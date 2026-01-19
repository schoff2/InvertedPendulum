module stepper_driver (
    // Standard Avalon-MM Signals
    input  wire        clk,                // 50MHz Clock
    input  wire        reset_n,            // Active Low Reset
    
    // Avalon-MM Slave Interface
    input  wire [3:0]  avs_s0_address,     // Address
    input  wire        avs_s0_write,       // Write Enable
    input  wire [31:0] avs_s0_writedata,   // Data from HPS
    input  wire        avs_s0_read,        // Read Enable
    output reg  [31:0] avs_s0_readdata,    // Data to HPS
    output wire        avs_s0_waitrequest,

    // Physical Conduits (Exported to Pins)
    output reg         step_pin,           // Connected to Stepper STEP
    output reg         dir_pin,            // Connected to Stepper DIR
    input  wire        home_switch         // Connected to NC/NO Switch
);

// --- Configuration Registers ---
    reg [31:0] target_speed, target_pos, accel_val, max_speed_val;
    reg [31:0] min_limit, max_limit;
    reg [31:0] current_speed; // The internal ramped speed
    reg [31:0] position_counter, step_accumulator;
    reg        mode_select; // 0 = Speed Mode, 1 = Position Mode
    reg        home_switch_triggered;
    
    // --- Constant for 50MHz Clock ---
    // Mapping speed to frequency: 
    // Pulse frequency = (abs_speed * 2^32) / 50,000,000
    // We use a simplified accumulator constant to achieve target Hz
    localparam ACCUM_CONST = 32'd86;       // Calibration for 50MHz

    // --- Avalon Slave Interface: Read/Write Logic ---
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            // Reset all regs
            target_speed <= 32'd0;
            target_pos   <= 32'd0;
            accel_val    <= 32'd1000;
            max_speed_val<= 32'd1000;
            min_limit    <= 32'h80000000;
            max_limit    <= 32'h7FFFFFFF;
            mode_select <= 1'b0;
            home_switch_triggered <= 1'b0;
        end else begin
            home_switch_triggered <= home_switch;
            // Write from HPS
            if (avs_s0_write) begin
                case (avs_s0_address)
                    4'd0: begin target_speed <= avs_s0_writedata; mode_select <= 0; end
                    4'd1: begin target_pos   <= avs_s0_writedata; mode_select <= 1; end
                    4'd2: accel_val     <= avs_s0_writedata;
                    4'd3: max_speed_val <= avs_s0_writedata;
                    4'd4: min_limit     <= avs_s0_writedata;
                    4'd5: max_limit     <= avs_s0_writedata;
                    // Skip 4'd6
                    // Write to 4'd7 in step Pulse Generation block
                endcase
            end
            
            // Read to HPS
            if (avs_s0_read) begin
                case (avs_s0_address)
                    4'd0: avs_s0_readdata <= target_speed;
                    4'd1: avs_s0_readdata <= target_pos;
                    4'd2: avs_s0_readdata <= accel_val;
                    4'd3: avs_s0_readdata <= max_speed_val;
                    4'd4: avs_s0_readdata <= min_limit;
                    4'd5: avs_s0_readdata <= max_limit;
                    4'd6: avs_s0_readdata <= current_speed;
                    4'd7: avs_s0_readdata <= position_counter;
                    4'd8: avs_s0_readdata <= home_switch_triggered;
                    default: avs_s0_readdata <= 32'hDEADBEEF;
                endcase
            end
        end
    end

    // --- Position & Speed Control Logic ---
    wire signed [31:0] dist_error = $signed(target_pos) - $signed(position_counter);
    wire signed [31:0] v_curr_s   = $signed(current_speed);
    
    // Position Mode: Braking Check
    wire [63:0] dist_abs_64 = (dist_error < 0) ? -dist_error : dist_error;
    wire [63:0] v_sq_64     = v_curr_s * v_curr_s;
    wire moving_towards = (v_curr_s != 0) && ((dist_error > 0) == (v_curr_s > 0));
    // Braking Threshold: v^2 >= 2 * a * d
    wire need_to_brake  = moving_towards && (v_sq_64 >= (64'd2 * accel_val * dist_abs_64));
    
    wire signed [31:0] pos_target_vel = (dist_error == 0) ? 32'd0 :
                                        (need_to_brake)   ? 32'd0 :
                                        (dist_error > 0)  ? $signed(max_speed_val) : -$signed(max_speed_val);

    // Mode Selection
    wire signed [31:0] raw_target = (mode_select) ? pos_target_vel : $signed(target_speed);

    // Safety Limits
    // If at limit, force target to 0 or away from limit
    wire limit_pos = ($signed(position_counter) >= $signed(max_limit));
    wire limit_neg = ($signed(position_counter) <= $signed(min_limit));
    
    wire signed [31:0] final_target = (raw_target > 0 && limit_pos) ? 32'd0 :
                                      (raw_target < 0 && limit_neg) ? 32'd0 :
                                      raw_target;

    // --- Acceleration Ramping Logic ---
    reg [31:0] ramp_timer;
    reg [15:0] speed_fraction;

    // Calculate increment based on accel_val (steps/s^2) and update rate (65536 Hz)
    // We use a fractional accumulator for the timer to achieve exactly 65536 Hz average update rate
    
    // Explicitly extend operands to 32-bit to ensure addition carries correctly
    wire [31:0] accel_fraction = accel_val & 32'hFFFF;
    wire [31:0] speed_frac_ext = {16'b0, speed_fraction};
    wire [31:0] frac_sum       = speed_frac_ext + accel_fraction;

    wire [31:0] step_inc  = (accel_val >> 16) + (frac_sum >> 16);
    wire [15:0] next_frac = frac_sum[15:0];

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            ramp_timer <= 0;
            current_speed <= 0;
            speed_fraction <= 0;
        end else begin
            ramp_timer <= ramp_timer + 1;
            if (ramp_timer >= 32'd763) begin // ~65.5 kHz Update
                ramp_timer <= 0;
            
                // Ramp current_speed towards final_target
                if ($signed(current_speed) < final_target) begin
                    // Accelerate
                    if (($signed(current_speed) + $signed(step_inc)) >= final_target)
                        current_speed <= final_target;
                    else
                        current_speed <= current_speed + step_inc;
                    speed_fraction <= next_frac;
                end else if ($signed(current_speed) > final_target) begin
                    // Decelerate
                    if (($signed(current_speed) - $signed(step_inc)) <= final_target)
                        current_speed <= final_target;
                    else
                        current_speed <= current_speed - step_inc;
                    speed_fraction <= next_frac;
                end else begin
                    // At target
                    speed_fraction <= 0;
                end
            end
        end
    end

    // --- Motor Control & Pulse Generation ---
    wire [31:0] abs_speed = (current_speed[31]) ? (~current_speed + 32'd1) : current_speed;
    wire [31:0] increment = 2 * abs_speed * ACCUM_CONST;
    wire [32:0] next_sum = step_accumulator + increment; // Use 33 bits to see the carry

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            step_accumulator <= 32'd0;
            position_counter <= 32'd0;
            step_pin <= 1'b0;
            dir_pin  <= 1'b0;
        end else begin
            if (abs_speed > 0) begin
            // Check limits before generating pulses
            // If moving positive (current_speed > 0) and limit_pos hit -> STOP
            // If moving negative (current_speed < 0) and limit_neg hit -> STOP
            if ((!current_speed[31] && limit_pos) || (current_speed[31] && limit_neg)) begin
                 step_accumulator <= 32'd0;
                 step_pin <= 1'b0;
            end else begin
                step_accumulator <= next_sum[31:0];
                dir_pin <= current_speed[31];

                // Use the 33rd bit (the carry out) to trigger the step
                if (next_sum[32]) begin 
                    step_pin <= ~step_pin;
        
                    // Increment position only on the rising edge of the step pulse
                    if (step_pin == 1'b0) begin
                        if (dir_pin) position_counter <= position_counter - 32'd1;
                        else         position_counter <= position_counter + 32'd1;
                    end
                end
            end
            end else begin
                 step_accumulator <= 32'd0;
                 step_pin <= 1'b0;
            end

            // Allow HPS to overwrite position (e.g. homing)
            if (avs_s0_write && (avs_s0_address == 4'd7)) begin
                position_counter <= avs_s0_writedata;
            end
        end
    end
    
    
    assign avs_s0_waitrequest = 1'b0;

endmodule