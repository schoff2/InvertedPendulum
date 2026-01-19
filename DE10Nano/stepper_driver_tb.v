`timescale 1ns/1ps

module stepper_driver_tb();
    reg clk, reset_n, avs_s0_write, avs_s0_read;
    reg [2:0] avs_s0_address;
    reg [31:0] avs_s0_writedata;
    wire [31:0] avs_s0_readdata;
    wire step_pin, dir_pin;
    reg home_switch;

    // Instantiate the module under test
    stepper_driver dut (
        .clk(clk), .reset_n(reset_n),
        .avs_s0_address(avs_s0_address), .avs_s0_write(avs_s0_write),
        .avs_s0_writedata(avs_s0_writedata), .avs_s0_read(avs_s0_read),
        .avs_s0_readdata(avs_s0_readdata), .step_pin(step_pin),
        .dir_pin(dir_pin), .home_switch(home_switch)
    );

    // 50MHz Clock Generation
    always #10 clk = ~clk;

    initial begin
        clk = 0; reset_n = 0; avs_s0_write = 0; home_switch = 0;
        #100 reset_n = 1;

        // Simulate HPS writing acceleration (Offset 0x08 -> Address 2)
        // Set accel to 500,000 steps/s^2 for faster simulation visibility
        #20; avs_s0_address = 3'd2; avs_s0_writedata = 32'd1000; avs_s0_write = 1;
        #20; avs_s0_write = 0;

        // Simulate HPS writing max speed (Offset 0x0C -> Address 3)
        #20; avs_s0_address = 3'd3; avs_s0_writedata = 32'd1000; avs_s0_write = 1;
        #20; avs_s0_write = 0;


        // Simulate HPS setting a target position (Offset 0x04 -> Address 1)
        #20; avs_s0_address = 3'd1; avs_s0_writedata = 32'd5000; avs_s0_write = 1;
        #20; avs_s0_write = 0;

        // Monitor Speed and Position
        repeat (20) begin
            // #1000000; // Wait 1ms
            #50000000; // Wait 50ms

            $display("Time: %t | Speed: %d | Pos: %d | StepPin: %b", $time, $signed(dut.current_speed), $signed(dut.position_counter), step_pin);
        end
        $stop;
    end
endmodule