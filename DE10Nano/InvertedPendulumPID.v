module InvertedPendulumPID_Top (
    // --- FPGA Pins ---
    input  wire        FPGA_CLK1_50,   // 50MHz clock
    output wire [7:0]  LED,            // Onboard LEDs

    // --- GPIO Pins (JP7 Header) ---
    output wire        GPIO_0_0,       // Step Pin
    output wire        GPIO_0_1,       // Dir Pin
    input  wire        GPIO_0_2,       // Home Switch

    //////////// HPS Pins //////////
    // Note: These must match the Pin Planner names exactly
    output   [14: 0]   HPS_DDR3_ADDR,
    output   [ 2: 0]   HPS_DDR3_BA,
    output             HPS_DDR3_CAS_N,
    output             HPS_DDR3_CK_N,
    output             HPS_DDR3_CK_P,
    output             HPS_DDR3_CKE,
    output             HPS_DDR3_CS_N,
    output   [ 3: 0]   HPS_DDR3_DM,
    inout    [31: 0]   HPS_DDR3_DQ,
    inout    [ 3: 0]   HPS_DDR3_DQS_N,
    inout    [ 3: 0]   HPS_DDR3_DQS_P,
    output             HPS_DDR3_ODT,
    output             HPS_DDR3_RAS_N,
    output             HPS_DDR3_RESET_N,
    input              HPS_DDR3_RZQ,
    output             HPS_DDR3_WE_N,
    output             HPS_ENET_GTX_CLK,
    output             HPS_ENET_MDC,
    inout              HPS_ENET_MDIO,
    input              HPS_ENET_RX_CLK,
    input    [ 3: 0]   HPS_ENET_RX_DATA,
    input              HPS_ENET_RX_DV,
    output   [ 3: 0]   HPS_ENET_TX_DATA,
    output             HPS_ENET_TX_EN,
    inout              HPS_I2C1_SCLK, // Built-in HPS I2C
    inout              HPS_I2C1_SDAT  // Built-in HPS I2C
);

// --- Internal Wires ---
wire hps_fpga_reset_n;

// --- Debugging Logic ---
assign LED[0] = GPIO_0_2; // LED0 lights up when Home Switch is triggered
assign LED[7:1] = 7'b1111111;    // Others on

// --- Qsys System Instantiation ---
InvertedPendulumPID u0 (
    .clk_clk                                  (FPGA_CLK1_50),
    
    // HPS DDR3
    .memory_mem_a                             (HPS_DDR3_ADDR),
    .memory_mem_ba                            (HPS_DDR3_BA),
    .memory_mem_ck                            (HPS_DDR3_CK_P),
    .memory_mem_ck_n                          (HPS_DDR3_CK_N),
    .memory_mem_cke                           (HPS_DDR3_CKE),
    .memory_mem_cs_n                          (HPS_DDR3_CS_N),
    .memory_mem_ras_n                         (HPS_DDR3_RAS_N),
    .memory_mem_cas_n                         (HPS_DDR3_CAS_N),
    .memory_mem_we_n                          (HPS_DDR3_WE_N),
    .memory_mem_reset_n                       (HPS_DDR3_RESET_N),
    .memory_mem_dq                            (HPS_DDR3_DQ),
    .memory_mem_dqs                           (HPS_DDR3_DQS_P),
    .memory_mem_dqs_n                         (HPS_DDR3_DQS_N),
    .memory_mem_odt                           (HPS_DDR3_ODT),
    .memory_mem_dm                            (HPS_DDR3_DM),
    .memory_oct_rzqin                         (HPS_DDR3_RZQ),

    // HPS Ethernet & I/O
    .hps_0_hps_io_hps_io_emac1_inst_TX_CLK    (HPS_ENET_GTX_CLK),
    .hps_0_hps_io_hps_io_emac1_inst_TXD0      (HPS_ENET_TX_DATA[0]),
    .hps_0_hps_io_hps_io_emac1_inst_TXD1      (HPS_ENET_TX_DATA[1]),
    .hps_0_hps_io_hps_io_emac1_inst_TXD2      (HPS_ENET_TX_DATA[2]),
    .hps_0_hps_io_hps_io_emac1_inst_TXD3      (HPS_ENET_TX_DATA[3]),
    .hps_0_hps_io_hps_io_emac1_inst_RXD0      (HPS_ENET_RX_DATA[0]),
    .hps_0_hps_io_hps_io_emac1_inst_MDIO      (HPS_ENET_MDIO),
    .hps_0_hps_io_hps_io_emac1_inst_MDC       (HPS_ENET_MDC),
    .hps_0_hps_io_hps_io_emac1_inst_RX_CTL    (HPS_ENET_RX_DV),
    .hps_0_hps_io_hps_io_emac1_inst_TX_CTL    (HPS_ENET_TX_EN),
    .hps_0_hps_io_hps_io_emac1_inst_RX_CLK    (HPS_ENET_RX_CLK),
    .hps_0_hps_io_hps_io_emac1_inst_RXD1      (HPS_ENET_RX_DATA[1]),
    .hps_0_hps_io_hps_io_emac1_inst_RXD2      (HPS_ENET_RX_DATA[2]),
    .hps_0_hps_io_hps_io_emac1_inst_RXD3      (HPS_ENET_RX_DATA[3]),

    // HPS Dedicated I2C1
    .hps_0_hps_io_hps_io_i2c1_inst_SDA        (HPS_I2C1_SDAT),
    .hps_0_hps_io_hps_io_i2c1_inst_SCL        (HPS_I2C1_SCLK),

    // Bridges & Resets
    .hps_0_h2f_reset_reset_n                  (hps_fpga_reset_n), 
    .reset_reset_n                            (hps_fpga_reset_n),

    // Stepper Driver Outputs
    .stepper_driver_0_conduit_end_step_pin    (GPIO_0_0),
    .stepper_driver_0_conduit_end_dir_pin     (GPIO_0_1),
    .stepper_driver_0_conduit_end_home_switch (GPIO_0_2) // No trailing comma here!
);

endmodule