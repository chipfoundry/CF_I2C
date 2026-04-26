`timescale 1ns/1ps

module top();
    reg     CLK = 0;
    reg     RESETn = 0;
    wire    irq;

    // I2C open-drain bus signals
    wire scl_i, scl_o, scl_oen_o;
    wire sda_i, sda_o, sda_oen_o;

    // Open-drain pull-up simulation
    wire SCL = scl_oen_o ? 1'bz : scl_o;
    wire SDA = sda_oen_o ? 1'bz : sda_o;
    pullup(SCL);
    pullup(SDA);
    assign scl_i = SCL;
    assign sda_i = SDA;

    `ifdef BUS_TYPE_APB
        wire        PCLK = CLK;
        wire        PRESETn = RESETn;
        wire [31:0] PADDR;
        wire        PWRITE;
        wire        PSEL;
        wire        PENABLE;
        wire [31:0] PWDATA;
        wire [31:0] PRDATA;
        wire        PREADY;
        CF_I2C_APB dut(
            .PCLK(PCLK), .PRESETn(PRESETn),
            .PADDR(PADDR), .PWRITE(PWRITE), .PSEL(PSEL),
            .PENABLE(PENABLE), .PWDATA(PWDATA), .PRDATA(PRDATA),
            .PREADY(PREADY),
            .scl_i(scl_i), .scl_o(scl_o), .scl_oen_o(scl_oen_o),
            .sda_i(sda_i), .sda_o(sda_o), .sda_oen_o(sda_oen_o),
            .IRQ(irq)
        );
    `endif

    `ifdef BUS_TYPE_WISHBONE
        wire        clk_i = CLK;
        wire        rst_i = ~RESETn;
        wire [31:0] adr_i;
        wire [31:0] dat_i;
        wire [31:0] dat_o;
        wire [3:0]  sel_i;
        wire        cyc_i;
        wire        stb_i;
        wire        we_i;
        wire        ack_o;
        CF_I2C_WB dut(
            .clk_i(clk_i), .rst_i(rst_i),
            .adr_i(adr_i), .dat_i(dat_i), .dat_o(dat_o),
            .sel_i(sel_i), .cyc_i(cyc_i), .stb_i(stb_i),
            .we_i(we_i), .ack_o(ack_o),
            .scl_i(scl_i), .scl_o(scl_o), .scl_oen_o(scl_oen_o),
            .sda_i(sda_i), .sda_o(sda_o), .sda_oen_o(sda_oen_o),
            .IRQ(irq)
        );
    `endif

    M24AA64 eeprom(
        .A0(1'b0), .A1(1'b0), .A2(1'b0), .WP(1'b0),
        .SDA(SDA), .SCL(SCL), .RESET(~RESETn)
    );

    initial begin
        #100 RESETn = 1;
    end

    `ifndef SKIP_WAVE_DUMP
        initial begin
            $dumpfile({"waves.vcd"});
            $dumpvars(0, top);
        end
    `endif
    always #10 CLK = !CLK;
endmodule
