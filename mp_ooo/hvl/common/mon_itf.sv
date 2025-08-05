interface mon_itf#(
            parameter       CHANNELS = 1
)(
    input   bit             clk,
    input   bit             rst
);

            logic           valid     [CHANNELS];
            logic   [63:0]  order     [CHANNELS];
            logic   [31:0]  inst      [CHANNELS];
            logic   [4:0]   rs1_addr  [CHANNELS];
            logic   [4:0]   rs2_addr  [CHANNELS];
            logic   [31:0]  rs1_rdata [CHANNELS];
            logic   [31:0]  rs2_rdata [CHANNELS];
            logic   [4:0]   rd_addr   [CHANNELS];
            logic   [31:0]  rd_wdata  [CHANNELS];
            `ifndef ECE411_NO_FLOAT
                logic   [5:0]   frs1_addr [CHANNELS];
                logic   [5:0]   frs2_addr [CHANNELS];
                logic   [5:0]   frs3_addr [CHANNELS];
                logic   [31:0]  frs1_rdata[CHANNELS];
                logic   [31:0]  frs2_rdata[CHANNELS];
                logic   [31:0]  frs3_rdata[CHANNELS];
                logic   [5:0]   frd_addr  [CHANNELS];
                logic   [31:0]  frd_wdata [CHANNELS];
            `endif
            logic   [31:0]  pc_rdata  [CHANNELS];
            logic   [31:0]  pc_wdata  [CHANNELS];
            logic   [31:0]  mem_addr  [CHANNELS];
            logic   [3:0]   mem_rmask [CHANNELS];
            logic   [3:0]   mem_wmask [CHANNELS];
            logic   [31:0]  mem_rdata [CHANNELS];
            logic   [31:0]  mem_wdata [CHANNELS];

            bit             halt  = 1'b0;
            bit             error = 1'b0;

endinterface
