module prf
import rv32i_types::*;
(
    input   logic                      clk,
    input   logic                      rst,

    input   logic   [PHYS_WIDTH-1:0]   rs1_paddr[NUM_RES_STATIONS],
    input   logic   [PHYS_WIDTH-1:0]   rs2_paddr[NUM_RES_STATIONS],

    input   logic   [PHYS_WIDTH-1:0]   rd_paddr[CDB_PORTS],
    input   logic   [31:0]             rd_data[CDB_PORTS],
    input   logic                      pregf_we[CDB_PORTS],

    output  logic   [31:0]             rs1_prdata[NUM_RES_STATIONS],
    output  logic   [31:0]             rs2_prdata[NUM_RES_STATIONS],

    input   logic   [PHYS_WIDTH-1:0]   store_paddr1,    // RVFI ONLY DO NOT USE
    input   logic   [PHYS_WIDTH-1:0]   store_paddr2,
    output  logic   [31:0]             store_paddr1_data,
    output  logic   [31:0]             store_paddr2_data
);
            logic   [31:0]  data [PHYS_REGS];

    always_ff @ (posedge clk) begin : UPDATE_PRF
        if (rst) begin
            for (integer i = 0; i < PHYS_REGS; i++)
                data[i] <= 'x;
        end
        else begin
            for (integer i = 0; i < CDB_PORTS; i++) begin
                if (pregf_we[i] && rd_paddr[i] != 0)
                    data[rd_paddr[i]] <= rd_data[i];
            end
        end
    end

    always_ff @ (posedge clk) begin : READ_PRF
        if (rst) begin
            for (integer i = 0; i < NUM_RES_STATIONS; i++) begin
                rs1_prdata[i] <= 'x;
                rs2_prdata[i] <= 'x;
            end
        end
        else begin
            for (integer i = 0; i < NUM_RES_STATIONS; i++) begin
                rs1_prdata[i] <= rs1_paddr[i] != 0 ? data[rs1_paddr[i]] : '0;
                rs2_prdata[i] <= rs2_paddr[i] != 0 ? data[rs2_paddr[i]] : '0;
            end
        end
    end
    
    assign store_paddr1_data = data[store_paddr1];
    assign store_paddr2_data = data[store_paddr2];

endmodule : prf
