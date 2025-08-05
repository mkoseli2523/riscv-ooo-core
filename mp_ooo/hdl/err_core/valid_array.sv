module valid_array
import rv32i_types::*;
(
    input logic                     clk,
    input logic                     rst,

    // rename (to invalidate)
    input logic                     rename_en[PROCESSOR_WIDTH],
    input logic   [PHYS_WIDTH-1:0]  rename_paddr[PROCESSOR_WIDTH],

    // broadcasted (to validate)
    input logic                     cdb_valid[CDB_PORTS],
    input logic   [PHYS_WIDTH-1:0]  cdb_paddr[CDB_PORTS],

    input logic                     stall_dispatch,

    input logic                     br_flush,
    input logic [PHYS_WIDTH-1:0]    rrf_copy [ARCH_REGS],

    // signals from/to RS
    input  logic  [PHYS_WIDTH-1:0]  rs1_source[NUM_RES_ENTRIES],
    input  logic  [PHYS_WIDTH-1:0]  rs2_source[NUM_RES_ENTRIES - NUM_MEM_RS_ENTRIES],
    output logic                    rs1_valid[NUM_RES_ENTRIES],
    output logic                    rs2_valid[NUM_RES_ENTRIES]
);
    logic [PHYS_REGS-1:0] valid_array;

    always_ff @(posedge clk) begin : UPDATE_VALID_ARRAY
        if (rst | br_flush) begin
            valid_array <= 'x;
            for (integer i = 0; i < ARCH_REGS; i++)
                valid_array[rrf_copy[i]] <= '1;
        end
        else begin
            for (integer i = 0; i < CDB_PORTS; i++) begin : BROADCAST_VALIDATE
                if (cdb_valid[i])
                    valid_array[cdb_paddr[i]] <= 1'b1;
            end

            for (integer i = 0; i < PROCESSOR_WIDTH; i++) begin : RENAME_INVALIDATE
                if (rename_en[i] && !stall_dispatch)
                    valid_array[rename_paddr[i]] <= 1'b0;
            end
        end
    end

    always_comb begin : READ_VALID_ARRAY_1
        for (integer i = 0; i < NUM_RES_ENTRIES; i++) begin
            rs1_valid[i] = valid_array[rs1_source[i]];
        end
    end

    always_comb begin : READ_VALID_ARRAY_2
        for (integer i = 0; i < NUM_RES_ENTRIES-NUM_MEM_RS_ENTRIES; i++) begin
            rs2_valid[i] = valid_array[rs2_source[i]];
        end
    end

endmodule : valid_array
