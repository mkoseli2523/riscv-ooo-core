module RAT
import rv32i_types::*;
(
    input   logic                  clk,
    input   logic                  rst,

    // source arch reg indices
    input   logic [ARCH_WIDTH-1:0] arch_src1[PROCESSOR_WIDTH],
    input   logic [ARCH_WIDTH-1:0] arch_src2[PROCESSOR_WIDTH],

    // rename inputs
    input   logic                  rename_en[PROCESSOR_WIDTH],
    input   logic [ARCH_WIDTH-1:0] arch_dst[PROCESSOR_WIDTH],
    input   logic [PHYS_WIDTH-1:0] new_phys_dst[PROCESSOR_WIDTH],

    input   logic                  br_flush, // branch mispredict
    input   logic [PHYS_WIDTH-1:0] rrf_copy [ARCH_REGS],

    // output physical reg mappings
    output  logic [PHYS_WIDTH-1:0] phys_src1[PROCESSOR_WIDTH],
    output  logic [PHYS_WIDTH-1:0] phys_src2[PROCESSOR_WIDTH]
);

    // alias table: maps each arch-reg to a phys-reg
    logic [PHYS_WIDTH-1:0] rat_table [ARCH_REGS];

    always_ff @ (posedge clk) begin : OVERWRITE_MAPPING
        if (rst) begin
            for (integer unsigned i = 0; i < ARCH_REGS; i++) begin
                rat_table[i] <= PHYS_WIDTH'(i);
            end
        end
        else if (br_flush) begin
            for (integer i = 0; i < ARCH_REGS; i++) begin
                rat_table[i] <= rrf_copy[i];
            end
        end
        else begin
            for (integer i = 0; i < PROCESSOR_WIDTH; i++) begin
                if (rename_en[i]) begin
                    rat_table[arch_dst[i]] <= new_phys_dst[i];
                end
            end
        end
    end

    generate 
        for (genvar i = 0; i < PROCESSOR_WIDTH; i++) begin
            assign phys_src1[i] = rat_table[arch_src1[i]];
            assign phys_src2[i] = rat_table[arch_src2[i]];
        end
    endgenerate

endmodule : RAT