/**
    IMPORTANT NOTE (if going superscalar):
        - make sure input signals that are parametrized by "PROCESSOR_WIDTH", are set
        in-order (where most recent mapping is inputted as the last), to prevent 
        collision during a WAW hazard
*/

module RRF
import rv32i_types::*;
(
    input   logic                   clk,
    input   logic                   rst,

    // commit regs 
    input   logic                   commit_en[PROCESSOR_WIDTH],
    input   logic [ARCH_WIDTH-1:0]  arch_dst[PROCESSOR_WIDTH],
    input   logic [PHYS_WIDTH-1:0]  new_phys_dst[PROCESSOR_WIDTH],

    // branch misprediction
    // input   logic                   br_result,
    input   logic                   head_is_jmp[PROCESSOR_WIDTH],
    output  logic [PHYS_WIDTH-1:0]  rrf_copy [ARCH_REGS],

    // registers to boot out
    output logic [PHYS_WIDTH-1:0]   retired_phys_reg[PROCESSOR_WIDTH]
);

    logic [PHYS_WIDTH-1:0] rrf_table [ARCH_REGS];

    always_ff @ (posedge clk) begin : UPDATE_RRF
        if (rst) begin
            for (integer unsigned i = 0; i < ARCH_REGS; i++) begin
                rrf_table[i] <= PHYS_WIDTH'(i); // identity mapping on reset
            end
        end else begin
            for (integer i = 0; i < PROCESSOR_WIDTH; i++) begin
                if (commit_en[i] && arch_dst[i] != 0) begin
                    retired_phys_reg[i] <= rrf_table[arch_dst[i]];  // output preg to retire
                    rrf_table[arch_dst[i]] <= new_phys_dst[i];      // update mapping
                end else begin
                    retired_phys_reg[i] <= '0; // treat this as invalid or unused
                end
            end
        end
    end

    always_comb begin
        for (integer unsigned i = 0; i < ARCH_REGS; i++)
            rrf_copy[i] = rrf_table[i];

        for (integer unsigned n = 0; n < PROCESSOR_WIDTH; n++) begin // if we have a jmp, 
            if (head_is_jmp[n])                                      // forward the mapping to copy so it's not lost.
                rrf_copy[arch_dst[n]] = new_phys_dst[n];
        end
    end

endmodule : RRF