module mul
import rv32i_types::*;
#(
    parameter NUM_STAGES = 6 // must be >= 3
)(
    input   logic       clk,
    input   logic       rst,

    input   instr_pkt   ex_pkt,
    output  instr_pkt   mul_pkt
);
    // RTDC for meaning of params...
    localparam a_width       = 33;
    localparam b_width       = 33;
    localparam stall_mode    = 1;
    localparam rst_mode      = 0;

            logic   [a_width-1:0]         a, b;
            logic                         en;
            logic   [a_width+b_width-1:0] product;

            instr_pkt                mul_pp[NUM_STAGES-1];
            logic   [NUM_STAGES-2:0] valid_muls;

            logic   [31:0] rd_v;

    //      Shift Register Pipeline (N = NUM_STAGES)

    //             +--------+  +--------+       +--------+
    // ex_pkt ---> | N-2    |->| N-3    |-> ... |   0    | ---> mul_pkt
    //             |        |  |        |       |        |
    //             +--------+  +--------+       +--------+
    //
    always_ff @ (posedge clk) begin : MUL_PP_REGS
        if (rst) begin
            for (integer i = 0; i < NUM_STAGES-1; i++)
                mul_pp[i].i_valid <= '0;
        end
        else begin
            mul_pp[NUM_STAGES-2] <= ex_pkt;
            for (integer i = 0; i < NUM_STAGES-2; i++)
                mul_pp[i] <= mul_pp[i+1];
        end
    end

    always_comb begin : OPERANDS
        {a, b} = 'x;

        unique case (ex_pkt.i_funct3)
            m_op_mul, m_op_mulh : begin // SIGNED * SIGNED
                a = {ex_pkt.rs1_data[31], ex_pkt.rs1_data};
                b = {ex_pkt.rs2_data[31], ex_pkt.rs2_data};
            end
            m_op_mulhsu : begin         // SIGNED * UNSIGNED
                a = {ex_pkt.rs1_data[31], ex_pkt.rs1_data};
                b = {1'b0, ex_pkt.rs2_data};                
            end 
            m_op_mulhu  : begin         // UNSIGNED * UNSIGNED
                a = {1'b0, ex_pkt.rs1_data};
                b = {1'b0, ex_pkt.rs2_data}; 
            end
            default:;
        endcase
    end

    always_comb begin : VALID_MUL_ONGOING
        for (integer i = 0; i < NUM_STAGES-1;i++)
            valid_muls[i] = mul_pp[i].i_valid;
    end

    assign en = ex_pkt.i_valid || |valid_muls;

    always_comb begin : RD_OUT
        unique case (mul_pp[0].i_funct3)
            m_op_mul    : rd_v = product[31:0];
            m_op_mulh   : rd_v = product[63:32];
            m_op_mulhsu : rd_v = product[63:32];
            m_op_mulhu  : rd_v = product[63:32];
            default     : rd_v = 'x;
        endcase
    end

    DW_mult_pipe #(
        a_width,
        b_width,
        NUM_STAGES,
        stall_mode,
        rst_mode
    ) mult_i (
        .clk,
        .rst_n('x),
        .en,
        .tc('1),
        .a,
        .b,
        .product
    );
    
    always_comb begin
        mul_pkt         = mul_pp[0];
        mul_pkt.rd_data = rd_v;
    end


endmodule : mul