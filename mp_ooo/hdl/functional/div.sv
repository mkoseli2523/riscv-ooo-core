module div
import rv32i_types::*;
#(
    parameter NUM_CYCLES = 12 // must be [3,a_width]
)(
    input   logic       clk,
    input   logic       rst,
    input   logic       br_flush,

    input   logic       stall_div,
    output  logic       div_in_progress,

    input   instr_pkt   ex_pkt,
    output  instr_pkt   div_pkt
);
    // RTDC for meaning of params...
    localparam a_width       = 33;
    localparam b_width       = 33;
    localparam rst_mode      = 0;
    localparam input_mode    = 1;
    localparam output_mode   = 1;
    localparam early_start   = 1;

            logic   [a_width-1:0] a, b;
            logic                 hold, start, complete;
            logic                 div_in_prog;
            logic   [a_width-1:0] quotient;
            logic   [b_width-1:0] remainder;
            logic                 divide_by_0;

            instr_pkt                div_out;

            logic   [31:0] rd_v;

    assign div_in_progress = start | (div_in_prog & ~complete);

    always_comb begin : OPERANDS
        {a, b} = 'x;

        if (ex_pkt.i_valid && ~div_in_prog && ~br_flush) begin
            unique case (ex_pkt.i_funct3)    // SIGNED [/][%] SIGNED
                m_op_div, m_op_rem : begin
                    a = {ex_pkt.rs1_data[31], ex_pkt.rs1_data};
                    b = {ex_pkt.rs2_data[31], ex_pkt.rs2_data};
                end
                m_op_divu, m_op_remu : begin // UNSIGNED [/][%] UNSIGNED
                    a = {1'b0, ex_pkt.rs1_data};
                    b = {1'b0, ex_pkt.rs2_data};
                end
                default:;
            endcase
        end
    end

    always_comb begin : START
        if (ex_pkt.i_valid && ~div_in_prog) begin
            start = 1'b1;
        end
        else begin
            start = 1'b0;
        end
    end

    always_ff @(posedge clk) begin : DIV_IN_PROG
        if (rst) begin
            div_in_prog <= '0;
        end
        else if (start) begin
            div_in_prog <= '1;
        end
        else if (complete | br_flush) begin
            div_in_prog <= '0;
        end
    end

    always_ff @(posedge clk) begin : DIV_OUT
        if (rst | br_flush) begin
            div_out.i_valid <= '0;
        end
        else if (ex_pkt.i_valid && ~div_in_prog) begin
            div_out <= ex_pkt;
        end
    end

    always_comb begin : RD_OUT
        rd_v = 'x;

        if (complete) begin
            if (divide_by_0) begin // CH 13.2 for div edge cases
                unique case (div_out.i_funct3)
                    m_op_div, m_op_divu : rd_v = '1;
                    m_op_rem, m_op_remu : rd_v = div_out.rs1_data;
                    default:;
                endcase
            end
            else begin
                unique case (div_out.i_funct3)
                    m_op_div, m_op_divu : rd_v = quotient[31:0];
                    m_op_rem, m_op_remu : rd_v = remainder[31:0];
                    default:;
                endcase
            end
        end
    end

    assign hold = stall_div;

    DW_div_seq #(
        a_width,
        b_width,
        1,      // tc_mode=1
        NUM_CYCLES,
        rst_mode,
        input_mode,
        output_mode,
        early_start
    ) div_i (
        .clk,
        .rst_n(~rst),
        .hold,
        .start,
        .a,
        .b,
        .complete,
        .divide_by_0,
        .quotient,
        .remainder
    );

    always_ff @(posedge clk) begin : DIV_PKT
        div_pkt         <= 'x;
        div_pkt.i_valid <= '0;

        if (complete & div_in_prog) begin
            div_pkt         <= div_out;
            div_pkt.i_valid <= '1;
            div_pkt.rd_data <= rd_v;
        end
    end

endmodule : div