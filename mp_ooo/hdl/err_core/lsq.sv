module lsq
import rv32i_types::*;
(
    input  logic                 clk,
    input  logic                 rst,
    input  logic                 br_flush,

    input  logic                 enqueue,
    input  logic                 dequeue_store,
    input  logic                 is_store, // 1 for store 0 for load
    input  res_entry             rse_in,

    output instr_pkt             mem_pkt_out,

    output logic                 sq_full,
    output logic                 lq_full,

    output logic                 sq_empty,
    output logic                 lq_empty,

    input  logic                 stall_mem,
    input  logic                 stall_mem_latch,

    output logic [ROB_ADDR_WIDTH-1:0]   sq_rob_idx,

    // valid array + PRF(might need to move to separate signals for more complex LSQ)
    output  logic [PHYS_WIDTH-1:0]  lq_rs1_paddr_valid[LSQ_DEPTH],
    input   logic                   lq_rs1_valid[LSQ_DEPTH],

    output  logic [PHYS_WIDTH-1:0]  lsq_rs1_paddr,  // shared across both queues
    output  logic [PHYS_WIDTH-1:0]  lsq_rs2_paddr,

    input   logic [31:0]            lsq_rs1_data,
    input   logic [31:0]            lsq_rs2_data,
    
    output  instr_pkt               store_out,   // RVFI ONLY!!!!!!!
    output  logic [PHYS_WIDTH-1:0]  paddr1,
    output  logic [PHYS_WIDTH-1:0]  paddr2,
    input   logic [31:0]            paddr1_data,
    input   logic [31:0]            paddr2_data
);

    logic   sq_head_valid;

    // logic [PHYS_WIDTH-1:0]  lq_rs1_paddr;   // paddr for lq
    
    instr_pkt   sq_out, lq_out;

    logic       select_valid;
    logic       select_store;  // 1 for store 0 for load

    logic [LSQ_DEPTH-1:0]       store_valid;
    logic [LSQ_ADDR_WIDTH-1:0]  store_inserted_index;
    logic [LSQ_ADDR_WIDTH-1:0]  store_selected_index;

    sq_fifo sq (
        .clk(clk),
        .rst(rst | br_flush),

        .enqueue(enqueue && is_store),
        .dequeue(dequeue_store),
        .din(rse_in),

        .dout(sq_out),
        .full(sq_full),
        .empty(sq_empty),

        .sq_rob_idx(sq_rob_idx),
        .sq_head_valid(sq_head_valid),

        .sq_rs1_paddr(),
        .sq_rs2_paddr(),

        .store_valid(store_valid),
        .store_inserted_index(store_inserted_index),
        .store_selected_index(store_selected_index)
    );

    always_comb begin
        store_out = dequeue_store ? sq_out : '0;
        paddr1 = store_out.rs1_paddr;
        paddr2 = store_out.rs2_paddr;
        store_out.rs1_data = store_out.rs1_paddr == '0 ? '0 : paddr1_data;
        store_out.rs2_data = store_out.rs2_paddr == '0 ? '0 : paddr2_data;

        store_out.i_dmem_addr = store_out.rs1_data + store_out.imm_data;

        unique case (store_out.i_funct3)
            store_f3_sb: store_out.i_dmem_wdata[(8*store_out.i_dmem_addr[1:0])+:8] = store_out.rs2_data[7 :0];
            store_f3_sh: store_out.i_dmem_wdata[(16*store_out.i_dmem_addr[1])+:16] = store_out.rs2_data[15:0];
            store_f3_sw: store_out.i_dmem_wdata = store_out.rs2_data;
            default    : store_out.i_dmem_wdata = 'x;
        endcase
    end

    lq_collapsing_queue lq (
        .clk(clk),
        .rst(rst),

        .rs1_out(lq_rs1_paddr_valid),
        .rs1_valid(lq_rs1_valid),

        .enable(enqueue && !is_store), 
        .res_entry_in(rse_in),
        
        .stall_fu(stall_mem),
        .br_flush(br_flush),
        
        .res_full(lq_full),
        .res_empty(lq_empty),
        .instr_pkt_out(lq_out),

        .rs1_paddr(),

        .store_selected(dequeue_store),
        .store_inserted(enqueue && is_store),
        .store_selected_index(store_selected_index),
        .store_inserted_index(store_inserted_index),
        .store_valid(store_valid)
    );

    always_ff @(posedge clk) begin : DOUT_REG
        if (rst | br_flush) begin
            mem_pkt_out.i_valid <= '0;
        end
        else begin
            if (mem_pkt_out.i_valid && ~stall_mem) begin
                mem_pkt_out.i_valid <= 1'b0; 
            end
            
            if (select_valid && select_store) begin
                mem_pkt_out <= sq_out;
            end
            else if (select_valid) begin
                mem_pkt_out <= lq_out;
            end

            if (mem_pkt_out.i_valid && stall_mem && !stall_mem_latch) begin
                mem_pkt_out.rs1_data <= lsq_rs1_data;
                mem_pkt_out.rs2_data <= lsq_rs2_data;
            end
        end
    end
    
    always_comb begin : SELECT_ST_V_LD
        select_valid = 1'b0;
        select_store = 'x;

        if (dequeue_store) begin
            select_valid = 1'b1;
            select_store = 1'b1;
        end
        else if (lq_out.i_valid) begin
            select_valid = 1'b1;
            select_store = 1'b0;
        end
    end

    logic [PHYS_WIDTH-1:0]  rs1_paddr_temp;
    logic [PHYS_WIDTH-1:0]  rs2_paddr_temp; 

    always_comb begin : PADDR_REQ
        rs1_paddr_temp = 'x;
        rs2_paddr_temp = 'x;

        if (select_valid && select_store) begin
            rs1_paddr_temp = sq_out.rs1_paddr;
            rs2_paddr_temp = sq_out.rs2_paddr;
        end
        else if (select_valid) begin
            rs1_paddr_temp = lq_out.rs1_paddr;
            rs2_paddr_temp = lq_out.rs2_paddr;
        end
    end

    assign lsq_rs1_paddr = rs1_paddr_temp;
    assign lsq_rs2_paddr = rs2_paddr_temp;

endmodule : lsq