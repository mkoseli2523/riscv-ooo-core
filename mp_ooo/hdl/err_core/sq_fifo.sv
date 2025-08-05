module sq_fifo
import rv32i_types::*;
(
    input   logic                       clk,
    input   logic                       rst,

    input   logic                       enqueue,
    input   logic                       dequeue,
    input   res_entry                   din,

    output  instr_pkt                   dout,
    output  logic                       full,
    output  logic                       empty,

    output  logic [ROB_ADDR_WIDTH-1:0]  sq_rob_idx,
    output  logic                       sq_head_valid,

    output  logic [PHYS_WIDTH-1:0]      sq_rs1_paddr,
    output  logic [PHYS_WIDTH-1:0]      sq_rs2_paddr,

    output  logic [LSQ_DEPTH-1:0]       store_valid,
    output  logic [LSQ_ADDR_WIDTH-1:0]  store_inserted_index,
    output  logic [LSQ_ADDR_WIDTH-1:0]  store_selected_index
);

    res_entry queue [LSQ_DEPTH];
    logic [LSQ_ADDR_WIDTH:0] head;
    logic [LSQ_ADDR_WIDTH:0] tail;

    always_ff @(posedge clk) begin : MODIFY_QUEUE
        if (rst) begin
            head <= '0;
            tail <= '0;

            for (integer i = 0; i < LSQ_DEPTH; i++) begin
                queue[i].valid <= '0;
                // queue[i].age <= 2**AGE_COUNTER_WIDTH-1;
                queue[i].stall <= 'x;
            end
        end
        else begin
            if (!empty && dequeue) begin
                // mem_pkt_out <= queue[head[LSQ_ADDR_WIDTH-1:0]].inst;
                queue[head[LSQ_ADDR_WIDTH-1:0]].valid   <= 1'b0;
            
                head <= head + 1'b1;
            end

            if (!full && enqueue) begin
                queue[tail[LSQ_ADDR_WIDTH-1:0]].inst  <= din.inst;
                // queue[tail[LSQ_ADDR_WIDTH-1:0]].age   <= age;
                queue[tail[LSQ_ADDR_WIDTH-1:0]].valid <= 1'b1;
                
                tail <= tail + 1'b1;
            end
        end
    end

    assign sq_rob_idx       = empty ? '0 : queue[head[LSQ_ADDR_WIDTH-1:0]].inst.i_rob_idx;
    assign sq_rs1_paddr     = dout.rs1_paddr;
    assign sq_rs2_paddr     = dout.rs2_paddr;
    assign dout             = queue[head[LSQ_ADDR_WIDTH-1:0]].inst;
    assign sq_head_valid    = queue[head[LSQ_ADDR_WIDTH-1:0]].valid;

    assign empty = (head == tail && !full);
    assign full  = (head[LSQ_ADDR_WIDTH-1:0] == tail[LSQ_ADDR_WIDTH-1:0]) && (head[LSQ_ADDR_WIDTH] != tail[LSQ_ADDR_WIDTH]);

    // assign sq_age_oldest    = queue[head[LSQ_ADDR_WIDTH-1:0]].age;

    generate
        for (genvar i = 0; i < LSQ_DEPTH; i++) begin
            assign store_valid[i] = queue[i].valid;
        end
    endgenerate

    assign store_inserted_index = tail[LSQ_ADDR_WIDTH-1:0];
    assign store_selected_index = head[LSQ_ADDR_WIDTH-1:0];

endmodule