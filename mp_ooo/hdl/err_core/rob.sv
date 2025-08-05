module rob
import rv32i_types::*;
(
    input  logic                        clk,
    input  logic                        rst,

    // signals coming from dispatch
    input  logic                        enqueue[PROCESSOR_WIDTH],
    input  instr_pkt                    dp_pkt[PROCESSOR_WIDTH],

    // broadcasted instruction
    input  instr_pkt                    cdb_pkt[CDB_PORTS],

    // signals going out to RRF
    output logic   [ARCH_WIDTH-1:0]     areg_addr[PROCESSOR_WIDTH],
    output logic   [PHYS_WIDTH-1:0]     preg_addr[PROCESSOR_WIDTH],
    output logic                        commit_en[PROCESSOR_WIDTH],

    // signals going back to dispatch
    output logic   [ROB_ADDR_WIDTH-1:0] rob_idx[PROCESSOR_WIDTH],  // tells dispatch what rob idx to put in res_entry
    output logic                        rob_full[PROCESSOR_WIDTH],
    
    // info lsq needs
    output logic   [ROB_ADDR_WIDTH-1:0] head_rob_idx[PROCESSOR_WIDTH],
    output logic                        head_is_mem[PROCESSOR_WIDTH],
    input  logic                        stall_store,
    input  logic                        dequeue_store,

    // branch
    output logic                        br_flush,
    output logic    [31:0]              br_target,
    output logic    [63:0]              br_order,
    output logic    [FL_ADDR_WIDTH:0]   rollback_fl_head[PROCESSOR_WIDTH],
    output logic                        head_is_jmp[PROCESSOR_WIDTH],

    // predictor
    output logic                        br_commit,
    output logic                        br_pred,
    output logic                        br_result,
    output logic    [9:0]               br_commit_ghr,
    output logic    [9:0]               br_commit_idx,

    // rvfi
    output instr_pkt                    rvfi_out[PROCESSOR_WIDTH],
    input  instr_pkt                    store_in
);
    // reset to '0
    
    // on insert from dispatch, mark as busy

    // when cdb broadcasts, mark inst as not busy

    // when head of rob is not busy, pop and mark as committed

    // how to handle rollback???

    
    /**
        
        How do we expect this ROB to function?

            ALLOCATE space for instruction dispatching
                - if ROB is not full, allocate space for the instruction in dispatch
                - else need to stall dispatch
    
            COMMIT when an instruction at the head node is marked not busy

            Mark BROADCASTed instruction as not-busy
                - if broadcasting a valid instruction, mark the rob-idx of broadcasted
                instruction as not-busy

    */

            // queue with random access!
            instr_pkt                rob_queue [PROCESSOR_WIDTH][ROB_DEPTH];
            logic [ROB_ADDR_WIDTH:0] head[PROCESSOR_WIDTH];
            logic [ROB_ADDR_WIDTH:0] tail[PROCESSOR_WIDTH];
            logic                    rob_empty[PROCESSOR_WIDTH];

            logic [FL_ADDR_WIDTH:0]  rob_fl_head[PROCESSOR_WIDTH];

            logic                    head_consumes_preg[PROCESSOR_WIDTH];

    always_comb for (integer n = 0; n < PROCESSOR_WIDTH; n++)
        head_consumes_preg[n] = rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].i_uses_rd && (rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].rd_addr != '0);

    always_ff @(posedge clk) begin : UPDATE_ROB
        if (rst) begin
            for (integer n = 0; n < PROCESSOR_WIDTH; n++) begin
                head[n]   <= '0;
                tail[n]   <= '0;
                rob_fl_head[n] <= '0;

                for (integer i = 0; i < ROB_DEPTH; i++) begin
                    rob_queue[n][i].rob_ready <= '0;
                end
            end
        end else begin
            for (integer n = 0; n < PROCESSOR_WIDTH; n++) begin
                // cdb broadcast
                for (integer i = 0; i < CDB_PORTS; i++) begin
                    if (cdb_pkt[i].i_valid) begin
                        rob_queue[n][cdb_pkt[i].i_rob_idx].rob_ready <= '1;
                        rob_queue[n][cdb_pkt[i].i_rob_idx].rd_data <= cdb_pkt[i].rd_data;
                        rob_queue[n][cdb_pkt[i].i_rob_idx].rs1_data <= cdb_pkt[i].rs1_data;
                        rob_queue[n][cdb_pkt[i].i_rob_idx].rs2_data <= cdb_pkt[i].rs2_data;

                        rob_queue[n][cdb_pkt[i].i_rob_idx].i_dmem_addr  <= cdb_pkt[i].i_dmem_addr;
                        rob_queue[n][cdb_pkt[i].i_rob_idx].i_dmem_rmask <= cdb_pkt[i].i_dmem_rmask;
                        // rob_queue[n][cdb_pkt[i].i_rob_idx].i_dmem_wmask <= cdb_pkt[i].i_dmem_wmask;
                        rob_queue[n][cdb_pkt[i].i_rob_idx].i_dmem_rdata <= cdb_pkt[i].i_dmem_rdata;
                        // rob_queue[n][cdb_pkt[i].i_rob_idx].i_dmem_wdata <= cdb_pkt[i].i_dmem_wdata;

                        rob_queue[n][cdb_pkt[i].i_rob_idx].br_result <= cdb_pkt[i].br_pred;
                        rob_queue[n][cdb_pkt[i].i_rob_idx].br_result <= cdb_pkt[i].br_result;
                        rob_queue[n][cdb_pkt[i].i_rob_idx].br_target <= cdb_pkt[i].br_target;
                        rob_queue[n][cdb_pkt[i].i_rob_idx].i_pc_next <= cdb_pkt[i].i_pc_next;
                    end
                end

                // insert inst from dispatch
                if (enqueue[n] && !rob_full[n]) begin
                    rob_queue[n][tail[n][ROB_ADDR_WIDTH-1:0]] <= dp_pkt[n];  // % depth
                    rob_queue[n][tail[n][ROB_ADDR_WIDTH-1:0]].rob_ready <= '0;
                    
                    tail[n] <= tail[n] + 1'b1;
                end

                // commit
                if (!rob_empty[n] && (rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].rob_ready || (head_is_mem[n] && dequeue_store))) begin
                    rob_fl_head[n] <= head_consumes_preg[n] ? (rob_fl_head[n] + 1'b1) : rob_fl_head[n];
                    
                    // check if branch misprediction
                    // in this case (static not taken), a branch taken is a mispredict
                    if (br_flush) begin
                        head[n]   <= '0;
                        tail[n]   <= '0;

                        for (integer i = 0; i < ROB_DEPTH; i++) begin
                            rob_queue[n][i].rob_ready <= '0;
                        end
                    end
                    else begin
                        head[n] <= head[n] + 1'b1;
                        rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].rob_ready <= 1'b0;
                    end
                end
            end
        end
    end

    generate 
        for (genvar n = 0; n < PROCESSOR_WIDTH; n++) begin
            assign areg_addr[n] = (rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].i_uses_rd) ? rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].rd_addr : '0;
            assign preg_addr[n] = rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].rd_paddr;
            assign commit_en[n] = rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].rob_ready || (head_is_mem[n] && dequeue_store);
            assign rob_empty[n] = (head[n] == tail[n] && !rob_full[n]);
            assign rob_full[n]  = (head[n][ROB_ADDR_WIDTH-1:0] == tail[n][ROB_ADDR_WIDTH-1:0]) && (head[n][ROB_ADDR_WIDTH] != tail[n][ROB_ADDR_WIDTH]);
            assign rob_idx[n]   = tail[n][ROB_ADDR_WIDTH-1:0];

            assign br_flush            = rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].rob_ready && ((rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].br_result != 
                                                                                                rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].br_pred));
            assign br_target           = br_flush ? (rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].br_result ?
                                         rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].br_target : rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].i_pc_next) : 'x;
            assign rollback_fl_head[n] = br_flush ? (head_consumes_preg[n] ? (rob_fl_head[n] + 1'b1) : rob_fl_head[n]) : 'x;
            assign br_order            = br_flush ? rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].i_order : 'x;
            assign head_is_jmp[n]      = !rob_empty[n] && rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].i_valid
                                    &&  ((rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].i_opcode == op_b_jalr));

            assign head_rob_idx[n] = rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].i_rob_idx;
            assign head_is_mem[n]  = !rob_empty[n] && rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].i_valid
                                  && ((rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].i_opcode == op_b_store)
                                  && !stall_store);

            // GShare
            assign br_commit = rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].i_opcode == op_b_br && commit_en[n];
            assign br_pred = rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].br_pred;
            assign br_result = rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].br_result;
            assign br_commit_ghr = rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].br_commit_ghr;
            assign br_commit_idx = br_commit_ghr ^ rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]].i_pc[11:2];
        end
    endgenerate

    always_comb begin : ASSIGN_RVFI
        // accomodate for store address and wmask for rvfi 
        // those won't propogate to the output
        for (integer n = 0; n < PROCESSOR_WIDTH; n++) begin
            rvfi_out[n] = rob_queue[n][head[n][ROB_ADDR_WIDTH-1:0]];

            if (rvfi_out[n].i_valid && rvfi_out[n].i_opcode == op_b_store && rvfi_out[n].i == store_in.i && dequeue_store) begin
                rvfi_out[n] = store_in;
                rvfi_out[n].i_dmem_rmask = '0;
                
                // rvfi_out[n].i_dmem_addr = rvfi_out[n].rs1_data + rvfi_out[n].imm_data;
            
                unique case (rvfi_out[n].i_funct3)
                    store_f3_sb: rvfi_out[n].i_dmem_wmask = 4'b0001 << rvfi_out[n].i_dmem_addr[1:0];
                    store_f3_sh: rvfi_out[n].i_dmem_wmask = 4'b0011 << rvfi_out[n].i_dmem_addr[1:0];
                    store_f3_sw: rvfi_out[n].i_dmem_wmask = 4'b1111;
                    default    : rvfi_out[n].i_dmem_wmask = 'x;
                endcase
                // unique case (rvfi_out[n].i_funct3)
                //     store_f3_sb: rvfi_out[n].i_dmem_wdata[(8*rvfi_out[n].i_dmem_addr[1:0])+:8] = rvfi_out[n].rs2_data[7 :0];
                //     store_f3_sh: rvfi_out[n].i_dmem_wdata[(16*rvfi_out[n].i_dmem_addr[1])+:16] = rvfi_out[n].rs2_data[15:0];
                //     store_f3_sw: rvfi_out[n].i_dmem_wdata = rvfi_out[n].rs2_data;
                //     default    : rvfi_out[n].i_dmem_wdata = 'x;
                // endcase
                rvfi_out[n].i_dmem_addr[1:0] = '0;
            end
        end 
    end

endmodule : rob
