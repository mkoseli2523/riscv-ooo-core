module cpu
import rv32i_types::*;
(
    input   logic               clk,
    input   logic               rst,

    output  logic   [31:0]      bmem_addr,
    output  logic               bmem_read,
    output  logic               bmem_write,
    output  logic   [63:0]      bmem_wdata,
    input   logic               bmem_ready,

    input   logic   [31:0]      bmem_raddr,
    input   logic   [63:0]      bmem_rdata,
    input   logic               bmem_rvalid
);
        // Fetch
        logic   [31:0]      icache_dfp_addr;
        logic               icache_dfp_read;
        logic               icache_dfp_write;
        logic   [255:0]     icache_dfp_rdata;
        logic   [255:0]     icache_dfp_wdata;
        logic               icache_dfp_resp;

        // branch / misprediction
        logic                     pc_sel;  // high when a taken branch is committed
        logic   [31:0]            br_pc;
        logic   [63:0]            br_order;
        logic   [FL_ADDR_WIDTH:0] rollback_fl_head[PROCESSOR_WIDTH];
        logic   [PHYS_WIDTH-1:0]  rrf_copy [ARCH_REGS];

        // Instruction queue
        logic               iq_enqueue;
        logic               iq_dequeue;
        iq_entry            iq_in;
        iq_entry            iq_out;
        logic               iq_full;
        logic               iq_empty;

        // RAT
        logic [ARCH_WIDTH-1:0]  arch_src1[PROCESSOR_WIDTH];
        logic [ARCH_WIDTH-1:0]  arch_src2[PROCESSOR_WIDTH];

        logic                   rename_en[PROCESSOR_WIDTH];
        logic [ARCH_WIDTH-1:0]  rat_arch_dst[PROCESSOR_WIDTH];
        logic [PHYS_WIDTH-1:0]  rat_phys_dst[PROCESSOR_WIDTH];

        logic [PHYS_WIDTH-1:0]  phys_src1[PROCESSOR_WIDTH];
        logic [PHYS_WIDTH-1:0]  phys_src2[PROCESSOR_WIDTH];

        // Free-list
        logic                  fl_enqueue;
        logic [PHYS_WIDTH-1:0] fl_preg_addr_in[PROCESSOR_WIDTH];
        logic                  fl_preg_request;
        logic [PHYS_WIDTH-1:0] fl_preg_addr_out;
        logic                  fl_full;
        logic                  fl_empty;

        // ROB
        logic                        rob_enqueue[PROCESSOR_WIDTH];
        logic   [ROB_ADDR_WIDTH-1:0] rob_idx[PROCESSOR_WIDTH];
        logic                        rob_full[PROCESSOR_WIDTH];
        instr_pkt                    rob_pkt[PROCESSOR_WIDTH];
        logic                        commit_en[PROCESSOR_WIDTH];
        logic   [ROB_ADDR_WIDTH-1:0] head_rob_idx[PROCESSOR_WIDTH];
        logic                        head_is_mem[PROCESSOR_WIDTH];
        logic                        head_is_jmp[PROCESSOR_WIDTH];

        // RRF
        logic [ARCH_WIDTH-1:0]  rrf_arch_dst[PROCESSOR_WIDTH];
        logic [PHYS_WIDTH-1:0]  rrf_new_phys_dst[PROCESSOR_WIDTH];
        logic [PHYS_WIDTH-1:0]  retired_phys_reg[PROCESSOR_WIDTH];

        // Dispatch
        logic                   stall_dispatch;
        res_entry               dp_res_entry;

        // Valid array
        logic   [PHYS_WIDTH-1:0]  rs1_source[NUM_RES_ENTRIES];
        logic   [PHYS_WIDTH-1:0]  rs2_source[NUM_RES_ENTRIES - NUM_MEM_RS_ENTRIES];
        logic                     rs1_valid[NUM_RES_ENTRIES];
        logic                     rs2_valid[NUM_RES_ENTRIES];
        logic                     cdb_valid[CDB_PORTS];
        logic   [PHYS_WIDTH-1:0]  cdb_paddr[CDB_PORTS];

        // RS
        logic                   res_full[NUM_RES_STATIONS];

            // BR MUL DIV RS
            logic [PHYS_WIDTH-1:0]  bmd_rs_rs1_out[NUM_BMD_RS_ENTRIES];
            logic [PHYS_WIDTH-1:0]  bmd_rs_rs2_out[NUM_BMD_RS_ENTRIES];
            logic                   bmd_rs_rs1_valid[NUM_BMD_RS_ENTRIES];
            logic                   bmd_rs_rs2_valid[NUM_BMD_RS_ENTRIES];
            
            logic       bmd_rs_en;
            logic       bmd_rs_full;
            instr_pkt   bmd_rs_instr_pkt;

            logic [PHYS_WIDTH-1:0]      bmd_rs1_paddr;
            logic [PHYS_WIDTH-1:0]      bmd_rs2_paddr;

            // ALU RS
            logic [PHYS_WIDTH-1:0]  alu_rs_rs1_out[NUM_ALU_RS_ENTRIES];
            logic [PHYS_WIDTH-1:0]  alu_rs_rs2_out[NUM_ALU_RS_ENTRIES];
            logic                   alu_rs_rs1_valid[NUM_ALU_RS_ENTRIES];
            logic                   alu_rs_rs2_valid[NUM_ALU_RS_ENTRIES];
            
            logic       alu_rs_en;
            logic       alu_rs_full;
            instr_pkt   alu_rs_instr_pkt;

            // LSQ
            logic [PHYS_WIDTH-1:0]     lsq_rs_rs1_out[NUM_MEM_RS_ENTRIES];
            // logic [PHYS_WIDTH-1:0]     lsq_rs_rs2_out[NUM_MEM_RS_ENTRIES];
            logic                      lsq_rs_rs1_valid[NUM_MEM_RS_ENTRIES];
            logic                      lsq_rs_rs2_valid[NUM_MEM_RS_ENTRIES];

            logic                      dequeue_store;
            instr_pkt                  lsq_out;
            logic                      lsq_full;
            logic                      lsq_empty;
            logic [ROB_ADDR_WIDTH-1:0] sq_rob_idx;
            logic                      lsq_head_is_ld;

        logic [PHYS_WIDTH-1:0]      alu_rs1_paddr;
        logic [PHYS_WIDTH-1:0]      alu_rs2_paddr;

        // PRF
        logic   [PHYS_WIDTH-1:0]   rs1_paddr[NUM_RES_STATIONS];
        logic   [PHYS_WIDTH-1:0]   rs2_paddr[NUM_RES_STATIONS];

        logic   [PHYS_WIDTH-1:0]   rd_paddr[CDB_PORTS];
        logic   [31:0]             rd_data[CDB_PORTS];
        logic                      pregf_we[CDB_PORTS];

        logic   [31:0]             rs1_prdata[NUM_RES_STATIONS];
        logic   [31:0]             rs2_prdata[NUM_RES_STATIONS];

        // Execute
        logic     [2:0]   stall_br_mul_div; // one-hot encoded
        logic             stall_alu;
        logic             stall_load;
        logic             stall_mem_latch;
        logic             stall_store;
        logic             stall_mem;

        logic   [31:0]    dcache_dfp_addr;
        logic             dcache_dfp_read;
        logic             dcache_dfp_write;
        logic   [255:0]   dcache_dfp_rdata;
        logic   [255:0]   dcache_dfp_wdata;
        logic             dcache_dfp_resp;

        cdb_t             ex_out[CDB_PORTS];

        // GShare
        logic br_commit;
        logic br_pred;
        logic br_result;
        logic [9:0] br_commit_ghr;
        logic [9:0] br_commit_idx;

        // RVFI
        instr_pkt   rvfi_out[PROCESSOR_WIDTH];
        instr_pkt   store_out;

        id_dp_t id_dp_reg, id_dp_reg_next;

    always_ff @ (posedge clk) begin : PIPELINE_REGS
        if (rst) begin
            id_dp_reg.inst.i_valid <= '0;
        end
        else if (pc_sel) begin  // flush i_queue, bubble to dispatch
            id_dp_reg.inst.i_valid <= '0;
        end
        else if (!stall_dispatch) begin
            id_dp_reg <= id_dp_reg_next;
        end
    end

    fetch fetch_i (
        .clk,
        .rst,
        .stall_pc(iq_full),
        .pc_sel,
        .br_pc,
        .br_order,
    
        .dfp_addr(icache_dfp_addr),
        .dfp_read(icache_dfp_read),
        .dfp_write(icache_dfp_write),
        .dfp_rdata(icache_dfp_rdata),
        .dfp_wdata(icache_dfp_wdata),
        .dfp_resp(icache_dfp_resp),
        
        .br_commit_valid(br_commit),
        .br_commit_result(br_result),
        .br_commit_idx(br_commit_idx),
        .br_commit_mispredict(br_commit && (br_pred != br_result)),
        .br_commit_ghr(br_commit_ghr),
        
        .iq_enqueue,
        .iq_in
    );

    assign iq_dequeue = !stall_dispatch;

    iqueue i_queue (
        .clk,
        .rst(rst | pc_sel),

        .enqueue(iq_enqueue),
        .dequeue(iq_dequeue),
        .din(iq_in),

        .dout(iq_out),
        .full(iq_full),
        .empty(iq_empty),

        .stall_dispatch(stall_dispatch)
    );

    decode decode_i (
        .iq_out,
        .id_dp_reg_next
    );

    cacheline_adapter adapter_i (
        .clk,
        .rst,

        .i_dfp_addr(icache_dfp_addr),
        .i_dfp_read(icache_dfp_read),
        .i_dfp_write(icache_dfp_write),
        .i_dfp_rdata(icache_dfp_rdata),
        .i_dfp_wdata(icache_dfp_wdata),
        .i_dfp_resp(icache_dfp_resp),
        
        .d_dfp_addr(dcache_dfp_addr),
        .d_dfp_read(dcache_dfp_read),
        .d_dfp_write(dcache_dfp_write),
        .d_dfp_rdata(dcache_dfp_rdata),
        .d_dfp_wdata(dcache_dfp_wdata),
        .d_dfp_resp(dcache_dfp_resp),

        .bmem_addr(bmem_addr),
        .bmem_read(bmem_read),
        .bmem_write(bmem_write),
        .bmem_wdata(bmem_wdata),
        .bmem_ready(bmem_ready),
        .bmem_raddr(bmem_raddr),
        .bmem_rdata(bmem_rdata),
        .bmem_rvalid(bmem_rvalid)
    );

    always_comb begin
        res_full[0] = bmd_rs_full;
        res_full[1] = alu_rs_full;
        res_full[2] = lsq_full; 
    end
    
    dispatch dispatch_i (
        .id_dp_reg(id_dp_reg),

        .rs1_mapping(phys_src1[0]),
        .rs2_mapping(phys_src2[0]),
        .rs1_archreg(arch_src1[0]),
        .rs2_archreg(arch_src2[0]),
        .rename_en(rename_en[0]),
        .rat_arch_dst(rat_arch_dst[0]),
        .rat_phys_dst(rat_phys_dst[0]), 

        .fl_empty(fl_empty),
        .preg_addr(fl_preg_addr_out),
        .preg_request(fl_preg_request),

        .rob_idx(rob_idx[0]),
        .rob_full(rob_full[0]),
        .rob_pkt(rob_pkt[0]),

        .res_full(res_full),
        .res_entry_out(dp_res_entry),

        .stall_dispatch(stall_dispatch) 
    );

    RAT RAT_i (
        .clk,
        .rst,

        .arch_src1(arch_src1),
        .arch_src2(arch_src2),

        .rename_en(rename_en),
        .arch_dst(rat_arch_dst),
        .new_phys_dst(rat_phys_dst),
        
        .br_flush(pc_sel),
        .rrf_copy(rrf_copy),

        .phys_src1(phys_src1),
        .phys_src2(phys_src2)
    );

    assign fl_enqueue = (fl_preg_addr_in[0] != '0);

    free_list free_list_i (
        .clk,
        .rst,

        .enqueue(fl_enqueue),
        .preg_addr_in(fl_preg_addr_in[0]),

        .br_flush(pc_sel),
        .rollback_fl_head(rollback_fl_head[0]),
        
        .preg_request(fl_preg_request),
        .preg_addr_out(fl_preg_addr_out),
        .fl_full(fl_full),
        .fl_empty(fl_empty)
    );

    assign rob_enqueue[0] = id_dp_reg.inst.i_valid && ~stall_dispatch;
    
    rob rob_i (
        .clk,
        .rst,

        .enqueue(rob_enqueue),
        .dp_pkt(rob_pkt),

        .cdb_pkt(ex_out),
        
        .areg_addr(rrf_arch_dst),
        .preg_addr(rrf_new_phys_dst),
        .commit_en(commit_en),


        .rob_idx(rob_idx),
        .rob_full(rob_full),

        .head_rob_idx,
        .head_is_mem,
        .stall_store(stall_store),
        .dequeue_store(dequeue_store),

        .br_flush(pc_sel),
        .br_target(br_pc),
        .br_order(br_order),
        .rollback_fl_head(rollback_fl_head),
        .head_is_jmp,

        .br_commit(br_commit),
        .br_pred(br_pred),
        .br_result(br_result),
        .br_commit_ghr(br_commit_ghr),
        .br_commit_idx(br_commit_idx),

        .rvfi_out(rvfi_out),
        .store_in(store_out)
    );

    RRF rrf_i (
        .clk,
        .rst,

        .commit_en(commit_en),
        .arch_dst(rrf_arch_dst),
        .new_phys_dst(rrf_new_phys_dst),

        .head_is_jmp,
        .rrf_copy(rrf_copy),

        .retired_phys_reg(fl_preg_addr_in)
    );
    
    generate
        for (genvar i = 0; i < CDB_PORTS; i++) begin
            assign cdb_valid[i] = ex_out[i].inst.i_valid & ex_out[i].inst.regfile_we;
            assign rd_paddr[i] = ex_out[i].inst.rd_paddr;
            assign rd_data[i]  = ex_out[i].inst.rd_data;
            assign pregf_we[i] = ex_out[i].inst.i_valid & ex_out[i].inst.regfile_we;
        end
    endgenerate

    // valid array - res station interfacing signals
    generate
        for (genvar i = 0; i < NUM_BMD_RS_ENTRIES; i++) begin
            assign rs1_source[i] = bmd_rs_rs1_out[i];
            assign rs2_source[i] = bmd_rs_rs2_out[i];
        end
        for (genvar i = 0; i < NUM_ALU_RS_ENTRIES; i++) begin
            assign rs1_source[NUM_BMD_RS_ENTRIES + i] = alu_rs_rs1_out[i];
            assign rs2_source[NUM_BMD_RS_ENTRIES + i] = alu_rs_rs2_out[i];
        end
        for (genvar i = 0; i < NUM_MEM_RS_ENTRIES; i++) begin
            assign rs1_source[NUM_BMD_RS_ENTRIES+NUM_ALU_RS_ENTRIES + i] = lsq_rs_rs1_out[i];
            // assign rs2_source[NUM_BMD_RS_ENTRIES+NUM_ALU_RS_ENTRIES + i] = lsq_rs_rs2_out[i];
        end
    endgenerate

    valid_array valid_i (
        .clk,
        .rst,

        .rename_en,
        .rename_paddr(rat_phys_dst),

        .cdb_valid(cdb_valid),//cdb
        .cdb_paddr(rd_paddr),//cdb

        .stall_dispatch(stall_dispatch),

        .br_flush(pc_sel),
        .rrf_copy(rrf_copy),

        .rs1_source(rs1_source),
        .rs2_source(rs2_source),
        .rs1_valid(rs1_valid),
        .rs2_valid(rs2_valid)
    );

    assign bmd_rs_rs1_valid = rs1_valid[0 +: NUM_BMD_RS_ENTRIES];
    assign bmd_rs_rs2_valid = rs2_valid[0 +: NUM_BMD_RS_ENTRIES];
    assign alu_rs_rs1_valid = rs1_valid[NUM_BMD_RS_ENTRIES +: NUM_ALU_RS_ENTRIES];
    assign alu_rs_rs2_valid = rs2_valid[NUM_BMD_RS_ENTRIES +: NUM_ALU_RS_ENTRIES];
    assign lsq_rs_rs1_valid = rs1_valid[(NUM_BMD_RS_ENTRIES+NUM_ALU_RS_ENTRIES) +: NUM_MEM_RS_ENTRIES];
    // assign lsq_rs_rs2_valid = rs2_valid[(NUM_BMD_RS_ENTRIES+NUM_ALU_RS_ENTRIES) +: NUM_MEM_RS_ENTRIES];


    bmd_res_station #(
        NUM_BMD_RS_ENTRIES,
        3
    ) bmd_rs (
        .clk,
        .rst,

        .rs1_out(bmd_rs_rs1_out),
        .rs2_out(bmd_rs_rs2_out),
        .rs1_valid(bmd_rs_rs1_valid),
        .rs2_valid(bmd_rs_rs2_valid),

        .enable(dp_res_entry.inst.i_valid && (dp_res_entry.inst.res_id == br_mul_div_res)),        // high - new inst to insert
        .res_entry_in(dp_res_entry),  // inst from dispatch
        
        .stall_fu(stall_br_mul_div), 
        .br_flush(pc_sel),

        .res_full(bmd_rs_full),
        .instr_pkt_out(bmd_rs_instr_pkt),

        .rs1_paddr(bmd_rs1_paddr),  // to prf
        .rs2_paddr(bmd_rs2_paddr)
    );

    res_station #(
        NUM_ALU_RS_ENTRIES,
        1
    ) alu_rs (
        .clk,
        .rst,

        .rs1_out(alu_rs_rs1_out),
        .rs2_out(alu_rs_rs2_out),
        .rs1_valid(alu_rs_rs1_valid),
        .rs2_valid(alu_rs_rs2_valid),

        .enable(dp_res_entry.inst.i_valid && (dp_res_entry.inst.res_id == alu_res)),        // high - new inst to insert
        .res_entry_in(dp_res_entry),  // inst from dispatch
        
        .stall_fu(stall_alu),
        .br_flush(pc_sel),

        .res_full(alu_rs_full),
        .instr_pkt_out(alu_rs_instr_pkt),

        .rs1_paddr(alu_rs1_paddr),  // to prf
        .rs2_paddr(alu_rs2_paddr)
    );

    // Issue from LSQ when the following conds. hold:
    //      0. Instruction at head of ROB is mem-type
    //      1. Head of ROB idx == Head of LSQ idx
    //      2. operands are valid
    //      -------     OR      ---------------
    //      0. Instruction at head of LSQ is a load
    //      1. operands are valid
    logic   [PHYS_WIDTH-1:0] lsq_rs1_paddr;   
    logic   [PHYS_WIDTH-1:0] lsq_rs2_paddr;   

    logic   lsq_enqueue;
    logic   is_store;

    assign  lsq_enqueue = id_dp_reg.inst.i_valid && (dp_res_entry.inst.res_id == mem_res);
    assign  is_store    = id_dp_reg.inst.i_valid && (dp_res_entry.inst.i_opcode == op_b_store);

    logic sq_full, lq_full;
    logic sq_empty, lq_empty;

    assign dequeue_store = ~stall_mem_latch && head_is_mem[0]
                      && head_rob_idx[0] == sq_rob_idx
                      && !sq_empty;

    assign stall_mem = (lsq_out.i_valid &&
                        (lsq_out.i_opcode == op_b_store && stall_store)
                        || (lsq_out.i_opcode != op_b_store && stall_load));

    always_ff @(posedge clk) stall_mem_latch <= stall_mem;

    assign lsq_full = (sq_full && lsq_enqueue && is_store) | (lq_full && lsq_enqueue && !is_store);
    
    logic   [PHYS_WIDTH-1:0]   store_paddr1; // RVFI ONLY DO NOT USE
    logic   [PHYS_WIDTH-1:0]   store_paddr2;
    logic   [31:0]             store_paddr1_data;
    logic   [31:0]             store_paddr2_data;

    lsq mem_rs (
        .clk(clk),
        .rst(rst),
        .br_flush(pc_sel),

        .enqueue(lsq_enqueue && !stall_dispatch),
        .dequeue_store(dequeue_store),
        .is_store(is_store),
        .rse_in(dp_res_entry),

        .mem_pkt_out(lsq_out),

        .sq_full(sq_full),
        .lq_full(lq_full),

        .sq_empty(sq_empty),
        .lq_empty(lq_empty),

        .stall_mem(stall_mem),
        .stall_mem_latch(stall_mem_latch),

        .sq_rob_idx(sq_rob_idx),

        .lq_rs1_paddr_valid(lsq_rs_rs1_out),
        .lq_rs1_valid(lsq_rs_rs1_valid),

        .lsq_rs1_paddr(lsq_rs1_paddr),
        .lsq_rs2_paddr(lsq_rs2_paddr),
        
        .lsq_rs1_data(rs1_prdata[2]),
        .lsq_rs2_data(rs2_prdata[2]),

        .store_out(store_out),
        .paddr1(store_paddr1),
        .paddr2(store_paddr2),
        .paddr1_data(store_paddr1_data),
        .paddr2_data(store_paddr2_data)
    );

    assign rs1_paddr[0] = bmd_rs1_paddr;
    assign rs2_paddr[0] = bmd_rs2_paddr;
    assign rs1_paddr[1] = alu_rs1_paddr;
    assign rs2_paddr[1] = alu_rs2_paddr;
    assign rs1_paddr[2] = lsq_rs1_paddr;
    assign rs2_paddr[2] = lsq_rs2_paddr;

    prf PRF_i (
        .clk,
        .rst,
        
        .rs1_paddr(rs1_paddr),
        .rs2_paddr(rs2_paddr),

        .rd_paddr(rd_paddr),
        .rd_data(rd_data),
        .pregf_we(pregf_we),

        .rs1_prdata(rs1_prdata),
        .rs2_prdata(rs2_prdata),

        .store_paddr1(store_paddr1), // RVFI ONLY DO NOT USE
        .store_paddr2(store_paddr2),
        .store_paddr1_data(store_paddr1_data),
        .store_paddr2_data(store_paddr2_data)        
    );

    execute execute_i (
        .clk,
        .rst,

        .rs1_prdata(rs1_prdata),
        .rs2_prdata(rs2_prdata),

        .br_mul_div_iss(bmd_rs_instr_pkt),
        .alu_iss(alu_rs_instr_pkt),
        .mem_iss(lsq_out),
        .is_store(lsq_out.i_opcode == op_b_store),

        .stall_br_mul_div(stall_br_mul_div),
        .stall_alu(stall_alu),
        .stall_load(stall_load),
        .stall_mem_latch(stall_mem_latch),
        .stall_store(stall_store),

        .br_flush(pc_sel),

        .dfp_addr(dcache_dfp_addr),
        .dfp_read(dcache_dfp_read),
        .dfp_write(dcache_dfp_write),
        .dfp_rdata(dcache_dfp_rdata),
        .dfp_wdata(dcache_dfp_wdata),
        .dfp_resp(dcache_dfp_resp), 

        .ex_out(ex_out)
    );


            // RVFI
            logic           monitor_valid;
            logic   [63:0]  monitor_order;
            logic   [31:0]  monitor_inst;
            logic   [4:0]   monitor_rs1_addr;
            logic   [4:0]   monitor_rs2_addr;
            logic   [31:0]  monitor_rs1_rdata;
            logic   [31:0]  monitor_rs2_rdata;
            logic           monitor_regf_we;
            logic   [4:0]   monitor_rd_addr;
            logic   [31:0]  monitor_rd_wdata;
            logic   [31:0]  monitor_pc_rdata;
            logic   [31:0]  monitor_pc_wdata;
            logic   [31:0]  monitor_mem_addr;
            logic   [3:0]   monitor_mem_rmask;
            logic   [3:0]   monitor_mem_wmask;
            logic   [31:0]  monitor_mem_rdata;
            logic   [31:0]  monitor_mem_wdata;

    assign monitor_valid     = commit_en[0];
    assign monitor_order     = rvfi_out[0].i_order;
    assign monitor_inst      = rvfi_out[0].i;
    assign monitor_rs1_addr  = rvfi_out[0].i_uses_rs1 ? rvfi_out[0].rs1_addr : '0;
    assign monitor_rs2_addr  = rvfi_out[0].i_uses_rs2 ? rvfi_out[0].rs2_addr : '0;
    assign monitor_rs1_rdata = rvfi_out[0].i_uses_rs1 ? rvfi_out[0].rs1_data : '0;
    assign monitor_rs2_rdata = rvfi_out[0].i_uses_rs2 ? rvfi_out[0].rs2_data : '0;
    assign monitor_rd_addr   = rvfi_out[0].rd_addr;
    assign monitor_rd_wdata  = rvfi_out[0].rd_data;
    assign monitor_pc_rdata  = rvfi_out[0].i_pc;
    assign monitor_pc_wdata  = rvfi_out[0].i_pc_next;
    assign monitor_mem_addr  = (rvfi_out[0].res_id == mem_res) ? rvfi_out[0].i_dmem_addr : '0;
    assign monitor_mem_rmask = (rvfi_out[0].res_id == mem_res) ? rvfi_out[0].i_dmem_rmask : '0;
    assign monitor_mem_wmask = (rvfi_out[0].res_id == mem_res && rvfi_out[0].i_opcode == op_b_store) ? rvfi_out[0].i_dmem_wmask : '0;
    assign monitor_mem_rdata = (rvfi_out[0].res_id == mem_res) ? rvfi_out[0].i_dmem_rdata : '0;
    assign monitor_mem_wdata = (rvfi_out[0].res_id == mem_res) ? rvfi_out[0].i_dmem_wdata : '0;

endmodule : cpu
