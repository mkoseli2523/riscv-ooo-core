// SDRAM timing model
// Based on MT48LC64M4A2-75, modified for ECE411
// Limitations:
//   Refresh not modeled
//   Fixed one channel and one rank
//   No auto precharge / Open page only
//   Precise bus transaction / bus state not modeled
//   First Access still requires precharge

module dram_w_burst_frfcfs_controller
#(
    parameter int DRAM_TIMMING_CL               = 20,   // In ns, aka tCAS, column access time
    parameter int DRAM_TIMMING_tRCD             = 20,   // in ns, active to r/w delay
    parameter int DRAM_TIMMING_tRP              = 20,   // in ns, precharge time
    parameter int DRAM_TIMMING_tRAS             = 44,   // in ns, active to precharge delay
    parameter int DRAM_TIMMING_tRC              = 66,   // in ns, active to active delay
    parameter int DRAM_TIMMING_tRRD             = 15,   // in ns, different bank active delay
    parameter int DRAM_TIMMING_tWR              = 15,   // in ns, write recovery time
    parameter int DRAM_PARAM_BA_WIDTH           = 4,    // in bits
    parameter int DRAM_PARAM_RA_WIDTH           = 20,   // in bits
    parameter int DRAM_PARAM_CA_WIDTH           = 3,    // in bits
    parameter int DRAM_PARAM_BUS_WIDTH          = 64,   // in bits
    parameter int DRAM_PARAM_BURST_LEN          = 4,    // in bursts
    parameter int DRAM_PARAM_IN_QUEUE_SIZE      = 16,   // in requests
    parameter int DRAM_PARAM_OUT_QUEUE_SIZE     = 16    // in requests
)(
    mem_itf_banked.mem itf
);

    int DRAM_TIMMING_CL_CYCLE   ;
    int DRAM_TIMMING_tRCD_CYCLE ;
    int DRAM_TIMMING_tRP_CYCLE  ;
    int DRAM_TIMMING_tRAS_CYCLE ;
    int DRAM_TIMMING_tRC_CYCLE  ;
    int DRAM_TIMMING_tRRD_CYCLE ;
    int DRAM_TIMMING_tWR_CYCLE  ;

    int BRAM_0_ON_X; // return 0 instead of x on rdata
    int CLOCK_PERIOD_PS;
    string memfile;
    initial begin
        $value$plusargs("BRAM_0_ON_X_ECE411=%d", BRAM_0_ON_X);
        $value$plusargs("CLOCK_PERIOD_PS_ECE411=%d", CLOCK_PERIOD_PS);
        $value$plusargs("MEMLST_ECE411=%s", memfile);
        DRAM_TIMMING_CL_CYCLE   = int'($ceil((DRAM_TIMMING_CL   *1000.0)/CLOCK_PERIOD_PS));
        DRAM_TIMMING_tRCD_CYCLE = int'($ceil((DRAM_TIMMING_tRCD *1000.0)/CLOCK_PERIOD_PS));
        DRAM_TIMMING_tRP_CYCLE  = int'($ceil((DRAM_TIMMING_tRP  *1000.0)/CLOCK_PERIOD_PS));
        DRAM_TIMMING_tRAS_CYCLE = int'($ceil((DRAM_TIMMING_tRAS *1000.0)/CLOCK_PERIOD_PS));
        DRAM_TIMMING_tRC_CYCLE  = int'($ceil((DRAM_TIMMING_tRC  *1000.0)/CLOCK_PERIOD_PS));
        DRAM_TIMMING_tRRD_CYCLE = int'($ceil((DRAM_TIMMING_tRRD *1000.0)/CLOCK_PERIOD_PS));
        DRAM_TIMMING_tWR_CYCLE  = int'($ceil((DRAM_TIMMING_tWR  *1000.0)/CLOCK_PERIOD_PS));
    end

    localparam int DRAM_PARAM_NUM_BANKS         = 2**DRAM_PARAM_BA_WIDTH;
    localparam int DRAM_PARAM_ACCESS_WIDTH      = DRAM_PARAM_BUS_WIDTH * DRAM_PARAM_BURST_LEN;
    localparam int DRAM_PARAM_OFFSET_WIDTH      = $clog2(DRAM_PARAM_ACCESS_WIDTH / 8);
    localparam int DRAM_PARAM_ACCESS_ADDR_WIDTH = DRAM_PARAM_BA_WIDTH + DRAM_PARAM_RA_WIDTH + DRAM_PARAM_CA_WIDTH;
    localparam int DRAM_PARAM_TOTAL_ADDR_WIDTH  = DRAM_PARAM_ACCESS_ADDR_WIDTH + DRAM_PARAM_OFFSET_WIDTH;

    function logic [DRAM_PARAM_OFFSET_WIDTH-1:0]      get_offset (logic [DRAM_PARAM_TOTAL_ADDR_WIDTH-1:0] addr);
        return addr[ 0                       +:  DRAM_PARAM_OFFSET_WIDTH];
    endfunction

    function logic [DRAM_PARAM_CA_WIDTH-1:0]          get_col    (logic [DRAM_PARAM_TOTAL_ADDR_WIDTH-1:0] addr);
        return addr[ DRAM_PARAM_OFFSET_WIDTH +:  DRAM_PARAM_CA_WIDTH];
    endfunction

    function logic [DRAM_PARAM_BA_WIDTH-1:0]          get_bank   (logic [DRAM_PARAM_TOTAL_ADDR_WIDTH-1:0] addr);
        return addr[(DRAM_PARAM_OFFSET_WIDTH +   DRAM_PARAM_CA_WIDTH) +: DRAM_PARAM_BA_WIDTH];
    endfunction

    function logic [DRAM_PARAM_RA_WIDTH-1:0]          get_row    (logic [DRAM_PARAM_TOTAL_ADDR_WIDTH-1:0] addr);
        return addr[(DRAM_PARAM_OFFSET_WIDTH +   DRAM_PARAM_CA_WIDTH  +  DRAM_PARAM_BA_WIDTH) +: DRAM_PARAM_RA_WIDTH];
    endfunction

    function logic [DRAM_PARAM_ACCESS_ADDR_WIDTH-1:0] get_access (logic [DRAM_PARAM_TOTAL_ADDR_WIDTH-1:0] addr);
        return addr[ DRAM_PARAM_OFFSET_WIDTH +: (DRAM_PARAM_CA_WIDTH  +  DRAM_PARAM_BA_WIDTH  +  DRAM_PARAM_RA_WIDTH)];
    endfunction

    typedef struct packed {
        bit                                         read;
        bit     [DRAM_PARAM_TOTAL_ADDR_WIDTH-1:0]   addr;
        logic   [DRAM_PARAM_ACCESS_WIDTH-1:0]       wdata;
        longint                                     qtime;
    } in_queue_t;

    typedef struct packed {
        bit     [DRAM_PARAM_TOTAL_ADDR_WIDTH-1:0]   raddr;
        logic   [DRAM_PARAM_ACCESS_WIDTH-1:0]       rdata;
    } out_queue_t;

    typedef enum int {
        DRAM_BANK_IDLE,
        DRAM_BANK_PRE,
        DRAM_BANK_ACT,
        DRAM_BANK_COL
    } bank_state_machine_t;

    typedef struct {
        bank_state_machine_t state;
        int active_row;
        int tRAS_counter;
        int tRC_counter;
        int tRP_counter;
        int tRCD_counter;
        int tCL_counter;
        int tWR_counter;
        in_queue_t req;
    } bank_state_t;

    logic [DRAM_PARAM_ACCESS_WIDTH-1:0] internal_memory_array [logic [DRAM_PARAM_ACCESS_ADDR_WIDTH-1:0]];

    in_queue_t  in_queue [DRAM_PARAM_IN_QUEUE_SIZE];
    in_queue_t  in_queue_next [DRAM_PARAM_IN_QUEUE_SIZE];
    out_queue_t out_queue [DRAM_PARAM_OUT_QUEUE_SIZE];
    out_queue_t out_queue_next [DRAM_PARAM_OUT_QUEUE_SIZE];

    int in_queue_tail;
    int in_queue_tail_next;
    int out_queue_tail;
    int out_queue_tail_next;

    int in_burst_counter;
    int in_burst_counter_next;
    int out_burst_counter;
    int out_burst_counter_next;

    bank_state_t bank_state [DRAM_PARAM_NUM_BANKS];
    bank_state_t bank_state_next [DRAM_PARAM_NUM_BANKS];

    int tRRD_counter;
    int tRRD_counter_next;

    longint cycle_counter;

    int bank_req_idx [DRAM_PARAM_NUM_BANKS];
    int bank_activate_arb;
    bit [DRAM_PARAM_IN_QUEUE_SIZE-1:0] in_queue_dequeue;
    longint bank_pre_out_queue_age [DRAM_PARAM_NUM_BANKS];
    bit [DRAM_PARAM_NUM_BANKS-1:0] bank_out_arb;

    bit [DRAM_PARAM_NUM_BANKS-1:0] bank_dequeue;
    logic [DRAM_PARAM_ACCESS_WIDTH-1:0] internal_memory_read_shim [DRAM_PARAM_NUM_BANKS];

    always_ff @(posedge itf.clk) begin
        if (itf.rst) begin
            internal_memory_array.delete();
            $readmemh(memfile, internal_memory_array);
            $display("using memory file %s", memfile);
            internal_memory_read_shim <= '{default: 'x};
        end else begin
            for (int bank = 0; bank < DRAM_PARAM_NUM_BANKS; bank++) begin
                if (bank_dequeue[bank]) begin
                    if (bank_state_next[bank].req.read) begin
                        internal_memory_read_shim[bank] <= internal_memory_array[get_access(bank_state_next[bank].req.addr)];
                    end else begin
                        internal_memory_array[get_access(bank_state_next[bank].req.addr)] = bank_state_next[bank].req.wdata;
                    end
                end
            end
        end
    end

    always_ff @(posedge itf.clk) begin
        if (itf.rst) begin
            automatic bank_state_t bank_state_rst = '{DRAM_BANK_IDLE, -1, 0, 0, 0, 0, 0, 0, '{'x, 'x, 'x, 'x}};
            in_queue <= '{default: '0};
            out_queue <= '{default: '0};
            in_queue_tail <= 0;
            out_queue_tail <= 0;
            in_burst_counter <= 0;
            out_burst_counter <= 0;
            bank_state <= '{default: bank_state_rst};
            tRRD_counter <= 0;
            cycle_counter <= longint'(0);
        end else begin
            in_queue <= in_queue_next;
            out_queue <= out_queue_next;
            in_queue_tail <= in_queue_tail_next;
            out_queue_tail <= out_queue_tail_next;
            in_burst_counter <= in_burst_counter_next;
            out_burst_counter <= out_burst_counter_next;
            bank_state <= bank_state_next;
            tRRD_counter <= tRRD_counter_next;
            cycle_counter <= cycle_counter + 1;
        end
    end

    always_comb begin
        itf.ready = in_queue_tail < DRAM_PARAM_IN_QUEUE_SIZE;
        in_queue_next = in_queue;
        out_queue_next = out_queue;
        in_queue_tail_next = in_queue_tail;
        out_queue_tail_next = out_queue_tail;
        in_burst_counter_next = in_burst_counter;
        out_burst_counter_next = out_burst_counter;
        bank_state_next = bank_state;
        for (int i = 0; i < DRAM_PARAM_NUM_BANKS; i++) begin
            bank_state_next[i].tRAS_counter = (bank_state[i].tRAS_counter != 0) ? bank_state[i].tRAS_counter - 1 : 0;
            bank_state_next[i].tRC_counter  = (bank_state[i].tRC_counter  != 0) ? bank_state[i].tRC_counter  - 1 : 0;
            bank_state_next[i].tRP_counter  = (bank_state[i].tRP_counter  != 0) ? bank_state[i].tRP_counter  - 1 : 0;
            bank_state_next[i].tRCD_counter = (bank_state[i].tRCD_counter != 0) ? bank_state[i].tRCD_counter - 1 : 0;
            bank_state_next[i].tCL_counter  = (bank_state[i].tCL_counter  != 0) ? bank_state[i].tCL_counter  - 1 : 0;
            bank_state_next[i].tWR_counter  = (bank_state[i].tWR_counter  != 0) ? bank_state[i].tWR_counter  - 1 : 0;
            bank_req_idx[i] = -1;
            bank_pre_out_queue_age[i] = 64'h7fff_ffff_ffff_ffff;
        end
        tRRD_counter_next = (tRRD_counter != 0) ? tRRD_counter - 1 : 0;
        itf.raddr = 'x;
        itf.rdata = 'x;
        itf.rvalid = 1'b0;
        bank_activate_arb = -1;
        in_queue_dequeue = '0;
        bank_out_arb = '0;
        bank_dequeue = '0;
        if ((itf.read || itf.write) && itf.ready) begin
            if (itf.read) begin
                in_queue_next[in_queue_tail_next].read  = 1'b1;
                in_queue_next[in_queue_tail_next].addr  = itf.addr;
                in_queue_next[in_queue_tail_next].wdata = 'x;
                in_queue_next[in_queue_tail_next].qtime = cycle_counter;
                in_queue_tail_next = in_queue_tail_next + 1;
            end
            if (itf.write) begin
                in_queue_next[in_queue_tail_next].read  = 1'b0;
                in_queue_next[in_queue_tail_next].addr  = itf.addr;
                in_queue_next[in_queue_tail_next].wdata[in_burst_counter*DRAM_PARAM_BUS_WIDTH+:DRAM_PARAM_BUS_WIDTH] = itf.wdata;
                in_queue_next[in_queue_tail_next].qtime = cycle_counter;
                if (in_burst_counter < 3) begin
                    in_burst_counter_next = in_burst_counter_next + 1;
                end else begin
                    in_burst_counter_next = 0;
                    in_queue_tail_next = in_queue_tail_next + 1;
                end
            end
        end
        for (int bank = 0; bank < DRAM_PARAM_NUM_BANKS; bank++) begin
            if (bank_state[bank].state == DRAM_BANK_IDLE) begin
                for (int i = 0; i < in_queue_tail; i++) begin
                    if (int'(get_bank(in_queue[i].addr)) == bank) begin
                        if (bank_req_idx[bank] < 0) begin
                            bank_req_idx[bank] = i;
                        end else begin
                            if (get_row(in_queue[bank_req_idx[bank]].addr) != DRAM_PARAM_RA_WIDTH'(bank_state[bank].active_row)) begin
                                if (get_row(in_queue[i].addr) == DRAM_PARAM_RA_WIDTH'(bank_state[bank].active_row)) begin
                                    bank_req_idx[bank] = i;
                                end
                            end
                        end
                    end
                end
            end
        end
        if (tRRD_counter_next <= DRAM_TIMMING_tRP_CYCLE) begin
            for (int bank = 0; bank < DRAM_PARAM_NUM_BANKS; bank++) begin
                if (bank_req_idx[bank] >= 0) begin
                    if (get_row(in_queue[bank_req_idx[bank]].addr) != DRAM_PARAM_RA_WIDTH'(bank_state[bank].active_row)) begin
                        if (bank_state_next[bank].tRAS_counter == 0 && bank_state_next[bank].tRC_counter <= DRAM_TIMMING_tRP_CYCLE) begin
                            if (bank_activate_arb < 0) begin
                                bank_activate_arb = bank;
                            end else begin
                                if (bank_req_idx[bank] < bank_req_idx[bank_activate_arb]) begin
                                    bank_activate_arb = bank;
                                end
                            end
                        end
                    end
                end
            end
        end
        for (int bank = 0; bank < DRAM_PARAM_NUM_BANKS; bank++) begin
            if (bank_state[bank].state == DRAM_BANK_COL) begin
                if (bank_state[bank].req.read) begin
                    if (bank_state_next[bank].tCL_counter == 0) begin
                        bank_pre_out_queue_age[bank] = bank_state[bank].req.qtime;
                    end
                end
            end
        end
        bank_pre_out_queue_age.sort();
        for (int i = 0; i < DRAM_PARAM_OUT_QUEUE_SIZE - out_queue_tail_next; i++) begin
            if (bank_pre_out_queue_age[i] == 64'h7fff_ffff_ffff_ffff) begin
                break;
            end
            for (int bank = 0; bank < DRAM_PARAM_NUM_BANKS; bank++) begin
                if (bank_state[bank].state == DRAM_BANK_COL) begin
                    if (bank_state[bank].req.read) begin
                        if (bank_state_next[bank].tCL_counter == 0) begin
                            if (bank_state[bank].req.qtime == bank_pre_out_queue_age[i]) begin
                                out_queue_next[out_queue_tail_next].raddr = bank_state[bank].req.addr;
                                out_queue_next[out_queue_tail_next].rdata = internal_memory_read_shim[bank];
                                if (BRAM_0_ON_X != 0) begin
                                    automatic bit [DRAM_PARAM_ACCESS_WIDTH-1:0] rdata_tmp;
                                    rdata_tmp = out_queue_next[out_queue_tail_next].rdata;
                                    out_queue_next[out_queue_tail_next].rdata = rdata_tmp;
                                end
                                out_queue_tail_next = out_queue_tail_next + 1;
                                bank_out_arb[bank] = 1'b1;
                                break;
                            end
                        end
                    end
                end
            end
        end
        for (int bank = 0; bank < DRAM_PARAM_NUM_BANKS; bank++) begin
            case (bank_state[bank].state)
            DRAM_BANK_IDLE: begin
                if (bank_req_idx[bank] >= 0) begin
                    if (get_row(in_queue[bank_req_idx[bank]].addr) == DRAM_PARAM_RA_WIDTH'(bank_state[bank].active_row)) begin
                        bank_state_next[bank].state = DRAM_BANK_COL;
                        bank_state_next[bank].req = in_queue[bank_req_idx[bank]];
                        in_queue_dequeue[bank_req_idx[bank]] = 1'b1;
                        bank_dequeue[bank] = 1'b1;
                        if (bank_state_next[bank].req.read) begin
                            bank_state_next[bank].tCL_counter = DRAM_TIMMING_CL_CYCLE;
                        end else begin
                            bank_state_next[bank].tWR_counter = DRAM_TIMMING_tWR_CYCLE;
                        end
                    end else begin
                        if (bank_activate_arb == bank) begin
                            bank_state_next[bank].state = DRAM_BANK_PRE;
                            bank_state_next[bank].active_row = -1;
                            bank_state_next[bank].tRP_counter = DRAM_TIMMING_tRP_CYCLE;
                            bank_state_next[bank].req = in_queue[bank_req_idx[bank]];
                            in_queue_dequeue[bank_req_idx[bank]] = 1'b1;
                            bank_dequeue[bank] = 1'b1;
                        end
                    end
                end
            end
            DRAM_BANK_PRE: begin
                if (bank_state_next[bank].tRP_counter == 0 && bank_state_next[bank].tRC_counter == 0 && tRRD_counter == 0) begin
                    bank_state_next[bank].state = DRAM_BANK_ACT;
                    bank_state_next[bank].tRAS_counter = DRAM_TIMMING_tRAS_CYCLE;
                    bank_state_next[bank].tRC_counter  = DRAM_TIMMING_tRC_CYCLE;
                    bank_state_next[bank].tRCD_counter = DRAM_TIMMING_tRCD_CYCLE;
                    tRRD_counter_next = DRAM_TIMMING_tRRD_CYCLE;
                end
            end
            DRAM_BANK_ACT: begin
                if (bank_state_next[bank].tRCD_counter == 0) begin
                    bank_state_next[bank].state = DRAM_BANK_COL;
                    bank_state_next[bank].active_row = int'(get_row(bank_state[bank].req.addr));
                    if (bank_state[bank].req.read) begin
                        bank_state_next[bank].tCL_counter = DRAM_TIMMING_CL_CYCLE;
                    end else begin
                        bank_state_next[bank].tWR_counter = DRAM_TIMMING_tWR_CYCLE;
                    end
                end
            end
            DRAM_BANK_COL: begin
                if (bank_state[bank].req.read) begin
                    if (bank_state_next[bank].tCL_counter == 0) begin
                        if (bank_out_arb[bank]) begin
                            bank_state_next[bank].state = DRAM_BANK_IDLE;
                        end
                    end
                end else begin
                    if (bank_state_next[bank].tWR_counter == 0) begin
                        bank_state_next[bank].state = DRAM_BANK_IDLE;
                    end
                end
            end
            endcase
        end
        for (int i = 0, int j = 0; i < DRAM_PARAM_IN_QUEUE_SIZE; i++, j++) begin
            while (j < DRAM_PARAM_IN_QUEUE_SIZE && in_queue_dequeue[j]) begin
                j = j + 1;
                in_queue_tail_next = in_queue_tail_next - 1;
            end
            if (j >= DRAM_PARAM_IN_QUEUE_SIZE) begin
                in_queue_next[i] = '0;
            end else begin
                in_queue_next[i] = in_queue_next[j];
            end
        end
        if (out_queue_tail != 0) begin
            itf.raddr = out_queue[0].raddr;
            itf.rdata = out_queue[0].rdata[out_burst_counter*DRAM_PARAM_BUS_WIDTH+:DRAM_PARAM_BUS_WIDTH];
            itf.rvalid = 1'b1;
            if (out_burst_counter < 3) begin
                out_burst_counter_next = out_burst_counter + 1;
            end else begin
                out_burst_counter_next = 0;
                out_queue_tail_next = out_queue_tail_next - 1;
                for (int i = 0; i < DRAM_PARAM_OUT_QUEUE_SIZE-1; i++) begin
                    out_queue_next[i] = out_queue_next[i+1];
                end
                out_queue_next[DRAM_PARAM_OUT_QUEUE_SIZE-1] = '0;
            end
        end
    end

    always @(posedge itf.clk iff !itf.rst) begin
        if ($isunknown(itf.read)) begin
            $error("Memory Error: read is 1'bx");
            itf.error <= 1'b1;
        end
        if ($isunknown(itf.write)) begin
            $error("Memory Error: write is 1'bx");
            itf.error <= 1'b1;
        end
        if (itf.read && itf.write) begin
            $error("Memory Error: simultaneous read and write");
            itf.error <= 1'b1;
        end
        if (in_burst_counter != 0 && !(itf.read || itf.write)) begin
            $error("Memory Error: burst incomplete");
            itf.error <= 1'b1;
        end;
        if (itf.read || itf.write) begin
            if ($isunknown(itf.addr)) begin
                $error("Memory Error: address contains 'x");
                itf.error <= 1'b1;
            end
            if (get_offset(itf.addr) != '0) begin
                $error("Memory Error: address not aligned");
                itf.error <= 1'b1;
            end
        end
    end

endmodule
