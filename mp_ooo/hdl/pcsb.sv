module pcsb 
#(
    parameter PCSB_DEPTH        = 8,
    parameter PCSB_ADDR_WIDTH   = $clog2(PCSB_DEPTH),
    parameter PCSB_WIDTH        = 1 + 32 + 4 + 32
) (
    input   logic               clk,
    input   logic               rst,

    input   logic               store_valid,
    input   logic [31:0]        store_address_in,
    input   logic [3:0]         store_wmask_in,
    input   logic [31:0]        store_data_in,

    input   logic               load_valid,
    input   logic [31:0]        load_address,

    input   logic               dcache_busy,
    input   logic               dcache_resp,
    input   logic               stall_pp,
    output  logic               pcsb_dcache_req,
    output  logic [31:0]        pcsb_dcache_address,
    output  logic [3:0]         pcsb_dcache_wmask,
    output  logic [31:0]        pcsb_dcache_data,

    output  logic               hit,
    output  logic [31:0]        store_address_out,
    output  logic [3:0]         store_wmask_out,
    output  logic [31:0]        store_data_out,

    output  logic               pcsb_full
);

    /**

        ROB-Store-Commit:
            - On a store commit, ROB will commit store immediately if it's at the head
            AND store buffer is not full
            - Now, first put the store address, data, and wmask in the pcsb
            - Check if the address matches any of the entries
                - If a match write coalesce and combine the masks
                - If not allocate an entry in the buffer
        
        Load-Forward:
            - Loads send a request to D-Cache in parallel with the store buffer
            - If the buffer wmask doesn't encapsulate load read-mask then concatinate
            the two results together after getting a response from D-Cache
            - Else ignore D-Cache response and just take data from pcsb's entry

        Write-Back:
            - Whenever D-Cache is not busy and buffer is not empty, drain the buffer

    */

    typedef struct packed {
        logic [31:0]    address;
        logic [3:0]     mask;
        logic [31:0]    data;
        logic           valid; 
    } pcsb_entry_t;
    
    pcsb_entry_t queue [PCSB_DEPTH-1:0];

    logic [PCSB_ADDR_WIDTH:0] head;
    logic [PCSB_ADDR_WIDTH:0] tail;

    logic enqueue;
    logic dequeue;
    // logic pcsb_full; // output
    logic pcsb_empty;

    logic store_match;
    logic [PCSB_ADDR_WIDTH-1:0] store_match_index;

    logic [3:0] wmask_next;
    logic [31:0] store_data_next;

    logic [2:0] counter;

    always_ff @(posedge clk) begin
        if (rst) begin
            counter <= '0;
        end
        else if ((store_valid && counter != 3'b111) && dequeue) begin

        end
        else if (store_valid && counter != 3'b111) begin
            counter <= counter + 1'b1;
        end
        else if (dequeue) begin
            counter <= counter - 1'b1;
        end
    end

    assign pcsb_empty = (head == tail && !pcsb_full);
    assign pcsb_full  = counter == 3'b111;

    logic [31:0]        store_address_in_latch;
    logic [3:0]         store_wmask_in_latch;
    logic [31:0]        store_data_in_latch;
    logic               store_valid_latch, use_latch;

    assign enqueue = store_valid_latch;
    assign dequeue = !dcache_busy && !pcsb_empty;

    always_ff @(posedge clk) begin
        if (!pcsb_full) begin
            store_address_in_latch <= store_address_in;
            store_wmask_in_latch <= store_wmask_in;
            store_data_in_latch <= store_data_in;
            store_valid_latch <= store_valid;
        end
    end

    always_ff @(posedge clk) begin : STORE_QUEUE
        if (rst) begin
            {head, tail} <= '0;

            for (integer i = 0; i < PCSB_DEPTH; i++) begin
                queue[i].valid <= '0;
            end
        end 
        else begin
            if (dequeue && !pcsb_empty) begin : WRITE_BACK
                queue[head[PCSB_ADDR_WIDTH-1:0]].valid <= 1'b0;
            
                head <= head + 1'b1;
            end

            if (store_match && store_valid_latch) begin
                queue[store_match_index].mask <= wmask_next;
                queue[store_match_index].data  <= store_data_next;
                queue[store_match_index].valid <= 1'b1;
                head <= (store_match_index != head[PCSB_ADDR_WIDTH-1:0] && dequeue) ? head + 1'b1 : head;
            end 
            else if (enqueue && !pcsb_full) begin
                queue[tail[PCSB_ADDR_WIDTH-1:0]].address <= store_address_in_latch;
                queue[tail[PCSB_ADDR_WIDTH-1:0]].mask   <= store_wmask_in_latch;
                queue[tail[PCSB_ADDR_WIDTH-1:0]].data    <= store_data_in_latch;
                queue[tail[PCSB_ADDR_WIDTH-1:0]].valid   <= 1'b1;

                tail <= tail + 1'b1;
            end
        end 
    end

    logic load_match;
    logic [PCSB_ADDR_WIDTH-1:0] load_match_index;

    always_ff @(posedge clk) begin : STORE_QUEUE_READ
        // If reg 1 only invalidate on Dcache resp
        if (rst) begin
            hit <= 1'b0;
        end 
        else if (dcache_resp) begin
            hit <= 1'b0;
        end

        if (load_match && !stall_pp && load_valid) begin
            store_address_out <= queue[load_match_index].address;
            store_wmask_out   <= queue[load_match_index].mask;
            store_data_out    <= queue[load_match_index].data;

            hit <= 1'b1;
        end

        if (load_match && !stall_pp && use_latch && load_valid) begin
            store_address_out <= store_address_in_latch;
            store_wmask_out   <= wmask_next;
            store_data_out    <= store_data_next;

            hit <= 1'b1;
        end
    end

    always_comb begin : WRITE_BACK_SIGNALS
        pcsb_dcache_req     = 1'b0;

        pcsb_dcache_address = queue[head[PCSB_ADDR_WIDTH-1:0]].address;
        pcsb_dcache_wmask   = queue[head[PCSB_ADDR_WIDTH-1:0]].mask;
        pcsb_dcache_data    = queue[head[PCSB_ADDR_WIDTH-1:0]].data;

        if (dequeue && !pcsb_empty) begin
            pcsb_dcache_req     = 1'b1;
        end
    end

    always_comb begin : STORE_MATCH
        store_match = 1'b0;
        store_match_index = 'x;

        for (integer unsigned i = 0; i < PCSB_DEPTH; i++) begin
            if (queue[i].valid && store_address_in_latch[30:2] == queue[i].address[30:2]) begin
                store_match = 1'b1;
                store_match_index = PCSB_ADDR_WIDTH'(i);
            end
        end
    end

    always_comb begin : WMASK_NEXT 
        wmask_next = 'x;
        store_data_next = 'x;
        
        if (store_match) begin
            store_data_next = store_data_in_latch;

            for (integer i = 0; i < 4; i++) begin
                if (!store_wmask_in_latch[i] && queue[store_match_index].mask[i]) begin
                    store_data_next[i*8+:8] = queue[store_match_index].data[i*8+:8];
                end
            end

            wmask_next = store_wmask_in_latch | queue[store_match_index].mask;
        end
        else if (store_valid_latch) begin
            wmask_next = store_wmask_in_latch;
            store_data_next = store_data_in_latch;
        end
    end

    always_comb begin : LOAD_MATCH
        load_match = 1'b0;
        load_match_index = 'x;
        use_latch = 1'b0;

        for (integer unsigned i = 0; i < PCSB_DEPTH; i++) begin
            if (queue[i].valid && load_address[30:2] == queue[i].address[30:2]) begin
                load_match = 1'b1;
                load_match_index = PCSB_ADDR_WIDTH'(i);            
            end
        end

        if (store_valid_latch && load_address[30:2] == store_address_in_latch[30:2]) begin
            load_match = 1'b1;
            use_latch = 1'b1;
        end
    end

endmodule : pcsb