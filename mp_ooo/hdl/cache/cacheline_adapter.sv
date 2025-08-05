module cacheline_adapter
import cache_types::*;
(
    input   logic           clk,
    input   logic           rst,

    // ICACHE
    input  logic [31:0]  i_dfp_addr,
    input  logic         i_dfp_read,
    input  logic         i_dfp_write,
    output logic [255:0] i_dfp_rdata,
    input  logic [255:0] i_dfp_wdata,
    output logic         i_dfp_resp,

    // DCACHE
    input  logic [31:0]  d_dfp_addr,
    input  logic         d_dfp_read,
    input  logic         d_dfp_write,
    output logic [255:0] d_dfp_rdata,
    input  logic [255:0] d_dfp_wdata,
    output logic         d_dfp_resp,

    // bmem
    output logic [31:0] bmem_addr,
    output logic        bmem_read,
    output logic        bmem_write,
    output logic [63:0] bmem_wdata,
    input  logic        bmem_ready,
    input  logic [31:0] bmem_raddr,
    input  logic [63:0] bmem_rdata,
    input  logic        bmem_rvalid
);

    /**
        New Cacheline Adapter:
        1) capture every new dfp read into a queue of mem requests
        2) update request in queue when bmem_rvalid
        3) issue outstanding reads in queue order
            - issue after entire cacheline data makes it into queue entry (cnt == 4)
            - continue to accept read requests until req queue is full (backpressure if full)
        
        - writes should happen as soon as they appear (after current req is serviced), dont put in queue
    */

    enum integer unsigned {
        idle,
        b_write,
        b_read
    } state, state_next;

    localparam MAX_REQS   = 2;
    localparam ADDR_WIDTH = $clog2(MAX_REQS);

    typedef struct packed {
        logic         valid;
        logic         source;    // 0 = icache, 1 = dcache
        logic [31:0]  addr;
        logic [2:0]   req_cnt;
        logic [255:0] rdata;
    } req_t;

            logic [2:0]   b_ctr, b_ctr_next; // => [0, 7]
            logic [31:0]  dfp_addr;
            logic         dfp_read;
            logic         dfp_write;
            logic [255:0] dfp_rdata;
            logic [255:0] dfp_wdata;
            logic         dfp_resp;

            logic i_read_serviced, d_read_serviced;
            logic i_rd_d, d_rd_d;
            logic new_i_read, new_d_read;

            always_ff @(posedge clk) begin
                if (rst) begin
                    i_read_serviced <= 1'b0;
                    d_read_serviced <= 1'b0;
                end 

                if (new_i_read & bmem_read) begin
                    i_read_serviced <= 1'b1;
                end
                else if (~i_dfp_read) begin
                    i_read_serviced <= 1'b0;
                end

                if (new_d_read & bmem_read) begin
                    d_read_serviced <= 1'b1;
                end
                else if (~d_dfp_read) begin
                    d_read_serviced <= 1'b0;
                end
            end

            always_ff @(posedge clk) begin
                if (rst) begin
                    i_rd_d <= 1'b0;
                    d_rd_d <= 1'b0;
                end
                else begin
                    i_rd_d <= i_dfp_read & ~bmem_write;
                    d_rd_d <= d_dfp_read & ~bmem_write & ~new_i_read;
                end
            end

            assign new_i_read = i_dfp_read & ~i_rd_d & ~i_read_serviced & ~bmem_write;
            assign new_d_read = d_dfp_read & ~d_rd_d & ~d_read_serviced & ~bmem_write & ~new_i_read;

    /*** request queue ***/
    req_t                reqs[MAX_REQS];
    logic                i_outstanding;
    logic                d_outstanding;

    always_ff @ (posedge clk) begin : MODIFY_QUEUE
        if (rst) begin
            for (integer i = 0; i < MAX_REQS; i++)
                reqs[i] <= '0;
        end

        else begin
            // add requests to queue
            if (new_i_read && !i_outstanding) begin  // push icache read req
                reqs[0] <= '{
                    valid      : 1'b1,
                    source     : 1'b0,
                    addr  : i_dfp_addr,
                    req_cnt   : 2'd0,
                    rdata : 256'd0
                };
            end

            else if (new_d_read && !d_outstanding) begin  // push dcache read req
                reqs[1] <= '{
                    valid      : 1'b1,
                    source     : 1'b1,
                    addr  : d_dfp_addr,
                    req_cnt   : 2'd0,
                    rdata : 256'd0
                };
            end

            // update entries when reads finish
            if (bmem_rvalid) begin
                for (integer i = 0; i < MAX_REQS; i++) begin
                    if ((bmem_raddr == reqs[i].addr) && reqs[i].req_cnt < 3'd4) begin
                        reqs[i].rdata <= reqs[i].rdata | 256'(bmem_rdata << (64*reqs[i].req_cnt));
                        reqs[i].req_cnt <= reqs[i].req_cnt + 1'b1;
                    end
                end
            end

            if (reqs[0].valid && (reqs[0].req_cnt == 3'd4) && i_outstanding)
                reqs[0].valid <= 1'b0;

            if (reqs[1].valid && (reqs[1].req_cnt == 3'd4) && d_outstanding)
                reqs[1].valid <= 1'b0;
        end
    end

    assign i_outstanding = reqs[0].valid;
    assign d_outstanding = reqs[1].valid;
            
    /*** issue r/w to bmem ***/
    always_comb begin : ARBITER
        {i_dfp_rdata, d_dfp_rdata} = 'x;
        {i_dfp_resp, d_dfp_resp}   = '0;

        dfp_addr = 'x;
        {dfp_read, dfp_write} = '0;

        if (d_dfp_write) begin
            dfp_addr    = d_dfp_addr;
            dfp_read    = d_dfp_read;
            dfp_write   = d_dfp_write;
            d_dfp_rdata = dfp_rdata;
            dfp_wdata   = d_dfp_wdata;
            d_dfp_resp  = dfp_resp;
        end
        else if (new_d_read) begin
            dfp_addr    = d_dfp_addr;
            dfp_read    = d_dfp_read;
            dfp_write   = d_dfp_write;
            dfp_wdata   = d_dfp_wdata;
        end
        else if (new_i_read) begin
            dfp_addr    = i_dfp_addr;
            dfp_read    = i_dfp_read;
            dfp_write   = i_dfp_write;
            dfp_wdata   = i_dfp_wdata;
        end
        else begin
            dfp_wdata   = i_dfp_wdata;
        end

        if (reqs[0].valid && (reqs[0].req_cnt == 3'd4) && i_outstanding) begin
            if (reqs[0].source == 1'b0) begin
                i_dfp_rdata = reqs[0].rdata;
                i_dfp_resp  = 1'b1;
                
            end
        end

        if (reqs[1].valid && (reqs[1].req_cnt == 3'd4) && d_outstanding) begin
            if (reqs[1].source == 1'b1) begin
                d_dfp_rdata = reqs[1].rdata;
                d_dfp_resp  = 1'b1;
            end
        end
    end

    /*** ***/
    always_ff @ (posedge clk) begin : STATE_TRANSITION
        if (rst) begin
            {state, b_ctr} <= '0;
            b_ctr                         <= '0;
        end
        else     {state, b_ctr} <= {state_next, b_ctr_next};
    end

    always_comb begin : OUTPUT__NEXT_STATE
        state_next         = state;
        b_ctr_next         = b_ctr;

        bmem_addr                = 'x;
        {bmem_read, bmem_write}  = '0;
        bmem_wdata               = 'x;

        dfp_rdata = 'x;
        dfp_resp  = '0;

        unique case (state)
            idle: begin // wait for valid DFP request

                // relay appropriate request to bmem
                if (new_i_read | new_d_read) begin
                    bmem_addr  = dfp_addr;
                    bmem_read  = '1;
                    state_next = idle;
                end

                if (dfp_write) begin
                    bmem_addr  = dfp_addr;
                    bmem_write = '1;
                    bmem_wdata = dfp_wdata[64*b_ctr +: 64];
                    state_next = b_write;
                    b_ctr_next = 3'(b_ctr + 'd1);
                end
            end
            b_read: begin

            end
            b_write: begin
                if (b_ctr == 3'd3) begin
                    dfp_resp   = '1;
                    state_next = idle;
                end

                bmem_addr  = dfp_addr;
                bmem_write = '1;
                bmem_wdata = dfp_wdata[64*b_ctr +: 64]; 
                b_ctr_next = (b_ctr == 3'd3) ? '0:3'(b_ctr + 'd1);
            end
            default: state_next = state;
        endcase
    end

    logic [256:0] d1; assign d1 = {bmem_raddr, bmem_ready};//to get rid of lint warnings
    logic [256:0] d2; assign d2 = {i_dfp_wdata, i_dfp_write};
endmodule : cacheline_adapter