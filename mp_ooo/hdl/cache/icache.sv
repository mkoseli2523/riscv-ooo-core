module icache
import rv32i_types::*;
import cache_types::*;
(
    input   logic           clk,
    input   logic           rst,

    // cpu side signals, ufp -> upward facing port
    input   logic   [31:0]  ufp_addr,
    input   logic   [3:0]   ufp_rmask,
    input   logic   [3:0]   ufp_wmask,
    output  logic   [31:0]  ufp_rdata,
    input   logic   [31:0]  ufp_wdata,
    output  logic           ufp_resp,

    // memory side signals, dfp -> downward facing port
    output  logic   [31:0]  dfp_addr,
    output  logic           dfp_read,
    output  logic           dfp_write,
    input   logic   [255:0] dfp_rdata,
    output  logic   [255:0] dfp_wdata,
    input   logic           dfp_resp
);
        cache_req   req, req_next;
        logic       stall_pp;

        cache_line_t line;
        assign line.l_dirty = 'x;

        logic                  valid_csb;
        logic                  valid_web;
        logic [SET_WIDTH-1:0]  valid_addr;
        logic                  valid_din;

        logic                  tag_csb;
        logic                  tag_web;
        logic [SET_WIDTH-1:0]  tag_addr;
        logic [TAG_WIDTH-1:0]  tag_din;

        logic                       data_csb;
        logic                       data_web;
        logic [DATA_MASK_WIDTH-1:0] data_wmask;
        logic [SET_WIDTH-1:0]       data_addr;
        logic [DATA_WIDTH-1:0]      data_din;

        logic [TAG_WIDTH-1:0]    tag, tag_reg;
        logic [SET_WIDTH-1:0]    set, set_reg;
        logic [OFFSET_WIDTH-1:0] offset, offset_reg;

        logic dfp_resp_reg;

        logic cache_hit;
        logic valid_incoming;

        typedef struct packed {
            logic [31:0]           next_addr;
            logic                  valid;

            logic [TAG_WIDTH-1:0]  l_tag;
            logic [DATA_WIDTH-1:0] l_data;
        } pf_buffer;

        pf_buffer pf_buf, pf_buf_next;

        typedef struct packed {
            logic                        active;  // we are in a burst
        } pf_state_t;

        pf_state_t pf_q, pf_d;

        logic [31:0] pf_addr;
        logic pf_req;  // high in cycle we issue a pf read
        logic pf_out;  // high when the read is outstanding
        logic stall_pf;

        logic pf_buf_hit;
    
    always_ff @ (posedge clk) begin : PF_ADDR_LATCH
        if (rst)
            pf_addr <= 'x;
        
        if (pf_req)
            pf_addr <= {req.addr[31:SET_LSB], OFFSET_WIDTH'(0)} + 32;
    end

    // dfp_resp_reg high means an sram write is happening
    always_ff @ (posedge clk) dfp_resp_reg <= rst ? '0:dfp_resp;

    assign valid_incoming = |ufp_rmask && ~stall_pp;

    assign {tag, set, offset} = ufp_addr;  // (pf_buf_hit & ~cache_hit) ? pf_buf_next.next_addr : 
    assign {tag_reg, set_reg, offset_reg} = req.addr;

    assign cache_hit = req.r_valid && line.l_valid
             &&  (tag_reg == line.l_tag);
    assign pf_buf_hit = req.r_valid && pf_buf.valid
             &&  (set_reg == pf_buf.next_addr[SET_MSB:SET_LSB])
             &&  (tag_reg == pf_buf.l_tag);

    assign stall_pp = req.r_valid && ~cache_hit && ~pf_buf_hit;
    assign stall_pf = ~cache_hit & pf_out;

    always_ff @ (posedge clk) begin : PP_CACHE_REG
        if (rst)
            req.r_valid <= '0;
        else
            req <= req_next;
    end

    always_comb begin : UFP
        ufp_resp  = '0;
        ufp_rdata = 'x;

        if (pf_buf_hit & ~cache_hit) begin
            ufp_resp  = '1;
            ufp_rdata = pf_buf.l_data[32*offset_reg[4:2] +: 32];
        end
        else if (cache_hit) begin
            ufp_resp  = '1;
            ufp_rdata = line.l_data[32*offset_reg[4:2] +: 32];
        end
    end

    always_comb begin : DFP
        dfp_write   = '0;
        dfp_wdata   = 'x;

        dfp_read = 1'b0;
        dfp_addr = 'x;

        // if miss, write address to sram (allocate)
        if (stall_pp & ~dfp_resp_reg) begin
            dfp_read = 1'b1;
            dfp_addr = {req.addr[31:SET_LSB], OFFSET_WIDTH'(0)};
        end

        // prefetch when cache hit
        if (pf_out) begin
            dfp_read = 1'b1;
            dfp_addr = pf_addr;  // next line
        end

        // if prefetching but no cache hit
        if (stall_pf) begin
            dfp_read = 1'b1;
            dfp_addr = pf_addr;
        end
    end

    always_comb begin : SRAM
        {valid_csb, valid_web}  = '1;
        {valid_addr, valid_din} = 'x;

        {tag_csb, tag_web}  = '1;
        {tag_addr, tag_din} = 'x;

        {data_csb, data_web}  = '1;
        data_wmask            = 'x;
        {data_addr, data_din} = 'x;

        if (valid_incoming) begin
            {valid_csb, tag_csb, data_csb}    = '0;
            {valid_addr, tag_addr, data_addr} = {3{set}};
        end
        else if (req.r_valid && ~dfp_resp) begin
            {valid_csb, tag_csb, data_csb}    = '0;
            {valid_addr, tag_addr, data_addr} = {3{set_reg}};
        end
        else if (req.r_valid && dfp_resp && ~pf_out) begin
            {valid_csb, tag_csb, data_csb}    = '0;
            {valid_web, tag_web, data_web}    = '0;
            {valid_addr, tag_addr, data_addr} = {3{set_reg}};

            valid_din  = '1;
            tag_din    = tag_reg;
            data_wmask = '1;
            data_din   = dfp_rdata;
        end
    end

    always_comb begin : PREFETCH
        pf_d = pf_q;

        // on miss, initiate prefetch
        if (stall_pp && ~pf_out && dfp_resp) begin
            pf_d.active    = 1'b1;  // set to 1'b0 to ignore prefetcher
        end

        // when the pf_req you issued comes back, stop
        if (pf_out && dfp_resp) begin
            pf_d.active = 1'b0;
        end

        if (rst)
            pf_d = '0;
    end

    always_ff @(posedge clk) pf_q <= pf_d;

    always_ff @(posedge clk) begin
        if (rst)
            pf_out <= 1'b0;
        else if (pf_req)
            pf_out <= 1'b1;  // just issued a pf
        else if (!pf_d.active)
            pf_out <= 1'b0;  // burst finished
    end

    // pf_req is true whenever we’re in allocate (i.e. stall_pp) 
    // and have not yet issued this line’s pf
    always_comb
        pf_req = dfp_resp_reg && pf_q.active && !pf_out; 

    always_comb begin : PF_BUF_NEXT
        pf_buf_next = pf_buf;

        if (pf_out & pf_d.active)
            pf_buf_next.valid = 1'b0;

        if (~pf_buf.valid && dfp_resp && pf_out) begin
            pf_buf_next = '{
                next_addr : pf_addr,
                valid     : 1'b1,
                l_tag     : pf_addr[TAG_MSB:TAG_LSB],
                l_data    : dfp_rdata
            };
        end

        if (rst)
            pf_buf_next = '0;
    end

    always_ff @(posedge clk) pf_buf <= pf_buf_next;

    sp_ff_array #(
        .S_INDEX(ICACHE_SET_WIDTH)
    ) valid_array (
        .clk0       (clk),
        .rst0       (rst),
        .csb0       (valid_csb),
        .web0       (valid_web),
        .addr0      (valid_addr),
        .din0       (valid_din),
        .dout0      (line.l_valid)
    );
    mp_cache_tag_array tag_array (
        .clk0       (clk),
        .csb0       (tag_csb),
        .web0       (tag_web),
        .addr0      (tag_addr),
        .din0       (tag_din),
        .dout0      (line.l_tag)
    );
    mp_cache_data_array data_array (
        .clk0       (clk),
        .csb0       (data_csb),
        .web0       (data_web),
        .wmask0     (data_wmask),
        .addr0      (data_addr),
        .din0       (data_din),
        .dout0      (line.l_data)
    );

    always_comb begin
        req_next = '{
            r_valid   :  valid_incoming,

            addr      :  ufp_addr,
            rmask     :  ufp_rmask,
            wmask     :  'x,
            wdata     :  'x
        };

        if (stall_pp) begin
            req_next = '{
                r_valid   :  req.r_valid,

                addr      :  req.addr,
                rmask     :  req.rmask,
                wmask     :  'x,
                wdata     :  'x
            };            
        end
    end

    logic [2+31+3-1:0] d0; assign d0 = {ufp_wmask, ufp_wdata};//stop linter from complaining

endmodule : icache