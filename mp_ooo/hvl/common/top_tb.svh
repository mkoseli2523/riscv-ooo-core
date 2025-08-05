longint timeout;
    initial begin
        $value$plusargs("TIMEOUT_ECE411=%d", timeout);
    end

    mem_itf_banked mem_itf(.*);
    dram_w_burst_frfcfs_controller mem(.itf(mem_itf));
    // random_tb mem(.itf(mem_itf));

    mon_itf #(.CHANNELS(8)) mon_itf(.*);
    monitor #(.CHANNELS(8)) monitor(.itf(mon_itf));

    cpu dut(
        .clk            (clk),
        .rst            (rst),

        .bmem_addr  (mem_itf.addr  ),
        .bmem_read  (mem_itf.read  ),
        .bmem_write (mem_itf.write ),
        .bmem_wdata (mem_itf.wdata ),
        .bmem_ready (mem_itf.ready ),
        .bmem_raddr (mem_itf.raddr ),
        .bmem_rdata (mem_itf.rdata ),
        .bmem_rvalid(mem_itf.rvalid)
    );

    `include "rvfi_reference.svh"

    int num_branches = 0;
    int num_correct = 0;

    logic is_branch, commit, predicted, actual, iqueue_full;

    int mul_cnt = 0;
    int div_cnt = 0;

    logic is_mul, is_div;
    
    assign is_branch = dut.rvfi_out[0].i_opcode == 7'b1100011;
    assign is_mul = dut.rvfi_out[0].i_opcode == 7'b0110011 && dut.rvfi_out[0].i_funct7 == 7'b0000001 && (dut.rvfi_out[0].i_funct3 < 3'b100);
    assign is_div = dut.rvfi_out[0].i_opcode == 7'b0110011 && dut.rvfi_out[0].i_funct7 == 7'b0000001 && (dut.rvfi_out[0].i_funct3 > 3'b011);
    assign commit = dut.commit_en[0];
    assign predicted = dut.rvfi_out[0].br_pred;
    assign actual = dut.rvfi_out[0].br_result;
    assign iqueue_full = dut.iq_full;

    always @(posedge clk) begin
        // if (iqueue_full) begin
        //     $display("iq full");
        // end
        
        if (is_branch && commit && predicted == actual) begin
            num_correct++;
        end 

        if (is_mul & commit) begin
            mul_cnt++;
        end

        if (is_div & commit) begin
            div_cnt++;
        end
        
        if (is_branch && commit) begin
            num_branches++;

            if (num_branches % 1000 == 0) begin
                $display("Total number of branches: %d", num_branches);
                $display("Number mispredicted: %d", num_correct);
            end
        end
    end

        longint cycle_count = longint'(0);

    int iq_full         = int'(0);
    int fl_empty        = int'(0);
    int br_mul_div_full = int'(0);
    int alu_full        = int'(0);
    int lsq_full        = int'(0);
    int rob_full        = int'(0);


    always @ (posedge clk) begin
        cycle_count += longint'(1);

        iq_full         += dut.iq_full ? 1 : 0;
        fl_empty        += dut.fl_empty ? 1 : 0;
        br_mul_div_full += dut.bmd_rs_full ? 1 : 0;
        alu_full        += dut.alu_rs_full ? 1 : 0;
        lsq_full        += dut.lsq_full ? 1 : 0;
        rob_full        += dut.rob_full[0] ? 1 : 0;
    end

    always @(posedge clk) begin
        // if (mon_itf.halt) begin
        //     $display("Total number of branches: %d", num_branches);
        //     $display("Number predictet:         %d", num_correct);
        //     $display("Percent accuracy:         %f", real'(num_correct)/real'(num_branches));
        //     $finish;
        // end
        if (mon_itf.halt) begin
            // Display cycle count and each variable with its fraction
            $display("cycle_cnt: %f", cycle_count);
            $display("iq_full: %d, iq_full_fraction: %f", iq_full, real'(iq_full) / real'(cycle_count));
            $display("fl_empty: %d, fl_empty_fraction: %f", fl_empty, real'(fl_empty) / real'(cycle_count));
            $display("br_mul_div_full: %d, br_mul_div_full_fraction: %f", br_mul_div_full, real'(br_mul_div_full) / real'(cycle_count));
            $display("alu_full: %d, alu_full_fraction: %f", alu_full, real'(alu_full) / real'(cycle_count));
            $display("lsq_full: %d, lsq_full_fraction: %f", lsq_full, real'(lsq_full) / real'(cycle_count));
            $display("rob_full: %d, rob_full_fraction: %f", rob_full, real'(rob_full) / real'(cycle_count));

            $display("mul_cnt: %d", mul_cnt);
            $display("div_cnt: %d", div_cnt);

            $finish;
        end
        if (timeout == 0) begin
            $error("TB Error: Timed out");
            $fatal;
        end
        if (mem_itf.error != 0 || mon_itf.error != 0) begin
            $fatal;
        end
        timeout <= timeout - 1;
    end