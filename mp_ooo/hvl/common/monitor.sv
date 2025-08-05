module monitor #(
    parameter CHANNELS = 1
)(
    mon_itf itf
);

    function bit is_halt(input logic [31:0] inst);
        is_halt = inst inside {32'h00000063, 32'h0000006f, 32'hF0002013};
    endfunction

    always @(posedge itf.clk iff !itf.rst) begin
        for (int unsigned channel = 0; channel < CHANNELS; channel++) begin
            if (itf.valid[channel] && is_halt(itf.inst[channel])) begin
                itf.halt <= 1'b1;
            end
        end
    end

    `ifndef ECE411_NO_ROI

        int time_fd;
        initial time_fd = $fopen("./time.txt", "w");

        longint inst_count = longint'(0);
        longint cycle_count = longint'(0);
        longint start_time = longint'(0);
        bit ipc_printed = 1'b0;

        longint power_start_time = longint'(0);
        bit power_printed = 1'b0;

        `ifdef ECE411_VERILATOR
            bit dump_on = 1'b0;
        `endif

        always @(posedge itf.clk iff !itf.rst) begin
            cycle_count += longint'(1);
            for (int unsigned channel = 0; channel < CHANNELS; channel++) begin
                if (itf.valid[channel]) begin
                    inst_count += longint'(1);
                    if (itf.inst[channel] == 32'h00102013) begin
                        $display("Monitor: Segment Start time is %t", $time);
                        inst_count = longint'(0);
                        cycle_count = longint'(0);
                        start_time = $time;
                    end
                    if (itf.inst[channel] == 32'h00202013) begin
                        ipc_printed = 1'b1;
                        $display("Monitor: Segment Stop time is %t", $time);
                        $display("Monitor: Segment IPC: %f", real'(inst_count) / cycle_count);
                        $display("Monitor: Segment Time: %t", $time - start_time);
                    end
                    if (itf.inst[channel] == 32'h00302013) begin
                        $display("Monitor: Power Start time is %t", $time);
                        power_start_time = $time;
                        if ($test$plusargs("NO_DUMP_ALL_ECE411")) begin
                            `ifdef ECE411_VERILATOR
                                // $dumpon();
                                dump_on = 1'b1;
                            `else
                                $fsdbDumpon();
                            `endif
                        end
                    end
                    if (itf.inst[channel] == 32'h00402013) begin
                        power_printed = 1'b1;
                        $display("Monitor: Power Stop time is %t", $time);
                        $fwrite(time_fd, "%0t\n", power_start_time);
                        $fwrite(time_fd, "%0t", $time);
                        if ($test$plusargs("NO_DUMP_ALL_ECE411")) begin
                            `ifdef ECE411_VERILATOR
                                // $dumpoff();
                                dump_on = 1'b0;
                            `else
                                $fsdbDumpoff();
                            `endif
                        end
                    end
                end
            end
        end

        final begin
            if (!ipc_printed) begin
                $display("Monitor: Total IPC: %f", real'(inst_count) / cycle_count);
                $display("Monitor: Total Time: %t", $time - start_time);
            end
            if (!power_printed) begin
                $fwrite(time_fd, "%0t\n", power_start_time);
                $fwrite(time_fd, "%0t", $time);
            end
            $fclose(time_fd);
        end

    `endif

    `ifndef ECE411_NO_SPIKE_PRINTER

        int spike_fd;
        initial spike_fd = $fopen("../spike/commit.log", "w");
        final $fclose(spike_fd);

        always @ (posedge itf.clk iff !itf.rst) begin
            if (!itf.halt) begin
                automatic struct {
                    int unsigned channel;
                    longint unsigned order;
                } sp, s[$:CHANNELS];
                for (int unsigned channel = 0; channel < CHANNELS; channel++) begin
                    if(itf.valid[channel]) begin
                        sp.channel = channel;
                        sp.order = itf.order[channel];
                        s.push_front(sp);
                    end
                end
                if (s.size() != 0) begin
                    s.rsort with(item.order);
                end
                while (s.size() != 0) begin
                    automatic int channel;
                    sp = s.pop_back();
                    channel = sp.channel;
                    if (itf.order[channel] % 1000 == 0) begin
                        $display("dut commit No.%d, rd_s: x%02d, rd: 0x%h", itf.order[channel], itf.rd_addr[channel], |itf.rd_addr[channel] ? itf.rd_wdata[channel] : 32'd0);
                    end
                    if (itf.inst[channel][1:0] == 2'b11) begin
                        $fwrite(spike_fd, "core   0: 3 0x%h (0x%h)", itf.pc_rdata[channel], itf.inst[channel]);
                    end else begin
                        $fwrite(spike_fd, "core   0: 3 0x%h (0x%h)", itf.pc_rdata[channel], itf.inst[channel][15:0]);
                    end
                    if (|itf.rd_addr[channel]) begin
                        if (itf.rd_addr[channel] < 10)
                            $fwrite(spike_fd, " x%0d  ", itf.rd_addr[channel]);
                        else
                            $fwrite(spike_fd, " x%0d ", itf.rd_addr[channel]);
                        $fwrite(spike_fd, "0x%h", itf.rd_wdata[channel]);
                    end
                    `ifndef ECE411_NO_FLOAT
                        if (|itf.frd_addr[channel]) begin
                            if (itf.frd_addr[channel] < 10)
                                $fwrite(spike_fd, " f%0d  ", itf.frd_addr[channel][4:0]);
                            else
                                $fwrite(spike_fd, " f%0d ", itf.frd_addr[channel][4:0]);
                            $fwrite(spike_fd, "0x%h", itf.frd_wdata[channel]);
                        end
                    `endif
                    if (|itf.mem_rmask[channel]) begin
                        automatic int first_1 = 0;
                        for(int i = 0; i < 4; i++) begin
                            if(itf.mem_rmask[channel][i]) begin
                                first_1 = i;
                                break;
                            end
                        end
                        $fwrite(spike_fd, " mem 0x%h", {itf.mem_addr[channel][31:2], 2'b0} + first_1);
                    end
                    if (|itf.mem_wmask[channel]) begin
                        automatic int amount_o_1 = 0;
                        automatic int first_1 = 0;
                        for(int i = 0; i < 4; i++) begin
                            if(itf.mem_wmask[channel][i]) begin
                                amount_o_1 += 1;
                            end
                        end
                        for(int i = 0; i < 4; i++) begin
                            if(itf.mem_wmask[channel][i]) begin
                                first_1 = i;
                                break;
                            end
                        end
                        $fwrite(spike_fd, " mem 0x%h", {itf.mem_addr[channel][31:2], 2'b0} + first_1);
                        case (amount_o_1)
                            1: begin
                                automatic logic[7:0] wdata_byte = itf.mem_wdata[channel][8*first_1 +: 8];
                                $fwrite(spike_fd, " 0x%h", wdata_byte);
                            end
                            2: begin
                                automatic logic[15:0] wdata_half = itf.mem_wdata[channel][8*first_1 +: 16];
                                $fwrite(spike_fd, " 0x%h", wdata_half);
                            end
                            4:
                                $fwrite(spike_fd, " 0x%h", itf.mem_wdata[channel]);
                        endcase
                    end
                    $fwrite(spike_fd, "\n");
                    if (is_halt(itf.inst[channel])) begin
                        break;
                    end
                end
            end
        end

    `endif

    `ifndef ECE411_NO_X_CHECK

        always @(posedge itf.clk iff !itf.rst) begin
            for (int unsigned channel = 0; channel < CHANNELS; channel++) begin
                if ($isunknown(itf.valid[channel])) begin
                    $error("RVFI Interface Error: valid is 1'bx");
                    itf.error <= 1'b1;
                end
            end
        end

        generate for (genvar channel = 0; channel < CHANNELS; channel++) begin : x_detection
            always @(posedge itf.clk iff !itf.rst) begin
                if (itf.valid[channel]) begin
                    if ($isunknown(itf.order[channel])) begin
                        $error("RVFI Interface Error: order contains 'x");
                        itf.error <= 1'b1;
                    end
                    if ($isunknown(itf.inst[channel])) begin
                        $error("RVFI Interface Error: inst contains 'x");
                        itf.error <= 1'b1;
                    end
                    if ($isunknown(itf.rs1_addr[channel])) begin
                        $error("RVFI Interface Error: rs1_addr contains 'x");
                        itf.error <= 1'b1;
                    end
                    if ($isunknown(itf.rs2_addr[channel])) begin
                        $error("RVFI Interface Error: rs2_addr contains 'x");
                        itf.error <= 1'b1;
                    end
                    if (|itf.rs1_addr[channel]) begin
                        if ($isunknown(itf.rs1_rdata[channel])) begin
                            $error("RVFI Interface Error: rs1_rdata contains 'x");
                            itf.error <= 1'b1;
                        end
                    end
                    if (|itf.rs2_addr[channel]) begin
                        if ($isunknown(itf.rs2_rdata[channel])) begin
                            $error("RVFI Interface Error: rs2_rdata contains 'x");
                            itf.error <= 1'b1;
                        end
                    end
                    if ($isunknown(itf.rd_addr[channel])) begin
                        $error("RVFI Interface Error: rd_addr contains 'x");
                        itf.error <= 1'b1;
                    end
                    if (|itf.rd_addr[channel]) begin
                        if ($isunknown(itf.rd_wdata[channel])) begin
                            $error("RVFI Interface Error: rd_wdata contains 'x");
                            itf.error <= 1'b1;
                        end
                    end
                    `ifndef ECE411_NO_FLOAT
                        if ($isunknown(itf.frs1_addr[channel])) begin
                            $error("RVFI Interface Error: frs1_addr contains 'x");
                            itf.error <= 1'b1;
                        end
                        if ($isunknown(itf.frs2_addr[channel])) begin
                            $error("RVFI Interface Error: frs2_addr contains 'x");
                            itf.error <= 1'b1;
                        end
                        if ($isunknown(itf.frs3_addr[channel])) begin
                            $error("RVFI Interface Error: frs3_addr contains 'x");
                            itf.error <= 1'b1;
                        end
                        if (|itf.frs1_addr[channel]) begin
                            if ($isunknown(itf.frs1_rdata[channel])) begin
                                $error("RVFI Interface Error: frs1_rdata contains 'x");
                                itf.error <= 1'b1;
                            end
                        end
                        if (|itf.frs2_addr[channel]) begin
                            if ($isunknown(itf.frs2_rdata[channel])) begin
                                $error("RVFI Interface Error: frs2_rdata contains 'x");
                                itf.error <= 1'b1;
                            end
                        end
                        if (|itf.frs3_addr[channel]) begin
                            if ($isunknown(itf.frs3_rdata[channel])) begin
                                $error("RVFI Interface Error: frs3_rdata contains 'x");
                                itf.error <= 1'b1;
                            end
                        end
                        if ($isunknown(itf.frd_addr[channel])) begin
                            $error("RVFI Interface Error: frd_addr contains 'x");
                            itf.error <= 1'b1;
                        end
                        if (|itf.frd_addr[channel]) begin
                            if ($isunknown(itf.frd_wdata[channel])) begin
                                $error("RVFI Interface Error: frd_wdata contains 'x");
                                itf.error <= 1'b1;
                            end
                        end
                    `endif
                    if ($isunknown(itf.pc_rdata[channel])) begin
                        $error("RVFI Interface Error: pc_rdata contains 'x");
                        itf.error <= 1'b1;
                    end
                    if ($isunknown(itf.pc_wdata[channel])) begin
                        $error("RVFI Interface Error: pc_wdata contains 'x");
                        itf.error <= 1'b1;
                    end
                    if ($isunknown(itf.mem_rmask[channel])) begin
                        $error("RVFI Interface Error: mem_rmask contains 'x");
                        itf.error <= 1'b1;
                    end
                    if ($isunknown(itf.mem_wmask[channel])) begin
                        $error("RVFI Interface Error: mem_wmask contains 'x");
                        itf.error <= 1'b1;
                    end
                    if (|itf.mem_rmask[channel] || |itf.mem_wmask[channel]) begin
                        if ($isunknown(itf.mem_addr[channel])) begin
                            $error("RVFI Interface Error: mem_addr contains 'x");
                            itf.error <= 1'b1;
                        end
                    end
                    if (|itf.mem_rmask[channel]) begin
                        for (int i = 0; i < 4; i++) begin
                            if (itf.mem_rmask[channel][i]) begin
                                if ($isunknown(itf.mem_rdata[channel][i*8 +: 8])) begin
                                    $error("RVFI Interface Error: mem_rdata contains 'x");
                                    itf.error <= 1'b1;
                                end
                            end
                        end
                    end
                    if (|itf.mem_wmask[channel]) begin
                        for (int i = 0; i < 4; i++) begin
                            if (itf.mem_wmask[channel][i]) begin
                                if ($isunknown(itf.mem_wdata[channel][i*8 +: 8])) begin
                                    $error("RVFI Interface Error: mem_wdata contains 'x");
                                    itf.error <= 1'b1;
                                end
                            end
                        end
                    end
                end
            end
        end endgenerate

    `endif

    `ifndef ECE411_NO_SPIKE_DPI

        typedef struct packed {
            bit [31:0] inst;
            bit [31:0] trapped;
            bit [31:0] rs1_addr;
            bit [31:0] rs2_addr;
            bit [31:0] rs1_rdata;
            bit [31:0] rs2_rdata;
            bit [31:0] rd_addr;
            bit [31:0] rd_wdata;
            bit [31:0] frs1_addr;
            bit [31:0] frs2_addr;
            bit [31:0] frs3_addr;
            bit [31:0] frs1_rdata;
            bit [31:0] frs2_rdata;
            bit [31:0] frs3_rdata;
            bit [31:0] frd_addr;
            bit [31:0] frd_wdata;
            bit [31:0] pc_rdata;
            bit [31:0] pc_wdata;
            bit [31:0] mem_addr;
            bit [31:0] mem_rmask;
            bit [31:0] mem_wmask;
            bit [31:0] mem_rdata;
            bit [31:0] mem_wdata;
        } spike_dpi_rvfi_itf_t;

        import "DPI-C" function void spike_dpi_init(string mem_space, string elf_file);
        import "DPI-C" function int unsigned spike_dpi_fin();
        import "DPI-C" function int unsigned spike_dpi_next(output spike_dpi_rvfi_itf_t r);
        import "DPI-C" function string spike_dpi_dasm();

        initial begin
            automatic string elf_file;
            $value$plusargs("ELF_ECE411=%s", elf_file);
            $display("using elf file %s", elf_file);
            spike_dpi_init("-m0xaaaaa000:0x55556000", elf_file);
        end

        final begin
            automatic int unsigned retval;
            retval = spike_dpi_fin();
        end

        spike_dpi_rvfi_itf_t spike_dpi_rvfi_itf;
        longint spike_dpi_order;
        initial spike_dpi_order = 64'd0;

        always @ (posedge itf.clk iff !itf.rst) begin
            automatic struct {
                int unsigned channel;
                longint unsigned order;
            } sp, s[$:CHANNELS];
            for (int unsigned channel = 0; channel < CHANNELS; channel++) begin
                if(itf.valid[channel]) begin
                    sp.channel = channel;
                    sp.order = itf.order[channel];
                    s.push_front(sp);
                end
            end
            if (s.size() != 0) begin
                s.rsort with(item.order);
            end
            while (s.size() != 0) begin
                automatic int channel;
                automatic bit [21:0] diff = '0;
                automatic int unsigned retval;
                sp = s.pop_back();
                channel = sp.channel;
                retval = spike_dpi_next(spike_dpi_rvfi_itf);
                diff[ 0] = itf.inst      [channel] != spike_dpi_rvfi_itf.inst          ;
                diff[ 1] = |spike_dpi_rvfi_itf.rs1_addr[4:0]  ? itf.rs1_addr  [channel] != spike_dpi_rvfi_itf.rs1_addr[4:0]  : 1'b0;
                diff[ 2] = |spike_dpi_rvfi_itf.rs1_addr[4:0]  ? itf.rs1_rdata [channel] != spike_dpi_rvfi_itf.rs1_rdata      : 1'b0;
                diff[ 3] = |spike_dpi_rvfi_itf.rs2_addr[4:0]  ? itf.rs2_addr  [channel] != spike_dpi_rvfi_itf.rs2_addr[4:0]  : 1'b0;
                diff[ 4] = |spike_dpi_rvfi_itf.rs2_addr[4:0]  ? itf.rs2_rdata [channel] != spike_dpi_rvfi_itf.rs2_rdata      : 1'b0;
                diff[ 5] = itf.rd_addr   [channel] != spike_dpi_rvfi_itf.rd_addr[4:0]  ;
                diff[ 6] = |spike_dpi_rvfi_itf.rd_addr[4:0]   ? itf.rd_wdata  [channel] != spike_dpi_rvfi_itf.rd_wdata       : 1'b0;
                `ifndef ECE411_NO_FLOAT
                    diff[ 7] = |spike_dpi_rvfi_itf.frs1_addr[5:0] ? itf.frs1_addr [channel] != spike_dpi_rvfi_itf.frs1_addr[5:0] : 1'b0;
                    diff[ 8] = |spike_dpi_rvfi_itf.frs1_addr[5:0] ? itf.frs1_rdata[channel] != spike_dpi_rvfi_itf.frs1_rdata     : 1'b0;
                    diff[ 9] = |spike_dpi_rvfi_itf.frs2_addr[5:0] ? itf.frs2_addr [channel] != spike_dpi_rvfi_itf.frs2_addr[5:0] : 1'b0;
                    diff[10] = |spike_dpi_rvfi_itf.frs2_addr[5:0] ? itf.frs2_rdata[channel] != spike_dpi_rvfi_itf.frs2_rdata     : 1'b0;
                    diff[11] = |spike_dpi_rvfi_itf.frs3_addr[5:0] ? itf.frs3_addr [channel] != spike_dpi_rvfi_itf.frs3_addr[5:0] : 1'b0;
                    diff[12] = |spike_dpi_rvfi_itf.frs3_addr[5:0] ? itf.frs3_rdata[channel] != spike_dpi_rvfi_itf.frs3_rdata     : 1'b0;
                    diff[13] = itf.frd_addr  [channel] != spike_dpi_rvfi_itf.frd_addr[5:0] ;
                    diff[14] = |spike_dpi_rvfi_itf.frd_addr[5:0]  ? itf.frd_wdata [channel] != spike_dpi_rvfi_itf.frd_wdata      : 1'b0;
                `endif
                diff[15] = itf.pc_rdata  [channel] != spike_dpi_rvfi_itf.pc_rdata      ;
                diff[16] = itf.pc_wdata  [channel] != spike_dpi_rvfi_itf.pc_wdata      ;
                diff[18] = itf.mem_rmask [channel] != spike_dpi_rvfi_itf.mem_rmask[3:0];
                diff[19] = itf.mem_wmask [channel] != spike_dpi_rvfi_itf.mem_wmask[3:0];
                if (spike_dpi_rvfi_itf.mem_rmask[3:0] != 4'd0) begin
                    diff[17] = {itf.mem_addr[channel][31:2], 2'b00} != spike_dpi_rvfi_itf.mem_addr;
                    diff[20] = 1'b0;
                    for (int i = 0; i < 4; i++) begin
                        if (spike_dpi_rvfi_itf.mem_rmask[i] && (itf.mem_rdata[channel][i*8 +: 8] != spike_dpi_rvfi_itf.mem_rdata[i*8 +: 8])) begin
                            diff[20] = 1'b1;
                        end
                    end
                end
                if (spike_dpi_rvfi_itf.mem_wmask[3:0] != 4'd0) begin
                    diff[17] = {itf.mem_addr[channel][31:2], 2'b00} != spike_dpi_rvfi_itf.mem_addr;
                    diff[21] = 1'b0;
                    for (int i = 0; i < 4; i++) begin
                        if (spike_dpi_rvfi_itf.mem_wmask[i] && (itf.mem_wdata[channel][i*8 +: 8] != spike_dpi_rvfi_itf.mem_wdata[i*8 +: 8])) begin
                            diff[21] = 1'b1;
                        end
                    end
                end
                if (spike_dpi_rvfi_itf.trapped) begin
                    $display("");
                    $error("Spike Monitor Error at time %0t channel %0d order %0d", $time, channel, itf.order[channel]);
                    $display("Trapped at pc x%08x inst x%08x dasm %s", spike_dpi_rvfi_itf.pc_rdata, spike_dpi_rvfi_itf.inst, spike_dpi_dasm());
                    $display("");
                    itf.error <= 1'b1;
                end else if (itf.order[channel] != spike_dpi_order) begin
                    $display("");
                    $error("Spike Monitor Error at time %0t channel %0d order %0d", $time, channel, itf.order[channel]);
                    $display("Expected order %0d, got %0d.", spike_dpi_order, itf.order[channel]);
                    $display("");
                    itf.error <= 1'b1;
                end else if (diff != 22'd0) begin
                    $display("");
                    $error("Spike Monitor Error at time %0t channel %0d order %0d", $time, channel, itf.order[channel]);
                    $display("-------begin spike mismatch--------");
                    $display("%010s %04s %09s %09s"              , "signal    ", "diff", "      dut", "    spike");
                    $display("%010s %04s h%08x h%08x %s"         , "inst      ", diff[ 0] ? "--->" : "    ", itf.inst      [channel], spike_dpi_rvfi_itf.inst, spike_dpi_dasm());
                    $display("%010s %04s        %02d        %02d", "rs1_addr  ", diff[ 1] ? "--->" : "    ", itf.rs1_addr  [channel], spike_dpi_rvfi_itf.rs1_addr  );
                    $display("%010s %04s h%08x h%08x"            , "rs1_rdata ", diff[ 2] ? "--->" : "    ", itf.rs1_rdata [channel], spike_dpi_rvfi_itf.rs1_rdata );
                    $display("%010s %04s        %02d        %02d", "rs2_addr  ", diff[ 3] ? "--->" : "    ", itf.rs2_addr  [channel], spike_dpi_rvfi_itf.rs2_addr  );
                    $display("%010s %04s h%08x h%08x"            , "rs2_rdata ", diff[ 4] ? "--->" : "    ", itf.rs2_rdata [channel], spike_dpi_rvfi_itf.rs2_rdata );
                    $display("%010s %04s        %02d        %02d", "rd_addr   ", diff[ 5] ? "--->" : "    ", itf.rd_addr   [channel], spike_dpi_rvfi_itf.rd_addr   );
                    $display("%010s %04s h%08x h%08x"            , "rd_wdata  ", diff[ 6] ? "--->" : "    ", itf.rd_wdata  [channel], spike_dpi_rvfi_itf.rd_wdata  );
                    `ifndef ECE411_NO_FLOAT
                        $display("%010s %04s        %02d        %02d", "frs1_addr ", diff[ 7] ? "--->" : "    ", itf.frs1_addr [channel], spike_dpi_rvfi_itf.frs1_addr );
                        $display("%010s %04s h%08x h%08x"            , "frs1_rdata", diff[ 8] ? "--->" : "    ", itf.frs1_rdata[channel], spike_dpi_rvfi_itf.frs1_rdata);
                        $display("%010s %04s        %02d        %02d", "frs2_addr ", diff[ 9] ? "--->" : "    ", itf.frs2_addr [channel], spike_dpi_rvfi_itf.frs2_addr );
                        $display("%010s %04s h%08x h%08x"            , "frs2_rdata", diff[10] ? "--->" : "    ", itf.frs2_rdata[channel], spike_dpi_rvfi_itf.frs2_rdata);
                        $display("%010s %04s        %02d        %02d", "frs3_addr ", diff[11] ? "--->" : "    ", itf.frs3_addr [channel], spike_dpi_rvfi_itf.frs3_addr );
                        $display("%010s %04s h%08x h%08x"            , "frs3_rdata", diff[12] ? "--->" : "    ", itf.frs3_rdata[channel], spike_dpi_rvfi_itf.frs3_rdata);
                        $display("%010s %04s        %02d        %02d", "frd_addr  ", diff[13] ? "--->" : "    ", itf.frd_addr  [channel], spike_dpi_rvfi_itf.frd_addr  );
                        $display("%010s %04s h%08x h%08x"            , "frd_wdata ", diff[14] ? "--->" : "    ", itf.frd_wdata [channel], spike_dpi_rvfi_itf.frd_wdata );
                    `endif
                    $display("%010s %04s h%08x h%08x"            , "pc_rdata  ", diff[15] ? "--->" : "    ", itf.pc_rdata  [channel], spike_dpi_rvfi_itf.pc_rdata  );
                    $display("%010s %04s h%08x h%08x"            , "pc_wdata  ", diff[16] ? "--->" : "    ", itf.pc_wdata  [channel], spike_dpi_rvfi_itf.pc_wdata  );
                    $display("%010s %04s h%08x h%08x"            , "mem_addr  ", diff[17] ? "--->" : "    ", itf.mem_addr  [channel], spike_dpi_rvfi_itf.mem_addr  );
                    $display("%010s %04s     b%04b     b%04b"    , "mem_rmask ", diff[18] ? "--->" : "    ", itf.mem_rmask [channel], spike_dpi_rvfi_itf.mem_rmask );
                    $display("%010s %04s     b%04b     b%04b"    , "mem_wmask ", diff[19] ? "--->" : "    ", itf.mem_wmask [channel], spike_dpi_rvfi_itf.mem_wmask );
                    $display("%010s %04s h%08x h%08x"            , "mem_rdata ", diff[20] ? "--->" : "    ", itf.mem_rdata [channel], spike_dpi_rvfi_itf.mem_rdata );
                    $display("%010s %04s h%08x h%08x"            , "mem_wdata ", diff[21] ? "--->" : "    ", itf.mem_wdata [channel], spike_dpi_rvfi_itf.mem_wdata );
                    $display("-------end spike mismatch----------");
                    $display("");
                    itf.error <= 1'b1;
                end
                spike_dpi_order = spike_dpi_order + 64'd1;
            end
        end

    `endif

    `ifndef ECE411_NO_RVFI

        logic [CHANNELS*1 -1:0] rvfi_valid;
        logic [CHANNELS*64-1:0] rvfi_order;
        logic [CHANNELS*32-1:0] rvfi_insn;
        logic [CHANNELS*1 -1:0] rvfi_trap;
        logic [CHANNELS*1 -1:0] rvfi_halt;
        logic [CHANNELS*1 -1:0] rvfi_intr;
        logic [CHANNELS*2 -1:0] rvfi_mode;
        logic [CHANNELS*5 -1:0] rvfi_rs1_addr;
        logic [CHANNELS*5 -1:0] rvfi_rs2_addr;
        logic [CHANNELS*32-1:0] rvfi_rs1_rdata;
        logic [CHANNELS*32-1:0] rvfi_rs2_rdata;
        logic [CHANNELS*5 -1:0] rvfi_rd_addr;
        logic [CHANNELS*32-1:0] rvfi_rd_wdata;
        logic [CHANNELS*32-1:0] rvfi_pc_rdata;
        logic [CHANNELS*32-1:0] rvfi_pc_wdata;
        logic [CHANNELS*32-1:0] rvfi_mem_addr;
        logic [CHANNELS*4 -1:0] rvfi_mem_rmask;
        logic [CHANNELS*4 -1:0] rvfi_mem_wmask;
        logic [CHANNELS*32-1:0] rvfi_mem_rdata;
        logic [CHANNELS*32-1:0] rvfi_mem_wdata;
        logic [CHANNELS*1 -1:0] rvfi_mem_extamo;

        assign rvfi_trap = '0;
        assign rvfi_intr = '0;
        assign rvfi_mode = '0;
        assign rvfi_mem_extamo = '0;
        generate for (genvar channel = 0; channel < CHANNELS; channel++) begin : assign_channels
            assign rvfi_valid    [channel*1  +: 1 ] =   itf.valid    [channel];
            assign rvfi_order    [channel*64 +: 64] =   itf.order    [channel];
            assign rvfi_insn     [channel*32 +: 32] =   itf.inst     [channel];
            assign rvfi_halt     [channel*1  +: 1 ] =   itf.halt              ;
            assign rvfi_rs1_addr [channel*5  +: 5 ] =   itf.rs1_addr [channel];
            assign rvfi_rs2_addr [channel*5  +: 5 ] =   itf.rs2_addr [channel];
            assign rvfi_rs1_rdata[channel*32 +: 32] = (|itf.rs1_addr [channel]) ? itf.rs1_rdata[channel] : '0;
            assign rvfi_rs2_rdata[channel*32 +: 32] = (|itf.rs2_addr [channel]) ? itf.rs2_rdata[channel] : '0;
            assign rvfi_rd_addr  [channel*5  +: 5 ] =   itf.rd_addr  [channel];
            assign rvfi_rd_wdata [channel*32 +: 32] = (|itf.rd_addr  [channel]) ? itf.rd_wdata[channel] : '0;
            assign rvfi_pc_rdata [channel*32 +: 32] =   itf.pc_rdata [channel];
            assign rvfi_pc_wdata [channel*32 +: 32] =   itf.pc_wdata [channel];
            assign rvfi_mem_addr [channel*32 +: 32] = { itf.mem_addr [channel][31:2], 2'b00};
            assign rvfi_mem_rmask[channel*4  +: 4 ] =   itf.mem_rmask[channel];
            assign rvfi_mem_wmask[channel*4  +: 4 ] =   itf.mem_wmask[channel];
            assign rvfi_mem_rdata[channel*32 +: 32] =   itf.mem_rdata[channel];
            assign rvfi_mem_wdata[channel*32 +: 32] =   itf.mem_wdata[channel];
        end endgenerate

        logic [15:0] errcode;

        riscv_formal_monitor_rv32imc monitor(
            .clock              (itf.clk),
            .reset              (itf.rst),
            .rvfi_valid         (rvfi_valid),
            .rvfi_order         (rvfi_order),
            .rvfi_insn          (rvfi_insn),
            .rvfi_trap          (rvfi_trap),
            .rvfi_halt          (rvfi_halt),
            .rvfi_intr          (rvfi_intr),
            .rvfi_mode          (rvfi_mode),
            .rvfi_rs1_addr      (rvfi_rs1_addr),
            .rvfi_rs2_addr      (rvfi_rs2_addr),
            .rvfi_rs1_rdata     (rvfi_rs1_rdata),
            .rvfi_rs2_rdata     (rvfi_rs2_rdata),
            .rvfi_rd_addr       (rvfi_rd_addr),
            .rvfi_rd_wdata      (rvfi_rd_wdata),
            .rvfi_pc_rdata      (rvfi_pc_rdata),
            .rvfi_pc_wdata      (rvfi_pc_wdata),
            .rvfi_mem_addr      (rvfi_mem_addr),
            .rvfi_mem_rmask     (rvfi_mem_rmask),
            .rvfi_mem_wmask     (rvfi_mem_wmask),
            .rvfi_mem_rdata     (rvfi_mem_rdata),
            .rvfi_mem_wdata     (rvfi_mem_wdata),
            .rvfi_mem_extamo    (rvfi_mem_extamo),
            .errcode            (errcode)
        );

        always @(posedge itf.clk iff !itf.rst) begin
            if (errcode != 0) begin
                $error("RVFI Monitor Error");
                itf.error <= 1'b1;
            end
        end

    `endif

endmodule
