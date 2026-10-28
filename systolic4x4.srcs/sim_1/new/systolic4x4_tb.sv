`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// systolic4x4_tb: self-checking constrained-random testbench (single file)
//   sa_if          : DUT interface with driver/monitor clocking blocks
//   sa_pkg         : transactions, generator, driver, monitors, scoreboard, env
//   systolic4x4_tb : top (clock, reset, DUT, run)
//------------------------------------------------------------------------------
interface sa_if #(
    parameter int N      = 4,
    parameter int DATA_W = 8,
    parameter int ACC_W  = 32
) (input logic clk);

    logic                rst;
    logic [N*DATA_W-1:0] w_in;
    logic                w_shift;
    logic                w_load;
    logic [N*DATA_W-1:0] x_in;
    logic                x_valid;
    logic [N*ACC_W-1:0]  y_out;
    logic                y_valid;
    logic                busy;

    clocking drv_cb @(posedge clk);
        default input #1step output #1;
        output w_in, w_shift, w_load, x_in, x_valid;
        input  busy;
    endclocking

    clocking mon_cb @(posedge clk);
        default input #1step;
        input rst, w_in, w_shift, w_load, x_in, x_valid, y_out, y_valid, busy;
    endclocking
endinterface

//------------------------------------------------------------------------------
// Class-based constrained-random verification environment for systolic4x4
//
//   generator -> driver -> DUT -> out_monitor -> scoreboard
//                           \--> in_monitor  --/
//
// Time agnostic: no component assumes the DUT latency. The input monitor
// reconstructs the weight matrix from the shift/load pins and forwards every
// accepted vector; the scoreboard queues expected results in order and
// matches them against whatever the output monitor sees, whenever it sees it.
// The driver uses the DUT's busy flag (not a cycle count) to know when it is
// safe to w_load new weights.
//------------------------------------------------------------------------------
package sa_pkg;

    parameter int N      = 4;
    parameter int DATA_W = 8;
    parameter int ACC_W  = 32;

    typedef bit signed [DATA_W-1:0] data_t;
    typedef bit signed [ACC_W-1:0]  acc_t;
    typedef acc_t                   yvec_t [N];
    typedef virtual sa_if #(N, DATA_W, ACC_W) vif_t;

    localparam data_t DMIN = data_t'(1 << (DATA_W-1));   // -128
    localparam data_t DMAX = ~DMIN;                      //  127

    //--------------------------------------------------------------------------
    // Reporting: every check goes through tb_error so output stays one line
    // per problem, and floods are capped at MAX_ERR_PRINT.
    //--------------------------------------------------------------------------
    localparam int MAX_ERR_PRINT = 10;
    int n_errors;

    function automatic void tb_error(string tag, string msg);
        n_errors++;
        if (n_errors <= MAX_ERR_PRINT)
            $display("%t  ERROR %-5s %s", $realtime, tag, msg);
        else if (n_errors == MAX_ERR_PRINT + 1)
            $display("%t  ... further errors suppressed, see summary", $realtime);
    endfunction

    // fixed-width "[a b c d]" so x/got/exp rows line up
    function automatic string fmt_vec(int v [N]);
        string s = "[";
        foreach (v[i]) s = {s, $sformatf("%7d", v[i])};
        return {s, " ]"};
    endfunction

    //--------------------------------------------------------------------------
    // Transactions
    //--------------------------------------------------------------------------
    class sa_item;
        virtual function string convert2str();
            return "sa_item";
        endfunction
    endclass

    typedef enum {W_RANDOM, W_IDENTITY, W_ZERO, W_EXTREME} w_kind_e;

    // Weight matrix load
    class w_item extends sa_item;
        rand w_kind_e     kind;
        rand data_t       w [N][N];
        rand int unsigned shift_gap [N];   // idle cycles before each shift beat

        constraint c_kind { kind dist {W_RANDOM := 70, W_IDENTITY := 10,
                                       W_ZERO := 5, W_EXTREME := 15}; }

        constraint c_w_kind {
            foreach (w[r, c]) {
                // (a ternary on r==c here crashes xelab 2026.1, hence the if/else)
                if (r == c) (kind == W_IDENTITY) -> (w[r][c] == 1);
                else        (kind == W_IDENTITY) -> (w[r][c] == 0);
                (kind == W_ZERO)     -> (w[r][c] == 0);
                (kind == W_EXTREME)  -> (w[r][c] inside {DMIN, DMAX});
            }
        }

        // value distribution for W_RANDOM (the kind constraints above override it)
        constraint c_w_dist {
            foreach (w[r, c]) w[r][c] dist {DMIN := 1, DMAX := 1, 0 := 1,
                                            [DMIN+1 : -1] :/ 10, [1 : DMAX-1] :/ 10};
        }

        constraint c_gap { foreach (shift_gap[i]) shift_gap[i] dist {0 := 80, [1:3] :/ 20}; }

        virtual function string convert2str();
            string s = $sformatf("W(%s)", kind.name());
            foreach (w[r]) s = {s, $sformatf(" [%0d %0d %0d %0d]", w[r][0], w[r][1], w[r][2], w[r][3])};
            return s;
        endfunction
    endclass

    // One activation vector
    class x_item extends sa_item;
        rand data_t       x [N];
        rand int unsigned gap;             // idle cycles before this vector

        constraint c_x {
            foreach (x[i]) x[i] dist {DMIN := 2, DMAX := 2, 0 := 2,
                                      [DMIN+1 : -1] :/ 10, [1 : DMAX-1] :/ 10};
        }

        constraint c_gap { gap dist {0 := 60, [1:3] :/ 30, [4:12] :/ 10}; }

        virtual function string convert2str();
            return $sformatf("X [%0d %0d %0d %0d]", x[0], x[1], x[2], x[3]);
        endfunction
    endclass

    // One result vector
    class y_item extends sa_item;
        yvec_t y;

        virtual function string convert2str();
            return $sformatf("Y [%0d %0d %0d %0d]", y[0], y[1], y[2], y[3]);
        endfunction
    endclass

    //--------------------------------------------------------------------------
    // Generator: num_jobs x (one weight load + a random burst of vectors).
    // A null item marks the end of the stream.
    //--------------------------------------------------------------------------
    class generator;
        mailbox #(sa_item) gen2drv;
        int                num_jobs;

        function new(mailbox #(sa_item) gen2drv, int num_jobs);
            this.gen2drv  = gen2drv;
            this.num_jobs = num_jobs;
        endfunction

        task run();
            w_item       w;
            x_item       x;
            int unsigned n_vec;

            for (int j = 0; j < num_jobs; j++) begin
                w = new();
                if (!w.randomize()) $fatal(1, "[GEN] w_item randomize failed");
                gen2drv.put(w);

                if (!std::randomize(n_vec) with { n_vec dist {[1:4] :/ 30, [5:32] :/ 60, [33:64] :/ 10}; })
                    $fatal(1, "[GEN] n_vec randomize failed");
                repeat (n_vec) begin
                    x = new();
                    if (!x.randomize()) $fatal(1, "[GEN] x_item randomize failed");
                    gen2drv.put(x);
                end
            end
            gen2drv.put(null);
        endtask
    endclass

    //--------------------------------------------------------------------------
    // Driver
    //--------------------------------------------------------------------------
    class driver;
        vif_t              vif;
        mailbox #(sa_item) gen2drv;

        function new(vif_t vif, mailbox #(sa_item) gen2drv);
            this.vif     = vif;
            this.gen2drv = gen2drv;
        endfunction

        // Drive one cycle's worth of inputs. Data buses get random garbage
        // whenever their qualifier is low, so the DUT must ignore them.
        task drive(bit shift, bit load, bit xv,
                   logic [N*DATA_W-1:0] wv = '0, logic [N*DATA_W-1:0] xvec = '0);
            vif.drv_cb.w_shift <= shift;
            vif.drv_cb.w_load  <= load;
            vif.drv_cb.x_valid <= xv;
            vif.drv_cb.w_in    <= shift ? wv   : rand_bus();
            vif.drv_cb.x_in    <= xv    ? xvec : rand_bus();
        endtask

        function logic [N*DATA_W-1:0] rand_bus();
            logic [N*DATA_W-1:0] v;
            for (int i = 0; i < N*DATA_W; i += 32) v[i +: 32] = $urandom();
            return v;
        endfunction

        task idle(int unsigned n);
            repeat (n) begin
                @(vif.drv_cb);
                drive(0, 0, 0);
            end
        endtask

        // Shift rows N-1 .. 0 down the columns (may overlap in-flight compute),
        // then wait for the array to drain before latching.
        task drive_w(w_item it);
            logic [N*DATA_W-1:0] wv;
            for (int r = N-1; r >= 0; r--) begin
                idle(it.shift_gap[r]);
                for (int c = 0; c < N; c++) wv[c*DATA_W +: DATA_W] = it.w[r][c];
                @(vif.drv_cb);
                drive(1, 0, 0, wv);
            end
            forever begin
                @(vif.drv_cb);
                if (!vif.drv_cb.busy) begin
                    drive(0, 1, 0);
                    break;
                end
                drive(0, 0, 0);
            end
        endtask

        task drive_x(x_item it);
            logic [N*DATA_W-1:0] xvec;
            idle(it.gap);
            foreach (it.x[i]) xvec[i*DATA_W +: DATA_W] = it.x[i];
            @(vif.drv_cb);
            drive(0, 0, 1, '0, xvec);
        endtask

        task run();
            sa_item it;
            w_item  w;
            x_item  x;
            forever begin
                gen2drv.get(it);
                if (it == null) break;
                if ($cast(w, it)) begin
                    drive_w(w);
                end else if ($cast(x, it)) begin
                    drive_x(x);
                end else begin
                    $fatal(1, "[DRV] unknown item type");
                end
            end
            idle(1);
        endtask
    endclass

    //--------------------------------------------------------------------------
    // Input monitor: observes pins only. Mirrors the shadow shift chain to
    // recover the weights captured by each w_load, and forwards every
    // accepted vector. Also flags w_load while busy (protocol violation).
    //--------------------------------------------------------------------------
    class in_monitor;
        vif_t              vif;
        mailbox #(sa_item) mon2scb;

        function new(vif_t vif, mailbox #(sa_item) mon2scb);
            this.vif     = vif;
            this.mon2scb = mon2scb;
        endfunction

        task run();
            data_t shadow [N][N];
            w_item w;
            x_item x;

            foreach (shadow[r, c]) shadow[r][c] = 0;
            forever begin
                @(vif.mon_cb);
                if (vif.mon_cb.rst) begin
                    foreach (shadow[r, c]) shadow[r][c] = 0;
                    continue;
                end

                // w_load latches the shadows as they were before this edge's shift
                if (vif.mon_cb.w_load) begin
                    if (vif.mon_cb.busy)
                        tb_error("[IMON]", "w_load asserted while busy");
                    w = new();
                    w.w = shadow;
                    mon2scb.put(w);
                end

                if (vif.mon_cb.w_shift) begin
                    for (int r = N-1; r > 0; r--) shadow[r] = shadow[r-1];
                    for (int c = 0; c < N; c++) shadow[0][c] = vif.mon_cb.w_in[c*DATA_W +: DATA_W];
                end

                if (vif.mon_cb.x_valid) begin
                    x = new();
                    foreach (x.x[i]) x.x[i] = vif.mon_cb.x_in[i*DATA_W +: DATA_W];
                    mon2scb.put(x);
                end
            end
        endtask
    endclass

    //--------------------------------------------------------------------------
    // Output monitor
    //--------------------------------------------------------------------------
    class out_monitor;
        vif_t              vif;
        mailbox #(y_item) mon2scb;

        function new(vif_t vif, mailbox #(y_item) mon2scb);
            this.vif     = vif;
            this.mon2scb = mon2scb;
        endfunction

        task run();
            y_item y;
            forever begin
                @(vif.mon_cb);
                if (!vif.mon_cb.rst && vif.mon_cb.y_valid) begin
                    if ($isunknown(vif.mon_cb.y_out))
                        tb_error("[OMON]", "X/Z on y_out while y_valid");
                    y = new();
                    foreach (y.y[c]) y.y[c] = vif.mon_cb.y_out[c*ACC_W +: ACC_W];
                    mon2scb.put(y);
                end
            end
        endtask
    endclass

    //--------------------------------------------------------------------------
    // Scoreboard with reference model. Expected results are matched to
    // outputs purely by order.
    //--------------------------------------------------------------------------
    class scoreboard;
        mailbox #(sa_item) in_mbx;
        mailbox #(y_item)  out_mbx;

        data_t  wcur [N][N];
        yvec_t  exp_q [$];
        x_item  xin_q [$];
        int     n_loads, n_pass, n_fail, n_unexpected;

        function new(mailbox #(sa_item) in_mbx, mailbox #(y_item) out_mbx);
            this.in_mbx  = in_mbx;
            this.out_mbx = out_mbx;
            foreach (wcur[r, c]) wcur[r][c] = 0;
        endfunction

        function yvec_t model(x_item x);
            yvec_t e;
            foreach (e[c]) begin
                e[c] = 0;
                foreach (x.x[r]) e[c] += acc_t'(x.x[r]) * acc_t'(wcur[r][c]);
            end
            return e;
        endfunction

        task run();
            fork
                forever begin : in_thread
                    sa_item it;
                    w_item  w;
                    x_item  x;
                    in_mbx.get(it);
                    if ($cast(w, it)) begin
                        wcur = w.w;
                        n_loads++;
                    end else if ($cast(x, it)) begin
                        exp_q.push_back(model(x));
                        xin_q.push_back(x);
                    end
                end
                forever begin : out_thread
                    y_item y;
                    yvec_t e;
                    x_item x;
                    out_mbx.get(y);
                    if (exp_q.size() == 0) begin
                        n_unexpected++;
                        tb_error("[SCB]", {"unexpected output ", fmt_vec(y.y)});
                        continue;
                    end
                    e = exp_q.pop_front();
                    x = xin_q.pop_front();
                    if (y.y == e) begin
                        n_pass++;
                    end else begin
                        int xv [N];
                        foreach (xv[i]) xv[i] = x.x[i];
                        n_fail++;
                        tb_error("[SCB]", $sformatf({"vector %0d mismatch\n%23s x   = %s",
                                                     "\n%23s got = %s\n%23s exp = %s"},
                                                    n_pass + n_fail, "", fmt_vec(xv),
                                                    "", fmt_vec(y.y), "", fmt_vec(e)));
                    end
                end
            join_none
        endtask

        function int pending();
            return exp_q.size();
        endfunction

        function bit report();
            int n_other = n_errors - n_fail - n_unexpected;
            bit ok = (n_errors == 0) && (exp_q.size() == 0) && (n_pass > 0);
            $display("-------------------------------------------------------------");
            $display(" %s  %0d/%0d vectors correct, %0d weight loads, %0.1f us sim time",
                     ok ? "PASS" : "FAIL", n_pass, n_pass + n_fail, n_loads, $realtime / 1000.0);
            if (!ok)
                $display("       mismatch %0d | unexpected %0d | missing %0d | other %0d",
                         n_fail, n_unexpected, exp_q.size(), n_other);
            $display("-------------------------------------------------------------");
            return ok;
        endfunction
    endclass

    //--------------------------------------------------------------------------
    // Environment
    //--------------------------------------------------------------------------
    class env;
        vif_t              vif;
        generator          gen;
        driver             drv;
        in_monitor         imon;
        out_monitor        omon;
        scoreboard         scb;
        mailbox #(sa_item) gen2drv, imon2scb;
        mailbox #(y_item)  omon2scb;

        function new(vif_t vif, int num_jobs);
            this.vif = vif;
            gen2drv  = new();
            imon2scb = new();
            omon2scb = new();
            gen  = new(gen2drv, num_jobs);
            drv  = new(vif, gen2drv);
            imon = new(vif, imon2scb);
            omon = new(vif, omon2scb);
            scb  = new(imon2scb, omon2scb);
        endfunction

        task run(output bit ok);
            fork
                imon.run();
                omon.run();
                scb.run();
            join_none

            fork
                gen.run();
                drv.run();
            join

            // drain: wait until the DUT is idle and every expected result is matched
            do @(vif.mon_cb); while (vif.mon_cb.busy || scb.pending() != 0);
            repeat (2 * N) @(vif.mon_cb);   // catch any spurious trailing outputs
            ok = scb.report();
        endtask
    endclass

endpackage

//------------------------------------------------------------------------------
// Top-level testbench for systolic4x4
//   plusargs: +NUM_JOBS=<n>   number of weight loads (each followed by a
//                             random burst of vectors), default 20
//   seed:     xsim -sv_seed <n> (or -sv_seed random)
//------------------------------------------------------------------------------
module systolic4x4_tb;
    import sa_pkg::*;

    logic clk = 1'b0;
    always #5 clk = ~clk;

    sa_if #(.N(N), .DATA_W(DATA_W), .ACC_W(ACC_W)) vif (clk);

    systolic4x4 #(.N(N), .DATA_W(DATA_W), .ACC_W(ACC_W)) dut (
        .clk    (clk),
        .rst    (vif.rst),
        .w_in   (vif.w_in),
        .w_shift(vif.w_shift),
        .w_load (vif.w_load),
        .x_in   (vif.x_in),
        .x_valid(vif.x_valid),
        .y_out  (vif.y_out),
        .y_valid(vif.y_valid),
        .busy   (vif.busy)
    );

    initial begin
        automatic env e;
        automatic int num_jobs = 20;
        automatic bit ok;

        if ($value$plusargs("NUM_JOBS=%d", num_jobs)) ;
        $timeformat(-9, 1, " ns", 10);
        $display("systolic4x4_tb: %0dx%0d array, int%0d x int%0d -> int%0d, %0d weight sets",
                 N, N, DATA_W, DATA_W, ACC_W, num_jobs);

        vif.rst     = 1'b1;
        vif.w_in    = '0;
        vif.w_shift = 1'b0;
        vif.w_load  = 1'b0;
        vif.x_in    = '0;
        vif.x_valid = 1'b0;
        repeat (5) @(posedge clk);
        vif.rst <= 1'b0;
        @(posedge clk);

        e = new(vif, num_jobs);
        e.run(ok);
        $finish;
    end

    initial begin
        #50ms;
        $fatal(1, "systolic4x4_tb: TIMEOUT (DUT hung or stopped producing outputs)");
    end
endmodule
