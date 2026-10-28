`timescale 1ns / 1ps
//------------------------------------------------------------------------------
// 4x4 weight-stationary systolic array
//
//   Computes y = x * W for a stream of row vectors x (1 x N) against a
//   stationary weight matrix W (N x N):
//       y[c] = sum_r x[r] * W[r][c]
//
//   Dataflow
//     - PE(r,c) holds W[r][c].
//     - Activations x[r] enter row r on the left and move right.
//     - Partial sums enter each column at the top (0) and move down.
//     - Input skew (row r delayed r cycles) and output de-skew (column c
//       delayed N-1-c cycles) are done internally, so x_in / y_out are
//       aligned vectors.
//
//   Weight loading (shift down columns, then latch)
//     - Each PE has a shadow weight register; the shadows of a column form
//       a shift chain fed from w_in[c] at the top.
//     - Each cycle w_shift=1 shifts the chain down one row. After N shifts,
//       row r holds the value shifted in N-1-r shifts ago, i.e. feed
//       W[N-1][*] first and W[0][*] last.
//     - A one-cycle w_load pulse copies all shadows into the active weights.
//     - Shifting may overlap with compute; w_load must only be asserted when
//       busy==0 (otherwise in-flight vectors see a mix of old/new weights).
//
//   Data format: signed two's complement. Vectors are packed with element k
//   at [k*W +: W].
//
//   Latency: y for an input accepted at cycle T is valid at cycle T + 2N.
//------------------------------------------------------------------------------
module systolic4x4 #(
    parameter N      = 4,
    parameter DATA_W = 8,
    parameter ACC_W  = 32
) (
    input  wire                clk,
    input  wire                rst,       // synchronous, active high

    // weight load
    input  wire [N*DATA_W-1:0] w_in,      // w_in[c]: next weight for column c
    input  wire                w_shift,
    input  wire                w_load,

    // activation stream
    input  wire [N*DATA_W-1:0] x_in,
    input  wire                x_valid,

    // result stream
    output wire [N*ACC_W-1:0]  y_out,
    output wire                y_valid,

    output wire                busy       // a valid vector is entering or in flight
);

    localparam LAT = 2*N;

    // a[r][c] : activation into PE(r,c)  (a[r][N] is the unused right edge)
    // p[r][c] : partial sum into PE(r,c) (p[N][c] is the bottom output)
    // w[r][c] : shadow weight into PE(r,c) from above
    wire signed [DATA_W-1:0] a [0:N-1][0:N];
    wire signed [ACC_W-1:0]  p [0:N][0:N-1];
    wire signed [DATA_W-1:0] w [0:N][0:N-1];

    genvar r, c;
    generate
        // input skew: row r delayed by r cycles
        for (r = 0; r < N; r = r + 1) begin : g_skew
            delay_line #(.WIDTH(DATA_W), .DEPTH(r)) u_skew (
                .clk(clk), .rst(rst),
                .d(x_in[r*DATA_W +: DATA_W]),
                .q(a[r][0])
            );
        end

        // top edge: zero partial sums, weight shift inputs
        for (c = 0; c < N; c = c + 1) begin : g_top
            assign p[0][c] = {ACC_W{1'b0}};
            assign w[0][c] = w_in[c*DATA_W +: DATA_W];
        end

        // PE grid
        for (r = 0; r < N; r = r + 1) begin : g_row
            for (c = 0; c < N; c = c + 1) begin : g_col
                pe #(.DATA_W(DATA_W), .ACC_W(ACC_W)) u_pe (
                    .clk    (clk),
                    .rst    (rst),
                    .w_in   (w[r][c]),
                    .w_shift(w_shift),
                    .w_load (w_load),
                    .w_out  (w[r+1][c]),
                    .a_in   (a[r][c]),
                    .a_out  (a[r][c+1]),
                    .p_in   (p[r][c]),
                    .p_out  (p[r+1][c])
                );
            end
        end

        // output de-skew: column c delayed by N-1-c cycles
        for (c = 0; c < N; c = c + 1) begin : g_deskew
            delay_line #(.WIDTH(ACC_W), .DEPTH(N-1-c)) u_deskew (
                .clk(clk), .rst(rst),
                .d(p[N][c]),
                .q(y_out[c*ACC_W +: ACC_W])
            );
        end
    endgenerate

    // valid pipeline, matched to the array latency
    reg [LAT-1:0] vpipe;
    always @(posedge clk) begin
        if (rst) vpipe <= {LAT{1'b0}};
        else     vpipe <= {vpipe[LAT-2:0], x_valid};
    end

    assign y_valid = vpipe[LAT-1];
    assign busy    = x_valid | (|vpipe);

endmodule


//------------------------------------------------------------------------------
// Processing element
//   a_reg <= a_in                   (register the activation on entry)
//   p_out <= p_in + a_reg * weight  (signed MAC)
//   a_out  = a_reg                  (pass activation right)
//   w_out <= w_in when w_shift      (shadow weight shift chain)
//   weight <= w_out when w_load     (latch shadow into active weight)
//   use_dsp maps the MAC (multiply + accumulate + p_out register) to a DSP58.
//   a_reg sits on the receiving side of the PE boundary so it can map to the
//   DSP58 A register; the PE-to-PE path is then a wire into a flop, which
//   keeps timing when PEs are pre-implemented with DONT_TOUCH boundaries.
//------------------------------------------------------------------------------
(* use_dsp = "yes" *)
module pe #(
    parameter DATA_W = 8,
    parameter ACC_W  = 32
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire signed [DATA_W-1:0] w_in,
    input  wire                     w_shift,
    input  wire                     w_load,
    output reg  signed [DATA_W-1:0] w_out,
    input  wire signed [DATA_W-1:0] a_in,
    output wire signed [DATA_W-1:0] a_out,
    input  wire signed [ACC_W-1:0]  p_in,
    output reg  signed [ACC_W-1:0]  p_out
);

    reg signed [DATA_W-1:0] weight;
    reg signed [DATA_W-1:0] a_reg;

    assign a_out = a_reg;

    always @(posedge clk) begin
        if (rst) begin
            w_out  <= {DATA_W{1'b0}};
            weight <= {DATA_W{1'b0}};
            a_reg  <= {DATA_W{1'b0}};
            p_out  <= {ACC_W{1'b0}};
        end else begin
            if (w_shift) w_out  <= w_in;
            if (w_load)  weight <= w_out;
            a_reg <= a_in;
            p_out <= p_in + a_reg * weight;
        end
    end

endmodule


//------------------------------------------------------------------------------
// Fixed-depth register delay line (DEPTH = 0 is a wire)
//------------------------------------------------------------------------------
module delay_line #(
    parameter WIDTH = 8,
    parameter DEPTH = 1
) (
    input  wire             clk,
    input  wire             rst,
    input  wire [WIDTH-1:0] d,
    output wire [WIDTH-1:0] q
);

    generate
        if (DEPTH == 0) begin : g_wire
            assign q = d;
        end else begin : g_sr
            reg [WIDTH-1:0] sr [0:DEPTH-1];
            integer k;
            always @(posedge clk) begin
                if (rst) begin
                    for (k = 0; k < DEPTH; k = k + 1) sr[k] <= {WIDTH{1'b0}};
                end else begin
                    sr[0] <= d;
                    for (k = 1; k < DEPTH; k = k + 1) sr[k] <= sr[k-1];
                end
            end
            assign q = sr[DEPTH-1];
        end
    endgenerate

endmodule
