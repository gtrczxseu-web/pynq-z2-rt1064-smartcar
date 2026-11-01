`timescale 1ns/1ps
// Floor(sqrt(radicand)), two input bits consumed per cycle.
module isqrt64(input wire clk, input wire resetn, input wire start,
    input wire [63:0] radicand,
    output reg busy, output reg done, output reg [31:0] root);
    reg [63:0] value;
    reg [65:0] remainder;
    reg [5:0] remaining;
    wire [65:0] shifted={remainder[63:0],value[63:62]};
    wire [65:0] trial={32'd0,root,2'b01};
    wire take=shifted>=trial;
    wire [31:0] next_root={root[30:0],take};
    always @(posedge clk) begin
        done<=0;
        if (!resetn) begin
            busy<=0; done<=0; root<=0; value<=0; remainder<=0; remaining<=0;
        end else if (start && !busy) begin
            busy<=1; value<=radicand; root<=0; remainder<=0; remaining<=32;
        end else if (busy) begin
            remainder<=take ? shifted-trial : shifted;
            value<=value<<2; root<=next_root; remaining<=remaining-1'b1;
            if (remaining==1) begin busy<=0; done<=1; end
        end
    end
endmodule
