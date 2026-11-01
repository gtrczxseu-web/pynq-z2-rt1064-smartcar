`timescale 1ns/1ps
// Unsigned restoring divider: fixed 64-cycle computation, no combinational '/'.
module udiv64(input wire clk, input wire resetn, input wire start,
    input wire [63:0] dividend, divisor,
    output reg busy, output reg done, output reg [63:0] quotient);
    reg [63:0] q, d;
    reg [64:0] remainder;
    reg [6:0] remaining;
    wire [64:0] shifted = {remainder[63:0], q[63]};
    wire take = shifted >= {1'b0,d};
    wire [63:0] next_q = {q[62:0],take};
    always @(posedge clk) begin
        done <= 0;
        if (!resetn) begin
            busy<=0; done<=0; quotient<=0; q<=0; d<=0; remainder<=0; remaining<=0;
        end else if (start && !busy) begin
            busy<=1; q<=dividend; d<=divisor; remainder<=0; remaining<=64;
        end else if (busy) begin
            remainder <= take ? shifted-{1'b0,d} : shifted;
            q<=next_q; remaining<=remaining-1'b1;
            if (remaining==1) begin busy<=0; done<=1; quotient<=next_q; end
        end
    end
endmodule
