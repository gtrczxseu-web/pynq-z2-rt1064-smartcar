`timescale 1ns/1ps
module tb_math;
    reg clk=0;always #5 clk=~clk;
    reg resetn=0,start_div=0,start_sqrt=0;
    reg [63:0] a,b;
    wire div_busy,div_done,sqrt_busy,sqrt_done;
    wire [63:0] q;wire [31:0] root;
    udiv64 divider(clk,resetn,start_div,a,b,div_busy,div_done,q);
    isqrt64 sqrt_unit(clk,resetn,start_sqrt,a,sqrt_busy,sqrt_done,root);
    task check(input [63:0] x,input [63:0] d,input [63:0] eq,input [31:0] er);
      begin
        @(negedge clk);a=x;b=d;start_div=1;start_sqrt=1;
        @(negedge clk);start_div=0;start_sqrt=0;
        wait(sqrt_done);@(negedge clk);if(root!==er)$fatal(1,"sqrt %h got %h expected %h",x,root,er);
        wait(div_done);@(negedge clk);if(q!==eq)$fatal(1,"divide %h / %h got %h expected %h",x,d,q,eq);
      end
    endtask
    initial begin
      repeat(3)@(negedge clk);resetn=1;
      check(0,1,0,0);check(1,1,1,1);check(2,3,0,1);check(15,2,7,3);
      check(16,3,5,4);check(64'hffffffffffffffff,1,64'hffffffffffffffff,32'hffffffff);
      check(64'hffffffffffffffff,64'hffffffffffffffff,1,32'hffffffff);
      check(64'h100000000,65536,65536,65536);
      $display("MATH_PASS 8 boundary cases");$finish;
    end
    initial begin #100000;$fatal(1,"math timeout");end
endmodule
