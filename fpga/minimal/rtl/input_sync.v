`timescale 1ns/1ps
module input_sync(input wire clk, input wire resetn,
    input wire uart_rx_async, input wire [3:0] btn_async,
    output wire uart_rx_sync, output wire [3:0] btn_sync);
    (* ASYNC_REG="TRUE" *) reg [4:0] stage1, stage2;
    always @(posedge clk) begin
        if (!resetn) begin stage1<=5'b10000; stage2<=5'b10000; end
        else begin stage1<={uart_rx_async,btn_async}; stage2<=stage1; end
    end
    assign {uart_rx_sync,btn_sync}=stage2;
endmodule
