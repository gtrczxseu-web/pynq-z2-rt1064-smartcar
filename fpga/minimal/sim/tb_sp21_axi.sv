`timescale 1ns/1ps
module tb_sp21_axi;
    reg clk=0; always #5 clk=~clk;
    reg resetn=0;
    reg [7:0] awaddr=0,araddr=0;
    reg awvalid=0,wvalid=0,bready=0,arvalid=0,rready=0;
    reg [31:0] wdata=0; reg [3:0] wstrb=0;
    wire awready,wready,bvalid,arready,rvalid;
    wire [31:0] rdata;wire [1:0] bresp,rresp;
    sp21_axi dut(.aclk(clk),.aresetn(resetn),.s_axi_awaddr(awaddr),.s_axi_awprot(3'b0),
      .s_axi_awvalid(awvalid),.s_axi_awready(awready),.s_axi_wdata(wdata),.s_axi_wstrb(wstrb),
      .s_axi_wvalid(wvalid),.s_axi_wready(wready),.s_axi_bresp(bresp),.s_axi_bvalid(bvalid),.s_axi_bready(bready),
      .s_axi_araddr(araddr),.s_axi_arprot(3'b0),.s_axi_arvalid(arvalid),.s_axi_arready(arready),
      .s_axi_rdata(rdata),.s_axi_rresp(rresp),.s_axi_rvalid(rvalid),.s_axi_rready(rready));
    `include "vectors_count.vh"
    reg [31:0] vectors[0:VECTOR_COUNT*16-1];
    integer i,j,polls,checks=0;
    reg [31:0] value,held;
    task write_word(input [7:0] addr,input [31:0] data,input [3:0] strobes,input integer mode);
      begin
        fork
          begin
            repeat(mode==0 ? 1:4) @(negedge clk);
            awaddr=addr;awvalid=1;
            do @(posedge clk);while(!awready);
            @(negedge clk);awvalid=0;
          end
          begin
            repeat(mode==0 ? 4:1) @(negedge clk);
            wdata=data;wstrb=strobes;wvalid=1;
            do @(posedge clk);while(!wready);
            @(negedge clk);wvalid=0;
          end
        join
        while(!bvalid) @(negedge clk);
        repeat(3) begin @(negedge clk);if(!bvalid||bresp!==0)$fatal(1,"B backpressure failed");end
        bready=1;@(posedge clk);@(negedge clk);bready=0;
      end
    endtask
    task read_word(input [7:0] addr,output [31:0] data);
      begin
        @(negedge clk);araddr=addr;arvalid=1;
        do @(posedge clk);while(!arready);
        @(negedge clk);arvalid=0;
        while(!rvalid) @(negedge clk);
        data=rdata;held=rdata;
        repeat(3) begin @(negedge clk);if(!rvalid||rdata!==held||rresp!==0)$fatal(1,"R backpressure failed");end
        rready=1;@(posedge clk);@(negedge clk);rready=0;
      end
    endtask
    task expect_word(input [7:0] addr,input [31:0] expected);
      begin read_word(addr,value);checks=checks+1;
        if(value!==expected)$fatal(1,"address=%h actual=%h expected=%h vector=%0d",addr,value,expected,i);
      end
    endtask
    task wait_idle;
      begin
        polls=0;value=1;
        while(value[0]) begin
          read_word(8'h64,value);polls=polls+1;
          if(polls>2000)$fatal(1,"Processor did not finish");
        end
      end
    endtask
    initial begin
      $readmemh("vectors.mem",vectors);
      repeat(5)@(negedge clk);resetn=1;
      expect_word(8'h00,32'h53504d31);expect_word(8'h34,32'h53503231);
      write_word(8'h40,32'h11223344,4'hf,0);
      write_word(8'h40,32'haaffbbcc,4'h5,1);expect_word(8'h40,32'h11ff33cc);
      write_word(8'h40,32'hffffffff,4'h0,0);expect_word(8'h40,32'h11ff33cc);
      // Read immediately after command acceptance, before its FSM capture.
      // Pending work must already be reported busy even with BREADY low.
      @(negedge clk);awaddr=8'h60;wdata=1;wstrb=4'hf;awvalid=1;wvalid=1;
      @(posedge clk);@(negedge clk);awvalid=0;wvalid=0;
      while(!bvalid)@(negedge clk);
      araddr=8'h64;arvalid=1;
      @(posedge clk);@(negedge clk);arvalid=0;
      if(!rvalid||!rdata[0])$fatal(1,"Accepted pending command was not visible as busy");
      bready=1;rready=1;@(posedge clk);@(negedge clk);bready=0;rready=0;
      wait_idle;write_word(8'h60,2,4'hf,0);
      for(i=0;i<VECTOR_COUNT;i=i+1) begin
        for(j=0;j<8;j=j+1)write_word(8'h40+4*j,vectors[i*16+j],4'hf,(i+j)%2);
        write_word(8'h60,1,4'hf,i%2);wait_idle;
        expect_word(8'h68,vectors[i*16+8]);expect_word(8'h6c,vectors[i*16+9]);
        expect_word(8'h80,vectors[i*16+10]);expect_word(8'h84,vectors[i*16+11]);
        expect_word(8'h70,vectors[i*16+12]);expect_word(8'h74,vectors[i*16+13]);
        expect_word(8'h78,vectors[i*16+14]);expect_word(8'h7c,vectors[i*16+15]);
      end
      write_word(8'h60,2,4'hf,0);expect_word(8'h68,0);expect_word(8'h6c,0);
      expect_word(8'h74,0);expect_word(8'h78,0);expect_word(8'h7c,0);
      // Prime after clear, then issue another command while the first is busy.
      for(j=0;j<8;j=j+1)write_word(8'h40+4*j,0,4'hf,j%2);
      write_word(8'h60,1,4'hf,0);wait_idle;
      for(j=0;j<4;j=j+1)write_word(8'h40+4*j,2430,4'hf,j%2);
      write_word(8'h54,5,4'hf,0);write_word(8'h58,20,4'hf,1);
      write_word(8'h60,1,4'hf,0);write_word(8'h60,1,4'hf,1);
      write_word(8'h40,999999,4'hf,0);write_word(8'h5c,7,4'hf,1);wait_idle;
      expect_word(8'h68,1);expect_word(8'h6c,0);expect_word(8'h74,200000);
      expect_word(8'h80,0);expect_word(8'h84,200000);expect_word(8'h7c,0);
      $display("SP21_AXI_PASS vectors=%0d register_checks=%0d AW/W independent, strobes, backpressure, clear, busy",VECTOR_COUNT,checks);
      $finish;
    end
    initial begin #10000000;$fatal(1,"Simulation timeout");end
endmodule
