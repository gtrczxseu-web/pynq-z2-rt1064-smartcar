`timescale 1ns/1ps
// New minimal implementation of the existing SP21 sensor register contract.
// No video/HUD implementation is advertised by this peripheral.
module sp21_axi(
    (* X_INTERFACE_INFO="xilinx.com:signal:clock:1.0 aclk CLK", X_INTERFACE_PARAMETER="ASSOCIATED_BUSIF S_AXI, ASSOCIATED_RESET aresetn, FREQ_HZ 100000000" *)
    input wire aclk,
    (* X_INTERFACE_INFO="xilinx.com:signal:reset:1.0 aresetn RST", X_INTERFACE_PARAMETER="POLARITY ACTIVE_LOW" *)
    input wire aresetn,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI AWADDR", X_INTERFACE_PARAMETER="PROTOCOL AXI4LITE, DATA_WIDTH 32, ADDR_WIDTH 8, READ_WRITE_MODE READ_WRITE" *)
    input wire [7:0] s_axi_awaddr,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI AWPROT" *) input wire [2:0] s_axi_awprot,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *) input wire s_axi_awvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *) output wire s_axi_awready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI WDATA" *) input wire [31:0] s_axi_wdata,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *) input wire [3:0] s_axi_wstrb,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI WVALID" *) input wire s_axi_wvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI WREADY" *) output wire s_axi_wready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI BRESP" *) output wire [1:0] s_axi_bresp,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI BVALID" *) output reg s_axi_bvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI BREADY" *) input wire s_axi_bready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *) input wire [7:0] s_axi_araddr,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI ARPROT" *) input wire [2:0] s_axi_arprot,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *) input wire s_axi_arvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *) output wire s_axi_arready,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI RDATA" *) output reg [31:0] s_axi_rdata,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI RRESP" *) output wire [1:0] s_axi_rresp,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI RVALID" *) output reg s_axi_rvalid,
    (* X_INTERFACE_INFO="xilinx.com:interface:aximm:1.0 S_AXI RREADY" *) input wire s_axi_rready
);
    reg aw_pending,w_pending;
    reg [7:0] aw;
    reg [31:0] wd,stage[0:7];
    reg [3:0] ws;
    reg submit,clear;
    reg [4:0] state;
    wire busy=(state!=0)||submit||clear;
    reg [31:0] samples,rejected,speed_um_s;
    reg [63:0] distance_um;
    reg signed [31:0] heading_q16_cdeg,dx_um,dy_um;
    integer k;
    assign s_axi_awready=!aw_pending&&!s_axi_bvalid;
    assign s_axi_wready=!w_pending&&!s_axi_bvalid;
    assign s_axi_arready=!s_axi_rvalid;
    assign s_axi_bresp=2'b00;
    assign s_axi_rresp=2'b00;
    function [31:0] read_word;
        input [7:0] addr;
        begin
            case (addr[7:2])
                0:read_word=32'h53504d31; // SPM1: deliberately not HUD2
                13:read_word=32'h53503231;
                16,17,18,19,20,21,22,23:read_word=stage[addr[4:2]];
                25:read_word={31'd0,busy};
                26:read_word=samples;
                27:read_word=rejected;
                28:read_word=speed_um_s;
                29:read_word=distance_um[31:0];
                30:read_word=distance_um[63:32];
                31:read_word=heading_q16_cdeg;
                32:read_word=dx_um;
                33:read_word=dy_um;
                default:read_word=0;
            endcase
        end
    endfunction
    always @(posedge aclk) begin
        submit<=0; clear<=0;
        if (!aresetn) begin
            aw_pending<=0; w_pending<=0; aw<=0; wd<=0; ws<=0;
            s_axi_bvalid<=0; s_axi_rvalid<=0; s_axi_rdata<=0;
            for(k=0;k<8;k=k+1) stage[k]<=0;
        end else begin
            if(s_axi_awvalid&&s_axi_awready) begin aw<=s_axi_awaddr; aw_pending<=1; end
            if(s_axi_wvalid&&s_axi_wready) begin wd<=s_axi_wdata; ws<=s_axi_wstrb; w_pending<=1; end
            if(s_axi_bvalid&&s_axi_bready) s_axi_bvalid<=0;
            if(aw_pending&&w_pending&&!s_axi_bvalid) begin
                aw_pending<=0; w_pending<=0; s_axi_bvalid<=1;
                if(aw[7:2]>=16&&aw[7:2]<=23)
                    for(k=0;k<4;k=k+1) if(ws[k]) stage[aw[4:2]][k*8+:8]<=wd[k*8+:8];
                if(aw[7:2]==24&&ws[0]&&!busy) begin
                    clear<=wd[1]; submit<=wd[0]&&!wd[1];
                end
            end
            if(s_axi_rvalid&&s_axi_rready) s_axi_rvalid<=0;
            if(s_axi_arvalid&&s_axi_arready) begin
                s_axi_rvalid<=1; s_axi_rdata<=read_word(s_axi_araddr);
            end
        end
    end

    reg primed;
    reg [31:0] previous[0:6], current[0:7];
    wire signed [31:0] e0=current[0]-previous[0],e1=current[1]-previous[1];
    wire signed [31:0] e2=current[2]-previous[2],e3=current[3]-previous[3];
    wire signed [33:0] x4={{2{e0[31]}},e0}-{{2{e1[31]}},e1}-{{2{e2[31]}},e2}+{{2{e3[31]}},e3};
    wire signed [33:0] y4={{2{e0[31]}},e0}+{{2{e1[31]}},e1}+{{2{e2[31]}},e2}+{{2{e3[31]}},e3};
    wire [31:0] dt20=current[5]-previous[5],dt5=current[6]-previous[6];
    wire signed [31:0] dg=current[4]-previous[4];
    reg signed [33:0] x4_work,y4_work;
    reg signed [31:0] gyro_work;
    reg signed [15:0] bias_work;
    reg [31:0] ticks5_work,magnitude_x,magnitude_y;
    reg signed [63:0] scaled_x,scaled_y,bias_product,gyro_corrected;
    reg [63:0] square_x,square_y;
    reg [63:0] abs_x,abs_y;
    reg neg_x,neg_y;
    reg [31:0] interval20,path_um,speed_work;
    reg signed [31:0] dx_work,dy_work;
    reg signed [63:0] yaw_delta,yaw_work;
    reg [63:0] squared;
    wire [31:0] dx_abs=dx_work[31] ? -dx_work : dx_work;
    wire [31:0] dy_abs=dy_work[31] ? -dy_work : dy_work;
    reg div_start,sqrt_start;
    reg [63:0] dividend,divisor;
    wire div_busy,div_done,sqrt_busy,sqrt_done;
    wire [63:0] quotient;
    wire [31:0] square_root;
    udiv64 divide(aclk,aresetn,div_start,dividend,divisor,div_busy,div_done,quotient);
    isqrt64 square_root_unit(aclk,aresetn,sqrt_start,squared,sqrt_busy,sqrt_done,square_root);
    integer j;
    always @(posedge aclk) begin
        div_start<=0; sqrt_start<=0;
        if(!aresetn || (clear&&state==0)) begin
            state<=0; primed<=0; samples<=0; rejected<=0; speed_um_s<=0;
            distance_um<=0; heading_q16_cdeg<=0; dx_um<=0; dy_um<=0;
            div_start<=0; sqrt_start<=0; dividend<=0; divisor<=1;
            abs_x<=0; abs_y<=0; neg_x<=0; neg_y<=0; interval20<=0;
            path_um<=0; speed_work<=0; dx_work<=0; dy_work<=0;
            yaw_delta<=0; yaw_work<=0; squared<=0;
            x4_work<=0; y4_work<=0; gyro_work<=0; bias_work<=0; ticks5_work<=0;
            magnitude_x<=0; magnitude_y<=0; scaled_x<=0; scaled_y<=0;
            bias_product<=0; gyro_corrected<=0; square_x<=0; square_y<=0;
            for(j=0;j<7;j=j+1) previous[j]<=0;
            for(j=0;j<8;j=j+1) current[j]<=0;
        end else case(state)
            0:if(submit) begin
                for(j=0;j<8;j=j+1) current[j]<=stage[j];
                state<=1;
            end
            1:begin
                for(j=0;j<7;j=j+1) previous[j]<=current[j];
                primed<=1;
                if(!primed) state<=0;
                else if(dt20==0||dt20>50||dt5==0||dt5>200||
                        dt20*4>dt5+5||dt5>dt20*4+5) begin
                    rejected<=rejected+1'b1; state<=0;
                end else begin
                    x4_work<=x4; y4_work<=y4; gyro_work<=dg;
                    bias_work<=current[7][15:0]; ticks5_work<=dt5;
                    interval20<=dt20; state<=14;
                end
            end
            2:begin dividend<=abs_x; divisor<=10140; div_start<=1; state<=3; end
            3:if(div_done) begin dx_work<=neg_x ? -quotient[31:0] : quotient[31:0]; state<=4; end
            4:begin dividend<=abs_y; divisor<=9720; div_start<=1; state<=5; end
            5:if(div_done) begin dy_work<=neg_y ? -quotient[31:0] : quotient[31:0]; state<=6; end
            6:begin magnitude_x<=dx_abs; magnitude_y<=dy_abs; state<=17; end
            7:begin sqrt_start<=1; state<=8; end
            8:if(sqrt_done) begin path_um<=square_root; state<=9; end
            9:begin dividend<={32'd0,path_um}*64'd50; divisor<={32'd0,interval20}; div_start<=1; state<=10; end
            10:if(div_done) begin speed_work<=quotient[31:0]; state<=11; end
            11:begin yaw_work<=$signed(heading_q16_cdeg)+yaw_delta; state<=12; end
            12:if(yaw_work>=64'sd1179648000) yaw_work<=yaw_work-64'sd2359296000;
               else if(yaw_work< -64'sd1179648000) yaw_work<=yaw_work+64'sd2359296000;
               else state<=13;
            13:begin
                samples<=samples+1'b1; speed_um_s<=speed_work;
                distance_um<=distance_um+{32'd0,path_um};
                heading_q16_cdeg<=yaw_work[31:0]; dx_um<=dx_work; dy_um<=dy_work;
                state<=0;
            end
            14:begin
                scaled_x<=$signed(x4_work)*64'sd200000;
                scaled_y<=$signed(y4_work)*64'sd200000;
                bias_product<=$signed(bias_work)*$signed({1'b0,ticks5_work});
                state<=15;
            end
            15:begin
                abs_x<=scaled_x[63] ? -scaled_x : scaled_x;
                abs_y<=scaled_y[63] ? -scaled_y : scaled_y;
                neg_x<=scaled_x[63]; neg_y<=scaled_y[63];
                gyro_corrected<=$signed(gyro_work)-bias_product;
                state<=16;
            end
            16:begin yaw_delta<=gyro_corrected*64'sd2291; state<=2; end
            17:begin
                square_x<={32'd0,magnitude_x}*magnitude_x;
                square_y<={32'd0,magnitude_y}*magnitude_y;
                state<=18;
            end
            18:begin squared<=square_x+square_y; state<=7; end
            default:state<=0;
        endcase
    end
endmodule
