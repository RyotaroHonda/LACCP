`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/18/2025 01:06:41 AM
// Design Name: 
// Module Name: CalcFineLantency_US
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module CalcFineLantency_US(
        CLK,
        CNTVALUEOUTInit,
        CNTVALUEOUT_slaveInit,
        idelay_tap,
        serdes_latency,
        result,
        is_ultrascale    
    );
    
    
    real revision_time = 1.024;      
    
    parameter kCNTVALUEbit = 9;
    parameter kDELAY_VALUE = 1000.0;     //=1000
    parameter kAlignDelay = 54;
    parameter kFreqFastClk = 500;
    parameter kBitPut = 4;      //4bit
    parameter kWidthTap = 5;
    parameter kWidthSerdesOffset = 4;
    parameter kFastClkFreq = 500;
    parameter kWidthLaccpFineOffset = 16;
    parameter kdiv_tap_shift = 8;       //div_tap -> fractual is 8bit
        
    
    input CLK;
    input [kCNTVALUEbit-1:0] CNTVALUEOUTInit;
    input [kCNTVALUEbit-1:0] CNTVALUEOUT_slaveInit;
    input [kWidthTap-1:0] idelay_tap;
    input signed [kWidthSerdesOffset-1:0] serdes_latency;
    output signed [kWidthLaccpFineOffset-1:0] result;
    output is_ultrascale;      
        
//-------------------------------------------------------------
//idelaye3
//-------------------------------------------------------------  
    localparam DELAY_VALUE_width = 16; 
    wire [DELAY_VALUE_width-1:0] DELAY_VALUE_set;
    assign DELAY_VALUE_set = kDELAY_VALUE * revision_time; // = 1000 (ps) * 1.024
    
    localparam quotient_integer_width = 16;
    localparam quotient_fractional_width = 8;
    wire [quotient_integer_width+quotient_fractional_width-1:0] tap_master;
    wire [quotient_integer_width+quotient_fractional_width-1:0] tap_slave;  
    
    wire [quotient_integer_width+quotient_fractional_width-1:0]s_axis_dividend_tdata_master;
    wire [quotient_integer_width+quotient_fractional_width-1:0] s_axis_divisor_tdata_master;
    assign s_axis_dividend_tdata_master = DELAY_VALUE_set;
    assign s_axis_divisor_tdata_master = CNTVALUEOUTInit - kAlignDelay;
    
    udiv_q_cbt_axis #(
        .DW(DELAY_VALUE_width), .QI(quotient_integer_width), .QF(quotient_fractional_width)
    ) div_tap_master (
        .clk(CLK),
        .rst(1'b0),
        .s_axis_tvalid(1'b1),
        .s_axis_tready(),
        .s_axis_dividend(s_axis_dividend_tdata_master),
        .s_axis_divisor(s_axis_divisor_tdata_master),
        .m_axis_tvalid(),
        .m_axis_tready(1'b1),
        .m_axis_div_by_zero(),
        .m_axis_q_int(tap_master[quotient_integer_width+quotient_fractional_width-1:quotient_fractional_width]),
        .m_axis_q_frac(tap_master[quotient_fractional_width-1:0]),
        .m_axis_remainder()
    );   

    wire [quotient_integer_width+quotient_fractional_width-1:0] s_axis_dividend_tdata_slave;
    wire [quotient_integer_width+quotient_fractional_width-1:0] s_axis_divisor_tdata_slave;
    assign s_axis_dividend_tdata_slave = DELAY_VALUE_set;
    assign s_axis_divisor_tdata_slave = CNTVALUEOUT_slaveInit;

    udiv_q_cbt_axis #(
        .DW(DELAY_VALUE_width), .QI(quotient_integer_width), .QF(quotient_fractional_width)
    ) div_tap_slave (
        .clk(CLK),
        .rst(1'b0),
        .s_axis_tvalid(1'b1),
        .s_axis_tready(),
        .s_axis_dividend(s_axis_dividend_tdata_slave),
        .s_axis_divisor(s_axis_divisor_tdata_slave),
        .m_axis_tvalid(),
        .m_axis_tready(1'b1),
        .m_axis_div_by_zero(),
        .m_axis_q_int(tap_slave[quotient_integer_width+quotient_fractional_width-1:quotient_fractional_width]),
        .m_axis_q_frac(tap_slave[quotient_fractional_width-1:0]),
        .m_axis_remainder()
    );        
    
    wire [quotient_integer_width+quotient_fractional_width-1:0] tap_delay;
    assign tap_delay = tap_master + tap_slave;
    
    reg [quotient_integer_width+quotient_fractional_width-1:0] tap_delay_old;
    always@(posedge CLK)begin
        tap_delay_old <= (tap_delay << kBitPut);
    end     
    
    wire [kWidthLaccpFineOffset + kdiv_tap_shift-1:0] idelay_val_level1;
    wire [kWidthLaccpFineOffset + kdiv_tap_shift-1:0] idelay_val_level2;
    reg [kWidthLaccpFineOffset-1:0] idelay_val;
    
    assign idelay_val_level1 = (idelay_tap*tap_delay_old[quotient_integer_width-1:0]);
    assign idelay_val_level2 = idelay_val_level1 >> kdiv_tap_shift;  //Ignore sub-picosecond values
    always@(posedge CLK)begin
        idelay_val <= idelay_val_level2;
    end
//-------------------------------------------------------------
//iserdese3
//-------------------------------------------------------------
    wire [11:0] period;
    
    //period = 1 / freq * revision_time
    //       = 1000*1000*1000*1000 / ( kFastClkFreq*1000*1000 * 2)  [ps] * revision_time
    //       = 1000*500 / kFastClkFreq [ps] * revision_time
    assign period = 1000 * 500 / kFastClkFreq * revision_time;         
    wire signed [kWidthLaccpFineOffset-1:0] serdes_val_level1;
    reg signed [kWidthLaccpFineOffset-1:0] serdes_val;
    assign serdes_val_level1 = serdes_latency * $signed({1'b0, period});
    always@(posedge CLK)begin
        serdes_val <= serdes_val_level1;
    end   
 
//-------------------------------------------------------------
//result
//-------------------------------------------------------------    
    assign result = idelay_val + serdes_val;
    assign is_ultrascale = (CNTVALUEOUTInit == 0) ? 1'b0 : 1'b1;
    
endmodule
