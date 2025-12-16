`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/06/2025 11:57:37 AM
// Design Name: 
// Module Name: CalcCalibDelay
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


module CalcCalibDelay(
        CLK,
        is_ultrascale_sec,
        is_ultrascale_pri,
        CalibDelay,
        calibdelay_rx,
        result
    );
    
    localparam TRUE  = 1'b1;
    localparam FALSE = 1'b0;    
    parameter enCalibDelay = TRUE;
    parameter kCalibDelayWidth = 12;
    parameter kWidthLaccpFineOffset = 16;
    parameter kFastClkFreq = 500;
    
    
    input CLK;
    input is_ultrascale_sec;
    input is_ultrascale_pri;    
    input signed [kCalibDelayWidth-1:0] CalibDelay;
    input signed [kCalibDelayWidth-1:0] calibdelay_rx;
    output signed [kWidthLaccpFineOffset-1:0] result;
    

//-------------------------
//caluculation analog    
//-------------------------

    wire signed [kCalibDelayWidth:0] diff;  // 12+1=13bit
    assign diff = CalibDelay - calibdelay_rx;

    wire signed [kWidthLaccpFineOffset-1:0] analog_result;
    assign analog_result = {{(kWidthLaccpFineOffset-(kCalibDelayWidth+1)){diff[kCalibDelayWidth]}}, diff};
    
    reg signed [kWidthLaccpFineOffset-1:0] analog_result_old;
    always@(posedge CLK)begin
        analog_result_old <= analog_result;
    end
    
//-------------------------
//caluculation digital    
//-------------------------  
  


    localparam integer period = (1000 * 500) / kFastClkFreq * 1.024;   //1.024 = revision

    parameter digital_bit = 4;
    parameter digital_delay_7S = 1;
    parameter digital_delay_US = 8;

    wire signed [kWidthLaccpFineOffset-1:0] sec_digital_delay;
    wire signed [kWidthLaccpFineOffset-1:0] prim_digital_delay;
    wire signed [kWidthLaccpFineOffset-1:0] digital_reslt;

    wire signed [15:0] sel_sec  = $signed(is_ultrascale_sec ? digital_delay_US : digital_delay_7S);
    wire signed [15:0] sel_prim = $signed(is_ultrascale_pri ? digital_delay_US : digital_delay_7S);

    wire signed [31:0] sec_tmp  = $signed(period) * sel_sec;
    wire signed [31:0] prim_tmp = $signed(period) * sel_prim;

    assign sec_digital_delay  = sec_tmp [kWidthLaccpFineOffset-1:0];
    assign prim_digital_delay = prim_tmp[kWidthLaccpFineOffset-1:0];

    assign digital_reslt = $signed(sec_digital_delay) - $signed(prim_digital_delay);                                
         
    reg signed [kWidthLaccpFineOffset-1:0] digital_reslt_old;         
    always@(posedge CLK)begin
        digital_reslt_old <= digital_reslt;
    end         
                                
//-------------------------
//caluculation all
//-------------------------                                  
    
    wire signed [kWidthLaccpFineOffset-1:0] all_reslt_level1; 
    assign all_reslt_level1 = digital_reslt_old + analog_result_old;

    
    reg signed [kWidthLaccpFineOffset-1:0] all_result_level1old; 
    always@(posedge CLK)begin
        all_result_level1old <= all_reslt_level1;
    end    
    
    generate
    if (enCalibDelay == TRUE) begin : g_TRUE  
        assign result = all_result_level1old;
    end else begin : g_False
        assign result = 0;
    end
    endgenerate      
        
        
endmodule
