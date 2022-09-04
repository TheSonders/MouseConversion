`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
/*
MIT License

Copyright (c) 2022 Antonio Sánchez (@TheSonders)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

PS2MOUSE -> MSMOUSE Conversion
Preliminary

References:
https://roborooter.com/post/serial-mice/
https://isdaman.com/alsos/hardware/mouse/ps2interface.htm
https://www.avrfreaks.net/sites/default/files/PS2%20Keyboard.pdf

*/
//////////////////////////////////////////////////////////////////////////////////
module MSMouseWrapper
	#(parameter CLKFREQ=50_000_000)
	(input wire clk,
	inout wire ps2dta,
	inout wire ps2clk,
	output reg rd=0
	);

assign ps2dta=(ps2dta_reg==0)?0:1'bZ;
assign ps2clk=(ps2clk_reg==0)?0:1'bZ;

localparam PS2BAUDRATE=15_000;
localparam PS2PERIOD=(CLKFREQ/PS2BAUDRATE);
localparam HUNDRED=(CLKFREQ/10_000);
localparam SERIALBAUDRATE=1_200;
localparam SERIALPERIOD=(CLKFREQ/SERIALBAUDRATE);

`define 	PAR_ODD	0
`define	PAR_EVEN	1

///////////////////////////////////////////
//////////////PS2 Reception////////////////
///////////////////////////////////////////
`define PS2CLKRISE	(ps2clkbuf==4'b0011)
`define PS2CLKFALL	(ps2clkbuf==4'b1100)
`define TXIDLE			(PS2Tr_STM==1)
`define PS2R_Start	 0
`define PS2R_Parity	 9
`define PS2R_Stop		10
`define PS2R_Delay	11

reg [3:0]ps2clkbuf=0;
reg [3:0]PS2R_STM=0;
reg PS2R_NewByte=0;
reg [7:0]PS2R_Byte=0;
reg PS2R_PAR=0;
reg [$clog2(PS2PERIOD)-1:0]PS2R_Counter=0;

always @(posedge clk)begin
	ps2clkbuf<={ps2clkbuf[2:0],ps2clk};
	if (PS2R_NewByte==1)PS2R_NewByte<=0;
	
	if (PS2R_STM==`PS2R_Delay)begin
		if (PS2R_Counter==0)begin
			PS2R_NewByte<=1;
			PS2R_STM<=0;
		end
		else PS2R_Counter<=PS2R_Counter-1;
	end
	else begin
	if (`PS2CLKRISE && `TXIDLE)begin
		case (PS2R_STM)
			`PS2R_Start:
				begin
					if (ps2dta==0)begin
						PS2R_STM<=PS2R_STM+1;
						PS2R_PAR<=`PAR_EVEN;
					end
				end
			`PS2R_Parity:
				begin
					if (ps2dta==PS2R_PAR)begin
						PS2R_STM<=PS2R_STM+1;
					end
					else begin
						PS2R_STM<=0;
					end
				end	
			`PS2R_Stop:
				begin
					if (ps2dta==1)begin
						PS2R_STM<=PS2R_STM+1;
						PS2R_Counter<=PS2PERIOD;
					end
					else PS2R_STM<=0;
				end
			default:
				begin
					PS2R_Byte<={ps2dta,PS2R_Byte[7:1]};
					PS2R_PAR<=PS2R_PAR^ps2dta;
					PS2R_STM<=PS2R_STM+1;
				end
		endcase
	end
	end	
end


///////////////////////////////////////////
//////////////PS2 Processing///////////////
///////////////////////////////////////////

`define PS2Pr_Idle		0
`define PS2Pr_WaitID		1
`define PS2Pr_WaitACK	2
`define PS2Pr_Query		3
`define PS2Pr_Wait0		4
`define PS2Pr_Wait1		5
`define PS2Pr_Wait2		6
`define PS2Pr_Wait3		7

`define PS2Pr_BAT			8'hAA
`define PS2Pr_ID			8'h00
`define PS2Pr_RESET		8'hFF
`define PS2Pr_REMOTE		8'hF0
`define PS2Pr_ACK			8'hFA
`define PS2Pr_READ		8'hEB

`define PS2Pr_M			30'h3FFFFE9A
`define PS2BitSYNC		3

`define LeftBt				PS2Byte1[0]
`define RightBt			PS2Byte1[1]
`define MSMByte1			{2'b11,`LeftBt,`RightBt,PS2Byte1[5],PS2R_Byte[7],PS2Byte1[4],PS2Byte2[7]}
`define MSMByte2			{2'b10,PS2Byte2[6:1]}
`define MSMByte3			{2'b10,PS2R_Byte[6:1]}

`define Serial_Reset		 0
`define Serial_Idle		 1
`define Serial_Stop		10

reg [$clog2(SERIALPERIOD)-1:0]Serial_Counter=0;
reg SerialSendRequest=0;
reg [4:0]Serial_STM=0;

reg [3:0]PS2Pr_STM=0;
reg PS2SendRequest=0;
reg [7:0]PS2SendData=0;
reg [7:0]PS2Byte1=0;
reg [7:0]PS2Byte2=0;
reg [29:0]SerialSendData=0;

always @(posedge clk)begin
	if (PS2SendRequest==1)PS2SendRequest<=0;
	if (SerialSendRequest==1)SerialSendRequest<=0;
		case (PS2Pr_STM)
			`PS2Pr_Idle:begin
				if (PS2R_NewByte==1)begin
				if (PS2R_Byte==`PS2Pr_BAT)begin
					PS2Pr_STM<=PS2Pr_STM+1;
				end
				else begin
					SendPS2(`PS2Pr_RESET);
				end
				end
			end
			`PS2Pr_WaitID:begin
				if (PS2R_NewByte==1)begin
				if (PS2R_Byte==`PS2Pr_ID)begin
					PS2Pr_STM<=PS2Pr_STM+1;
					SendPS2(`PS2Pr_REMOTE);
				end
				else begin
					PS2Pr_STM<=0;
					SendPS2(`PS2Pr_RESET);
				end
				end
			end
			`PS2Pr_WaitACK:begin
				if (PS2R_NewByte==1)begin
				if (PS2R_Byte==`PS2Pr_ACK)begin
					PS2Pr_STM<=PS2Pr_STM+1;
					SendSerial(`PS2Pr_M);
				end
				else begin
					PS2Pr_STM<=0;
					SendPS2(`PS2Pr_RESET);
				end
				end
			end
			`PS2Pr_Query:begin
				if (SerialSendRequest==0 && Serial_STM==1)begin
					SendPS2(`PS2Pr_READ);
					PS2Pr_STM<=PS2Pr_STM+1;
				end
			end
			`PS2Pr_Wait0:begin
				if (PS2R_NewByte==1)begin
				if (PS2R_Byte==`PS2Pr_ACK)begin
					PS2Pr_STM<=PS2Pr_STM+1;
				end
				else begin
					PS2Pr_STM<=0;
					SendPS2(`PS2Pr_RESET);
				end
				end
			end
			`PS2Pr_Wait1:begin
				if (PS2R_NewByte==1)begin
				if (PS2R_Byte[`PS2BitSYNC]==1)begin
					PS2Byte1<=PS2R_Byte;
					PS2Pr_STM<=PS2Pr_STM+1;
				end
				end
			end
			`PS2Pr_Wait2:begin
				if (PS2R_NewByte==1)begin
					PS2Byte2<=PS2R_Byte;
					PS2Pr_STM<=PS2Pr_STM+1;
				end
			end
			`PS2Pr_Wait3:begin
				if (PS2R_NewByte==1)begin
					PS2Pr_STM<=`PS2Pr_Query;
					SendSerial({1'b1,`MSMByte3,2'b01,`MSMByte2,2'b01,`MSMByte1,1'b0});
				end
			end
		endcase
///////////////////////////////////////////
/////////////Serial Transmision////////////
///////////////////////////////////////////
	case (Serial_STM)
		`Serial_Reset:begin
			rd<=1;
			Serial_STM<=Serial_STM+1;
		end
		`Serial_Idle:begin
			if (SerialSendRequest==1)begin
				Serial_STM<=Serial_STM+1;
				{SerialSendData,rd}<={1'b1,SerialSendData};
				Serial_Counter<=SERIALPERIOD;
			end
		end
		default:begin
			if (Serial_Counter==0)begin
				Serial_STM<=Serial_STM+1;
				{SerialSendData,rd}<={1'b1,SerialSendData};
				Serial_Counter<=SERIALPERIOD;
			end
			else begin
				Serial_Counter<=Serial_Counter-1;
			end
		end
	endcase
end

task SendPS2 (input [7:0] ByteToSend);
begin
	PS2SendRequest<=1;
	PS2SendData<=ByteToSend;
end
endtask

task SendSerial (input [29:0] ByteToSend);
begin
	SerialSendRequest<=1;
	SerialSendData<=ByteToSend;
end
endtask


///////////////////////////////////////////
//////////////PS2 Transmision//////////////
///////////////////////////////////////////
`define PS2Tr_Reset		 0
`define PS2Tr_Idle		 1
`define PS2Tr_ClockLow	 2
`define PS2Tr_Parity		11
`define PS2Tr_Stop		12
`define PS2Tr_ACK			13
`define PS2Tr_End			14
`define STARTBIT			 0
`define STOPBIT			 1

reg [$clog2(HUNDRED)-1:0]PS2Tr_Counter=0;
reg ps2dta_reg=0; 	
reg ps2clk_reg=0;
reg [3:0]PS2Tr_STM=0;
reg PS2Tr_PAR=0;
reg [7:0]PS2SendBuffer=0;

always @(posedge clk)begin
	case (PS2Tr_STM)
		`PS2Tr_Reset:begin
			ps2dta_reg<=1; 			//Requerido para algunas CPLD
			ps2clk_reg<=1;
			PS2Tr_STM<=PS2Tr_STM+1;
		end
		`PS2Tr_Idle:begin
			if (PS2SendRequest==1)begin
				ps2clk_reg<=0;
				PS2Tr_Counter<=HUNDRED;
				PS2Tr_STM<=PS2Tr_STM+1;
				PS2SendBuffer<=PS2SendData;
			end
		end
		`PS2Tr_ClockLow:begin
			if (PS2Tr_Counter==0)begin
				ps2dta_reg<=`STARTBIT;
				ps2clk_reg<=1;
				PS2Tr_STM<=PS2Tr_STM+1;
				PS2Tr_PAR<=`PAR_EVEN;
			end
			else begin
				PS2Tr_Counter<=PS2Tr_Counter-1;
			end
		end
		`PS2Tr_Parity:begin
			if (`PS2CLKFALL)begin
				ps2dta_reg<=PS2Tr_PAR;
				PS2Tr_STM<=PS2Tr_STM+1;
			end
		end
		`PS2Tr_Stop:begin
			if (`PS2CLKFALL)begin
				ps2dta_reg<=`STOPBIT;
				PS2Tr_STM<=PS2Tr_STM+1;
			end
		end
		`PS2Tr_ACK:begin
			if (`PS2CLKFALL)begin
				PS2Tr_STM<=PS2Tr_STM+1;
			end
		end
		`PS2Tr_End:begin
			if (`PS2CLKRISE)begin
				PS2Tr_STM<=`PS2Tr_Idle;
			end
		end
		default:begin
			if (`PS2CLKFALL)begin
				{PS2SendBuffer,ps2dta_reg}<={1'b1,PS2SendBuffer};
				PS2Tr_PAR<=PS2Tr_PAR^PS2SendBuffer[0];
				PS2Tr_STM<=PS2Tr_STM+1;
			end
		end
	endcase
end
endmodule
