`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    01:41:03 12/17/2016 
// Design Name: 
// Module Name:    all 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module UART(
    input clk, // The master clock for this module
    input rst, // Synchronous reset.
    input rx, // Incoming serial line
    output tx, // Outgoing serial line
    input transmit, // Signal to transmit
    input [7:0] tx_byte, // Byte to transmit
    output received, // Indicated that a byte has been received.
    output [7:0] rx_byte, // Byte received
    output is_receiving, // Low when receive line is idle.
    output is_transmitting, // Low when transmit line is idle.
    output recv_error // Indicates error in receiving packet.
    );

parameter CLOCK_DIVIDE = 1302; // clock rate (50Mhz) / (baud rate (9600) * 4)

// States for the receiving state machine.
// These are just constants, not parameters to override.
parameter RX_IDLE = 0;
parameter RX_CHECK_START = 1;
parameter RX_READ_BITS = 2;
parameter RX_CHECK_STOP = 3;
parameter RX_DELAY_RESTART = 4;
parameter RX_ERROR = 5;
parameter RX_RECEIVED = 6;

// States for the transmitting state machine.
// Constants - do not override.
parameter TX_IDLE = 0;
parameter TX_SENDING = 1;
parameter TX_DELAY_RESTART = 2;

reg [10:0] rx_clk_divider = CLOCK_DIVIDE;
reg [10:0] tx_clk_divider = CLOCK_DIVIDE;

reg [2:0] recv_state = RX_IDLE;
reg [5:0] rx_countdown;
reg [3:0] rx_bits_remaining;
reg [7:0] rx_data;

reg tx_out = 1'b1;
reg [1:0] tx_state = TX_IDLE;
reg [5:0] tx_countdown;
reg [3:0] tx_bits_remaining;
reg [7:0] tx_data;

assign received = recv_state == RX_RECEIVED;
assign recv_error = recv_state == RX_ERROR;
assign is_receiving = recv_state != RX_IDLE;
assign rx_byte = rx_data;

assign tx = tx_out;
assign is_transmitting = tx_state != TX_IDLE;

always @(posedge clk) begin
	if (rst) begin
		recv_state = RX_IDLE;
		tx_state = TX_IDLE;
	end
	
	
	// The clk_divider counter counts down from
	// the CLOCK_DIVIDE constant. Whenever it
	// reaches 0, 1/16 of the bit period has elapsed.
   // Countdown timers for the receiving and transmitting
	// state machines are decremented.
	rx_clk_divider = rx_clk_divider - 1;
	if (!rx_clk_divider) begin
		rx_clk_divider = CLOCK_DIVIDE;
		rx_countdown = rx_countdown - 1;
	end
	tx_clk_divider = tx_clk_divider - 1;
	if (!tx_clk_divider) begin
		tx_clk_divider = CLOCK_DIVIDE;
		tx_countdown = tx_countdown - 1;
	end
	
	// Receive state machine
	case (recv_state)
		RX_IDLE: begin
			// A low pulse on the receive line indicates the
			// start of data.
			if (!rx) begin
				// Wait half the period - should resume in the
				// middle of this first pulse.
				rx_clk_divider = CLOCK_DIVIDE;
				rx_countdown = 2;
				recv_state = RX_CHECK_START;
			end
		end
		RX_CHECK_START: begin
			if (!rx_countdown) begin
				// Check the pulse is still there
				if (!rx) begin
					// Pulse still there - good
					// Wait the bit period to resume half-way
					// through the first bit.
					rx_countdown = 4;
					rx_bits_remaining = 8;
					recv_state = RX_READ_BITS;
				end else begin
					// Pulse lasted less than half the period -
					// not a valid transmission.
					recv_state = RX_ERROR;
				end
			end
		end
		RX_READ_BITS: begin
			if (!rx_countdown) begin
				// Should be half-way through a bit pulse here.
				// Read this bit in, wait for the next if we
				// have more to get.
				rx_data = {rx, rx_data[7:1]};
				rx_countdown = 4;
				rx_bits_remaining = rx_bits_remaining - 1;
				recv_state = rx_bits_remaining ? RX_READ_BITS : RX_CHECK_STOP;
			end
		end
		RX_CHECK_STOP: begin
			if (!rx_countdown) begin
				// Should resume half-way through the stop bit
				// This should be high - if not, reject the
				// transmission and signal an error.
				recv_state = rx ? RX_RECEIVED : RX_ERROR;
			end
		end
		RX_DELAY_RESTART: begin
			// Waits a set number of cycles before accepting
			// another transmission.
			recv_state = rx_countdown ? RX_DELAY_RESTART : RX_IDLE;
		end
		RX_ERROR: begin
			// There was an error receiving.
			// Raises the recv_error flag for one clock
			// cycle while in this state and then waits
			// 2 bit periods before accepting another
			// transmission.
			rx_countdown = 8;
			recv_state = RX_DELAY_RESTART;
		end
		RX_RECEIVED: begin
			// Successfully received a byte.
			// Raises the received flag for one clock
			// cycle while in this state.
			recv_state = RX_IDLE;
		end
	endcase
	
	// Transmit state machine
	case (tx_state)
		TX_IDLE: begin
			if (transmit) begin
				// If the transmit flag is raised in the idle
				// state, start transmitting the current content
				// of the tx_byte input.
				tx_data = tx_byte;
				// Send the initial, low pulse of 1 bit period
				// to signal the start, followed by the data
				tx_clk_divider = CLOCK_DIVIDE;
				tx_countdown = 4;
				tx_out = 0;
				tx_bits_remaining = 8;
				tx_state = TX_SENDING;
			end
		end
		TX_SENDING: begin
			if (!tx_countdown) begin
				if (tx_bits_remaining) begin
					tx_bits_remaining = tx_bits_remaining - 1;
					tx_out = tx_data[0];
					tx_data = {1'b0, tx_data[7:1]};
					tx_countdown = 4;
					tx_state = TX_SENDING;
				end else begin
					// Set delay to send out 2 stop bits.
					tx_out = 1;
					tx_countdown = 8;
					tx_state = TX_DELAY_RESTART;
				end
			end
		end
		TX_DELAY_RESTART: begin
			// Wait until tx_countdown reaches the end before
			// we send another transmission. This covers the
			// "stop bit" delay.
			tx_state = tx_countdown ? TX_DELAY_RESTART : TX_IDLE;
		end
	endcase
end

endmodule


module Receiver ( outreg, clk, rst, rx, transmit, tx, received,is_receiving,is_transmitting,recv_error);
input clk, rst, rx, transmit; // Incoming serial line
output tx, received,is_receiving,is_transmitting,recv_error; // Outgoing serial line
//output done;
output reg [104:0] outreg;
//wire [6:0] sample;
//assign sample = outreg[6:0];
parameter state0 = 2'd0, state1 = 2'd1, state2 = 2'd2;
reg [1:0] currentState;
reg [4:0] i;				//this might be a problem in some cases... however, it's good for simulation
//assign done = (i >= 5'd15);

wire [7:0] tx_byte, rx_byte;

UART uart1(clk, rst, rx, tx, transmit, tx_byte, received, rx_byte, is_receiving, is_transmitting, recv_error);

always @ (posedge clk) begin
if (rst) begin
i <= 5'd0;
currentState <=state0;
end
else
case (currentState)
state0: begin
if (is_receiving) begin currentState <= state1; end
else begin currentState <= state0; end
i <= i;
outreg <= outreg;
end
state1: begin
if (is_receiving) begin currentState <= state1; end
else begin currentState <= state2; end
i <= i;
outreg <= outreg;
end
state2: begin
if (is_receiving) begin currentState <= state1; end
else begin currentState <= state0; end
if (i < 5'd15) begin
outreg <= {rx_byte[6:0], outreg[104:7]};
i <= i + 5'd1;
end
else begin
i <= i;
outreg <= outreg;
end
end
endcase
end
endmodule

module CLkDiv(inputclock, outputclock, rst);
parameter n = 1;
parameter c =50;
input inputclock, rst;
output reg outputclock;
reg [31:0] counter;

always @  (posedge inputclock) begin 
if (rst) begin
outputclock <= 1'b0;
counter <= 32'b0;
end
else begin 
if (counter == (c/n)) 
counter <= 32'b0;
else counter <= counter + 1'b1;
if (counter == (c/n))
     outputclock <= 1'b0;
else
    if(counter == (c/(2 * n)))
        outputclock <= 1'b1;
end
end
endmodule



module Motor(clkmotor, pulseM);
parameter n= 2;
input clkmotor;
output reg pulseM = 1'b1;
reg [2:0] counter = 3'b000;

always @ (posedge clkmotor) begin
if(counter == 4'b111)
    counter = 4'b000;
else
    counter = counter + 1'b1;

if(counter < n)
    pulseM = 1'b1;
else
    pulseM = 1'b0;
end
endmodule

/*
module PC(clock, addrout, rst);
input  clock, rst;
output reg [7:0] addrout;

always @ (posedge clock) begin
if (rst)
    addrout <= 8'd0;
else 
    addrout <= addrout + 1'b1;
end

endmodule
*/

module PC_7(clock, addrout, rst);
input  clock, rst;
output reg [7:0] addrout;

always @ (posedge clock) begin
if (rst)
    addrout <= 8'd0;
else 
    addrout <= addrout + 8'd6;
end

endmodule

/*
module SavePath(echo, trig,Path, clk, US, rst, right);
input clk, rst; 
output trig;
input echo;
output [1:0] US; 
output reg [7:0] Path [15:0];
reg clockDivided;
output  right;
reg [3:0] count;
reg [3:0] duration;
reg [31:0] state;


CLkDiv #(200000, 50000000) bit0(clk, clkdivid,rst);
UltraSonic Ultra_1( echo, trig , clk, US, rst, right);
always @(posedge clockDivided) begin
if (rst)begin
count <= 32'b0;
state <= 32'b0;
end
if (state == 32'd100)
state <= 32'b0;

else begin 
if (US != 2'b00)begin
if (US == 2'b11)
count <= count + 1'b1;
else if (US == 2'b10)begin
Path [state] <= ({1'b0, count [3:0], 2'b11, 1'b0});
Path [state+1] <= ({1'b0, count [3:0], 2'b10, 1'b0});
count<= 32'b0;
state <= state + 1'b1;
end 
else if (US == 2'b01)begin
Path [state] <= ({1'b0, count [3:0], 2'b11, 1'b0});
Path [state+1] <= ({1'b0, count [3:0], 2'b01, 1'b0});
count<= 32'b0;
state <= state + 1'b1;

end
end

end
end
endmodule
*/

module UltraSonic ( echo, trig , clk, US, rst, right);
output reg trig;
input echo;
input clk, rst;
output reg [1:0]US;                // to choose the path
wire clkdivid;
reg [12:0] countertrig;
reg[31:0] counterecho;                         //to check if we turned right before;
reg [31:0] temp;
reg[31:0] timer;
output reg right;
reg [31:0] counterdelayer;
CLkDiv #(200000, 50000000) bit0(clk, clkdivid,rst);


always @(posedge clkdivid) begin // stop = 00, left = 01, right = 10, front = 11
if(rst) begin
countertrig <= 13'b0;
counterecho <= 32'b0;
temp <= 32'd250;
timer <=32'd0;
right <= 1'b0;
counterdelayer <= 32'b0;
US <= 2'b00;
end
else begin
if(countertrig < 13'd3) //  && US != 2'b10 && US != 2'b01      // to send signal to trigger
    trig <= 1'b1;
else
    trig <= 1'b0;   
    
 if(US == 2'b11 && right == 1)
    counterdelayer <= counterdelayer + 1'b1;
else
    counterdelayer <= 32'b0;
   
    
if(countertrig == 13'd7644 )                  // count for trigger pulse
    countertrig <= 13'b0;
else
    countertrig <= countertrig + 1'b1;

if(echo == 1'b1)                             // count for echo pulse to calculate the distance;
    counterecho <= counterecho +1'b1;
else begin 
    if(counterecho != 32'b0)   
	    temp <= counterecho;
    if(temp == counterecho)
		counterecho <= 32'b0;
	
    if(temp > 32'd200 && US != 2'b10 && US != 2'b01) begin
        US <= 2'b11;
		  if(counterdelayer == 32'd16000) right <= 1'b0;
		  end
    else begin
	 if (timer != 32'd0 && US == 2'b11) timer <=32'd0;
	 if (right == 0) begin
		
		  US <= 2'b10;
		  timer <= timer + 1'b1;
		  if (timer == 32'd200000) begin US <= 2'b11; right <= 1; timer <=32'd0; end
		  end
	else if (right == 1) begin 
			US <= 2'b01;
		  timer <= timer + 1'b1;
		  if (timer == 32'd400000) begin US <= 2'b11; right <= 0; timer <=32'd0; end
		  end
    end
    end
/*	 

if(counterdelayer == 32'd16000)
    right <= 1'b0;


if(US == 2'b01 || US == 2'b10)
    timer <= timer + 1'b1;
else
    timer <= 32'b0;
 */   
   /* 
if(US == 2'b00) // stop
    if(right == 1'b0) begin
        US <= 2'b10; // right
        right <=1'b1;
        end
    else
        US <= 2'b01; // left */
      /* 
if(US == 2'b10) begin // right
    
    if(timer == 32'd400000)
        US <= 2'b11;
    end*/
    /*
if(US == 2'b01)
    if(timer == 32'd800000) begin // left
        US <= 2'b11;
         right <= 1'b0;
        end*/
end
end
endmodule


/*

module Car(clk, rst, Direction, mst, mt, dt, mct);
parameter n = 2000;
output mst, mt;
output[1:0] dt;
output[3:0] mct;
output [1:0] Direction;
wire [3:0] Motorclk;
input clk, rst;
wire m;
wire [7:0] addrout;
reg [31:0]counter;
reg start;
wire  MotorStop;
 PC  coco(start, addrout, rst);
//wire  M1, M2;    //M1 left wheel , M2 right wheel;
//CLkDiv #(1, 50000000) bit1(clk, ClkDivided, rst);
/*
RAM motorrre (
  .clka(start), // input clka
  .addra(addrout), // input [7 : 0] addra
  .douta({m,Direction, Motorclk,  MotorStop}) // output [7 : 0] douta
);

 assign mst =MotorStop;
 assign mt =m;
 assign dt =Direction;
  assign mct =Motorclk;
always @ (posedge clk) begin
if (rst) begin // active low push buttons
/*Direction <= 2'd0;
Motorclk <= 4'd0;
MotorStop <= 1'b0;

	counter <= 32'b0;
	start <= 1'b0;
end


else begin
	if((counter == (Motorclk * n)))
		counter <= 32'd0;
	else
		counter <= counter +32'd1; 

if( MotorStop == 1'b1 && (counter == (Motorclk * n)))
start <= 1'b0;
	if(counter == 32'd0)
		start <= 1'b1;
	else
		start <= 1'b0;

end


end
endmodule

*/


module CarUART(clk, rst, Data, Directions, mst, mt, dt, mct, enable);
parameter n = 200;
input [104:0] Data;
output mst, mt;
output[1:0] dt;
output[3:0] mct;
output reg [1:0] Directions;
reg [3:0] Motorclk;
input clk, rst, enable;
reg [31:0]counter;
reg start;
wire m;
wire [7:0] Countposition;
reg  MotorStop;

PC_7  cc(start, Countposition, rst);

assign mst =MotorStop;
assign mt =m;
assign dt =Directions;
assign mct =Motorclk;
 
always @ (posedge clk) begin
 
if (rst) begin 
  	counter <= 32'b0;
	start <= 1'b0;
  end
    else if (enable) begin
	 
 if (start)
  if (Countposition == 32'd6)
  	    {Directions,  Motorclk,  MotorStop} <= Data[6:0];
	else if (Countposition == 32'd13)
    	   {Directions,  Motorclk,  MotorStop} <= Data[13:7];
	else if (Countposition == 32'd20)
    	   {Directions,  Motorclk,  MotorStop} <= Data[20:14];
	else if (Countposition == 32'd27)
    	   {Directions,  Motorclk,  MotorStop} <= Data[27:21];
	else if (Countposition == 32'd34)
    	   {Directions,  Motorclk,  MotorStop} <= Data[34:28];
	else if (Countposition == 32'd41)
    	   {Directions,  Motorclk,  MotorStop} <= Data[41:35];
	else if (Countposition == 32'd48)
    	   {Directions,  Motorclk,  MotorStop} <= Data[48:42];
	else if (Countposition == 32'd55)
    	   {Directions,  Motorclk,  MotorStop} <= Data[55:49];
	else if (Countposition == 32'd62)
    	   {Directions,  Motorclk,  MotorStop} <= Data[62:56];
	else if (Countposition == 32'd69)
    	   {Directions,  Motorclk,  MotorStop} <= Data[69:63];
	else if (Countposition == 32'd76)
    	   {Directions,  Motorclk,  MotorStop} <= Data[76:70];
	else if (Countposition == 32'd83)
    	   {Directions,  Motorclk,  MotorStop} <= Data[83:77];
	else if (Countposition == 32'd90)
    	   {Directions,  Motorclk,  MotorStop} <= Data[90:84];		
	else if (Countposition == 32'd97)
    	   {Directions,  Motorclk,  MotorStop} <= Data[97:91];		
	else if (Countposition == 32'd104)
    	   {Directions,  Motorclk,  MotorStop} <= Data[104:98];		
  
    
if(MotorStop == 1'b0 &&(counter == (Motorclk * n)))
	counter <= 32'd0;
else
	counter <= counter +32'd1; 

if( MotorStop == 1'b1 && (counter == (Motorclk * n)))
	start <= 1'b0;

if(counter == 32'd0)
	start <= 1'b1;
else
	start <= 1'b0;

end
end
endmodule

module Mux(aa, bb, cc, dd, sel, out);
parameter n = 2;
input [n-1:0] aa, bb, cc, dd;
input [1:0]sel;
output reg [n-1:0] out;
always@* begin
if (sel == 2'b00)
    out = aa;
  else if (sel == 2'b01)
    out = bb;
  else  if (sel == 2'b10)
    out = cc;
  else  if (sel == 2'b11)
     out = dd;
     
     end
endmodule

module MotorDriverPath(clk, M1, M2, rst, mst, mt, dt, mct, rx, transmit, tx, received,is_receiving,is_transmitting,recv_error, readEn);
output mst, mt, tx, received,is_receiving,is_transmitting,recv_error;
output [1:0] dt;
output [3:0] mct;
input clk, rst, rx, transmit, readEn; 
output  M1, M2;    //M1 left wheel , M2 right wheel;
wire [2:0]Motor; 
wire [1:0] Direction;
wire ClkDivided;
wire [104:0]outreg;
Receiver receiver1(outreg, clk, rst, rx, transmit, tx, received,is_receiving,is_transmitting,recv_error);
//assign dat = 105'b101101011011101111110111111001000100111110000011010001010101010111111111000110101010101000101010101011010100101011111010000001010101011110;

CLkDiv #(2000, 50000000) bit1(clk, ClkDivided, rst);
//Car ultra( ClkDivided, rst, Direction, mst, mt, dt, mct);

CarUART ultra( ClkDivided, rst, outreg,Direction, mst, mt, dt, mct, readEn);

Motor #(2) motora(ClkDivided, Motor[0]);
Motor #(3) motorb(ClkDivided, Motor[1]);
Motor #(4) motorc(ClkDivided, Motor[2]);
Mux   #(2) Mux1({Motor[1], Motor[1]}, {Motor[0], Motor[0]}, {Motor[2], Motor[2]}, {Motor[2], Motor[0]}, Direction, {M1, M2});
endmodule



module MotorDriverUltra(clk, M1, M2, echo,inputTrigger, rst, outUltraSonic, right2);
input clk, rst; 
output  M1, M2;    //M1 left wheel , M2 right wheel;
wire [2:0]Motor;
output inputTrigger; 
input echo; 
output [1:0] outUltraSonic;
wire ClkDivided;
output right2;
CLkDiv #(2000, 50000000) bit1(clk, ClkDivided, rst);
UltraSonic ultra( echo, inputTrigger, clk, outUltraSonic, rst, right2);

Motor #(2) motora(ClkDivided, Motor[0]);
Motor #(30000000) motorb(ClkDivided, Motor[1]);
Motor #(4) motorc(ClkDivided, Motor[2]);
Mux  #(2) Mux1({Motor[1], Motor[1]}, {Motor[0], Motor[0]}, {Motor[2], Motor[2]}, {Motor[2], Motor[0]}, outUltraSonic, {M1, M2}); // stop, left, write, front
endmodule


module mux2x1(aa, bb, sel, out);
parameter n = 1;
input [n-1:0] aa, bb;
input sel;
output reg  [n-1:0] out;
always @(sel) begin
if (sel == 1'b0)
out <= aa;
else 
 out <= bb;
end
endmodule


module CarGeneral (clk,en, M1, M2, echo,inputTrigger, rst,outUltraSonic,right2, mst, mt, dt, mct, tx, received,is_receiving,is_transmitting,recv_error, rx, transmit, readEn);
output mst, mt, tx, received,is_receiving,is_transmitting,recv_error;
output [1:0] dt;
output [3:0] mct;
input clk, rst, en, echo, rx, transmit, readEn;
output inputTrigger;
output [1:0] outUltraSonic;
output right2;
output  M1, M2;//M1 left wheel , M2 right wheel;
wire M3, M4, M5, M6;
wire [2:0]Motor; 
wire [1:0] Direction;
wire [6:0]sample; 

MotorDriverPath  D_1(clk, M3, M4, rst, mst, mt, dt, mct, rx, transmit, tx, received,is_receiving,is_transmitting,recv_error, readEn);
MotorDriverUltra D_2(clk, M5, M6, echo,inputTrigger, rst, outUltraSonic, right2);
mux2x1 #(2) m({M3, M4}, { M5, M6}, en, {M1, M2});

endmodule
