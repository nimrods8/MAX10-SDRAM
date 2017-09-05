////////////////////////////////////////////////////////////////////////////////
// Author: lsilvest/nimrod stoler
//
// Create Date:   02/03/2008
//
// Module Name:    sdram_rw
//
// Target Devices: Altera DE2
//
// Tool versions:  Quartus II 7.2 Web Edition
//
//
// Description: This module provides a simple test bench for the SDRAM
//              controller.  It sequentially writes all positions in
//              memory, pauses for a while and then reads back all
//              positions comparing them to the written value. The
//              green LEDG1 indicates the test passed. The red LEDR0
//              indicates at least one of the readbacks failed
//
////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2008 Authors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
////////////////////////////////////////////////////////////////////////////////
module sdram_rw
  (
   input clk_i,
   input rst_i,
   output [24:0] addr_i,
   output [31:0] dat_i,
   input [31:0] dat_o,
   output we_i,
   input ack_o,
   output stb_i,
   output cyc_i,
	input  init_done,
   output [0:0] green_led,
   output [0:0] red_led,
	output [7:0] hex0, 
	output [7:0] hex1, 
	output [7:0] hex2, 
	output [7:0] hex3, 
	output [7:0] hex4, 
	output [7:0] hex5,
	input  [9:0] switches
   );

  parameter START_WRITE_ST        = 4'b0000,
              WRITE_ST           = 4'b0001,
              WAIT_WRITE_ACK_ST  = 4'b0010,
              READ_ST            = 4'b0011,
              WAIT_READ_ACK_ST   = 4'b0100,
              WRITE_WAIT_ST      = 4'b0101,
              START_READ_ST      = 4'b0110,
              READ_WAIT_ST       = 4'b0111,
              DONE_ST            = 4'b1000,
				  DONE_WAIT_ST       = 4'b1001,
              WRITE_SECOND_BANK	= 4'b1010,
              WAIT_WRITE2_ACK_ST	= 4'b1011;
				  
  
  parameter MAX_RW = 24'd0002000; // 200000 is the full 8 Mbytes of memory
  parameter R_TO_W_WAIT_TIME  = 24'd1;//24'd12500000;
  parameter INITIAL_MEM_VALUE = /*32'h12345678*/32'h00005678;
  
  reg [24:0] addr_i_r;
  reg [31:0] dat_i_r;
  reg        we_i_r;
  reg        stb_i_r;
  reg        cyc_i_r;

  reg [23:0] rw_cntr;  
  reg [23:0] cntr;
  reg [31:0] number;
  reg [31:0] mem_value;
  reg [3:0]  state;   
  reg [31:0]  read_cntr, time_count;
  reg [7:0]  bad_count;

  reg [0:0]       red_led_r;
  reg [0:0]       green_led_r;
  

  assign dat_i = dat_i_r;
  assign addr_i = addr_i_r;
  assign we_i = we_i_r;
  assign stb_i = stb_i_r;
  assign cyc_i = cyc_i_r;
  assign red_led = red_led_r;
  assign green_led = green_led_r;


  initial begin
    mem_value <= INITIAL_MEM_VALUE;
    cntr <= 24'b0;
    rw_cntr <= 24'b0;
    state <= DONE_WAIT_ST;
    we_i_r <= 1'b0;
    addr_i_r <= 25'b0;
    stb_i_r <= 1'b0;
    cyc_i_r <= 1'b0;
    red_led_r <= 1'b0;
    green_led_r <= 1'b0;
	 read_cntr <= 0;
  end
  
  
  always@ (posedge clk_i or posedge rst_i) begin
    if( rst_i && state == DONE_WAIT_ST ) begin
		state <= START_WRITE_ST;
		red_led_r <= 1'b0;
		green_led_r <= 1'b0;
		rw_cntr <= 24'b0;
		addr_i_r <= 25'd0;
		bad_count <= switches[7:0]; 
		time_count <= 0;
    end else begin
	 
      case (state) 
        START_WRITE_ST:
          begin
            state <= WRITE_ST;
          end
        
        WRITE_ST:
          begin
            stb_i_r <= 1'b1;
            cyc_i_r <= 1'b1;
            dat_i_r <= mem_value;
            we_i_r <= 1'b1;
            //addr_i_r <= { 15'b0, switches };
			   addr_i_r <= { 1'b0, switches[9:8], 12'b0, switches };
            state <= WAIT_WRITE_ACK_ST;
          end

        WAIT_WRITE_ACK_ST:
          if (ack_o) begin
            state <= WRITE_SECOND_BANK; //READ_ST; //WRITE_WAIT_ST;
            stb_i_r <= 1'b0;
            cyc_i_r <= 1'b0;
				read_cntr <= 0;
				bad_count <= 0;
          end
			 else rw_cntr <= rw_cntr + 24'b1;
			 
			 
		  WRITE_SECOND_BANK:
          begin
            stb_i_r <= 1'b1;
            cyc_i_r <= 1'b1;
            dat_i_r <= mem_value + 1;
            we_i_r <= 1'b1;
			   addr_i_r <= { 1'b1, switches[9:8], 12'b0, switches };
            state <= WAIT_WRITE2_ACK_ST;
          end
		  
        WAIT_WRITE2_ACK_ST:
          if (ack_o) begin
            state <= READ_ST;
            stb_i_r <= 1'b0;
            cyc_i_r <= 1'b0;
				read_cntr <= 0;
				bad_count <= 0;
          end
			 else rw_cntr <= rw_cntr + 24'b1;

        READ_ST:
          begin
            stb_i_r <= 1'b1;
            cyc_i_r <= 1'b1;
            we_i_r <= 1'b0;
				addr_i_r <= { 1'b0, switches[9:8], 12'b0, switches };
            state <= WAIT_READ_ACK_ST;
          end
        
        WAIT_READ_ACK_ST:
          if (ack_o) begin
            state <= READ_WAIT_ST;
            number <= dat_o;
            stb_i_r <= 1'b0;
            cyc_i_r <= 1'b0;
          end
			 else rw_cntr <= rw_cntr + 24'b1;


        READ_WAIT_ST:
          begin
				  if( (read_cntr & 32'h01))
				  begin
					  if (mem_value + 1 != number) begin
						 bad_count <= bad_count + 1;
						 red_led_r[0] <= 1'b1;
					  end 
				  end else
				  begin
					  if (mem_value != number) begin
						 bad_count <= bad_count + 1;
						 red_led_r[0] <= 1'b1;
					  end 
				  end
				  read_cntr <= read_cntr + 1;
				  if( read_cntr == 'h00ffffff)
				  begin
					  mem_value <= mem_value + 1'b1;
					  state <= DONE_ST;
					  time_count <= number;
				  end else begin
					  //state <= READ_ST;
					  stb_i_r <= 1'b1;
					  cyc_i_r <= 1'b1;
					  we_i_r <= 1'b0;
					  if( read_cntr & 32'h01)
						  addr_i_r <= { 1'b0, switches[9:8], 12'b0, switches };
					  else
						  addr_i_r <= { 1'b1, switches[9:8], 12'b0, switches };
					  state <= WAIT_READ_ACK_ST;
              end
          end // case: READ_WAIT_ST

        DONE_ST:
          begin
            state <= DONE_WAIT_ST;
            if (!red_led_r[0])
              green_led_r[0] <= 1'b1;
 			   else
              green_led_r[0] <= 1'b0;
          end

        DONE_WAIT_ST:
          begin
			 end
			 
      endcase // case (state)
    end // else: !if(rst_i)
  end // always@ (posedge clk_i)


  always@ (posedge clk_i) begin
    if (rst_i) begin
      cntr <= 24'b0;
    end else if (state == WRITE_WAIT_ST) begin
      cntr <= R_TO_W_WAIT_TIME;
    end else
      cntr <= cntr - 24'b1;
  end



  SEG7_LUT seg0 ( hex0, state[3:0]);
  SEG7_LUT seg1 ( hex1, time_count[3:0]);
  SEG7_LUT seg2 ( hex2, time_count[7:4]);
  SEG7_LUT seg3 ( hex3, time_count[11:8]);
  SEG7_LUT seg4 ( hex4, time_count[15:12]);
  SEG7_LUT seg5 ( hex5, time_count[19:16]);

  
endmodule

