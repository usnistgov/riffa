// ----------------------------------------------------------------------
// Copyright (c) 2016, The Regents of the University of California All
// rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
// 
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
// 
//     * Neither the name of The Regents of the University of California
//       nor the names of its contributors may be used to endorse or
//       promote products derived from this software without specific
//       prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REGENTS OF THE
// UNIVERSITY OF CALIFORNIA BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
// TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
// USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
// ----------------------------------------------------------------------
//----------------------------------------------------------------------------
// Filename:			async_fifo.v
// Version:				0.0.1a
// Verilog Standard:	Verilog-2001
// Description:			Asynchronous capable parameterized FIFO. As with all
// traditional FIFOs, the RD_DATA will be valid one cycle following a RD_EN 
// assertion. 
// Author:				Matt Jacobsen, modified by Gabe Ycas with code written by CE Cummings
// History:				@mattj: Version 2.0
// Additional Comments: Based on design by CE Cummings in Simulation 
// and Synthesis Techniques for Asynchronous FIFO Design
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "functions.vh"
module async_fifo #(
	parameter C_WIDTH = 32,	// Data bus width
	parameter C_DEPTH = 1024,	// Depth of the FIFO
	// Local parameters
	parameter C_REAL_DEPTH = 2**`clog2(C_DEPTH),
	parameter C_DEPTH_BITS = `clog2(C_REAL_DEPTH),
	parameter C_DEPTH_P1_BITS = `clog2(C_REAL_DEPTH+1)
)
(
	input RD_CLK,							// Read clock
	input RD_RST,							// Read synchronous reset
	input WR_CLK,						 	// Write clock
	input WR_RST,							// Write synchronous reset
	input [C_WIDTH-1:0] WR_DATA, 			// Write data input (WR_CLK)
	input WR_EN, 							// Write enable, high active (WR_CLK)
	output [C_WIDTH-1:0] RD_DATA, 			// Read data output (RD_CLK)
	input RD_EN,							// Read enable, high active (RD_CLK)
	output WR_FULL, 						// Full condition (WR_CLK)
	output RD_EMPTY 						// Empty condition (RD_CLK)
);



wire	[C_DEPTH_BITS-1:0]	waddr, raddr;
wire	[C_DEPTH_BITS  :0]	wptr, rptr, wq2_rptr, rq2_wptr;

sync_r2w #(.ADDRSIZE (C_DEPTH_BITS) )     sync_r2w  (.wq2_rptr(wq2_rptr), .rptr(rptr),
                           .wclk(WR_CLK), .wrst(WR_RST));
sync_w2r #(.ADDRSIZE (C_DEPTH_BITS) )     sync_w2r  (.rq2_wptr(rq2_wptr), .wptr(wptr),
                           .rclk(RD_CLK), .rrst(RD_RST));

// Memory block (synthesis attributes applied to this module will
// determine the memory option).
ram_2clk_1w_1r #(.C_RAM_WIDTH(C_WIDTH), .C_RAM_DEPTH(C_REAL_DEPTH)) mem (
	.CLKA(WR_CLK),
	.ADDRA(waddr),
	.WEA(WR_EN & !WR_FULL),
	.DINA(WR_DATA),
	.CLKB(RD_CLK),
	.ADDRB(raddr),
	.DOUTB(RD_DATA)
);

rptr_empty #( .ADDRSIZE (C_DEPTH_BITS) ) rptr_empty_i
(
    .rempty   (RD_EMPTY),
    .raddr    (raddr),
    .rptr     (rptr),
    .rq2_wptr (rq2_wptr),
    .rinc     (RD_EN),
    .rclk     (RD_CLK),
    .rrst     (RD_RST)
);

wptr_full #( .ADDRSIZE (C_DEPTH_BITS) ) wptr_full_i
(
    .wfull    (WR_FULL),
    .waddr    (waddr),
    .wptr     (wptr),
    .wq2_rptr (wq2_rptr),
    .winc     (WR_EN),
    .wclk     (WR_CLK),
    .wrst     (WR_RST)
);
endmodule

module rptr_empty #(parameter ADDRSIZE = 4)
  (output reg                rempty,
   output     [ADDRSIZE-1:0] raddr,
   output reg [ADDRSIZE  :0] rptr,
   input      [ADDRSIZE  :0] rq2_wptr,
   input                     rinc, rclk, rrst);
  reg  [ADDRSIZE:0] rbin;
  wire [ADDRSIZE:0] rgraynext, rbinnext;
  //-------------------
  // GRAYSTYLE2 pointer
  //-------------------
  always @(posedge rclk or posedge rrst)
    if (rrst) {rbin, rptr} <= 0;
    else         {rbin, rptr} <= {rbinnext, rgraynext};
  // Memory read-address pointer (okay to use binary to address memory)
  assign raddr     = rbin[ADDRSIZE-1:0];
  assign rbinnext  = rbin + (rinc & ~rempty);
  assign rgraynext = (rbinnext>>1) ^ rbinnext;
  //---------------------------------------------------------------
  // FIFO empty when the next rptr == synchronized wptr or on reset
  //---------------------------------------------------------------
  assign rempty_val = (rgraynext == rq2_wptr);
  always @(posedge rclk or posedge rrst)
    if (rrst) rempty <= 1'b1;
    else         rempty <= rempty_val;
endmodule

module wptr_full  #(parameter ADDRSIZE = 4)
  (output reg                wfull,
   output     [ADDRSIZE-1:0] waddr,
   output reg [ADDRSIZE  :0] wptr,
   input      [ADDRSIZE  :0] wq2_rptr,
   input                     winc, wclk, wrst);
  reg  [ADDRSIZE:0] wbin;
  wire [ADDRSIZE:0] wgraynext, wbinnext;
  // GRAYSTYLE2 pointer
  always @(posedge wclk or posedge wrst)
    if (wrst) {wbin, wptr} <= 0;
    else         {wbin, wptr} <= {wbinnext, wgraynext};
  // Memory write-address pointer (okay to use binary to address memory)
  assign waddr = wbin[ADDRSIZE-1:0];
  assign wbinnext  = wbin + (winc & ~wfull);
  assign wgraynext = (wbinnext>>1) ^ wbinnext;
  //------------------------------------------------------------------
  // Simplified version of the three necessary full-tests:
  // assign wfull_val=((wgnext[ADDRSIZE]    !=wq2_rptr[ADDRSIZE]  ) &&
  //                   (wgnext[ADDRSIZE-1]  !=wq2_rptr[ADDRSIZE-1]) &&
  //                   (wgnext[ADDRSIZE-2:0]==wq2_rptr[ADDRSIZE-2:0]));
  //------------------------------------------------------------------
  assign wfull_val = (wgraynext=={~wq2_rptr[ADDRSIZE:ADDRSIZE-1],
                                   wq2_rptr[ADDRSIZE-2:0]});
  always @(posedge wclk or posedge wrst)
    if (wrst) wfull  <= 1'b0;
    else         wfull  <= wfull_val;
endmodule

module sync_r2w #(parameter ADDRSIZE = 4)
  ((*ASYNC_REG = "true" *)
   output reg [ADDRSIZE:0] wq2_rptr,
   input      [ADDRSIZE:0] rptr,
   input                   wclk, wrst);
  (*ASYNC_REG = "true" *) reg [ADDRSIZE:0] wq1_rptr;
  always @(posedge wclk or posedge wrst)
    if (wrst) {wq2_rptr,wq1_rptr} <= 0;
    else         {wq2_rptr,wq1_rptr} <= {wq1_rptr,rptr};
endmodule

module sync_w2r #(parameter ADDRSIZE = 4)
  (
   (*ASYNC_REG = "true" *)
   output reg [ADDRSIZE:0] rq2_wptr,
   input      [ADDRSIZE:0] wptr,
   input                   rclk, rrst);
  (*ASYNC_REG = "true" *) reg [ADDRSIZE:0] rq1_wptr;
  always @(posedge rclk or posedge rrst)
    if (rrst)    {rq2_wptr,rq1_wptr} <= 0;
    else         {rq2_wptr,rq1_wptr} <= {rq1_wptr,wptr};
endmodule

