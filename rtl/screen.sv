// SS1 Controller Tester
// Copyright (C) 2026 f3bandit
// MIT License (see LICENSE-MIT).
//
// screen: what the TV shows. 240p, 15 kHz (MiSTer scales it for HDMI): a 64x30 grid of 8x8
// characters. Port 1 on rows 7-11, port 2 on rows 13-17: the controller type, the buttons
// (a pressed button's name lights up), both sticks and whether there's a memory card.

module screen
(
	input             clk,          // 20 MHz
	input             reset,

	input       [1:0] pad_on,       // port 1 low half, port 2 high half
	input      [15:0] pad_id,
	input      [31:0] pad_btn,
	input      [63:0] pad_ana,      // RX RY LX LY
	input       [3:0] pad_card,
	input      [39:0] wdiag,        // {pin 6 ID, pin 6 answered, pins low: pin 6, port 2, port 1}
	input             tool,         // SS1 Tool's report file is mounted
	input             analog,       // the core switches digital pads to analog

	output reg        ce_pix,
	output reg        hblank,
	output reg        vblank,
	output reg        hsync,
	output reg        vsync,
	output reg  [7:0] r,
	output reg  [7:0] g,
	output reg  [7:0] b
);

`include "messages.vh"
`include "padcols.vh"

////////////////////////////// timing (same as MiSTer's template core) //////////////////////////////

reg [9:0] hc = 0, vc = 0;
initial ce_pix = 0;
always @(posedge clk) begin
	ce_pix <= ~ce_pix;
	if (reset) begin hc <= 0; vc <= 0; end
	else if (ce_pix) begin
		if (hc == 637) begin
			hc <= 0;
			vc <= (vc == 261) ? 10'd0 : vc + 1'd1;
		end
		else hc <= hc + 1'd1;
	end
end

always @(posedge clk) if (ce_pix) begin
	if (hc == 529) hblank <= 1; else if (hc == 0) hblank <= 0;
	if (hc == 544) begin
		hsync <= 1;
		if (vc == 245) vsync <= 1; else if (vc == 248) vsync <= 0;
		if (vc == 240) vblank <= 1; else if (vc == 0) vblank <= 0;
	end
	if (hc == 590) hsync <= 0;
end

////////////////////////////// text //////////////////////////////

reg [5:0] msgs[0:2047];
initial $readmemh("rtl/messages.hex", msgs);

function [5:0] digit(input [10:0] v, input [1:0] pos);   // pos 0 = thousands
	reg [10:0] t;
	begin
		case (pos)
			0: t = v / 1000;
			1: t = (v / 100) % 10;
			2: t = (v / 10) % 10;
			3: t = v % 10;
		endcase
		digit = 6'd16 + t[5:0];   // '0' - 32
	end
endfunction

// the next pixel's cell
wire [9:0] nx = hc + 1'd1 - 10'd8;   // 8 pixels of border on the left
wire [5:0] ncol = nx[8:3];
wire [4:0] nrow = vc[7:3];

// which port's block this row is in, and the row within it (7 = none)
wire        sp   = (nrow >= 13);
wire  [2:0] prow = (nrow >= 7 && nrow <= 11) ? 3'(nrow - 5'd7) : (nrow >= 13 && nrow <= 17) ? 3'(nrow - 5'd13) : 3'd7;
wire        s_on   = sp ? pad_on[1] : pad_on[0];
wire  [7:0] s_id   = sp ? pad_id[15:8] : pad_id[7:0];
wire [31:0] s_ana  = sp ? pad_ana[63:32] : pad_ana[31:0];
wire  [1:0] s_card = sp ? pad_card[3:2] : pad_card[1:0];
wire        s_sticks = s_id[3:0] >= 3;   // the ID says sticks are sent

reg  [4:0] msg;       // message on this row
reg  [5:0] mcol;      // column where it starts
reg        is_num;
reg [10:0] num;
reg  [1:0] npos;
reg        blank;

always @(*) begin
	msg = M_NONE; mcol = 2; is_num = 0; num = 0; npos = 0; blank = 0;
	if (prow != 7) case (prow)
		0: if (ncol < 10) msg = sp ? M_P2 : M_P1; else begin mcol = 10;
				msg = !s_on ? M_PD_NONE : (s_id[7:4] == 4) ? M_PD_DIG : (s_id[7:4] == 7) ? M_PD_DS : (s_id[7:4] == 5) ? M_PD_ANA :
				      (s_id[7:4] == 2) ? M_PD_NEG : (s_id[7:4] == 1) ? M_PD_MOUSE : (s_id[7:4] == 6) ? M_PD_GUN : M_PD_OTHER; end
		1: if (s_on) msg = M_B1; else blank = 1;
		2: if (s_on) msg = M_B2; else blank = 1;
		3: if (!s_on) blank = 1;
			else if (!s_sticks) msg = M_NOSTICKS;
			else begin
				msg = M_STICKS;
				if (ncol >= 2 + SX_LX && ncol <= 4 + SX_LX) begin is_num = 1; num = s_ana[23:16]; npos = ncol - 6'(1 + SX_LX); end
				if (ncol >= 2 + SX_LY && ncol <= 4 + SX_LY) begin is_num = 1; num = s_ana[31:24]; npos = ncol - 6'(1 + SX_LY); end
				if (ncol >= 2 + SX_RX && ncol <= 4 + SX_RX) begin is_num = 1; num = s_ana[7:0];   npos = ncol - 6'(1 + SX_RX); end
				if (ncol >= 2 + SX_RY && ncol <= 4 + SX_RY) begin is_num = 1; num = s_ana[15:8];  npos = ncol - 6'(1 + SX_RY); end
			end
		4: if (ncol < 15) msg = M_MC; else begin mcol = 15;
				msg = (s_card == 1) ? M_MC_YES : (s_card == 2) ? M_MC_BLANK : M_MC_NO; end
		default: blank = 1;
	endcase
	else case (nrow)
		1:  msg = M_TITLE;
		2:  begin msg = M_RULE; mcol = 1; end
		4:  msg = M_READING;
		5:  if (tool) msg = M_TOOL; else blank = 1;
		20: begin
				msg = M_WIRE;
				if (ncol >= 2 + WX_P1 && ncol <= 4 + WX_P1) begin is_num = 1; num = wdiag[7:0];   npos = ncol - 6'(1 + WX_P1); end
				if (ncol >= 2 + WX_P2 && ncol <= 4 + WX_P2) begin is_num = 1; num = wdiag[15:8];  npos = ncol - 6'(1 + WX_P2); end
				if (ncol >= 2 + WX_P6 && ncol <= 4 + WX_P6) begin is_num = 1; num = wdiag[23:16]; npos = ncol - 6'(1 + WX_P6); end
				if (ncol >= 2 + WX_ID && ncol <= 4 + WX_ID) begin is_num = 1; num = wdiag[24] ? wdiag[39:32] : 11'd0; npos = ncol - 6'(1 + WX_ID); end
			end
		22: msg = analog ? M_AN_ON : M_AN_OFF;
		29: msg = M_HELP;
		default: blank = 1;
	endcase
	if (ncol < mcol || nx[9]) blank = 1;
end

wire [5:0] off = ncol - mcol;
reg  [5:0] ch;
always @(posedge clk) if (ce_pix) begin
	if (blank) ch <= 0;
	else if (is_num) ch <= ((npos == 0 && num < 1000) || (npos == 1 && num < 100) || (npos == 2 && num < 10)) ? 6'd0 : digit(num, npos);
	else ch <= msgs[{msg, off}];
end

wire [7:0] font_q;
font font(.ch(ch), .row(vc[2:0]), .q(font_q));

////////////////////////////// colours //////////////////////////////

wire [9:0] x = hc - 10'd8;
wire [4:0] row = vc[7:3];
wire       pix = font_q[~x[2:0]];
// the button under this pixel (button rows 8-9 port 1, 14-15 port 2)
wire [4:0]  pbit = btn_bit(row == 9 || row == 15, x[8:3] - 6'd2);
wire [15:0] row_btn = (row >= 13) ? pad_btn[31:16] : pad_btn[15:0];

reg [23:0] c;
always @(*) begin
	c = 24'h101428;   // background
	if (pix) begin
		case (row)
			1, 7, 13: c = 24'h60D0FF;
			8, 9, 14, 15: c = (pbit != 0 && row_btn[pbit - 1'd1]) ? 24'h48E070 : 24'h6A7290;   // pressed: green
			5:  c = 24'h48E070;
			20, 22, 29: c = 24'h9098B0;
			default: c = 24'hE8ECF4;
		endcase
	end
end

always @(posedge clk) if (ce_pix) begin
	{r, g, b} <= (hblank || vblank) ? 24'h000000 : c;
end

endmodule
