// SS1 Controller Tester
// Copyright (C) 2026 f3bandit
// MIT License (see LICENSE-MIT).
//
// pad_ctrl: reads the controllers in the SuperStation One's front ports 1 and 2 (SNAC).
//
// From the moment the core loads it reads both ports in turn, about 60 times a second each, and
// looks for a memory card in each port about once a second. A controller that answers as a
// digital pad (ID 41) is sent the DualShock commands for analog mode, and again about once a
// second while it stays digital (a wireless receiver answers as a digital pad before its
// controller has paired), so the sticks and L3/R3 come through. Twice a second it also tries
// USER_OUT[6] as a select (the wiring check) and records which input pins something pulls low
// during each read.
//
// SS1 Tool mounts a report file (drive 0, 512 bytes or more). Block 0 of it is read once (the
// request: "SCTQ" and a 4-byte id at 8), then rewritten about 12 times a second:
//   0  "SCTR"         4  version          8  id (4 bytes, from the request)
//   32 port 1: port (1), controller there (0/1), its ID, buttons (16-bit LE, pressed = 1:
//      0 Select 1 L3 2 R3 3 Start 4 Up 5 Right 6 Down 7 Left 8 L2 9 R2 10 L1 11 R1 12 Triangle
//      13 Circle 14 Cross 15 Square), RX RY LX LY, reads so far, memory card (0 none,
//      1 formatted, 2 not formatted), analog mode asked for (0/1)
//   44 analog switches sent so far
//   48 port 2: the same as port 1 (port is 2)
//   64 wiring check: pins pulled low (bit n = USER_IN[n]) reading port 1, port 2 (65), with
//      USER_OUT[6] as the select (66); a controller answered there (67), its ID (68)

module pad_ctrl
#(
	// times in 20 MHz clock cycles (the testbench shortens them)
	parameter HALF      = 39,       // 250 kHz bus
	parameter ACK_TO    = 4000,     // 200 us
	parameter SEL_DLY   = 400,      // 20 us
	parameter PAD_GAP   = 165000,   // a read every ~8 ms, the two ports in turn
	parameter REPORT    = 1666666   // the report into the file ~12 times a second
)
(
	input             clk,
	input             reset,
	input             opt_analog,     // switch digital pads to analog mode

	// MiSTer file interface (hps_io, 512-byte blocks): the report file
	input             img_mounted,
	input      [63:0] img_size,
	output reg [31:0] sd_lba,
	output reg        sd_rd,
	output reg        sd_wr,
	input             sd_ack,
	input       [8:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	input             sd_buff_wr,
	output reg  [7:0] sd_buff_din,

	// the front ports
	output reg        port2,        // the port being read: 0 port 1, 1 port 2
	output reg        pin6,         // wiring check: select on USER_OUT[6] instead
	input       [6:0] bus_low,      // USER_IN pins something else pulls low

	// for the screen: port 1 in the low half, port 2 in the high half
	output reg  [1:0] pad_on,       // a controller answers
	output reg [15:0] pad_id,
	output reg [31:0] pad_btn,
	output reg [63:0] pad_ana,      // RX RY LX LY
	output reg  [3:0] pad_card,
	output     [39:0] wdiag,        // {pin 6 ID, pin 6 answered, pins low: pin 6, port 2, port 1}
	output reg        tool,         // SS1 Tool's report file is mounted

	output            sio_sel_n,
	output            sio_clk,
	output            sio_cmd,
	input             sio_dat,
	input             sio_ack
);

localparam VERSION = 1;
localparam R_OK = 0, R_NOCARD = 1, R_BAD = 2, R_RANGE = 3;
localparam C_NONE = 0, C_FORMATTED = 1, C_BLANK = 2;

////////////////////////////// the bus //////////////////////////////

reg        cs_start, cs_pad;
reg  [1:0] cs_pcmd;
wire       cs_done;
wire [1:0] cs_result;
wire [7:0] cs_hdr0, cs_hdr1, cs_pid;
wire [15:0] cs_pbtn;
wire [31:0] cs_pana;

card_sector card_sector
(
	.clk(clk), .reset(reset),
	.half_period(16'(HALF)), .ack_timeout(16'(ACK_TO)), .sel_delay(16'(SEL_DLY)),
	.start(cs_start), .pad(cs_pad), .pcmd(cs_pcmd), .sector(10'd0), .mode(2'd2),
	.busy(), .done(cs_done), .result(cs_result), .mismatch(),
	.hdr0(cs_hdr0), .hdr1(cs_hdr1), .pad_id(cs_pid), .pad_btn(cs_pbtn), .pad_ana(cs_pana),
	.mem_addr(), .mem_wdata(), .mem_we(), .mem_rdata(8'd0),
	.sio_sel_n(sio_sel_n), .sio_clk(sio_clk), .sio_cmd(sio_cmd), .sio_dat(sio_dat), .sio_ack(sio_ack)
);

////////////////////////////// reading the ports //////////////////////////////

reg        st;           // 0 waiting for the next read, 1 a read on the bus
reg [23:0] tmr;
reg        pp;           // the port being read
reg  [5:0] pad_n1, pad_n2;   // reads until the next look for a memory card in that port
reg  [1:0] cfg1, cfg2;   // switching that port's controller to analog: the next step (0 none)
reg  [5:0] swc1, swc2;   // reads until the switch is sent again while that pad stays digital
reg  [1:0] tried;        // the switch was sent to that port's controller
reg  [7:0] sw_cnt;       // analog switches sent
reg  [7:0] pad_cnt;      // reads so far (port 1)
reg  [6:0] acc, act1, act2, act6;
reg        on6, scanning;
reg  [7:0] id6;
reg  [4:0] scn;          // reads until the next pin 6 try
assign wdiag = {id6, 7'd0, on6, 1'b0, act6, 1'b0, act2, 1'b0, act1};

wire [1:0] cfg_pp = pp ? cfg2 : cfg1;
wire [5:0] pn_pp  = pp ? pad_n2 : pad_n1;
wire [5:0] swc_pp = pp ? swc2 : swc1;
wire [1:0] card_now = (cs_result == R_NOCARD) ? 2'(C_NONE) : ((cs_hdr0 == "M" && cs_hdr1 == "C") ? 2'(C_FORMATTED) : 2'(C_BLANK));

always @(posedge clk) begin
	cs_start <= 0;
	if (tmr) tmr <= tmr - 1'd1;

	if (reset) begin
		st <= 0; tmr <= 24'(PAD_GAP); pp <= 0; port2 <= 0; pin6 <= 0;
		pad_n1 <= 0; pad_n2 <= 0; cfg1 <= 0; cfg2 <= 0; swc1 <= 0; swc2 <= 0; tried <= 0;
		sw_cnt <= 0; pad_cnt <= 0; pad_on <= 0; pad_btn <= 0; pad_id <= 0;
		pad_ana <= {8{8'h80}}; pad_card <= 0;
		act1 <= 0; act2 <= 0; act6 <= 0; on6 <= 0; id6 <= 0; scn <= 5'd4; scanning <= 0;
	end
	else if (!st) begin
		if (!tmr) begin
			scanning <= (scn == 0);
			pin6     <= (scn == 0);
			port2    <= pp;
			acc      <= 0;
			cs_pad   <= (scn == 0) || (pn_pp != 0) || (cfg_pp != 0);
			cs_pcmd  <= (scn == 0) ? 2'd0 : cfg_pp;
			cs_start <= 1;
			st       <= 1;
		end
	end
	else if (!cs_done) begin
		if (!sio_sel_n) acc <= acc | bus_low;
	end
	else begin
		st   <= 0;
		tmr  <= 24'(PAD_GAP);
		pin6 <= 0;
		if (scanning) begin
			// the wiring check's read: record it, don't touch the ports
			act6 <= acc;
			on6  <= (cs_result == R_OK);
			id6  <= (cs_result == R_OK) ? cs_pid : 8'd0;
			scn  <= 5'd30;
		end
		else begin
			if (scn != 0) scn <= scn - 1'd1;
			if (pp) act2 <= acc; else act1 <= acc;
			pp <= ~pp;
			if (cs_pad && cs_pcmd != 0) begin
				// a step of the switch to analog: on to the next one
				if (pp) cfg2 <= (cfg2 == 3) ? 2'd0 : cfg2 + 1'd1;
				else    cfg1 <= (cfg1 == 3) ? 2'd0 : cfg1 + 1'd1;
			end
			else if (cs_pad) begin
				if (!pp) pad_cnt <= pad_cnt + 1'd1;
				if (pp) pad_n2 <= pad_n2 - 1'd1; else pad_n1 <= pad_n1 - 1'd1;
				if (pp) pad_on[1] <= (cs_result == R_OK); else pad_on[0] <= (cs_result == R_OK);
				if (cs_result == R_OK) begin
					if (pp) begin
						pad_id[15:8]   <= cs_pid;
						pad_btn[31:16] <= ~cs_pbtn;
						pad_ana[63:32] <= (cs_pid[3:0] >= 3) ? cs_pana : 32'h80808080;
					end
					else begin
						pad_id[7:0]    <= cs_pid;
						pad_btn[15:0]  <= ~cs_pbtn;
						pad_ana[31:0]  <= (cs_pid[3:0] >= 3) ? cs_pana : 32'h80808080;
					end
					// a digital pad that may be a DualShock: ask for analog mode, and again about
					// once a second while it stays digital
					if (cs_pid == 8'h41 && opt_analog) begin
						if (swc_pp == 0) begin
							if (pp) begin cfg2 <= 1; swc2 <= 6'd60; end else begin cfg1 <= 1; swc1 <= 6'd60; end
							tried[pp] <= 1;
							sw_cnt    <= sw_cnt + 1'd1;
						end
						else if (pp) swc2 <= swc2 - 1'd1; else swc1 <= swc1 - 1'd1;
					end
					// left in config mode somehow: leave it
					if (cs_pid == 8'hF3) begin
						if (pp) cfg2 <= 3; else cfg1 <= 3;
					end
				end
				else begin
					if (pp) begin pad_btn[31:16] <= 0; pad_ana[63:32] <= 32'h80808080; end
					else    begin pad_btn[15:0]  <= 0; pad_ana[31:0]  <= 32'h80808080; end
					if (cs_result == R_NOCARD) begin   // unplugged: switch it straight away when it's back
						tried[pp] <= 0;
						if (pp) swc2 <= 0; else swc1 <= 0;
					end
				end
			end
			else begin
				// the look for a memory card (a broken reply keeps what we had)
				if (cs_result == R_NOCARD || cs_result == R_OK) begin
					if (pp) pad_card[3:2] <= card_now; else pad_card[1:0] <= card_now;
				end
				if (pp) pad_n2 <= 6'd60; else pad_n1 <= 6'd60;
			end
		end
	end
end

////////////////////////////// the report file //////////////////////////////

reg  [7:0] req_id[0:3];
reg        req_rd;       // reading the request
reg  [1:0] fs;           // file transfer state
reg [23:0] rtmr;
reg        old_ack;

always @(*) begin
	case (sd_buff_addr)
		0: sd_buff_din = "S";  1: sd_buff_din = "C";  2: sd_buff_din = "T";  3: sd_buff_din = "R";
		4: sd_buff_din = VERSION;
		8: sd_buff_din = req_id[0];  9: sd_buff_din = req_id[1];  10: sd_buff_din = req_id[2];  11: sd_buff_din = req_id[3];
		32: sd_buff_din = 1;              48: sd_buff_din = 2;
		33: sd_buff_din = pad_on[0];      49: sd_buff_din = pad_on[1];
		34: sd_buff_din = pad_id[7:0];    50: sd_buff_din = pad_id[15:8];
		35: sd_buff_din = pad_btn[7:0];   51: sd_buff_din = pad_btn[23:16];
		36: sd_buff_din = pad_btn[15:8];  52: sd_buff_din = pad_btn[31:24];
		37: sd_buff_din = pad_ana[7:0];   53: sd_buff_din = pad_ana[39:32];
		38: sd_buff_din = pad_ana[15:8];  54: sd_buff_din = pad_ana[47:40];
		39: sd_buff_din = pad_ana[23:16]; 55: sd_buff_din = pad_ana[55:48];
		40: sd_buff_din = pad_ana[31:24]; 56: sd_buff_din = pad_ana[63:56];
		41: sd_buff_din = pad_cnt;        57: sd_buff_din = pad_cnt;
		42: sd_buff_din = pad_card[1:0];  58: sd_buff_din = pad_card[3:2];
		43: sd_buff_din = tried[0];       59: sd_buff_din = tried[1];
		44: sd_buff_din = sw_cnt;
		64: sd_buff_din = act1;  65: sd_buff_din = act2;  66: sd_buff_din = act6;  67: sd_buff_din = on6;  68: sd_buff_din = id6;
		default: sd_buff_din = 0;
	endcase
end

always @(posedge clk) begin
	old_ack <= sd_ack;
	if (rtmr) rtmr <= rtmr - 1'd1;

	if (reset) begin
		tool <= 0; sd_rd <= 0; sd_wr <= 0; req_rd <= 0; fs <= 0; rtmr <= 0;
		req_id[0] <= 0; req_id[1] <= 0; req_id[2] <= 0; req_id[3] <= 0;
	end
	else begin
		if (img_mounted) begin
			tool  <= (img_size >= 512);
			sd_wr <= 0;
			fs    <= 0;
			if (img_size >= 512) begin
				// read the request first: the id goes back in every report
				sd_lba <= 0;
				sd_rd  <= 1;
				req_rd <= 1;
				req_id[0] <= 0; req_id[1] <= 0; req_id[2] <= 0; req_id[3] <= 0;
			end
		end
		if (sd_ack && sd_buff_wr && req_rd && sd_buff_addr >= 8 && sd_buff_addr < 12) req_id[sd_buff_addr[1:0]] <= sd_buff_dout;
		if (sd_ack) sd_rd <= 0;
		if (old_ack && !sd_ack && req_rd) begin req_rd <= 0; rtmr <= 0; end

		// the report, about 12 times a second
		case (fs)
			0: if (tool && !req_rd && !sd_rd && !rtmr) begin sd_lba <= 0; sd_wr <= 1; fs <= 1; end
			1: if (sd_ack) begin sd_wr <= 0; fs <= 2; end
			2: if (!sd_ack) begin rtmr <= 24'(REPORT); fs <= 0; end
			default: fs <= 0;
		endcase
	end
end

endmodule
