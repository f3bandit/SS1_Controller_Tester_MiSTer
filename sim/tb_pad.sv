// Testbench for pad_ctrl: two modelled PlayStation controllers, one in each front port (port 1
// with a memory card), and a stand-in for MiSTer's file transfers (hps_io). Run: sh sim/run.sh
`timescale 1ns/1ns

module card_model
(
	input        sel_n,
	input        sclk,
	input        cmd,
	output reg   dat,
	output reg   ack_n
);
	reg  [7:0] mem[0:131071];
	integer    present = 1;
	integer    flaky_sector = -1;   // returns a bad checksum this many times
	integer    flaky_count = 0;
	integer    dead_sector = -1;    // always bad
	integer    ack_delay = 150;     // ns after the byte

	integer    idx;
	reg  [7:0] in_sh, out_sh, nxt;
	integer    bitn;
	reg  [9:0] addr;
	reg  [7:0] chk;
	reg        active;
	reg        wmode;
	reg        pmode;               // a controller is being read
	integer    pad_present = 1;
	reg [15:0] pad_raw = 16'hFFFF;    // active low
	reg [31:0] pad_ana = 32'h80808080;
	reg        pad_analog = 1;      // sends 73 (analog); 0: 41 (digital) until switched
	reg        pad_cfg = 0;         // in config mode (sends F3)
	reg  [7:0] pad_cmd;
	integer    pad_len;
	integer    pad_switches = 0;    // times it was put into analog mode
	integer    refuse = 0;          // ignores the analog command (a receiver with no controller yet)
	reg  [7:0] wbuf[0:127];
	reg  [7:0] wchk;
	integer    writes = 0;
	integer    bad_write_sector = -1;   // refuses the first write
	integer    bad_write_count = 0;

	initial begin dat = 1; ack_n = 1; active = 0; wmode = 0; end

	always @(negedge sel_n) begin idx = 0; bitn = 0; active = present | pad_present; out_sh = 8'hFF; wmode = 0; pmode = 0; end
	always @(posedge sel_n) begin active = 0; dat = 1; end

	// card drives DAT while CLK is low, the console samples on the rising edge
	always @(negedge sclk) if (!sel_n && active) dat = out_sh[bitn];

	always @(posedge sclk) if (!sel_n && active) begin
		in_sh = {cmd, in_sh[7:1]};
		bitn  = bitn + 1;
		if (bitn == 8) begin
			bitn = 0;
			respond();
		end
	end

	function [7:0] reply(input integer i);
		integer d;
		begin
			if (pmode) begin
				case (i)
					0: reply = pad_cfg ? 8'hF3 : pad_analog ? 8'h73 : 8'h41;
					1: reply = 8'h5A;
					2: reply = pad_raw[7:0];
					3: reply = pad_raw[15:8];
					default: reply = pad_ana[(i - 4) * 8 +: 8];
				endcase
			end
			else if (wmode && i >= 3) begin
				if (i == 134) reply = 8'h5C;
				else if (i == 135) reply = 8'h5D;
				else if (i == 136) reply = (wchk == 0) ? 8'h47 : 8'h4E;
				else reply = 8'h00;
			end
			else case (i)
				0: reply = 8'h08;       // FLAG, answered during byte 1
				1: reply = 8'h5A;
				2: reply = 8'h5D;
				3: reply = 8'h00;
				4: reply = addr[9:8];
				5: reply = 8'h5C;
				6: reply = 8'h5D;
				7: reply = addr[9:8];
				8: reply = addr[7:0];
				default: begin
					if (i >= 9 && i < 137) reply = mem[{addr, 7'd0} + i - 9];
					else if (i == 137) reply = chk;
					else reply = 8'h47;
				end
			endcase
		end
	endfunction

	task respond;
		integer i;
		begin
			// in_sh is the byte just received (index idx); out byte for idx+1 is prepared now
			if (idx == 0 && in_sh == 8'h01 && pad_present) pmode = 1;
			else if (idx == 0 && (in_sh != 8'h81 || !present)) begin active = 0; disable respond; end
			if (idx == 0 && pmode) pad_len = (!pad_cfg && !pad_analog) ? 5 : 9;
			if (idx == 1 && pmode) pad_cmd = in_sh;
			if (idx == 3 && pmode && pad_cmd == 8'h43) pad_cfg = in_sh[0];
			if (idx == 3 && pmode && pad_cmd == 8'h44 && pad_cfg && !refuse) begin pad_analog = in_sh[0]; pad_switches = pad_switches + 1; end
			if (idx == 1 && !pmode) begin
				if (in_sh == 8'h52) wmode = 0;
				else if (in_sh == 8'h57) wmode = 1;
				else begin active = 0; disable respond; end
			end
			if (wmode && idx == 4) addr[9:8] = in_sh[1:0];
			if (wmode && idx == 5) begin addr[7:0] = in_sh; wchk = addr[9:8] ^ addr[7:0]; end
			if (wmode && idx >= 6 && idx < 134) begin wbuf[idx-6] = in_sh; wchk = wchk ^ in_sh; end
			if (wmode && idx == 134) begin
				wchk = wchk ^ in_sh;   // 0 when the checksum was right
				if (addr == bad_write_sector && bad_write_count > 0) begin bad_write_count = bad_write_count - 1; end
				else if (wchk == 0) begin for (i = 0; i < 128; i = i + 1) mem[{addr, 7'd0} + i] = wbuf[i]; writes = writes + 1; end
			end
			if (!wmode && idx == 4) addr[9:8] = in_sh[1:0];
			if (!wmode && idx == 5) begin
				addr[7:0] = in_sh;
				chk = addr[9:8] ^ addr[7:0];
				for (i = 0; i < 128; i = i + 1) chk = chk ^ mem[{addr, 7'd0} + i];
				if (addr == flaky_sector && flaky_count > 0) begin chk = ~chk; flaky_count = flaky_count - 1; end
				if (addr == dead_sector) chk = ~chk;
			end
			out_sh = reply(idx);
			idx = idx + 1;
			if (idx < (pmode ? pad_len : wmode ? 138 : 140)) begin
				#(ack_delay) ack_n = 0;
				#200 ack_n = 1;
			end
		end
	endtask
endmodule

module tb;
	reg clk = 0;
	always #25 clk = ~clk;   // 20 MHz

	reg reset = 1;
	reg opt_analog = 1;
	reg        img_mounted = 0;
	reg [63:0] img_size = 0;
	wire [31:0] sd_lba;
	wire        sd_rd, sd_wr;
	reg         sd_ack = 0;
	reg   [8:0] sd_buff_addr = 0;
	reg   [7:0] sd_buff_dout = 0;
	reg         sd_buff_wr = 0;
	wire  [7:0] sd_buff_din;

	wire sel_n, sclk, scmd, dat1, ack1_n, dat2, ack2_n, port2, pin6;
	// the front ports: one select line each, DAT and ACK shared (open collector); nothing on pin 6
	wire sel1_n = (port2 || pin6) ? 1'b1 : sel_n;
	wire sel2_n = (port2 && !pin6) ? sel_n : 1'b1;
	wire dat = dat1 & dat2;
	wire ack_n = ack1_n & ack2_n;

	pad_ctrl #(.HALF(3), .ACK_TO(60), .SEL_DLY(4), .PAD_GAP(2000), .REPORT(30000)) dut
	(
		.clk(clk), .reset(reset), .opt_analog(opt_analog),
		.img_mounted(img_mounted), .img_size(img_size),
		.sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack),
		.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_wr(sd_buff_wr), .sd_buff_din(sd_buff_din),
		.port2(port2), .pin6(pin6), .bus_low(7'd0),
		.pad_on(), .pad_id(), .pad_btn(), .pad_ana(), .pad_card(), .wdiag(), .tool(),
		.sio_sel_n(sel_n), .sio_clk(sclk), .sio_cmd(scmd), .sio_dat(dat), .sio_ack(ack_n)
	);

	card_model card(.sel_n(sel1_n), .sclk(sclk), .cmd(scmd), .dat(dat1), .ack_n(ack1_n));
	card_model card2(.sel_n(sel2_n), .sclk(sclk), .cmd(scmd), .dat(dat2), .ack_n(ack2_n));
	initial begin card2.present = 0; end

	// the report file
	reg [7:0] rf[0:511];
	reg rd_req;
	task automatic serve;
		integer i;
		begin
			@(posedge clk) sd_ack <= 1;
			sd_buff_addr <= 0;
			repeat (4) @(posedge clk);
			for (i = 0; i < 512; i = i + 1) begin
				sd_buff_addr <= i;
				if (rd_req) begin
					sd_buff_dout <= rf[i];
					@(posedge clk) sd_buff_wr <= 1;
					@(posedge clk) sd_buff_wr <= 0;
					repeat (2) @(posedge clk);
				end
				else begin
					repeat (3) @(posedge clk);
					rf[i] = sd_buff_din;
				end
			end
			@(posedge clk) sd_ack <= 0;
			repeat (4) @(posedge clk);
		end
	endtask
	always @(posedge clk) if (!sd_ack && (sd_rd || sd_wr)) begin rd_req = sd_rd; serve(); end

	task mount(input integer size);
		begin
			@(posedge clk) begin img_size <= size; img_mounted <= 1; end
			@(posedge clk) img_mounted <= 0;
		end
	endtask

	integer fails = 0, i;
	task check(input [8*64-1:0] what, input ok);
		begin
			if (ok) $display("PASS %0s", what);
			else begin $display("FAIL %0s", what); fails = fails + 1; end
		end
	endtask

	initial begin
		for (i = 0; i < 512; i = i + 1) rf[i] = 0;
		for (i = 0; i < 131072; i = i + 1) card.mem[i] = 0;
		card.mem[0] = "M"; card.mem[1] = "C";
		// port 1: a DualShock in analog mode with a card; port 2: a pad that starts digital, no card
		card.pad_raw = ~16'h4011;      // Cross, Up and Select held
		card.pad_ana = 32'h10203040;   // RX 40 RY 30 LX 20 LY 10
		card2.pad_present = 1; card2.pad_analog = 0;
		card2.pad_raw = ~16'h0108;     // Start and L2
		card2.pad_ana = 32'hA0B0C0D0;
		repeat (10) @(posedge clk);
		reset = 0;

		// 1. it starts reading as soon as it's running, with no file
		repeat (400000) @(posedge clk);
		check("port 1 read without a file", dut.pad_on[0] && dut.pad_id[7:0] == 8'h73 && dut.pad_btn[15:0] == 16'h4011 && dut.pad_ana[31:0] == 32'h10203040);
		check("port 2 switched to analog", card2.pad_switches == 1 && card2.pad_analog && !card2.pad_cfg);
		check("port 2 read", dut.pad_on[1] && dut.pad_id[15:8] == 8'h73 && dut.pad_btn[31:16] == 16'h0108 && dut.pad_ana[63:32] == 32'hA0B0C0D0);
		check("memory cards: port 1 yes, port 2 no", dut.pad_card == 4'b0001);
		check("no file, no report", !dut.tool && rf[0] == 0);

		// 2. SS1 Tool's report file: the request's id comes back in every report
		rf[0] = "S"; rf[1] = "C"; rf[2] = "T"; rf[3] = "Q"; rf[8] = 8'hDE; rf[9] = 8'hAD; rf[10] = 8'hBE; rf[11] = 8'hEF;
		mount(512);
		repeat (200000) @(posedge clk);
		check("report header and id", rf[0] == "S" && rf[1] == "C" && rf[2] == "T" && rf[3] == "R" && rf[4] == 1 && rf[8] == 8'hDE && rf[11] == 8'hEF && dut.tool);
		check("both ports in the report", rf[32] == 1 && rf[33] == 1 && rf[34] == 8'h73 && rf[35] == 8'h11 && rf[36] == 8'h40 && rf[37] == 8'h40 && rf[40] == 8'h10
		      && rf[48] == 2 && rf[49] == 1 && rf[50] == 8'h73 && rf[51] == 8'h08 && rf[52] == 8'h01 && rf[53] == 8'hD0 && rf[56] == 8'hA0 && rf[59] == 1);
		check("memory cards in the report", rf[42] == 1 && rf[58] == 0);
		check("wiring check: nothing on pin 6, tried", rf[67] == 0 && dut.scn != 0);

		// 3. a receiver whose controller pairs late: refuses analog at first, accepts it later
		card2.pad_analog = 0; card2.pad_switches = 0; card2.refuse = 1;
		repeat (1000000) @(posedge clk);
		check("still digital: asked again", card2.pad_switches == 0 && dut.sw_cnt >= 3 && dut.pad_id[15:8] == 8'h41 && dut.pad_ana[63:32] == 32'h80808080);
		card2.refuse = 0;
		repeat (400000) @(posedge clk);
		check("paired later: switched to analog", card2.pad_switches == 1 && dut.pad_id[15:8] == 8'h73);

		// 4. unplugged and back
		card2.pad_present = 0;
		repeat (400000) @(posedge clk);
		check("port 2 unplugged, port 1 still read", !dut.pad_on[1] && dut.pad_btn[31:16] == 0 && rf[49] == 0 && dut.pad_on[0] && rf[33] == 1);
		card2.pad_present = 1; card2.pad_analog = 0;
		repeat (400000) @(posedge clk);
		check("plugged back in: switched again", card2.pad_switches == 2 && dut.pad_id[15:8] == 8'h73);

		// 5. "Switch pads to analog" off: a digital pad stays digital
		opt_analog = 0;
		card2.pad_analog = 0; card2.pad_switches = 0;
		repeat (600000) @(posedge clk);
		check("option off: left digital", card2.pad_switches == 0 && dut.pad_id[15:8] == 8'h41 && dut.pad_on[1]);

		if (fails) $display("%0d FAILED", fails); else $display("ALL PASSED");
		$finish;
	end

	initial begin #(64'd2000000000); $display("TIMEOUT"); $finish; end
endmodule
