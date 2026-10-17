// Renders one frame of the screen to frame.ppm with made-up controller values.
`timescale 1ns/1ns
module tb_screen;
	reg clk = 0; always #25 clk = ~clk;
	wire ce, hb, vb, hs, vs; wire [7:0] r, g, b;
	// port 1: DualShock, Up + Cross + Select, sticks moved, card; port 2: digital pad, Start held
	screen dut(.clk(clk), .reset(1'b0), .pad_on(2'b11), .pad_id(16'h4173), .pad_btn({16'h0008, 16'h4011}),
		.pad_ana({32'h80808080, 32'h10FF8040}), .pad_card(4'b0001), .wdiag(40'h0001181818), .tool(1'b1), .analog(1'b1),
		.ce_pix(ce), .hblank(hb), .vblank(vb), .hsync(hs), .vsync(vs), .r(r), .g(g), .b(b));
	integer f, x, y;
	initial begin
		wait (vb); wait (!vb);
		f = $fopen("frame.ppm", "w");
		$fwrite(f, "P3\n512 240\n255\n");
		for (y = 0; y < 240; y = y + 1) begin
			x = 0;
			while (x < 512) begin
				@(posedge clk);
				if (ce && !hb && dut.hc >= 9 && dut.hc < 9 + 512) begin
					$fwrite(f, "%0d %0d %0d\n", r, g, b); x = x + 1;
				end
			end
			wait (hb);
			wait (!hb);
		end
		$fclose(f);
		$finish;
	end
endmodule
