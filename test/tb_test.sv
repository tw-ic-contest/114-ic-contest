`timescale 1ns/10ps

module tb;

    logic [31:0] in1;
    logic [31:0] in2;
    wire  [31:0] out;

    top dut (
        .in1(in1),
        .in2(in2),
        .out(out)
    );

    initial begin
        $display("--------------------------------------------------");
        $display("開始測試: out = sqrt(in1 / in2)");
        $display("--------------------------------------------------");

        in1 = 32'd100; in2 = 32'd4;
        #10;
        $display("[Time %0t] in1=%0d, in2=%0d | out = %0d (預期: 5)", $time, in1, in2, out);

        in1 = 32'd81; in2 = 32'd1;
        #10;
        $display("[Time %0t] in1=%0d, in2=%0d | out = %0d (預期: 9)", $time, in1, in2, out);

        in1 = 32'd200; in2 = 32'd2;
        #10;
        $display("[Time %0t] in1=%0d, in2=%0d | out = %0d (預期: 10)", $time, in1, in2, out);

        in1 = 32'd50; in2 = 32'd2;
        #10;
        $display("[Time %0t] in1=%0d, in2=%0d | out = %0d (預期: 5)", $time, in1, in2, out);

        $display("--------------------------------------------------");
        $display("測試完成！");
        $finish;
    end

endmodule
