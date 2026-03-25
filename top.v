module top (
    input  [31:0] in1,
    input  [31:0] in2,
    output [31:0] out
);

    wire [31:0] div_res;

    DW_div #(.a_width(32), .b_width(32), .tc_mode(0)) 
    U_DIV (
        .a(in1), 
        .b(in2), 
        .quotient(div_res), 
        .remainder()
    );

    DW_sqrt #(.width(32), .tc_mode(0)) 
    U_SQRT (
        .a(div_res), 
        .root(out)
    );

endmodule