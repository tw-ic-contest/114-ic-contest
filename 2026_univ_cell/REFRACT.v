module REFRACT(
    input  wire        CLK,
    input  wire        RST,
    input  wire [3:0]  RI,   
    output reg  [8:0]  SRAM_A,
    output reg  [15:0] SRAM_D,
    input  wire [15:0] SRAM_Q,   // unused
    output reg         SRAM_WE,
    output reg         DONE
);


reg [3:0] state;
reg [3:0] next_state;
reg [8:0] iteration;
reg [3:0] x_idx;
reg [3:0] y_idx;

reg signed [16:0] big_x;
reg signed [16:0] big_y;

reg signed [15:0] x8; // 2 * ((X - 8) / 8) ^ 8
reg signed [15:0] y8; // 2 * ((Y - 8) / 8) ^ 8
reg signed [16:0] big_z; // z = 6 - 2 * ((X - 8) / 8) ^ 8 - 2 * ((Y - 8) / 8) ^ 8

reg signed [15:0] gx; // 2 * ((X - 8) / 8) ^ 7
reg signed [15:0] gy; // 2 * ((X - 8) / 8) ^ 7
reg [15:0] gx2; // gx ^ 2
reg [15:0] gy2; // gy ^ 2
reg signed [16:0] g2; // g ^ 2 = gx ^ 2 + gy ^ 2 + 1

reg [15:0] eta;
reg [15:0] eta2;

reg signed [31:0] kgg;
reg signed [15:0] sqrt_kgg_r;
reg signed [16:0] coef; 
reg signed [16:0] t;

reg signed [16:0] z_x;
reg signed [16:0] z_y;

// -----------------------------
// combinational helper wires
// -----------------------------
wire signed [16:0] big_z_w;
wire signed [16:0] g2_w;

wire signed [33:0] mul_eta2_g2_w;
wire signed [16:0] eta2_g2_q412_w;
wire signed [31:0] kgg_w;

wire signed [16:0] eta_m_sqrt_kgg_w;
wire signed [16:0] eta_m_coef_w;

wire signed [31:0] sqrt_in_w;
wire [31:0] sqrt_kgg_w;

wire signed [31:0] coef_num_w;
wire signed [31:0] coef_den_w;
wire signed [31:0] coef_w;

wire signed [31:0] t_num_w;
wire signed [31:0] t_den_w;
wire signed [31:0] t_w;

wire signed [33:0] t_mul_coef_w;
wire signed [16:0] t_coef_q412_w;

wire signed [33:0] gx_mul_t_mul_coef_w;
wire signed [33:0] gy_mul_t_mul_coef_w;
wire signed [16:0] gx_t_coef_q412_w;
wire signed [16:0] gy_t_coef_q412_w;

wire signed [16:0] z_x_w;
wire signed [16:0] z_y_w;


// -----------------------------
// stage-A combinational wires
// get big_z, g2, and kgg
// -----------------------------
assign big_z_w = 17'sd24576 - $signed({1'b0, x8}) - $signed({1'b0, y8}); // Z = 6 - x8 - y8

assign g2_w = $signed({1'b0, gx2}) + $signed({1'b0, gy2}) + 17'sd4096; // g^2 = gx^2 + gy^2 + 1

assign mul_eta2_g2_w = $signed(eta2) * $signed(g2_w); // eta2 * g2 (Q4.12 * Q4.12 = Q8.24)

assign eta2_g2_q412_w = mul_eta2_g2_w >>> 12; // eta2 * g2 (back to Q4.12)

assign kgg_w = g2_w - eta2_g2_q412_w + 17'sd4096; // kgg = g2 - eta2 * g2 + 1



// -----------------------------
// sqrt
// get sqrt_kgg
// input shift so sqrt output stays around Q4.12
// -----------------------------
assign sqrt_in_w = kgg_w <<< 12; //shift to Q8.24 before sqrt

DW_sqrt #(.width(32), .tc_mode(0)) 
U_SQRT (.a(sqrt_in_w), .root(sqrt_kgg_w));



// -----------------------------
// coef divider
// coef = (eta - sqrt_kgg) / g2
// keep quotient in Q4.12 => numerator << 12
// -----------------------------

assign eta_m_sqrt_kgg_w = $signed({1'b0, eta}) - $signed(sqrt_kgg_w[15:0]); // eta - sqrt_kgg

assign coef_num_w = $signed(eta_m_sqrt_kgg_w) <<< 12; //shift for numeration of divider (Q4.12 -> Q8.24)

assign coef_den_w = $signed({{15{g2[16]}}, g2}); // assign g^2 for denominator

DW_div #(.a_width(32), .b_width(32), .tc_mode(1)) 
U_DIV1 (.a(coef_num_w), .b(coef_den_w), .quotient(coef_w), .remainder());


// -----------------------------
// t divider
// t = big_z / (eta - coef)
// keep quotient in Q4.12 => numerator << 12
// -----------------------------
assign eta_m_coef_w = $signed({1'b0, eta}) - $signed(coef); // eta - coef

assign t_num_w = $signed(big_z) <<< 12;

assign t_den_w = $signed(eta_m_coef_w);

DW_div #(.a_width(32), .b_width(32), .tc_mode(1)) 
U_DIV2 (.a(t_num_w), .b(t_den_w), .quotient(t_w), .remainder());



// -----------------------------
// final zx / zy
// z = big_x/y + t * coef * g
// multiply twice, each time >> 12
// -----------------------------
assign t_mul_coef_w   = $signed(t) * $signed(coef); // t * coef
assign t_coef_q412_w  = t_mul_coef_w >>> 12; // Q8.24 -> Q4.12

assign gx_mul_t_mul_coef_w = $signed(t_coef_q412_w) * $signed(gx); // t * coef * gx
assign gy_mul_t_mul_coef_w = $signed(t_coef_q412_w) * $signed(gy); // t * coef * gy

assign gx_t_coef_q412_w = gx_mul_t_mul_coef_w >>> 12; // Q8.24 -> Q4.12
assign gy_t_coef_q412_w = gy_mul_t_mul_coef_w >>> 12; // Q8.24 -> Q4.12

assign z_x_w = big_x + gx_t_coef_q412_w;
assign z_y_w = big_y + gy_t_coef_q412_w;




always @(posedge CLK or posedge RST) begin
    if (RST) begin
        state   <= 4'd0;
        SRAM_A  <= 9'd0;
        SRAM_D  <= 16'd0;
        SRAM_WE <= 1'd0;
        DONE    <= 1'd0;
        iteration <= 9'd0;
        x_idx <= 4'd0;
        y_idx <= 4'd0;
        z_x        <= 17'd0;
        z_y        <= 17'd0;
        big_z <= 16'd0;
        
        //add all the variables later

    end
    else begin
        state <= next_state;

        case (state)
            4'd0: begin // INIT
                iteration <= 9'd0;
                DONE <= 1'b0;
            end

            4'd1: begin // GET_COOR
                x_idx <= iteration[3:0];
                y_idx <= iteration[7:4];

                big_x <= $signed({1'b0, iteration[3:0], 12'd0}); // x * 4096, Q4.12
                big_y <= $signed({1'b0, iteration[7:4], 12'd0}); // y * 4096, Q4.12
            end

            4'd2: begin // COMPUTING
                big_z <= big_z_w;
                g2    <= g2_w;
                kgg   <= kgg_w;
            end    
               
               
            4'd3: begin //COMPUTING_2
                sqrt_kgg_r <= sqrt_kgg_w[15:0];
                coef       <= coef_w[16:0];
            end

            4'd4: begin //COMPUTING_3
                t   <= t_w[16:0];
            end            
                        
            4'd5: begin //COMPUTING_4
                z_x <= z_x_w;
                z_y <= z_y_w;
            end

            4'd6: begin // WRITE_X
                SRAM_WE <= 1'b1;
                SRAM_A <= iteration << 1;
                SRAM_D <= z_x[15:0];
            end

            4'd7: begin // WRITE_Y
                SRAM_WE <= 1'b1;
                SRAM_A <= (iteration << 1) + 9'd1;
                SRAM_D <= z_y[15:0];
            end

            4'd8: begin // NEXT
                SRAM_WE <= 1'b0;
                if (iteration != 9'd255)
                    iteration <= iteration + 9'd1;

                    $display("(%0d,%0d) -> (%f,%f) | kgg=%f sqrt=%f coef=%f t=%f",
                        x_idx,
                        y_idx,
                        $itor(z_x) / 4096.0,
                        $itor(z_y) / 4096.0,
                        $itor(kgg) / 4096.0,
                        $itor(sqrt_kgg_r) / 4096.0,
                        $itor(coef) / 4096.0,
                        $itor(t) / 4096.0
                    );
            end

            4'd9: begin // FINISH
                SRAM_WE <= 1'b0;
                DONE <= 1'b1;
            end

            default: begin
                DONE <= 1'b0;
            end
        endcase

    end
end

always @(*) begin
    case(x_idx)
        4'd0:  x8 = 16'h2000;
        4'd1:  x8 = 16'h0aff;
        4'd2:  x8 = 16'h0334;
        4'd3:  x8 = 16'h00bf;
        4'd4:  x8 = 16'h0020;
        4'd5:  x8 = 16'h0003;
        4'd6:  x8 = 16'h0000;
        4'd7:  x8 = 16'h0000;
        4'd8:  x8 = 16'h0000;
        4'd9:  x8 = 16'h0000;
        4'd10: x8 = 16'h0000;
        4'd11: x8 = 16'h0003;
        4'd12: x8 = 16'h0020;
        4'd13: x8 = 16'h00bf;
        4'd14: x8 = 16'h0334;
        4'd15: x8 = 16'h0aff;
        default: x8 = 16'h0000;
    endcase
end

always @(*) begin
    case(y_idx)
        4'd0:  y8 = 16'h2000;
        4'd1:  y8 = 16'h0aff;
        4'd2:  y8 = 16'h0334;
        4'd3:  y8 = 16'h00bf;
        4'd4:  y8 = 16'h0020;
        4'd5:  y8 = 16'h0003;
        4'd6:  y8 = 16'h0000;
        4'd7:  y8 = 16'h0000;
        4'd8:  y8 = 16'h0000;
        4'd9:  y8 = 16'h0000;
        4'd10: y8 = 16'h0000;
        4'd11: y8 = 16'h0003;
        4'd12: y8 = 16'h0020;
        4'd13: y8 = 16'h00bf;
        4'd14: y8 = 16'h0334;
        4'd15: y8 = 16'h0aff;
        default: y8 = 16'h0000;
    endcase
end

always @(*) begin
    case(x_idx)
        4'd0:  gx = 16'he000; // -2.0000
        4'd1:  gx = 16'hf36f; // -0.7854
        4'd2:  gx = 16'hfbba; // -0.2670
        4'd3:  gx = 16'hfecf; // -0.0745
        4'd4:  gx = 16'hffc0; // -0.0156
        4'd5:  gx = 16'hfff7; // -0.0021
        4'd6:  gx = 16'h0000; // ~0
        4'd7:  gx = 16'h0000; // ~0
        4'd8:  gx = 16'h0000; // 0.0000
        4'd9:  gx = 16'h0000; // ~0
        4'd10: gx = 16'h0000; // ~0
        4'd11: gx = 16'h0009; // 0.0021
        4'd12: gx = 16'h0040; // 0.0156
        4'd13: gx = 16'h0131; // 0.0745
        4'd14: gx = 16'h0446; // 0.2670
        4'd15: gx = 16'h0c91; // 0.7854
        default: gx = 16'h0000;
    endcase
end

always @(*) begin
    case(y_idx)
        4'd0:  gy = 16'he000;
        4'd1:  gy = 16'hf36f;
        4'd2:  gy = 16'hfbba;
        4'd3:  gy = 16'hfecf;
        4'd4:  gy = 16'hffc0;
        4'd5:  gy = 16'hfff7;
        4'd6:  gy = 16'h0000;
        4'd7:  gy = 16'h0000;
        4'd8:  gy = 16'h0000;
        4'd9:  gy = 16'h0000;
        4'd10: gy = 16'h0000;
        4'd11: gy = 16'h0009;
        4'd12: gy = 16'h0040;
        4'd13: gy = 16'h0131;
        4'd14: gy = 16'h0446;
        4'd15: gy = 16'h0c91;
        default: gy = 16'h0000;
    endcase
end

always @(*) begin
    case(x_idx)
        4'd0:  gx2 = 16'h4000;
        4'd1:  gx2 = 16'h09df;
        4'd2:  gx2 = 16'h0124;
        4'd3:  gx2 = 16'h0017;
        4'd4:  gx2 = 16'h0001;
        4'd5:  gx2 = 16'h0000;
        4'd6:  gx2 = 16'h0000;
        4'd7:  gx2 = 16'h0000;
        4'd8:  gx2 = 16'h0000;
        4'd9:  gx2 = 16'h0000;
        4'd10: gx2 = 16'h0000;
        4'd11: gx2 = 16'h0000;
        4'd12: gx2 = 16'h0001;
        4'd13: gx2 = 16'h0017;
        4'd14: gx2 = 16'h0124;
        4'd15: gx2 = 16'h09df;
        default: gx2 = 16'h0000;
    endcase
end

always @(*) begin
    case(y_idx)
        4'd0:  gy2 = 16'h4000;
        4'd1:  gy2 = 16'h09df;
        4'd2:  gy2 = 16'h0124;
        4'd3:  gy2 = 16'h0017;
        4'd4:  gy2 = 16'h0001;
        4'd5:  gy2 = 16'h0000;
        4'd6:  gy2 = 16'h0000;
        4'd7:  gy2 = 16'h0000;
        4'd8:  gy2 = 16'h0000;
        4'd9:  gy2 = 16'h0000;
        4'd10: gy2 = 16'h0000;
        4'd11: gy2 = 16'h0000;
        4'd12: gy2 = 16'h0001;
        4'd13: gy2 = 16'h0017;
        4'd14: gy2 = 16'h0124;
        4'd15: gy2 = 16'h09df;
        default: gy2 = 16'h0000;
    endcase
end
    
always @(*) begin
    case(RI)
        4'd2:  eta = 16'h0800;
        4'd3:  eta = 16'h0555;
        4'd4:  eta = 16'h0400;
        4'd5:  eta = 16'h0333;
        4'd6:  eta = 16'h02ab;
        4'd7:  eta = 16'h0249;
        4'd8:  eta = 16'h0200;
        4'd9:  eta = 16'h01c7;
        4'd10: eta = 16'h019a;
        4'd11: eta = 16'h0174;
        4'd12: eta = 16'h0155;
        4'd13: eta = 16'h013b;
        4'd14: eta = 16'h0125;
        4'd15: eta = 16'h0111;
        default: eta = 16'h0000;
    endcase
end

always @(*) begin
    case(RI)
        4'd2:  eta2 = 16'h0400;
        4'd3:  eta2 = 16'h01c7;
        4'd4:  eta2 = 16'h0100;
        4'd5:  eta2 = 16'h00a4;
        4'd6:  eta2 = 16'h0072;
        4'd7:  eta2 = 16'h0054;
        4'd8:  eta2 = 16'h0040;
        4'd9:  eta2 = 16'h0033;
        4'd10: eta2 = 16'h0029;
        4'd11: eta2 = 16'h0022;
        4'd12: eta2 = 16'h001c;
        4'd13: eta2 = 16'h0018;
        4'd14: eta2 = 16'h0015;
        4'd15: eta2 = 16'h0012;
        default: eta2 = 16'h0000;
    endcase
end

always @(*) begin
    case (state)
        4'd0: next_state = 4'd1;
        4'd1: next_state = 4'd2;
        4'd2: next_state = 4'd3;
        4'd3: next_state = 4'd4;
        4'd4: next_state = 4'd5;
        4'd5: next_state = 4'd6; 
        4'd6: next_state = 4'd7;
        4'd7: next_state = 4'd8;
        4'd8: next_state = (iteration == 9'd255) ? 4'd9 : 4'd1;
        4'd9: next_state = 4'd9;
        default: next_state = 4'd0;
    endcase
end

endmodule