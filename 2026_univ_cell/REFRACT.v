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

    reg [3:0] state, next_state;
    reg [8:0] iteration;
    reg signed [16:0] big_x, big_y, big_z;
    reg signed [15:0] x8, y8, gx, gy, gx2, gy2;
    reg signed [16:0] g2;
    reg [15:0] eta, eta2;
    reg signed [31:0] kgg;
    reg signed [15:0] sqrt_kgg_r;
    reg signed [16:0] coef, t, z_x, z_y;

    // --- 這裡保留你原本使用的所有 DW 元件 ---
    wire [31:0] eta_w, div1_q, div2_q, sqrt_q;
    
    // eta = 1/RI (Q4.12)
    DW_div #(.a_width(32), .b_width(32), .tc_mode(0)) 
    U_ETA (.a(32'd1 << 24), .b({28'd0, RI}), .quotient(eta_w), .remainder());

    // sqrt(k*g2)
    DW_sqrt #(.width(32), .tc_mode(0)) 
    U_SQRT (.a(kgg[31] ? 32'd0 : (kgg << 12)), .root(sqrt_q));

    // coef = (eta - sqrt) / g2
    DW_div #(.a_width(32), .b_width(32), .tc_mode(1)) 
    U_DIV1 (.a(($signed({1'b0, eta}) - $signed(sqrt_kgg_r)) << 12), .b({{15{g2[16]}}, g2}), .quotient(div1_q), .remainder());

    // t = -Z / (-eta + coef)
    DW_div #(.a_width(32), .b_width(32), .tc_mode(1)) 
    U_DIV2 (.a((-big_z) << 12), .b($signed(coef) - $signed({1'b0, eta})), .quotient(div2_q), .remainder());

    // --- 連乘積邏輯 (修正位移以保留精度) ---
    wire signed [16:0] ax = (big_x - 17'sd32768) >>> 3; 
    wire signed [16:0] ay = (big_y - 17'sd32768) >>> 3;
    wire signed [16:0] ax2 = (ax * ax) >>> 12;
    wire signed [16:0] ax4 = (ax2 * ax2) >>> 12;
    wire signed [16:0] ax7 = (ax4 * (ax * ax2 >>> 12)) >>> 12;
    wire signed [16:0] ax8 = (ax4 * ax4) >>> 12;
    wire signed [16:0] ay2 = (ay * ay) >>> 12;
    wire signed [16:0] ay4 = (ay2 * ay2) >>> 12;
    wire signed [16:0] ay7 = (ay4 * (ay * ay2 >>> 12)) >>> 12;
    wire signed [16:0] ay8 = (ay4 * ay4) >>> 12;

    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            state <= 4'd0; iteration <= 9'd0; DONE <= 1'b0; SRAM_WE <= 1'b0;
        end else begin
            state <= next_state;
            case (state)
                4'd0: begin // INIT
                    eta <= eta_w[15:0];
                    eta2 <= (eta_w[15:0] * eta_w[15:0]) >> 12;
                    iteration <= 9'd0; DONE <= 1'b0;
                end
                4'd1: begin // GET_COOR
                    big_x <= {1'b0, iteration[3:0], 12'd0};
                    big_y <= {1'b0, iteration[7:4], 12'd0};
                    SRAM_WE <= 1'b0;
                end
                4'd2: begin // COMPUTING
                    x8 <= ax8[15:0]; y8 <= ay8[15:0];
                    gx <= (ax7 << 1); gy <= (ay7 << 1);
                end    
                4'd3: begin // COMPUTING_2
                    big_z <= 17'sd24576 - (x8 << 1) - (y8 << 1);
                    gx2 <= (gx * gx) >>> 12; gy2 <= (gy * gy) >>> 12;
                end
                4'd4: begin // COMPUTING_3
                    g2 <= $signed(gx2) + $signed(gy2) + 17'sd4096;
                    // kgg = g2 - eta2*g2 + eta2
                    kgg <= ($signed(gx2) + $signed(gy2) + 17'sd4096) - (((eta2 * ($signed(gx2) + $signed(gy2) + 17'sd4096)) >>> 12) - {1'b0, eta2});
                end
                4'd5: begin // COMPUTING_4
                    sqrt_kgg_r <= sqrt_q[15:0];
                    coef <= div1_q[16:0];
                end            
                4'd6: begin // COMPUTING_5
                    t <= div2_q[16:0];
                end
                4'd7: begin // PRE_WRITE
                    z_x <= big_x + (((t * coef) >>> 12) * gx >>> 12);
                    z_y <= big_y + (((t * coef) >>> 12) * gy >>> 12);
                end
                4'd8: begin // WRITE_X
                    SRAM_WE <= 1'b1; SRAM_A <= iteration << 1; SRAM_D <= z_x[15:0];
                end
                4'd9: begin // WRITE_Y
                    SRAM_WE <= 1'b1; SRAM_A <= (iteration << 1) + 9'd1; SRAM_D <= z_y[15:0];
                    iteration <= iteration + 9'd1; // 修正：在這裡累加，確保跑完 256 次
                end
                4'd10: begin // FINISH
                    SRAM_WE <= 1'b0; DONE <= 1'b1;
                end
            endcase
        end
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
            4'd8: next_state = 4'd9;
            4'd9: next_state = (iteration == 9'd255) ? 4'd10 : 4'd1;
            4'd10: next_state = 4'd10;
            default: next_state = 4'd0;
        endcase
    end
endmodule