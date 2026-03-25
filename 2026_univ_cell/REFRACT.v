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

    // --- 狀態定義 ---
    reg [3:0] state, next_state;
    reg [8:0] iteration;
    reg [5:0] cnt; // 用於計數除法與開根號的週期

    // --- 暫存器 ---
    reg signed [16:0] big_x, big_y, big_z;
    reg signed [15:0] x8, y8, gx, gy, gx2, gy2;
    reg signed [16:0] g2;
    reg [15:0] eta, eta2;
    reg signed [31:0] kgg;
    reg signed [15:0] sqrt_kgg_r;
    reg signed [16:0] coef, t, z_x, z_y;

    // --- 手寫除法器介面 ---
    reg [31:0] div_num;
    reg [31:0] div_den;
    wire [31:0] div_q;
    reg div_start;
    wire div_done;

    // --- 手寫開根號介面 ---
    reg [31:0] sqrt_in;
    wire [15:0] sqrt_out;
    reg sqrt_start;
    wire sqrt_done;

    // --- 實例化手寫模組 ---
    divider u_div (
        .clk(CLK), .rst(RST), .start(div_start),
        .num(div_num), .den(div_den),
        .quotient(div_q), .done(div_done)
    );

    sqrt_module u_sqrt (
        .clk(CLK), .rst(RST), .start(sqrt_start),
        .in(sqrt_in), .out(sqrt_out), .done(sqrt_done)
    );

    // --- 連乘積邏輯 (與原本一致) ---
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

    // --- 主狀態機 ---
    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            state <= 4'd0; iteration <= 9'd0; DONE <= 1'b0; SRAM_WE <= 1'b0;
            div_start <= 1'b0; sqrt_start <= 1'b0;
        end else begin
            case (state)
                4'd0: begin // INIT: 計算 eta = 1/RI
                    if (!div_start && !div_done) begin
                        div_num <= 32'd1 << 24;
                        div_den <= {28'd0, RI};
                        div_start <= 1'b1;
                    end else if (div_done) begin
                        div_start <= 1'b0;
                        eta <= div_q[15:0];
                        eta2 <= (div_q[15:0] * div_q[15:0]) >> 12;
                        state <= 4'd1;
                    end
                end
                4'd1: begin // GET_COOR
                    big_x <= {1'b0, iteration[3:0], 12'd0};
                    big_y <= {1'b0, iteration[7:4], 12'd0};
                    SRAM_WE <= 1'b0;
                    state <= 4'd2;
                end
                4'd2: begin // COMPUTING: 幾何形狀
                    x8 <= ax8[15:0]; y8 <= ay8[15:0];
                    gx <= (ax7 << 1); gy <= (ay7 << 1);
                    state <= 4'd3;
                end    
                4'd3: begin // COMPUTING_2: Z 與 法向量平方
                    big_z <= 17'sd24576 - (x8 << 1) - (y8 << 1);
                    gx2 <= (gx * gx) >>> 12; gy2 <= (gy * gy) >>> 12;
                    state <= 4'd4;
                end
                4'd4: begin // COMPUTING_3: g2 與 kgg
                    g2 <= $signed(gx2) + $signed(gy2) + 17'sd4096;
                    kgg <= ($signed(gx2) + $signed(gy2) + 17'sd4096) - (((eta2 * ($signed(gx2) + $signed(gy2) + 17'sd4096)) >>> 12) - {1'b0, eta2});
                    state <= 4'd5;
                end
                4'd5: begin // COMPUTING_4: 開根號與 coef 除法
                    if (!sqrt_start && !sqrt_done) begin
                        sqrt_in <= (kgg[31] ? 32'd0 : (kgg << 12));
                        sqrt_start <= 1'b1;
                    end else if (sqrt_done && !div_start) begin
                        sqrt_start <= 1'b0;
                        sqrt_kgg_r <= sqrt_out;
                        div_num <= ($signed({1'b0, eta}) - $signed(sqrt_out)) << 12;
                        div_den <= {{15{g2[16]}}, g2};
                        div_start <= 1'b1;
                    end else if (div_done) begin
                        div_start <= 1'b0;
                        coef <= div_q[16:0];
                        state <= 4'd6;
                    end
                end            
                4'd6: begin // COMPUTING_5: t 除法
                    if (!div_start && !div_done) begin
                        div_num <= (-big_z) << 12;
                        div_den <= $signed(coef) - $signed({1'b0, eta});
                        div_start <= 1'b1;
                    end else if (div_done) begin
                        div_start <= 1'b0;
                        t <= div_q[16:0];
                        state <= 4'd7;
                    end
                end
                4'd7: begin // PRE_WRITE: 折射點
                    z_x <= big_x + (((t * coef) >>> 12) * gx >>> 12);
                    z_y <= big_y + (((t * coef) >>> 12) * gy >>> 12);
                    state <= 4'd8;
                end
                4'd8: begin // WRITE_X
                    SRAM_WE <= 1'b1; SRAM_A <= iteration << 1; SRAM_D <= z_x[15:0];
                    state <= 4'd9;
                end
                4'd9: begin // WRITE_Y
                    SRAM_WE <= 1'b1; SRAM_A <= (iteration << 1) + 9'd1; SRAM_D <= z_y[15:0];
                    iteration <= iteration + 9'd1;
                    state <= (iteration == 9'd255) ? 4'd10 : 4'd1;
                end
                4'd10: begin // FINISH
                    SRAM_WE <= 1'b0; DONE <= 1'b1;
                end
            endcase
        end
    end

endmodule

// --- 手寫非還原除法器 (32-bit) ---
module divider (
    input clk, rst, start,
    input [31:0] num, den,
    output reg [31:0] quotient,
    output reg done
);
    reg [63:0] temp_num;
    reg [31:0] temp_den;
    reg [5:0] count;
    reg busy;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            busy <= 0; done <= 0;
        end else if (start && !busy) begin
            busy <= 1; done <= 0;
            count <= 0;
            temp_num <= {32'd0, num};
            temp_den <= den;
        end else if (busy) begin
            if (count < 32) begin
                if (temp_num[63:31] >= {1'b0, temp_den}) begin
                    temp_num[63:31] <= temp_num[63:31] - temp_den;
                    temp_num <= {temp_num[62:0], 1'b1};
                end else begin
                    temp_num <= {temp_num[62:0], 1'b0};
                end
                count <= count + 1;
            end else begin
                quotient <= temp_num[31:0];
                busy <= 0; done <= 1;
            end
        end else begin
            done <= 0;
        end
    end
endmodule

// --- 手寫逐位開根號器 (32-bit) ---
module sqrt_module (
    input clk, rst, start,
    input [31:0] in,
    output reg [15:0] out,
    output reg done
);
    reg [31:0] x, y, res;
    reg [4:0] count;
    reg busy;
    reg [31:0] t;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            busy <= 0; done <= 0;
        end else if (start && !busy) begin
            busy <= 1; done <= 0;
            count <= 15;
            x <= in; y <= 0; res <= 0;
        end else if (busy) begin
            t = (res << (count + 1)) | (32'd1 << (count << 1));
            if (x >= t) begin
                x <= x - t;
                res <= res | (32'd1 << count);
            end
            if (count == 0) begin
                out <= res[15:0];
                busy <= 0; done <= 1;
            end else begin
                count <= count - 1;
            end
        end else begin
            done <= 0;
        end
    end
endmodule