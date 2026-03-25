module REFRACT(
    input  wire        CLK,
    input  wire        RST,
    input  wire [3:0]  RI,   
    output reg  [8:0]  SRAM_A,
    output reg  [15:0] SRAM_D,
    input  wire [15:0] SRAM_Q,   
    output reg         SRAM_WE,
    output reg         DONE
);

    // FSM States
    localparam IDLE      = 3'd0;
    localparam CALC_PRE  = 3'd1; 
    localparam WAIT_SQRT = 3'd2; 
    localparam WAIT_DIV1 = 3'd3; 
    localparam WAIT_DIV2 = 3'd4; 
    localparam WRITE_ZX  = 3'd5;
    localparam WRITE_ZY  = 3'd6;
    localparam FINISH    = 3'd7;

    reg [2:0] state;
    reg [3:0] x_idx, y_idx;

    // 運算暫存器
    reg signed [31:0] gx_reg, gy_reg, Z_reg, eta_reg, g2_reg, inner_reg;
    reg signed [31:0] sqrt_reg, coef_reg, t_reg;

    // 手寫運算器信號
    reg  div_start, sqrt_start;
    reg  [31:0] div_num, div_den, sqrt_in;
    wire [31:0] div_q, sqrt_out;
    wire div_done, sqrt_done;

    // 實例化手寫模組
    my_div  u_div  (.clk(CLK), .rst(RST), .start(div_start),  .num(div_num), .den(div_den), .q(div_q), .done(div_done));
    my_sqrt u_sqrt (.clk(CLK), .rst(RST), .start(sqrt_start), .in(sqrt_in), .out(sqrt_out), .done(sqrt_done));

    // Stage 1 Logic: Powers & kgg [cite: 122, 126, 146, 150]
    wire signed [31:0] dx = {28'd0, x_idx} - 32'd8;
    wire signed [31:0] dy = {28'd0, y_idx} - 32'd8;
    wire signed [31:0] dx_8 = (dx*dx)*(dx*dx)*(dx*dx)*(dx*dx);
    wire signed [31:0] dy_8 = (dy*dy)*(dy*dy)*(dy*dy)*(dy*dy);
    
    wire signed [31:0] gx_w = ((dx*dx*dx*dx)*(dx*dx*dx)) >>> 8; 
    wire signed [31:0] gy_w = ((dy*dy*dy*dy)*(dy*dy*dy)) >>> 8;
    wire signed [31:0] Z_w  = 32'd24576 - (dx_8 >>> 11) - (dy_8 >>> 11); // 6*4096 = 24576 [cite: 70, 146]

    // Main FSM [cite: 91, 92]
    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            state <= IDLE; {x_idx, y_idx} <= 8'd0; DONE <= 1'b0;
            div_start <= 0; sqrt_start <= 0;
        end else begin
            case (state)
                IDLE: begin
                    eta_reg <= 32'd4096 / RI; // 初始計算 RI [cite: 86, 116]
                    state <= CALC_PRE;
                end
                CALC_PRE: begin
                    gx_reg <= gx_w; gy_reg <= gy_w; Z_reg <= Z_w;
                    g2_reg <= ((gx_w * gx_w) >>> 12) + ((gy_w * gy_w) >>> 12) + 32'd4096;
                    inner_reg <= g2_w - ((( (eta_reg*eta_reg)>>>12 ) * g2_w) >>> 12) + ((eta_reg*eta_reg)>>>12);
                    sqrt_in <= inner_w << 12; sqrt_start <= 1; state <= WAIT_SQRT;
                end
                WAIT_SQRT: begin
                    sqrt_start <= 0;
                    if (sqrt_done) begin
                        sqrt_reg <= sqrt_out;
                        div_num <= (eta_reg - sqrt_out) << 12; div_den <= g2_reg;
                        div_start <= 1; state <= WAIT_DIV1;
                    end
                end
                WAIT_DIV1: begin
                    div_start <= 0;
                    if (div_done) begin
                        coef_reg <= div_q;
                        div_num <= Z_reg << 12; div_den <= eta_reg - div_q;
                        div_start <= 1; state <= WAIT_DIV2;
                    end
                end
                WAIT_DIV2: begin
                    div_start <= 0;
                    if (div_done) begin
                        t_reg <= div_q; state <= WRITE_ZX;
                    end
                end
                WRITE_ZX: state <= WRITE_ZY;
                WRITE_ZY: begin
                    if (x_idx == 15 && y_idx == 15) state <= FINISH;
                    else begin
                        if (x_idx == 15) begin x_idx <= 0; y_idx <= y_idx + 1; end
                        else x_idx <= x_idx + 1;
                        state <= CALC_PRE;
                    end
                end
                FINISH: DONE <= 1'b1;
            endcase
        end
    end

    // SRAM Data Output [cite: 160, 162, 170]
    wire signed [31:0] t_coef = (t_reg * coef_reg) >>> 12;
    always @(posedge CLK) begin
        SRAM_WE <= (state == WRITE_ZX || state == WRITE_ZY);
        SRAM_A  <= {y_idx, x_idx, (state == WRITE_ZY)};
        SRAM_D  <= (state == WRITE_ZX) ? ({28'd0, x_idx} << 12) + ((t_coef * gx_reg) >>> 12) :
                                         ({28'd0, y_idx} << 12) + ((t_coef * gy_reg) >>> 12);
    end
endmodule

// 手寫時序除法器 (32-bit)
module my_div(input clk, rst, start, input [31:0] num, den, output reg [31:0] q, output reg done);
    reg [63:0] a; reg [31:0] b; reg [5:0] i; reg busy;
    always @(posedge clk or posedge rst) begin
        if (rst) begin busy <= 0; done <= 0; end
        else if (start) begin busy <= 1; i <= 0; a <= {32'd0, num}; b <= den; done <= 0; end
        else if (busy) begin
            if (i == 32) begin q <= a[31:0]; busy <= 0; done <= 1; end
            else begin
                if (a[63:32] >= b) begin a[63:32] <= a[63:32] - b; a <= {a[62:0], 1'b1}; end
                else a <= {a[62:0], 1'b0};
                i <= i + 1;
            end
        end else done <= 0;
    end
endmodule

// 手寫時序開根號器 (32-bit)
module my_sqrt(input clk, rst, start, input [31:0] in, output reg [31:0] out, output reg done);
    reg [31:0] x, r, q; reg [5:0] i; reg busy;
    always @(posedge clk or posedge rst) begin
        if (rst) begin busy <= 0; done <= 0; end
        else if (start) begin busy <= 1; i <= 15; x <= in; r <= 0; q <= 0; done <= 0; end
        else if (busy) begin
            wire [31:0] t = (q << (i+1)) | (32'd1 << (i<<1));
            if (x >= t) begin x <= x - t; q <= q | (32'd1 << i); end
            if (i == 0) begin out <= q; busy <= 0; done <= 1; end
            else i <= i - 1;
        end else done <= 0;
    end
endmodule