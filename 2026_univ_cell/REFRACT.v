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

    // ==========================================
    // Compressed FSM States
    // ==========================================
    localparam S_IDLE       = 4'd0;
    localparam S_WAIT_ETA   = 4'd1;
    localparam S_CALC_POW   = 4'd2;
    localparam S_WAIT_SQRT  = 4'd3;
    localparam S_WAIT_COEF  = 4'd4;
    localparam S_WAIT_T     = 4'd5;
    localparam S_CALC_COORD = 4'd6;
    localparam S_WRITE_ZY   = 4'd7;
    localparam S_FINISH     = 4'd8;

    reg [3:0] current_state;
    reg [3:0] x_idx, y_idx;

    reg signed [31:0] eta_12, eta2_12;
    reg signed [31:0] gx_r, gy_r, bigZ_r, g2_r;
    reg signed [31:0] sqrt_kgg_12, coef_12, t_12;
    
    // ==========================================
    // Modules Connection
    // ==========================================
    reg div_start;
    reg signed [31:0] div_num, div_den;
    wire signed [31:0] div_quo;
    wire div_done;

    Seq_Div u_div (
        .clk(CLK), .rst(RST), .start(div_start),
        .num(div_num), .den(div_den),
        .quo(div_quo), .done(div_done)
    );

    reg sqrt_start;
    reg [31:0] sqrt_rad;
    wire [15:0] sqrt_root;
    wire sqrt_done;

    Seq_Sqrt u_sqrt (
        .clk(CLK), .rst(RST), .start(sqrt_start),
        .rad(sqrt_rad),
        .root(sqrt_root), .done(sqrt_done)
    );

    // ==========================================
    // Combinational Math Wires (MUST be before logic)
    // ==========================================
    wire signed [31:0] x_val = {27'd0, x_idx};
    wire signed [31:0] y_val = {27'd0, y_idx};

    wire signed [31:0] dx = x_val - 32'sd8;
    wire signed [31:0] dy = y_val - 32'sd8;

    wire signed [31:0] dx_2 = dx * dx;
    wire signed [31:0] dx_4 = dx_2 * dx_2;
    wire signed [31:0] dx_7 = dx_4 * dx_2 * dx;
    wire signed [31:0] dx_8 = dx_4 * dx_4;

    wire signed [31:0] dy_2 = dy * dy;
    wire signed [31:0] dy_4 = dy_2 * dy_2;
    wire signed [31:0] dy_7 = dy_4 * dy_2 * dy;
    wire signed [31:0] dy_8 = dy_4 * dy_4;

    wire signed [31:0] gx_w = dx_7 >>> 8;
    wire signed [31:0] gy_w = dy_7 >>> 8;
    wire signed [31:0] g2_w = ((gx_w * gx_w) >>> 12) + ((gy_w * gy_w) >>> 12) + 32'sd4096;
    
    wire signed [31:0] bigZ_w  = 32'sd24576 - (dx_8 >>> 11) - (dy_8 >>> 11);
    wire signed [31:0] kgg_w = g2_w - ((eta2_12 * g2_w) >>> 12) + eta2_12;

    wire signed [31:0] t_mul_coef_w = (t_12 * coef_12) >>> 12;
    wire signed [31:0] t_coef_gx_w = t_mul_coef_w * gx_r;
    wire signed [31:0] t_coef_gy_w = t_mul_coef_w * gy_r;
    wire signed [31:0] zx_w = (x_val <<< 12) + (t_coef_gx_w >>> 12);
    wire signed [31:0] zy_w = (y_val <<< 12) + (t_coef_gy_w >>> 12);

    // ==========================================
    // Zero-Overhead Combinational Start Logic
    // ==========================================
    always @(*) begin
        div_start = 1'b0; div_num = 32'd0; div_den = 32'd1;
        
        if (current_state == S_IDLE) begin
            div_start = 1'b1;
            div_num   = 32'sd4096;
            div_den   = {28'd0, RI};
        end 
        else if (current_state == S_WAIT_SQRT && sqrt_done) begin
            div_start = 1'b1; // 無縫啟動 COEF 除法
            div_num   = (eta_12 - {16'd0, sqrt_root}) <<< 12;
            div_den   = g2_r;
        end 
        else if (current_state == S_WAIT_COEF && div_done) begin
            div_start = 1'b1; // 無縫啟動 T 除法
            div_num   = bigZ_r <<< 12;
            div_den   = eta_12 - div_quo; 
        end
    end

    always @(*) begin
        sqrt_start = 1'b0; sqrt_rad = 32'd0;
        if (current_state == S_CALC_POW) begin
            sqrt_start = 1'b1;
            sqrt_rad   = kgg_w <<< 12;
        end
    end

    // ==========================================
    // Main FSM
    // ==========================================
    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            current_state <= S_IDLE;
            x_idx <= 4'd0; y_idx <= 4'd0;
            SRAM_WE <= 1'b0; DONE <= 1'b0;
        end else begin
            SRAM_WE <= 1'b0; 
            DONE <= (current_state == S_FINISH) ? 1'b1 : 1'b0;

            case (current_state)
                S_IDLE: begin
                    current_state <= S_WAIT_ETA;
                end

                S_WAIT_ETA: begin
                    if (div_done) begin
                        eta_12  <= div_quo;
                        eta2_12 <= (div_quo * div_quo) >>> 12;
                        current_state <= S_CALC_POW;
                    end
                end

                S_CALC_POW: begin
                    gx_r <= gx_w; gy_r <= gy_w;
                    bigZ_r <= bigZ_w; g2_r <= g2_w;
                    current_state <= S_WAIT_SQRT;
                end

                S_WAIT_SQRT: begin
                    if (sqrt_done) begin
                        sqrt_kgg_12 <= {16'd0, sqrt_root};
                        current_state <= S_WAIT_COEF;
                    end
                end

                S_WAIT_COEF: begin
                    if (div_done) begin
                        coef_12 <= div_quo;
                        current_state <= S_WAIT_T;
                    end
                end

                S_WAIT_T: begin
                    if (div_done) begin
                        t_12 <= div_quo;
                        current_state <= S_CALC_COORD;
                    end
                end

                S_CALC_COORD: begin
                    SRAM_WE <= 1'b1;
                    SRAM_A  <= {y_idx, x_idx, 1'b0}; 
                    SRAM_D  <= zx_w[15:0];
                    current_state <= S_WRITE_ZY;
                end

                S_WRITE_ZY: begin
                    SRAM_WE <= 1'b1;
                    SRAM_A  <= {y_idx, x_idx, 1'b1}; 
                    SRAM_D  <= zy_w[15:0];
                    
                    if (x_idx == 4'd15 && y_idx == 4'd15) begin
                        current_state <= S_FINISH;
                    end else begin
                        if (x_idx == 4'd15) begin
                            x_idx <= 4'd0; y_idx <= y_idx + 1'b1;
                        end else begin
                            x_idx <= x_idx + 1'b1;
                        end
                        current_state <= S_CALC_POW;
                    end
                end

                S_FINISH: begin
                    current_state <= S_FINISH;
                end
            endcase
        end
    end

endmodule

// ==========================================
// Divider Module
// ==========================================
module Seq_Div (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire signed [31:0] num,
    input  wire signed [31:0] den,
    output wire signed [31:0] quo,
    output reg         done
);
    reg [5:0] count;
    reg [63:0] remainder;
    reg [31:0] divisor;
    reg sign;

    wire [31:0] abs_num = (num[31]) ? -num : num;
    wire [31:0] abs_den = (den[31]) ? -den : den;
    wire [32:0] sub_res = {1'b0, remainder[62:31]} - {1'b0, divisor};

    assign quo = sign ? -remainder[31:0] : remainder[31:0];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            count <= 0; done <= 0;
            remainder <= 0; divisor <= 0; sign <= 0;
        end else begin
            if (start) begin
                count <= 6'd32;
                remainder <= {32'd0, abs_num};
                divisor <= abs_den;
                sign <= num[31] ^ den[31];
                done <= 0;
            end else if (count > 0) begin
                if (!sub_res[32]) remainder <= {sub_res[31:0], remainder[30:0], 1'b1};
                else remainder <= {remainder[62:0], 1'b0};
                
                count <= count - 1;
                if (count == 1) done <= 1;
            end else begin
                done <= 0;
            end
        end
    end
endmodule

// ==========================================
// Square Root Module
// ==========================================
module Seq_Sqrt (
    input wire clk,
    input wire rst,
    input wire start,
    input wire [31:0] rad,
    output reg [15:0] root,
    output reg done
);
    reg [4:0] count;
    reg [31:0] acc;
    reg [31:0] rad_reg;
    
    wire [31:0] next_acc = {acc[29:0], rad_reg[31:30]};
    wire [31:0] test_val = {14'd0, root, 2'b01};
    wire can_sub = (next_acc >= test_val);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            count <= 0; acc <= 0; root <= 0;
            rad_reg <= 0; done <= 0;
        end else begin
            if (start) begin
                count <= 5'd16;
                rad_reg <= rad;
                acc <= 32'd0; root <= 16'd0; done <= 1'b0;
            end else if (count > 0) begin
                if (can_sub) begin
                    acc <= next_acc - test_val;
                    root <= {root[14:0], 1'b1};
                end else begin
                    acc <= next_acc;
                    root <= {root[14:0], 1'b0};
                end
                rad_reg <= {rad_reg[29:0], 2'b00};
                count <= count - 1'b1;
                if (count == 5'd1) done <= 1'b1;
            end else begin
                done <= 1'b0;
            end
        end
    end
endmodule