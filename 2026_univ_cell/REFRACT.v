module REFRACT(
    input  wire        CLK,
    input  wire        RST,
    input  wire [3:0]  RI,   
    output reg  [8:0]  SRAM_A,
    output reg  [15:0] SRAM_D,
    input  wire [15:0] SRAM_Q,   // unused [cite: 255]
    output reg         SRAM_WE,
    output reg         DONE
);

    // FSM States
    localparam IDLE      = 3'd0;
    localparam CALC_PRE  = 3'd1; // Stage 1: Powers, Z, g2, kgg
    localparam CALC_SQRT = 3'd2; // Stage 2: Sqrt(kgg) & Coef Division
    localparam CALC_FIN  = 3'd3; // Stage 3: t Division & Final ZX/ZY
    localparam WRITE_ZX  = 3'd4;
    localparam WRITE_ZY  = 3'd5;
    localparam FINISH    = 3'd6;

    reg [2:0] current_state, next_state;
    reg [3:0] x_idx, y_idx;

    // Pipeline Registers
    reg signed [31:0] gx_reg, gy_reg, Z_reg, eta_reg, g2_reg, inner_reg;
    reg signed [31:0] sqrt_kgg_reg, coef_reg;

    // =========================================================================
    // Stage 1: Combinational (Powers & kgg)
    // =========================================================================
    wire signed [31:0] dx = {28'd0, x_idx} - 32'd8;
    wire signed [31:0] dy = {28'd0, y_idx} - 32'd8;

    wire signed [31:0] dx_2 = dx * dx;
    wire signed [31:0] dx_4 = dx_2 * dx_2;
    wire signed [31:0] dx_7 = dx_4 * dx_2 * dx;
    wire signed [31:0] dx_8 = dx_4 * dx_4;

    wire signed [31:0] dy_2 = dy * dy;
    wire signed [31:0] dy_4 = dy_2 * dy_2;
    wire signed [31:0] dy_7 = dy_4 * dy_2 * dy;
    wire signed [31:0] dy_8 = dy_4 * dy_4;

    wire signed [31:0] gx_w = dx_7 >>> 8; // Adjust based on your scaling
    wire signed [31:0] gy_w = dy_7 >>> 8;
    wire signed [31:0] Z_w  = 32'd24576 - (dx_8 >>> 11) - (dy_8 >>> 11);

    wire signed [31:0] ri_s   = {28'd0, RI};
    wire signed [31:0] eta_w  = 32'd4096 / ri_s;
    wire signed [31:0] eta2_w = (eta_w * eta_w) >>> 12;

    wire signed [31:0] g2_w    = ((gx_w * gx_w) >>> 12) + ((gy_w * gy_w) >>> 12) + 32'd4096;
    wire signed [31:0] inner_w = g2_w - ((eta2_w * g2_w) >>> 12) + eta2_w;

    // =========================================================================
    // Stage 2 & 3 Hardware: Using DesignWare for faster Synthesis 
    // =========================================================================
    wire [31:0] sqrt_res;
    // 使用 DW_sqrt 替代迴圈 function
    DW_sqrt #(.width(32), .tc_mode(0)) U_SQRT (.a(inner_reg << 12), .root(sqrt_res));

    wire [31:0] coef_q;
    // 使用 DW_div 處理 coef = num / g2
    DW_div #(.a_width(32), .b_width(32), .tc_mode(1)) 
    U_DIV_COEF (.a((eta_reg - {16'd0, sqrt_res[15:0]}) << 12), .b(g2_reg), .quotient(coef_q), .remainder());

    wire [31:0] t_q;
    // 使用 DW_div 處理 t = Z / (eta - coef)
    DW_div #(.a_width(32), .b_width(32), .tc_mode(1)) 
    U_DIV_T (.a(Z_reg << 12), .b(eta_reg - coef_reg), .quotient(t_q), .remainder());

    // Final calculations (Combinational at Stage 3)
    wire signed [31:0] t_coef = (t_q * coef_reg) >>> 12;
    wire signed [31:0] zx_out = ({28'd0, x_idx} << 12) + ((t_coef * gx_reg) >>> 12);
    wire signed [31:0] zy_out = ({28'd0, y_idx} << 12) + ((t_coef * gy_reg) >>> 12);

    // =========================================================================
    // Sequential Control
    // =========================================================================
    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            current_state <= IDLE;
            {x_idx, y_idx} <= 8'd0;
            DONE <= 1'b0;
        end else begin
            current_state <= next_state;
            
            case (current_state)
                CALC_PRE: begin
                    gx_reg <= gx_w; gy_reg <= gy_w; Z_reg <= Z_w;
                    eta_reg <= eta_w; g2_reg <= g2_w; inner_reg <= inner_w;
                end
                CALC_SQRT: begin
                    coef_reg <= coef_q;
                end
                WRITE_ZY: begin
                    if (x_idx == 4'd15) begin
                        x_idx <= 4'd0;
                        if (y_idx != 4'd15) y_idx <= y_idx + 1'b1;
                    end else x_idx <= x_idx + 1'b1;
                end
                FINISH: DONE <= 1'b1;
            endcase
        end
    end

    always @(*) begin
        next_state = current_state;
        case (current_state)
            IDLE:      next_state = CALC_PRE;
            CALC_PRE:  next_state = CALC_SQRT;
            CALC_SQRT: next_state = CALC_FIN;
            CALC_FIN:  next_state = WRITE_ZX;
            WRITE_ZX:  next_state = WRITE_ZY;
            WRITE_ZY:  next_state = (x_idx == 15 && y_idx == 15) ? FINISH : CALC_PRE;
            default:   next_state = IDLE;
        endcase
    end

    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            SRAM_WE <= 1'b0; SRAM_A <= 9'd0; SRAM_D <= 16'd0;
        end else begin
            SRAM_WE <= 1'b0;
            case (current_state)
                WRITE_ZX: begin
                    SRAM_WE <= 1'b1;
                    SRAM_A  <= {y_idx, x_idx, 1'b0};
                    SRAM_D  <= zx_out[15:0];
                end
                WRITE_ZY: begin
                    SRAM_WE <= 1'b1;
                    SRAM_A  <= {y_idx, x_idx, 1'b1};
                    SRAM_D  <= zy_out[15:0];
                end
            endcase
        end
    end

endmodule