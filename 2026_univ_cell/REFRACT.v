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

    // =========================================================================
    // FSM States
    // =========================================================================
    localparam IDLE      = 4'd0;
    localparam CALC_1    = 4'd1;
    localparam SQRT_INIT = 4'd2;
    localparam SQRT_RUN  = 4'd3;
    localparam CALC_2    = 4'd4;
    localparam WRITE_ZX  = 4'd5;
    localparam WRITE_ZY  = 4'd6;
    localparam FINISH    = 4'd7;

    reg [3:0] current_state, next_state;
    reg [3:0] x_idx, y_idx;

    // =========================================================================
    // Pipeline Registers
    // =========================================================================
    reg signed [31:0] gx_12_reg;
    reg signed [31:0] gy_12_reg;
    reg signed [31:0] Z_12_reg;
    reg signed [31:0] eta_12_reg;
    reg signed [31:0] g2_12_reg;
    reg signed [31:0] inner_12_reg;

    // =========================================================================
    // Sequential sqrt Registers
    // =========================================================================
    reg [31:0] sqrt_val_reg;
    reg [15:0] sqrt_root_reg;
    reg [4:0]  sqrt_bit_reg;

    // =========================================================================
    // Stage 1: Combinational Logic
    // =========================================================================
    wire signed [31:0] x_val = {28'd0, x_idx};
    wire signed [31:0] y_val = {28'd0, y_idx};

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

    wire signed [31:0] gx_12 = dx_7 >>> 8;
    wire signed [31:0] gy_12 = dy_7 >>> 8;
    wire signed [31:0] Z_12  = 32'd24576 - (dx_8 >>> 11) - (dy_8 >>> 11);

    // eta = 1 / RI
    wire signed [31:0] ri_signed = {28'd0, RI};
    wire signed [31:0] eta_12    = 32'd4096 / ri_signed;
    wire signed [31:0] eta2_12   = (eta_12 * eta_12) >>> 12;

    wire signed [31:0] g2_12    = ((gx_12 * gx_12) >>> 12) + ((gy_12 * gy_12) >>> 12) + 32'd4096;
    wire signed [31:0] inner_12 = g2_12 - ((eta2_12 * g2_12) >>> 12) + eta2_12;

    // =========================================================================
    // Sequential sqrt datapath
    // =========================================================================
    wire [15:0] sqrt_try_root = sqrt_root_reg | (16'd1 << sqrt_bit_reg[3:0]);
    wire [31:0] sqrt_try_sq   = sqrt_try_root * sqrt_try_root;
    wire        sqrt_take     = (sqrt_try_sq <= sqrt_val_reg);

    // =========================================================================
    // Stage 2: Combinational Logic
    // =========================================================================
    wire signed [31:0] sqrt_kgg_12 = {16'd0, sqrt_root_reg};

    wire signed [31:0] coef_num = (eta_12_reg - sqrt_kgg_12) <<< 12;
    wire signed [31:0] coef_12  = coef_num / g2_12_reg;

    wire signed [31:0] t_num = Z_12_reg <<< 12;
    wire signed [31:0] t_den = eta_12_reg - coef_12;
    wire signed [31:0] t_12  = t_num / t_den;

    wire signed [31:0] t_coef_12 = (t_12 * coef_12) >>> 12;
    wire signed [31:0] zx_12     = (x_val <<< 12) + ((t_coef_12 * gx_12_reg) >>> 12);
    wire signed [31:0] zy_12     = (y_val <<< 12) + ((t_coef_12 * gy_12_reg) >>> 12);

    // =========================================================================
    // FSM Sequential Logic
    // =========================================================================
    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            current_state <= IDLE;
            x_idx         <= 4'd0;
            y_idx         <= 4'd0;

            gx_12_reg     <= 32'd0;
            gy_12_reg     <= 32'd0;
            Z_12_reg      <= 32'd0;
            eta_12_reg    <= 32'd0;
            g2_12_reg     <= 32'd0;
            inner_12_reg  <= 32'd0;

            sqrt_val_reg  <= 32'd0;
            sqrt_root_reg <= 16'd0;
            sqrt_bit_reg  <= 5'd0;
        end else begin
            current_state <= next_state;

            // Latch stage 1 results
            if (current_state == CALC_1) begin
                gx_12_reg    <= gx_12;
                gy_12_reg    <= gy_12;
                Z_12_reg     <= Z_12;
                eta_12_reg   <= eta_12;
                g2_12_reg    <= g2_12;
                inner_12_reg <= inner_12;
            end

            // Init sqrt
            if (current_state == SQRT_INIT) begin
                sqrt_val_reg  <= inner_12_reg <<< 12;
                sqrt_root_reg <= 16'd0;
                sqrt_bit_reg  <= 5'd15;
            end
            // Run sqrt bit-by-bit
            else if (current_state == SQRT_RUN) begin
                if (sqrt_take)
                    sqrt_root_reg <= sqrt_try_root;

                if (sqrt_bit_reg != 5'd0)
                    sqrt_bit_reg <= sqrt_bit_reg - 1'b1;
            end

            // Advance coordinate grid after Y is written
            if (current_state == WRITE_ZY) begin
                if (x_idx == 4'd15) begin
                    x_idx <= 4'd0;
                    if (y_idx != 4'd15)
                        y_idx <= y_idx + 1'b1;
                end else begin
                    x_idx <= x_idx + 1'b1;
                end
            end
        end
    end

    // =========================================================================
    // FSM Next State Logic
    // =========================================================================
    always @(*) begin
        next_state = current_state;
        case (current_state)
            IDLE:      next_state = CALC_1;
            CALC_1:    next_state = SQRT_INIT;
            SQRT_INIT: next_state = SQRT_RUN;
            SQRT_RUN: begin
                if (sqrt_bit_reg == 5'd0)
                    next_state = CALC_2;
                else
                    next_state = SQRT_RUN;
            end
            CALC_2:    next_state = WRITE_ZX;
            WRITE_ZX:  next_state = WRITE_ZY;
            WRITE_ZY: begin
                if (x_idx == 4'd15 && y_idx == 4'd15)
                    next_state = FINISH;
                else
                    next_state = CALC_1;
            end
            FINISH:    next_state = FINISH;
            default:   next_state = IDLE;
        endcase
    end

    // =========================================================================
    // Output Registers
    // =========================================================================
    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            SRAM_WE <= 1'b0;
            SRAM_A  <= 9'd0;
            SRAM_D  <= 16'd0;
            DONE    <= 1'b0;
        end else begin
            SRAM_WE <= 1'b0;
            DONE    <= (current_state == FINISH) ? 1'b1 : 1'b0;

            case (current_state)
                WRITE_ZX: begin
                    SRAM_WE <= 1'b1;
                    SRAM_A  <= {y_idx, x_idx, 1'b0};
                    SRAM_D  <= zx_12[15:0];
                end
                WRITE_ZY: begin
                    SRAM_WE <= 1'b1;
                    SRAM_A  <= {y_idx, x_idx, 1'b1};
                    SRAM_D  <= zy_12[15:0];
                end
            endcase
        end
        always @(posedge CLK) begin
            if (!RST) begin
                $display("T=%0t | state=%0d | x=%0d y=%0d | sqrt_bit=%0d | sqrt_root=%0d | inner=%0d",
                        $time,
                        current_state,
                        x_idx,
                        y_idx,
                        sqrt_bit_reg,
                        sqrt_root_reg,
                        inner_12_reg);
            end
        end
    end

endmodule