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

    // FSM States (Expanded for Pipelining)
    localparam IDLE     = 3'd0;
    localparam CALC_1   = 3'd1;  // Pipeline Stage 1
    localparam CALC_2   = 3'd2;  // Pipeline Stage 2
    localparam WRITE_ZX = 3'd3;
    localparam WRITE_ZY = 3'd4;
    localparam FINISH   = 3'd5;

    reg [2:0] current_state, next_state;
    reg [3:0] x_idx, y_idx;

    // =========================================================================
    // Pipeline Registers (Barrier between CALC_1 and CALC_2)
    // =========================================================================
    reg signed [31:0] gx_12_reg;
    reg signed [31:0] gy_12_reg;
    reg signed [31:0] Z_12_reg;
    reg signed [31:0] eta_12_reg;
    reg signed [31:0] g2_12_reg;
    reg signed [31:0] inner_12_reg;

    // =========================================================================
    // Stage 1: Combinational Logic (Evaluated during CALC_1)
    // =========================================================================
    wire signed [31:0] x_val = {27'd0, x_idx};
    wire signed [31:0] y_val = {27'd0, y_idx};
    
    // dx = X - 8, dy = Y - 8
    wire signed [31:0] dx = x_val - 32'd8;
    wire signed [31:0] dy = y_val - 32'd8;

    // Calculate powers for X
    wire signed [31:0] dx_2 = dx * dx;
    wire signed [31:0] dx_4 = dx_2 * dx_2;
    wire signed [31:0] dx_7 = dx_4 * dx_2 * dx;
    wire signed [31:0] dx_8 = dx_4 * dx_4;

    // Calculate powers for Y
    wire signed [31:0] dy_2 = dy * dy;
    wire signed [31:0] dy_4 = dy_2 * dy_2;
    wire signed [31:0] dy_7 = dy_4 * dy_2 * dy;
    wire signed [31:0] dy_8 = dy_4 * dy_4;

    wire signed [31:0] gx_12 = dx_7 >>> 8;
    wire signed [31:0] gy_12 = dy_7 >>> 8;
    wire signed [31:0] Z_12 = 32'd24576 - (dx_8 >>> 11) - (dy_8 >>> 11);

    // eta = 1 / RI
    wire signed [31:0] ri_signed = {28'd0, RI};
    wire signed [31:0] eta_12 = 32'd4096 / ri_signed;
    wire signed [31:0] eta2_12 = (eta_12 * eta_12) >>> 12;

    wire signed [31:0] g2_12 = ((gx_12 * gx_12) >>> 12) + ((gy_12 * gy_12) >>> 12) + 32'd4096;
    wire signed [31:0] inner_12 = g2_12 - ((eta2_12 * g2_12) >>> 12) + eta2_12;

    // =========================================================================
    // Stage 2: Combinational Logic (Evaluated during CALC_2)
    // =========================================================================
    wire [31:0] sqrt_input = inner_12_reg <<< 12;
    wire signed [31:0] sqrt_kgg_12 = {16'd0, sqrt_func(sqrt_input)};

    wire signed [31:0] coef_num = (eta_12_reg - sqrt_kgg_12) <<< 12;
    wire signed [31:0] coef_12 = coef_num / g2_12_reg;

    wire signed [31:0] t_num = Z_12_reg <<< 12;
    wire signed [31:0] t_den = eta_12_reg - coef_12;
    wire signed [31:0] t_12 = t_num / t_den;

    wire signed [31:0] t_coef_12 = (t_12 * coef_12) >>> 12;
    wire signed [31:0] zx_12 = (x_val <<< 12) + ((t_coef_12 * gx_12_reg) >>> 12);
    wire signed [31:0] zy_12 = (y_val <<< 12) + ((t_coef_12 * gy_12_reg) >>> 12);

    // -------------------------------------------------------------------------
    // Combinational Integer Square Root Function
    // -------------------------------------------------------------------------
    function [15:0] sqrt_func;
        input [31:0] val;
        reg [31:0] temp, root;
        integer i;
        begin
            root = 0;
            for (i = 15; i >= 0; i = i - 1) begin
                temp = root | (1 << i);
                if ((temp * temp) <= val)
                    root = temp;
            end
            sqrt_func = root[15:0];
        end
    endfunction

    // -------------------------------------------------------------------------
    // FSM Sequential Logic & Pipeline Register Updating
    // -------------------------------------------------------------------------
    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            current_state <= IDLE;
            x_idx         <= 4'd0;
            y_idx         <= 4'd0;
            
            gx_12_reg    <= 32'd0;
            gy_12_reg    <= 32'd0;
            Z_12_reg     <= 32'd0;
            eta_12_reg   <= 32'd0;
            g2_12_reg    <= 32'd0;
            inner_12_reg <= 32'd0;
        end else begin
            current_state <= next_state;
            
            // Latch Stage 1 data into Pipeline Registers
            if (current_state == CALC_1) begin
                gx_12_reg    <= gx_12;
                gy_12_reg    <= gy_12;
                Z_12_reg     <= Z_12;
                eta_12_reg   <= eta_12;
                g2_12_reg    <= g2_12;
                inner_12_reg <= inner_12;
            end
            
            // Advance coordinate grid after Y is written
            if (current_state == WRITE_ZY) begin
                if (x_idx == 4'd15) begin
                    x_idx <= 4'd0;
                    if (y_idx != 4'd15) begin
                        y_idx <= y_idx + 1'b1;
                    end
                end else begin
                    x_idx <= x_idx + 1'b1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // FSM Next State Logic
    // -------------------------------------------------------------------------
    always @(*) begin
        next_state = current_state; 
        case (current_state)
            IDLE:     next_state = CALC_1;
            CALC_1:   next_state = CALC_2;   // Transition to Stage 2
            CALC_2:   next_state = WRITE_ZX; // Stage 2 evaluates here
            WRITE_ZX: next_state = WRITE_ZY;
            WRITE_ZY: begin
                if (x_idx == 4'd15 && y_idx == 4'd15)
                    next_state = FINISH;
                else
                    next_state = CALC_1; // Loop back to Stage 1
            end
            FINISH:   next_state = FINISH;
            default:  next_state = IDLE;
        endcase
    end

    // -------------------------------------------------------------------------
    // Output Registers (SRAM Control & DONE)
    // -------------------------------------------------------------------------
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
    end

endmodule