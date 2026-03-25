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
    // FSM State Definitions
    // =========================================================================
    localparam IDLE           = 6'd0;
    localparam INIT_DIV_ETA   = 6'd1;
    localparam WAIT_ETA       = 6'd2;
    localparam CALC_ETA2      = 6'd3;
    
    localparam X_POW_1        = 6'd4;
    localparam X_POW_2        = 6'd5;
    localparam X_POW_3        = 6'd6;
    localparam X_POW_4        = 6'd7;
    localparam X_POW_5        = 6'd8;
    
    localparam Y_POW_1        = 6'd9;
    localparam Y_POW_2        = 6'd10;
    localparam Y_POW_3        = 6'd11;
    localparam Y_POW_4        = 6'd12;
    localparam Y_POW_5        = 6'd13;
    
    localparam G_POW_1        = 6'd14;
    localparam G_POW_2        = 6'd15;
    localparam G_POW_3        = 6'd16;
    
    localparam INNER_CALC     = 6'd17;
    localparam START_SQRT     = 6'd18;
    localparam WAIT_SQRT      = 6'd19;
    
    localparam START_DIV_COEF = 6'd20;
    localparam WAIT_COEF      = 6'd21;
    localparam START_DIV_T    = 6'd22;
    localparam WAIT_T         = 6'd23;
    
    localparam CALC_T_COEF    = 6'd24;
    localparam CALC_ZX        = 6'd25;
    localparam WRITE_ZX       = 6'd26;
    localparam CALC_ZY        = 6'd27;
    localparam WRITE_ZY       = 6'd28;
    
    localparam NEXT_COORD     = 6'd29;
    localparam FINISH         = 6'd30;

    reg [5:0] current_state;
    reg [3:0] x_idx, y_idx;
    reg       calc_x_flag; // 用來標記是否需要重新計算 X 的高次方以節省時間

    // =========================================================================
    // Shared Resource & Registers (Q20.12 Signed Fixed-Point)
    // =========================================================================
    reg signed [31:0] mult_a, mult_b;
    wire signed [63:0] mult_out = mult_a * mult_b; // 系統中唯一的實體乘法器
    
    reg signed [31:0] dx_reg, dx2, dx3, dx4;
    reg signed [31:0] dy_reg, dy2, dy3, dy4;
    
    reg signed [31:0] gx_12, gy_12;
    reg signed [31:0] dx8_s, dy8_s;
    
    reg signed [31:0] eta_12, eta2_12;
    reg signed [31:0] gx2_12, gy2_12, g2_12;
    reg signed [31:0] inner_12, sqrt_kgg_12;
    reg signed [31:0] coef_12, t_12, t_coef_12;
    reg signed [31:0] Z_12;
    
    reg signed [31:0] zx_out, zy_out;

    // Divider Control Signals
    reg div_start;
    reg signed [31:0] div_num, div_den;
    wire signed [31:0] div_quo;
    wire div_done;

    // SQRT Control Signals
    reg sqrt_start;
    reg [31:0] sqrt_rad;
    wire [15:0] sqrt_root;
    wire sqrt_done;

    // Instantiate Sequential Modules
    Seq_Div u_div (
        .clk(CLK), .rst(RST), .start(div_start),
        .num(div_num), .den(div_den),
        .quo(div_quo), .done(div_done)
    );

    Seq_Sqrt u_sqrt (
        .clk(CLK), .rst(RST), .start(sqrt_start),
        .rad(sqrt_rad),
        .root(sqrt_root), .done(sqrt_done)
    );

    // =========================================================================
    // FSM & Datapath Logic
    // =========================================================================
    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            current_state <= IDLE;
            x_idx <= 4'd0;
            y_idx <= 4'd0;
            calc_x_flag <= 1'b1;
            SRAM_WE <= 1'b0;
            DONE <= 1'b0;
            div_start <= 1'b0;
            sqrt_start <= 1'b0;
        end else begin
            // Default pulldowns
            SRAM_WE <= 1'b0;
            div_start <= 1'b0;
            sqrt_start <= 1'b0;

            case (current_state)
                IDLE: begin
                    x_idx <= 4'd0;
                    y_idx <= 4'd0;
                    calc_x_flag <= 1'b1;
                    
                    // eta = 1 / RI -> 4096 / RI
                    div_num <= 32'd4096;
                    div_den <= {28'd0, RI};
                    div_start <= 1'b1;
                    current_state <= INIT_DIV_ETA;
                end
                
                INIT_DIV_ETA: current_state <= WAIT_ETA;
                
                WAIT_ETA: begin
                    if (div_done) begin
                        eta_12 <= div_quo;
                        mult_a <= div_quo;
                        mult_b <= div_quo;
                        current_state <= CALC_ETA2;
                    end
                end
                
                CALC_ETA2: begin
                    eta2_12 <= mult_out[43:12]; // (eta * eta) >> 12
                    current_state <= calc_x_flag ? X_POW_1 : Y_POW_1;
                end
                
                // --- X 座標高次方計算 (只有 X 改變時才執行) ---
                X_POW_1: begin
                    dx_reg <= {27'd0, x_idx} - 32'd8; // X - 8
                    mult_a <= {27'd0, x_idx} - 32'd8;
                    mult_b <= {27'd0, x_idx} - 32'd8;
                    current_state <= X_POW_2;
                end
                X_POW_2: begin
                    dx2 <= mult_out[31:0];
                    mult_a <= mult_out[31:0];
                    mult_b <= dx_reg;
                    current_state <= X_POW_3;
                end
                X_POW_3: begin
                    dx3 <= mult_out[31:0];
                    mult_a <= dx2;
                    mult_b <= dx2;
                    current_state <= X_POW_4;
                end
                X_POW_4: begin
                    dx4 <= mult_out[31:0];
                    mult_a <= mult_out[31:0];
                    mult_b <= dx3; // dx4 * dx3 = dx7
                    current_state <= X_POW_5;
                end
                X_POW_5: begin
                    // 數學平移優化: gx12 = dx^7 >> 8
                    gx_12 <= mult_out[39:8]; 
                    mult_a <= dx4;
                    mult_b <= dx4; // dx4 * dx4 = dx8
                    current_state <= Y_POW_1;
                end
                
                // --- Y 座標高次方計算 ---
                Y_POW_1: begin
                    if (calc_x_flag) dx8_s <= mult_out[42:11]; // dx^8 >> 11
                    dy_reg <= {27'd0, y_idx} - 32'd8;
                    mult_a <= {27'd0, y_idx} - 32'd8;
                    mult_b <= {27'd0, y_idx} - 32'd8;
                    current_state <= Y_POW_2;
                end
                Y_POW_2: begin
                    dy2 <= mult_out[31:0];
                    mult_a <= mult_out[31:0];
                    mult_b <= dy_reg;
                    current_state <= Y_POW_3;
                end
                Y_POW_3: begin
                    dy3 <= mult_out[31:0];
                    mult_a <= dy2;
                    mult_b <= dy2;
                    current_state <= Y_POW_4;
                end
                Y_POW_4: begin
                    dy4 <= mult_out[31:0];
                    mult_a <= mult_out[31:0];
                    mult_b <= dy3;
                    current_state <= Y_POW_5;
                end
                Y_POW_5: begin
                    gy_12 <= mult_out[39:8]; // gy12 = dy^7 >> 8
                    mult_a <= dy4;
                    mult_b <= dy4;
                    current_state <= G_POW_1;
                end
                
                // --- 曲面幾何與法向量運算 ---
                G_POW_1: begin
                    dy8_s <= mult_out[42:11]; // dy^8 >> 11
                    // 數學平移優化: Z = 6*4096 - dx^8 - dy^8
                    Z_12 <= 32'd24576 - dx8_s - mult_out[42:11]; 
                    mult_a <= gx_12;
                    mult_b <= gx_12;
                    current_state <= G_POW_2;
                end
                G_POW_2: begin
                    gx2_12 <= mult_out[43:12];
                    mult_a <= gy_12;
                    mult_b <= gy_12;
                    current_state <= G_POW_3;
                end
                G_POW_3: begin
                    gy2_12 <= mult_out[43:12];
                    g2_12 <= gx2_12 + mult_out[43:12] + 32'd4096; // g^2 = gx^2 + gy^2 + 1
                    current_state <= INNER_CALC;
                end
                
                INNER_CALC: begin
                    mult_a <= eta2_12;
                    mult_b <= g2_12; // (eta2 * g2) >> 12
                    current_state <= START_SQRT;
                end
                START_SQRT: begin
                    inner_12 <= g2_12 - mult_out[43:12] + eta2_12;
                    sqrt_rad <= (g2_12 - mult_out[43:12] + eta2_12) << 12; 
                    sqrt_start <= 1'b1;
                    current_state <= WAIT_SQRT;
                end
                WAIT_SQRT: begin
                    if (sqrt_done) begin
                        sqrt_kgg_12 <= {16'd0, sqrt_root};
                        current_state <= START_DIV_COEF;
                    end
                end
                
                // --- 折射係數與時間 t 運算 ---
                START_DIV_COEF: begin
                    div_num <= (eta_12 - sqrt_kgg_12) <<< 12;
                    div_den <= g2_12;
                    div_start <= 1'b1;
                    current_state <= WAIT_COEF;
                end
                WAIT_COEF: begin
                    if (div_done) begin
                        coef_12 <= div_quo;
                        current_state <= START_DIV_T;
                    end
                end
                START_DIV_T: begin
                    div_num <= Z_12 <<< 12;
                    div_den <= eta_12 - coef_12;
                    div_start <= 1'b1;
                    current_state <= WAIT_T;
                end
                WAIT_T: begin
                    if (div_done) begin
                        t_12 <= div_quo;
                        mult_a <= div_quo;
                        mult_b <= coef_12;
                        current_state <= CALC_T_COEF;
                    end
                end
                
                // --- 最終座標運算與寫入 ---
                CALC_T_COEF: begin
                    t_coef_12 <= mult_out[43:12];
                    mult_a <= mult_out[43:12];
                    mult_b <= gx_12;
                    current_state <= CALC_ZX;
                end
                CALC_ZX: begin
                    zx_out <= ({28'd0, x_idx} <<< 12) + mult_out[43:12];
                    mult_a <= t_coef_12;
                    mult_b <= gy_12;
                    current_state <= WRITE_ZX;
                end
                WRITE_ZX: begin
                    SRAM_WE <= 1'b1;
                    SRAM_A  <= {y_idx, x_idx, 1'b0}; // 依據 X 優先寫入 [cite: 165]
                    SRAM_D  <= zx_out[15:0];
                    zy_out  <= ({28'd0, y_idx} <<< 12) + mult_out[43:12];
                    current_state <= WRITE_ZY;
                end
                WRITE_ZY: begin
                    SRAM_WE <= 1'b1;
                    SRAM_A  <= {y_idx, x_idx, 1'b1};
                    SRAM_D  <= zy_out[15:0];
                    current_state <= NEXT_COORD;
                end
                NEXT_COORD: begin
                    if (x_idx == 4'd15 && y_idx == 4'd15) begin
                        current_state <= FINISH;
                    end else if (x_idx == 4'd15) begin
                        x_idx <= 4'd0;
                        y_idx <= y_idx + 1'b1;
                        calc_x_flag <= 1'b1; // Y 跑滿 16 次，X 更新，標記需重算 
                        current_state <= X_POW_1;
                    end else begin
                        x_idx <= x_idx + 1'b1;
                        calc_x_flag <= 1'b0; // X 沒變，直接跳過 X 計算，管線隱藏延遲發揮作用
                        current_state <= Y_POW_1;
                    end
                end
                
                FINISH: begin
                    DONE <= 1'b1;
                    current_state <= FINISH;
                end
            endcase
        end
    end

endmodule


// =========================================================================
// Area-Optimized Sequential Divider (Signed, 32-bit)
// =========================================================================
module Seq_Div (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire signed [31:0] num,
    input  wire signed [31:0] den,
    output reg  signed [31:0] quo,
    output reg         done
);
    reg [5:0] count;
    reg [63:0] remainder;
    reg [31:0] divisor;
    reg sign;

    wire [31:0] abs_num = (num[31]) ? -num : num;
    wire [31:0] abs_den = (den[31]) ? -den : den;
    
    // Compare the shifted 32-bit accumulator against the divisor safely
    wire [32:0] sub_res = {1'b0, remainder[62:31]} - {1'b0, divisor};

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            count <= 0;
            quo <= 0;
            done <= 0;
            remainder <= 0;
            divisor <= 0;
            sign <= 0;
        end else begin
            if (start) begin
                count <= 6'd32;
                remainder <= {32'd0, abs_num};
                divisor <= abs_den;
                sign <= num[31] ^ den[31];
                done <= 0;
            end else if (count > 0) begin
                if (!sub_res[32]) begin // If subtraction didn't underflow (positive)
                    // Replace top bits with result, shift bottom bits, append 1
                    remainder <= {sub_res[31:0], remainder[30:0], 1'b1};
                end else begin
                    // Shift the entire 64-bit register cleanly by 1
                    remainder <= {remainder[62:0], 1'b0};
                end
                count <= count - 1;
                if (count == 1) done <= 1;
            end else begin
                done <= 0;
                // The quotient naturally ends up in the lower 32 bits of the remainder
                quo <= sign ? -remainder[31:0] : remainder[31:0];
            end
        end
    end
endmodule

// =========================================================================
// Area-Optimized Sequential SQRT (Digit-by-digit algorithm)
// =========================================================================
module Seq_Sqrt (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [31:0] rad,
    output reg  [15:0] root,
    output reg         done
);
    reg [4:0]  count;
    reg [31:0] acc;
    reg [31:0] rad_reg;
    
    wire [31:0] next_acc  = {acc[29:0], rad_reg[31:30]};
    wire [31:0] test_val  = {14'd0, root, 2'b01};
    wire        can_sub   = (next_acc >= test_val);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            count   <= 0;
            acc     <= 0;
            root    <= 0;
            rad_reg <= 0;
            done    <= 0;
        end else begin
            if (start) begin
                count   <= 5'd16;
                rad_reg <= rad;
                acc     <= 32'd0;
                root    <= 16'd0;
                done    <= 1'b0;
            end else if (count > 0) begin
                if (can_sub) begin
                    acc  <= next_acc - test_val;
                    root <= {root[14:0], 1'b1};
                end else begin
                    acc  <= next_acc;
                    root <= {root[14:0], 1'b0};
                end
                rad_reg <= {rad_reg[29:0], 2'b00};
                count   <= count - 1'b1;
                if (count == 5'd1) done <= 1'b1;
            end else begin
                done <= 1'b0;
            end
        end
    end
endmodule