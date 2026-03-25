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

    // =========================================================
    // State
    // =========================================================
    localparam S_IDLE      = 5'd0;
    localparam S_PRECALC   = 5'd1;
    localparam S_LOADXY    = 5'd2;
    localparam S_AXAY      = 5'd3;
    localparam S_POW2      = 5'd4;
    localparam S_POW4      = 5'd5;
    localparam S_POW68     = 5'd6;
    localparam S_GEO1      = 5'd7;
    localparam S_GEO2      = 5'd8;
    localparam S_GEO3      = 5'd9;
    localparam S_KGG       = 5'd10;
    localparam S_COEF      = 5'd11;
    localparam S_T         = 5'd12;
    localparam S_FINAL     = 5'd13;
    localparam S_WRITE_X   = 5'd14;
    localparam S_WRITE_Y   = 5'd15;
    localparam S_NEXT      = 5'd16;
    localparam S_DONE      = 5'd17;

    reg [4:0] state, next_state;

    // =========================================================
    // Main iteration
    // =========================================================
    reg [8:0] iteration;
    reg [3:0] x_idx, y_idx;

    // =========================================================
    // Fixed-point registers (Q4.12 unless noted)
    // =========================================================
    reg signed [16:0] big_x, big_y;
    reg signed [16:0] ax, ay;

    reg signed [16:0] ax2, ay2;
    reg signed [16:0] ax4, ay4;
    reg signed [16:0] ax6, ay6;
    reg signed [16:0] ax7, ay7;
    reg signed [16:0] ax8, ay8;

    reg signed [16:0] x8, y8;
    reg signed [16:0] gx, gy;
    reg signed [16:0] gx2, gy2;
    reg signed [16:0] g2;
    reg signed [31:0] kgg;

    reg signed [16:0] big_z;

    reg [15:0] eta;
    reg [15:0] eta2;

    reg signed [15:0] sqrt_kgg_r;
    reg signed [16:0] coef;
    reg signed [16:0] t;
    reg signed [16:0] z_x, z_y;

    // =========================================================
    // Combinational helper wires
    // =========================================================
    wire [31:0] eta_num_w;
    wire [31:0] eta_den_w;
    wire [31:0] eta_div_w;
    wire [63:0] eta_mul_eta_w;
    wire [15:0] eta2_w;

    assign eta_num_w = 32'd1 << 24;
    assign eta_den_w = (RI == 4'd0) ? 32'd1 : {28'd0, RI};

    DW_div #(.a_width(32), .b_width(32), .tc_mode(0))
    U_DIV_ETA (
        .a(eta_num_w),
        .b(eta_den_w),
        .quotient(eta_div_w),
        .remainder()
    );

    assign eta_mul_eta_w = eta_div_w * eta_div_w;
    assign eta2_w        = eta_mul_eta_w >> 36;

    // =========================================================
    // Coordinate -> normalized coordinate
    // =========================================================
    wire signed [16:0] ax_w, ay_w;
    assign ax_w = (big_x - 17'sd32768) >>> 3;   // (X - 8) / 8
    assign ay_w = (big_y - 17'sd32768) >>> 3;   // (Y - 8) / 8

    // =========================================================
    // Power chain
    // =========================================================
    wire signed [33:0] ax2_full_w, ay2_full_w;
    wire signed [16:0] ax2_calc_w, ay2_calc_w;

    assign ax2_full_w = ax * ax;
    assign ay2_full_w = ay * ay;
    assign ax2_calc_w = ax2_full_w >>> 12;
    assign ay2_calc_w = ay2_full_w >>> 12;

    wire signed [33:0] ax4_full_w, ay4_full_w;
    wire signed [16:0] ax4_calc_w, ay4_calc_w;

    assign ax4_full_w = ax2 * ax2;
    assign ay4_full_w = ay2 * ay2;
    assign ax4_calc_w = ax4_full_w >>> 12;
    assign ay4_calc_w = ay4_full_w >>> 12;

    wire signed [33:0] ax6_full_w, ay6_full_w;
    wire signed [33:0] ax8_full_w, ay8_full_w;
    wire signed [16:0] ax6_calc_w, ay6_calc_w;
    wire signed [16:0] ax8_calc_w, ay8_calc_w;

    assign ax6_full_w = ax4 * ax2;
    assign ay6_full_w = ay4 * ay2;
    assign ax8_full_w = ax4 * ax4;
    assign ay8_full_w = ay4 * ay4;

    assign ax6_calc_w = ax6_full_w >>> 12;
    assign ay6_calc_w = ay6_full_w >>> 12;
    assign ax8_calc_w = ax8_full_w >>> 12;
    assign ay8_calc_w = ay8_full_w >>> 12;

    wire signed [33:0] ax7_full_w, ay7_full_w;
    wire signed [16:0] ax7_calc_w, ay7_calc_w;

    assign ax7_full_w = ax6 * ax;
    assign ay7_full_w = ay6 * ay;
    assign ax7_calc_w = ax7_full_w >>> 12;
    assign ay7_calc_w = ay7_full_w >>> 12;

    // =========================================================
    // Geometry chain
    // =========================================================
    wire signed [16:0] x8_w, y8_w;
    wire signed [16:0] gx_w, gy_w;

    assign x8_w = ax8 <<< 1;
    assign y8_w = ay8 <<< 1;
    assign gx_w = ax7 <<< 1;
    assign gy_w = ay7 <<< 1;

    wire signed [33:0] gx2_full_w, gy2_full_w;
    wire signed [16:0] gx2_calc_w, gy2_calc_w;

    assign gx2_full_w = gx * gx;
    assign gy2_full_w = gy * gy;
    assign gx2_calc_w = gx2_full_w >>> 12;
    assign gy2_calc_w = gy2_full_w >>> 12;

    wire signed [16:0] big_z_w, g2_w;
    assign big_z_w = 17'sd24576 - x8 - y8;      // 6 - x8 - y8
    assign g2_w    = gx2 + gy2 + 17'sd4096;     // gx^2 + gy^2 + 1

    // =========================================================
    // kgg / sqrt / coef
    // =========================================================
    wire signed [33:0] eta2_mul_g2_w;
    wire signed [16:0] eta2_g2_q412_w;
    wire signed [31:0] kgg_w;

    assign eta2_mul_g2_w  = $signed({1'b0, eta2}) * $signed(g2);
    assign eta2_g2_q412_w = eta2_mul_g2_w >>> 12;
    assign kgg_w          = $signed(g2) - $signed(eta2_g2_q412_w) + $signed({1'b0, eta2});

    wire [31:0] sqrt_in_w;
    wire [31:0] sqrt_kgg_w;

    assign sqrt_in_w = (kgg[31]) ? 32'd0 : (kgg <<< 12);

    DW_sqrt #(.width(32), .tc_mode(0))
    U_SQRT (
        .a(sqrt_in_w),
        .root(sqrt_kgg_w)
    );

    wire signed [16:0] eta_m_sqrt_w;
    wire signed [31:0] coef_num_w, coef_den_w;
    wire signed [31:0] coef_w;

    assign eta_m_sqrt_w = $signed({1'b0, eta}) - $signed(sqrt_kgg_w[15:0]);
    assign coef_num_w   = $signed(eta_m_sqrt_w) <<< 12;
    assign coef_den_w   = (g2 == 17'sd0) ? 32'sd1 : $signed({{15{g2[16]}}, g2});

    DW_div #(.a_width(32), .b_width(32), .tc_mode(1))
    U_DIV1 (
        .a(coef_num_w),
        .b(coef_den_w),
        .quotient(coef_w),
        .remainder()
    );

    // =========================================================
    // t
    // =========================================================
    wire signed [16:0] eta_m_coef_w;
    wire signed [31:0] t_num_w, t_den_w;
    wire signed [31:0] t_w;

    assign eta_m_coef_w = $signed({1'b0, eta}) - $signed(coef);
    assign t_num_w      = $signed(big_z) <<< 12;
    assign t_den_w      = (eta_m_coef_w == 17'sd0) ? 32'sd1 : $signed({{15{eta_m_coef_w[16]}}, eta_m_coef_w});

    DW_div #(.a_width(32), .b_width(32), .tc_mode(1))
    U_DIV2 (
        .a(t_num_w),
        .b(t_den_w),
        .quotient(t_w),
        .remainder()
    );

    // =========================================================
    // Final zx / zy
    // =========================================================
    wire signed [33:0] t_mul_coef_w;
    wire signed [16:0] t_coef_q412_w;
    wire signed [33:0] gx_tcoef_full_w, gy_tcoef_full_w;
    wire signed [16:0] gx_tcoef_w, gy_tcoef_w;
    wire signed [16:0] z_x_w, z_y_w;

    assign t_mul_coef_w   = $signed(t) * $signed(coef);
    assign t_coef_q412_w  = t_mul_coef_w >>> 12;

    assign gx_tcoef_full_w = $signed(t_coef_q412_w) * $signed(gx);
    assign gy_tcoef_full_w = $signed(t_coef_q412_w) * $signed(gy);

    assign gx_tcoef_w = gx_tcoef_full_w >>> 12;
    assign gy_tcoef_w = gy_tcoef_full_w >>> 12;

    assign z_x_w = big_x + gx_tcoef_w;
    assign z_y_w = big_y + gy_tcoef_w;

    // =========================================================
    // Sequential
    // =========================================================
    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            state      <= S_IDLE;
            SRAM_A     <= 9'd0;
            SRAM_D     <= 16'd0;
            SRAM_WE    <= 1'b0;
            DONE       <= 1'b0;
            iteration  <= 9'd0;
            x_idx      <= 4'd0;
            y_idx      <= 4'd0;

            big_x      <= 17'sd0;
            big_y      <= 17'sd0;
            ax         <= 17'sd0;
            ay         <= 17'sd0;
            ax2        <= 17'sd0;
            ay2        <= 17'sd0;
            ax4        <= 17'sd0;
            ay4        <= 17'sd0;
            ax6        <= 17'sd0;
            ay6        <= 17'sd0;
            ax7        <= 17'sd0;
            ay7        <= 17'sd0;
            ax8        <= 17'sd0;
            ay8        <= 17'sd0;
            x8         <= 17'sd0;
            y8         <= 17'sd0;
            gx         <= 17'sd0;
            gy         <= 17'sd0;
            gx2        <= 17'sd0;
            gy2        <= 17'sd0;
            g2         <= 17'sd0;
            kgg        <= 32'sd0;
            big_z      <= 17'sd0;
            eta        <= 16'd0;
            eta2       <= 16'd0;
            sqrt_kgg_r <= 16'sd0;
            coef       <= 17'sd0;
            t          <= 17'sd0;
            z_x        <= 17'sd0;
            z_y        <= 17'sd0;
        end
        else begin
            state <= next_state;

            case (state)
                S_IDLE: begin
                    SRAM_WE   <= 1'b0;
                    DONE      <= 1'b0;
                    iteration <= 9'd0;
                end

                // eta / eta2 only once
                S_PRECALC: begin
                    eta  <= eta_div_w[27:12];
                    eta2 <= eta2_w;
                end

                S_LOADXY: begin
                    x_idx <= iteration[3:0];
                    y_idx <= iteration[7:4];
                    big_x <= $signed({1'b0, iteration[3:0], 12'd0});
                    big_y <= $signed({1'b0, iteration[7:4], 12'd0});
                end

                S_AXAY: begin
                    ax <= ax_w;
                    ay <= ay_w;
                end

                S_POW2: begin
                    ax2 <= ax2_calc_w;
                    ay2 <= ay2_calc_w;
                end

                S_POW4: begin
                    ax4 <= ax4_calc_w;
                    ay4 <= ay4_calc_w;
                end

                S_POW68: begin
                    ax6 <= ax6_calc_w;
                    ay6 <= ay6_calc_w;
                    ax8 <= ax8_calc_w;
                    ay8 <= ay8_calc_w;
                end

                S_GEO1: begin
                    ax7 <= ax7_calc_w;
                    ay7 <= ay7_calc_w;
                    x8  <= x8_w;
                    y8  <= y8_w;
                end

                S_GEO2: begin
                    gx    <= gx_w;
                    gy    <= gy_w;
                    big_z <= big_z_w;
                end

                S_GEO3: begin
                    gx2 <= gx2_calc_w;
                    gy2 <= gy2_calc_w;
                end

                S_KGG: begin
                    g2  <= g2_w;
                    kgg <= kgg_w;
                end

                S_COEF: begin
                    sqrt_kgg_r <= sqrt_kgg_w[15:0];
                    coef       <= coef_w[16:0];
                end

                S_T: begin
                    t <= t_w[16:0];
                end

                S_FINAL: begin
                    z_x <= z_x_w;
                    z_y <= z_y_w;
                end

                S_WRITE_X: begin
                    SRAM_WE <= 1'b1;
                    SRAM_A  <= iteration << 1;
                    SRAM_D  <= z_x[15:0];
                end

                S_WRITE_Y: begin
                    SRAM_WE <= 1'b1;
                    SRAM_A  <= (iteration << 1) + 9'd1;
                    SRAM_D  <= z_y[15:0];
                end

                S_NEXT: begin
                    SRAM_WE <= 1'b0;
                    if (iteration != 9'd255)
                        iteration <= iteration + 9'd1;
                end

                S_DONE: begin
                    SRAM_WE <= 1'b0;
                    DONE    <= 1'b1;
                end

                default: begin
                    SRAM_WE <= 1'b0;
                    DONE    <= 1'b0;
                end
            endcase
        end
    end

    // =========================================================
    // Next-state logic
    // =========================================================
    always @(*) begin
        case (state)
            S_IDLE    : next_state = S_PRECALC;
            S_PRECALC : next_state = S_LOADXY;
            S_LOADXY  : next_state = S_AXAY;
            S_AXAY    : next_state = S_POW2;
            S_POW2    : next_state = S_POW4;
            S_POW4    : next_state = S_POW68;
            S_POW68   : next_state = S_GEO1;
            S_GEO1    : next_state = S_GEO2;
            S_GEO2    : next_state = S_GEO3;
            S_GEO3    : next_state = S_KGG;
            S_KGG     : next_state = S_COEF;
            S_COEF    : next_state = S_T;
            S_T       : next_state = S_FINAL;
            S_FINAL   : next_state = S_WRITE_X;
            S_WRITE_X : next_state = S_WRITE_Y;
            S_WRITE_Y : next_state = S_NEXT;
            S_NEXT    : next_state = (iteration == 9'd255) ? S_DONE : S_LOADXY;
            S_DONE    : next_state = S_DONE;
            default   : next_state = S_IDLE;
        endcase
    end

endmodule