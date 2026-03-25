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

    // FSM States
    localparam IDLE      = 4'd0;
    localparam GET_COOR  = 4'd1;
    localparam COMP_G    = 4'd2;
    localparam COMP_KGG  = 4'd3;
    localparam COMP_SQRT = 4'd4;
    localparam COMP_COEF = 4'd5;
    localparam COMP_T    = 4'd6;
    localparam COMP_ZXY  = 4'd7;
    localparam WRITE_X   = 4'd8;
    localparam WRITE_Y   = 4'd9;
    localparam FINISH    = 4'd10;

    reg [3:0] state, next_state;
    reg [8:0] iteration;
    
    // Internal Registers (Fixed Point Q4.12)
    reg [15:0] eta, eta2;
    reg signed [16:0] big_x, big_y, big_z;
    reg signed [16:0] gx, gy;
    reg signed [16:0] g2;
    reg signed [31:0] kgg;
    reg signed [15:0] sqrt_kgg_r;
    reg signed [16:0] coef, t;
    reg signed [16:0] z_x, z_y;

    // --- Shared Divider Logic ---
    reg [31:0] div_a, div_b;
    wire [31:0] div_q;
    reg div_tc; // 0 for unsigned (eta), 1 for signed (coef, t)

    // Resource Sharing: Only ONE DW_div to save Area 
    DW_div #(.a_width(32), .b_width(32), .tc_mode(1)) 
    u_divider (.a(div_a), .b(div_b), .quotient(div_q), .remainder());

    always @(*) begin
        div_a = 32'd0;
        div_b = 32'd1;
        div_tc = 1'b1;
        case (state)
            IDLE: begin // Calculate eta = 1/RI [cite: 116, 148]
                div_a = 32'd1 << 24; 
                div_b = {28'd0, RI};
                div_tc = 1'b0;
            end
            COMP_COEF: begin // coef = (eta - sqrt_kgg) / g2 [cite: 136, 154]
                div_a = ($signed({1'b0, eta}) - $signed(sqrt_kgg_r)) << 12;
                div_b = (g2 == 0) ? 32'd1 : {{15{g2[16]}}, g2};
                div_tc = 1'b1;
            end
            COMP_T: begin // t = -Z / (-eta + coef) [cite: 140, 155]
                div_a = (-big_z) << 12;
                div_b = $signed(coef) - $signed({1'b0, eta});
                div_tc = 1'b1;
            end
        endcase
    end

    // --- Shared Sqrt Logic ---
    wire [31:0] sqrt_in = (kgg[31]) ? 32'd0 : (kgg << 12);
    wire [31:0] sqrt_out;
    DW_sqrt #(.width(32), .tc_mode(0)) u_sqrt (.a(sqrt_in), .root(sqrt_out));

    // --- Power Calculation (x^7, x^8) ---
    // Note: Shifting by 3 (divide by 8) [cite: 126, 150]
    wire signed [16:0] ax = (big_x - 17'sd32768) >>> 3; 
    wire signed [16:0] ay = (big_y - 17'sd32768) >>> 3;
    
    // Intermediate products for power
    wire signed [33:0] px2 = ax * ax;
    wire signed [16:0] ax2 = px2 >>> 12;
    wire signed [33:0] px4 = ax2 * ax2;
    wire signed [16:0] ax4 = px4 >>> 12;
    wire signed [33:0] px6 = ax4 * ax2;
    wire signed [16:0] ax6 = px6 >>> 12;
    wire signed [33:0] px7 = ax6 * ax;
    wire signed [16:0] ax7 = px7 >>> 12;
    wire signed [33:0] px8 = px4 * px4; // ax8
    wire signed [16:0] ax8 = px8 >>> 24; // Q4.12

    wire signed [33:0] py2 = ay * ay;
    wire signed [16:0] ay2 = py2 >>> 12;
    wire signed [33:0] py4 = ay2 * ay2;
    wire signed [16:0] ay4 = py4 >>> 12;
    wire signed [33:0] py6 = ay4 * ay2;
    wire signed [16:0] ay6 = py6 >>> 12;
    wire signed [33:0] py7 = ay6 * ay;
    wire signed [16:0] ay7 = py7 >>> 12;
    wire signed [33:0] py8 = py4 * py4;
    wire signed [16:0] ay8 = py8 >>> 24;

    // --- FSM Control ---
    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            state <= IDLE;
            iteration <= 9'd0;
            DONE <= 1'b0;
            SRAM_WE <= 1'b0;
        end else begin
            state <= next_state;
            case (state)
                IDLE: begin
                    eta  <= div_q[15:0];
                    eta2 <= (div_q[15:0] * div_q[15:0]) >> 12;
                    iteration <= 9'd0;
                    DONE <= 1'b0;
                end
                GET_COOR: begin
                    big_x <= {1'b0, iteration[3:0], 12'd0}; 
                    big_y <= {1'b0, iteration[7:4], 12'd0};
                    SRAM_WE <= 1'b0;
                end
                COMP_G: begin
                    gx <= ax7 << 1; // 2 * ((X-8)/8)^7 [cite: 126]
                    gy <= ay7 << 1;
                    big_z <= 17'sd24576 - (ax8 << 1) - (ay8 << 1); // 6 - 2*... [cite: 146]
                end
                COMP_KGG: begin
                    g2 <= ((gx * gx) >>> 12) + ((gy * gy) >>> 12) + 17'sd4096;
                    // kgg = g2 - eta^2*(g2-1) = g2*(1-eta2) + eta2
                    kgg <= g2 - (((eta2 * g2) >>> 12) - {1'b0, eta2}); [cite: 154]
                end
                COMP_SQRT: begin
                    sqrt_kgg_r <= sqrt_out[15:0];
                end
                COMP_COEF: begin
                    coef <= div_q[16:0]; [cite: 154]
                end
                COMP_T: begin
                    t <= div_q[16:0]; [cite: 155]
                end
                COMP_ZXY: begin
                    z_x <= big_x + (((t * coef) >>> 12) * gx >>> 12); [cite: 157]
                    z_y <= big_y + (((t * coef) >>> 12) * gy >>> 12); [cite: 158]
                end
                WRITE_X: begin
                    SRAM_WE <= 1'b1;
                    SRAM_A <= iteration << 1;
                    SRAM_D <= z_x[15:0]; [cite: 162, 170]
                end
                WRITE_Y: begin
                    SRAM_WE <= 1'b1;
                    SRAM_A <= (iteration << 1) + 9'd1;
                    SRAM_D <= z_y[15:0]; [cite: 162, 170]
                end
                FINISH: begin
                    DONE <= 1'b1; [cite: 69, 93]
                    SRAM_WE <= 1'b0;
                end
            endcase
        end
    end

    always @(*) begin
        case (state)
            IDLE:      next_state = GET_COOR;
            GET_COOR:  next_state = COMP_G;
            COMP_G:    next_state = COMP_KGG;
            COMP_KGG:  next_state = COMP_SQRT;
            COMP_SQRT: next_state = COMP_COEF;
            COMP_COEF: next_state = COMP_T;
            COMP_T:    next_state = COMP_ZXY;
            COMP_ZXY:  next_state = WRITE_X;
            WRITE_X:   next_state = WRITE_Y;
            WRITE_Y:   next_state = (iteration == 9'd255) ? FINISH : GET_COOR;
            FINISH:    next_state = FINISH;
            default:   next_state = IDLE;
        endcase
    end

endmodule