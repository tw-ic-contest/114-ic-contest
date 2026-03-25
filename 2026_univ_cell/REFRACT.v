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


    reg [3:0] state;
    reg [3:0] next_state;
    reg [8:0] iteration;
    reg [3:0] x_idx;
    reg [3:0] y_idx;

    reg signed [16:0] big_x;
    reg signed [16:0] big_y;

    reg signed [15:0] x8; // 2 * ((X - 8) / 8) ^ 8
    reg signed [15:0] y8; // 2 * ((Y - 8) / 8) ^ 8
    reg signed [16:0] big_z; // z = 6 - 2 * ((X - 8) / 8) ^ 8 - 2 * ((Y - 8) / 8) ^ 8

    reg signed [15:0] gx; // 2 * ((X - 8) / 8) ^ 7
    reg signed [15:0] gy; // 2 * ((X - 8) / 8) ^ 7
    reg signed [15:0] gx2;
    reg signed [15:0] gy2;
    reg signed [16:0] g2; // g ^ 2 = gx ^ 2 + gy ^ 2 + 1

    reg [15:0] eta;
    reg [15:0] eta2;

    reg signed [31:0] kgg;
    reg signed [15:0] sqrt_kgg_r;
    reg signed [16:0] coef; 
    reg signed [16:0] t;

    reg signed [16:0] z_x;
    reg signed [16:0] z_y;


    // -----------------------------
    // combinational helper wires
    // -----------------------------
    wire signed [16:0] big_z_w;
    wire signed [16:0] g2_w;

    wire [31:0] eta_num_w;
    wire [31:0] eta_den_w;
    wire [31:0] eta_w;

    wire [63:0] eta_mul_eta_w;
    wire [15:0] eta2_w;

    wire signed [33:0] mul_eta2_g2_w;
    wire signed [16:0] eta2_g2_q412_w;
    wire signed [31:0] kgg_w;

    wire signed [16:0] eta_m_sqrt_kgg_w;
    wire signed [16:0] eta_m_coef_w;

    wire signed [31:0] sqrt_in_w;
    wire [31:0] sqrt_kgg_w;

    wire signed [31:0] coef_num_w;
    wire signed [31:0] coef_den_w;
    wire signed [31:0] coef_w;

    wire signed [31:0] t_num_w;
    wire signed [31:0] t_den_w;
    wire signed [31:0] t_w;

    wire signed [33:0] t_mul_coef_w;
    wire signed [16:0] t_coef_q412_w;

    wire signed [33:0] gx_mul_t_mul_coef_w;
    wire signed [33:0] gy_mul_t_mul_coef_w;
    wire signed [16:0] gx_t_coef_q412_w;
    wire signed [16:0] gy_t_coef_q412_w;

    wire signed [16:0] z_x_w;
    wire signed [16:0] z_y_w;

    wire signed [16:0] ax_w;
    wire signed [16:0] ay_w;

    wire signed [33:0] ax2_full_w;
    wire signed [16:0] ax2_w;
    wire signed [33:0] ax4_full_w;
    wire signed [16:0] ax4_w;
    wire signed [33:0] ax6_full_w;
    wire signed [16:0] ax6_w;
    wire signed [33:0] ax7_full_w;
    wire signed [16:0] ax7_w;
    wire signed [33:0] ax8_full_w;
    wire signed [16:0] ax8_w;

    wire signed [33:0] ay2_full_w;
    wire signed [16:0] ay2_w;
    wire signed [33:0] ay4_full_w;
    wire signed [16:0] ay4_w;
    wire signed [33:0] ay6_full_w;
    wire signed [16:0] ay6_w;
    wire signed [33:0] ay7_full_w;
    wire signed [16:0] ay7_w;
    wire signed [33:0] ay8_full_w;
    wire signed [16:0] ay8_w;

    wire signed [16:0] gx_w;
    wire signed [16:0] gy_w;

    wire signed [16:0] x8_w;
    wire signed [16:0] y8_w;

    wire signed [33:0] gx2_full_w;
    wire signed [16:0] gx2_w;
    wire signed [33:0] gy2_full_w;
    wire signed [16:0] gy2_w;


    // -----------------------------
    // eta = 1 / RI, output in Q4.12
    // 1.0 in Q4.12 = 4096
    // -----------------------------
    assign eta_num_w = 32'd1 << 24;
    assign eta_den_w = {28'd0, RI};

    DW_div #(.a_width(32), .b_width(32), .tc_mode(0)) 
    U_DIV_ETA (.a(eta_num_w), .b(eta_den_w), .quotient (eta_w), .remainder());

    // -----------------------------
    // eta2 = eta * eta
    // Q4.12 * Q4.12 = Q8.24
    // shift back to Q4.12
    // -----------------------------
    assign eta_mul_eta_w = eta_w * eta_w;
    assign eta2_w = eta_mul_eta_w >> 24;



    // -----------------------------
    // compute gx, gy
    // -----------------------------
    assign ax_w = (big_x - 17'sd32768) >>> 3; // (X - 8) / 8
    assign ay_w = (big_y - 17'sd32768) >>> 3; // (Y - 8) / 8

    assign ax2_full_w = ax_w * ax_w;
    assign ax2_w      = ax2_full_w >>> 12;

    assign ax4_full_w = ax2_w * ax2_w;
    assign ax4_w      = ax4_full_w >>> 12;

    assign ax6_full_w = ax4_w * ax2_w;
    assign ax6_w      = ax6_full_w >>> 12;

    assign ax7_full_w = ax6_w * ax_w;
    assign ax7_w      = ax7_full_w >>> 12;

    assign ay2_full_w = ay_w * ay_w;
    assign ay2_w      = ay2_full_w >>> 12;

    assign ay4_full_w = ay2_w * ay2_w;
    assign ay4_w      = ay4_full_w >>> 12;

    assign ay6_full_w = ay4_w * ay2_w;
    assign ay6_w      = ay6_full_w >>> 12;

    assign ay7_full_w = ay6_w * ay_w;
    assign ay7_w      = ay7_full_w >>> 12;

    assign gx_w = ax7_w <<< 1; // gx = ax7 * 2
    assign gy_w = ay7_w <<< 1; // gy = ax7 * 2

    assign gx2_full_w = gx_w * gx_w;
    assign gx2_w      = gx2_full_w >>> 12;

    assign gy2_full_w = gy_w * gy_w;
    assign gy2_w      = gy2_full_w >>> 12;

    // -----------------------------
    // compute x8, y8
    // -----------------------------

    assign ax8_full_w = ax4_w * ax4_w;
    assign ax8_w = ax8_full_w >>> 12;
    assign x8_w = ax8_w <<< 1;

    assign ay8_full_w = ay4_w * ay4_w;
    assign ay8_w = ay8_full_w >>> 12;
    assign y8_w = ay8_w <<< 1;


    // -----------------------------
    // stage-A combinational wires
    // get big_z, g2, and kgg
    // -----------------------------
    assign big_z_w = 17'sd24576 - $signed({1'b0, x8_w}) - $signed({1'b0, y8_w}); // Z = 6 - x8 - y8

    assign g2_w = $signed(gx2_w) + $signed(gy2_w) + 17'sd4096; // g^2 = gx^2 + gy^2 + 1

    assign mul_eta2_g2_w = $signed(eta2_w) * $signed(g2_w); // eta2 * g2 (Q4.12 * Q4.12 = Q8.24)

    assign eta2_g2_q412_w = mul_eta2_g2_w >>> 12; // eta2 * g2 (back to Q4.12)

    assign kgg_w = g2_w - eta2_g2_q412_w + $signed({1'b0, eta2_w}); // kgg = g2 - eta2 * g2 + eta2



    // -----------------------------
    // sqrt
    // get sqrt_kgg
    // input shift so sqrt output stays around Q4.12
    // -----------------------------
    assign sqrt_in_w = kgg_w <<< 12; //shift to Q8.24 before sqrt

    DW_sqrt #(.width(32), .tc_mode(0)) 
    U_SQRT (.a(sqrt_in_w), .root(sqrt_kgg_w));



    // -----------------------------
    // coef divider
    // coef = (eta - sqrt_kgg) / g2
    // keep quotient in Q4.12 => numerator << 12
    // -----------------------------

    assign eta_m_sqrt_kgg_w = $signed({1'b0, eta}) - $signed(sqrt_kgg_w[27:12]); // eta - sqrt_kgg

    assign coef_num_w = $signed(eta_m_sqrt_kgg_w) <<< 12; //shift for numeration of divider (Q4.12 -> Q8.24)

    assign coef_den_w = $signed({{15{g2[16]}}, g2}); // assign g^2 for denominator

    DW_div #(.a_width(32), .b_width(32), .tc_mode(1)) 
    U_DIV1 (.a(coef_num_w), .b(coef_den_w), .quotient(coef_w), .remainder());


    // -----------------------------
    // t divider
    // t = big_z / (eta - coef)
    // keep quotient in Q4.12 => numerator << 12
    // -----------------------------
    assign eta_m_coef_w = $signed({1'b0, eta}) - $signed(coef); // eta - coef

    assign t_num_w = $signed(big_z) <<< 12;

    assign t_den_w = $signed(eta_m_coef_w);

    DW_div #(.a_width(32), .b_width(32), .tc_mode(1)) 
    U_DIV2 (.a(t_num_w), .b(t_den_w), .quotient(t_w), .remainder());



    // -----------------------------
    // final zx / zy
    // z = big_x/y + t * coef * g
    // multiply twice, each time >> 12
    // -----------------------------
    assign t_mul_coef_w   = $signed(t) * $signed(coef); // t * coef
    assign t_coef_q412_w  = t_mul_coef_w >>> 12; // Q8.24 -> Q4.12

    assign gx_mul_t_mul_coef_w = $signed(t_coef_q412_w) * $signed(gx); // t * coef * gx
    assign gy_mul_t_mul_coef_w = $signed(t_coef_q412_w) * $signed(gy); // t * coef * gy

    assign gx_t_coef_q412_w = gx_mul_t_mul_coef_w >>> 12; // Q8.24 -> Q4.12
    assign gy_t_coef_q412_w = gy_mul_t_mul_coef_w >>> 12; // Q8.24 -> Q4.12

    assign z_x_w = big_x + gx_t_coef_q412_w;
    assign z_y_w = big_y + gy_t_coef_q412_w;




    always @(posedge CLK or posedge RST) begin
        if (RST) begin
            state   <= 4'd0;
            SRAM_A  <= 9'd0;
            SRAM_D  <= 16'd0;
            SRAM_WE <= 1'd0;
            DONE <= 1'd0;
            iteration <= 9'd0;
            x_idx <= 4'd0;
            y_idx <= 4'd0;
            z_x <= 17'd0;
            z_y <= 17'd0;
            big_z <= 16'd0;
            
            //add all the variables later

        end
        else begin
            state <= next_state;

            case (state)
                4'd0: begin // INIT
                    iteration <= 9'd0;
                    DONE <= 1'b0;
                end

                4'd1: begin // GET_COOR
                    x_idx <= iteration[3:0];
                    y_idx <= iteration[7:4];

                    big_x <= $signed({1'b0, iteration[3:0], 12'd0}); // x * 4096, Q4.12
                    big_y <= $signed({1'b0, iteration[7:4], 12'd0}); // y * 4096, Q4.12
                end

                4'd2: begin // COMPUTING
                    big_z <= big_z_w;
                    x8 <= x8_w;
                    y8 <= y8_w;
                    g2 <= g2_w;
                    gx <= gx_w[15:0];
                    gy <= gy_w[15:0];
                    kgg <= kgg_w;
                    eta <= eta_w[27:12];
                    eta2 <= eta2_w;
                    // 1 / RI
                    // eta * eta
                end    
                
                
                4'd3: begin //COMPUTING_2
                    sqrt_kgg_r <= sqrt_kgg_w[27:12];
                    coef <= coef_w[16:0];
                end

                4'd4: begin //COMPUTING_3
                    t <= t_w[16:0];
                end            
                            
                4'd5: begin //COMPUTING_4
                    z_x <= z_x_w;
                    z_y <= z_y_w;
                end

                4'd6: begin // WRITE_X
                    SRAM_WE <= 1'b1;
                    SRAM_A <= iteration << 1;
                    SRAM_D <= z_x[15:0];
                end

                4'd7: begin // WRITE_Y
                    SRAM_WE <= 1'b1;
                    SRAM_A <= (iteration << 1) + 9'd1;
                    SRAM_D <= z_y[15:0];
                end

                4'd8: begin // NEXT
                    SRAM_WE <= 1'b0;
                    if (iteration != 9'd255)
                        iteration <= iteration + 9'd1;

                    $display("%2d %2d  %8.4f  %8.4f  %8.4f  %8.4f  %8.4f  %8.4f  %8.4f",
                        x_idx,
                        y_idx,
                        $itor(big_z) / 4096.0,
                        $itor(g2) / 4096.0,
                        $itor(sqrt_kgg_r) / 4096.0,
                        $itor(coef) / 4096.0,
                        $itor(t) / 4096.0,
                        $itor($unsigned(z_x)) / 4096.0,
                        $itor($unsigned(z_y)) / 4096.0
                    );
                end

                4'd9: begin // FINISH
                    SRAM_WE <= 1'b0;
                    DONE <= 1'b1;
                end

                default: begin
                    DONE <= 1'b0;
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
            4'd8: next_state = (iteration == 9'd255) ? 4'd9 : 4'd1;
            4'd9: next_state = 4'd9;
            default: next_state = 4'd0;
        endcase
    end

    endmodule