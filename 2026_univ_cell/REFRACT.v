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
reg [3:0] read_ri;
reg [8:0] iteration;
reg [15:0] input_x;
reg [15:0] input_y;

reg [15:0] temp_x;
reg [15:0] temp_y;

reg [15:0] z_x;
reg [15:0] z_y;

reg [15:0] z_output;


always @(posedge CLK or posedge RST) begin
    if (RST) begin
        state   <= 4'd0;
        SRAM_A  <= 9'd0;
        SRAM_D  <= 16'd0;
        SRAM_WE <= 1'd0;
        DONE    <= 1'd0;
        read_ri <= 4'd0;
        iteration <= 9'd0;
    end
    else begin
        state <= next_state;

        case (state)
            4'd0: begin // INIT
                read_ri <= RI;
                iteration <= 9'd0;
                SRAM_WE <= 1'b0;
                DONE <= 1'b0;
            end

            4'd1: begin // READ_X_ADDR
                SRAM_A  <= iteration << 1;
            end

            4'd2: begin // READ_X_DATA
                input_x <= SRAM_Q;
            end

            4'd3: begin // READ_Y_ADDR
                SRAM_A  <= (iteration << 1) + 9'd1;
            end

            4'd4: begin // READ_Y_DATA
                input_y <= SRAM_Q;
            end

            4'd5: begin // COMPUTING

                //temp_x = (input_x - (16'd8 << 12)) >>> 3;
                z_x = input_x;
                z_y = input_y;
            end

            4'd6: begin // WRITE_X
                SRAM_WE <= 1'b1;
                SRAM_A <= iteration << 1;
                SRAM_D <= z_x;
            end

            4'd7: begin // WRITE_Y
                SRAM_WE <= 1'b1;
                SRAM_A <= (iteration << 1) + 9'd1;
                SRAM_D <= z_y;
            end

            4'd8: begin // NEXT
                SRAM_WE <= 1'b0;
                iteration <= iteration + 9'd1;
            end

            4'd9: begin // FINISH
                SRAM_WE <= 1'b0;
                DONE <= 1'b1;
            end

            default: begin
                DONE <= 1'b0;
            end
        endcase

        $display("t=%0t | state=%0d iter=%0d | A=%0d Q=%0d | D=%0d WE=%0d DONE=%0d",
                 $time, state, iteration, SRAM_A, SRAM_Q, SRAM_D, SRAM_WE, DONE);
    end
end

always @(*) begin
    case (state)
        4'd0: next_state = 4'd1;                              // INIT -> READ_X_ADDR
        4'd1: next_state = 4'd2;                              // READ_X_ADDR -> READ_X_DATA
        4'd2: next_state = 4'd3;                              // READ_X_DATA -> READ_Y_ADDR
        4'd3: next_state = 4'd4;                              // READ_Y_ADDR -> READ_Y_DATA
        4'd4: next_state = 4'd5;                              // READ_Y_DATA -> COMPUTING
        4'd5: next_state = 4'd6;                              // COMPUTING -> WRITE_X
        4'd6: next_state = 4'd7;                              // WRITE_X -> WRITE_Y
        4'd7: next_state = 4'd8;                              // WRITE_Y -> NEXT
        4'd8: next_state = (iteration == 9'd255) ? 4'd9 : 4'd1; // NEXT -> FINISH / READ_X_ADDR
        4'd9: next_state = 4'd9;                              // FINISH
        default: next_state = 4'd0;
    endcase
end

endmodule