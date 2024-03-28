module Decoder(
    input [39:0] GPIO_0,
    output [39:0] GPIO_1,
    input KEY[3:3]);
//Yellow high = Turn left -> GPIO_1[0]
//Green high=Turn right -> GPIO_1[1]
//Channel A = GPIO_0[0] White cable on GPIO
//Channel B = GPIO_0[1] Grey cable on GPIO
reg chaa;
reg chab;
reg [6:0] degree;
reg [5:0] ticks;
wire direction;

assign chaa = GPIO_0[0];
assign chab = GPIO_0[1];
assign GPIO_1[8:2] = degree;
assign GPIO_1[9] = direction;//1 is right, 0 is left

always_ff @(posedge chaa) begin

if(KEY[3] == 1) begin //No reset

    
    if(chab == 1) begin //Turning left
        direction <= 1'b0;
        ticks <= ticks - 1;

        if(ticks == 0) begin //Subtract degree
            if(degree == 0) begin //Rollover degrees
                degree <= 89;
                ticks <= 33;
            end else begin
                degree <= degree - 1;
                ticks <= 33;
            end
        end 

    end 

    if (chab == 0) begin //Turning right
        direction <= 1'b1;
        ticks <= ticks +1;

        if(ticks == 33) begin //Add degree
            if(degree == 89) begin //Rollover degrees
                degree <= 0;
                ticks <= 0;
            end else begin
                degree <= degree + 1;
                ticks <= 0;
            end
     end 
    
    end 

    
end else begin //Reset condition
    ticks <= 0;
    degree <= 0;
    direction <= 1'b1;
end


end


endmodule: Decoder