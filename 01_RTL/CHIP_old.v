//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//
module CHIP #(                                                                                  //
    parameter BIT_W = 32                                                                        //
)(                                                                                              //
    // clock                                                                                    //
        input               i_clk,                                                              //
        input               i_rst_n,                                                            //
    // instruction memory                                                                       //
        input  [BIT_W-1:0]  i_IMEM_data,                                                        //
        output [BIT_W-1:0]  o_IMEM_addr,                                                        //
        output              o_IMEM_cen,                                                         //
    // data memory                                                                              //
        input               i_DMEM_stall,                                                       //
        input  [BIT_W-1:0]  i_DMEM_rdata,                                                       //
        output              o_DMEM_cen,                                                         //
        output              o_DMEM_wen,                                                         //
        output [BIT_W-1:0]  o_DMEM_addr,                                                        //
        output [BIT_W-1:0]  o_DMEM_wdata,                                                       //
    // finnish procedure                                                                        //
        output              o_finish,                                                           //
    // cache                                                                                    //
        input               i_cache_finish,                                                     //
        output              o_proc_finish                                                       //
);                                                                                              //
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any declaration
    parameter auipc_op = 7'b0010111;
    parameter jal_op = 7'b1101111;
    parameter jalr_op = 7'b1100111;
    parameter R_op = 7'b0110011;
    parameter I_op = 7'b0010011;
    parameter LW_op = 7'b0000011;
    parameter SW_op = 7'b0100011;
    parameter MUL_op = 7'b0110011;
    parameter B_op = 7'b1100011;
    parameter ecall_op = 7'b1110011;

    // FSM States
    parameter IDLE = 2'd0, IF = 2'd1, SINGLE = 2'd2, MULTI = 2'd3;

    

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
    reg [BIT_W-1:0] PC, next_PC;
    wire mem_cen, mem_wen;
    wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
    wire mem_stall;

    // Control
    reg Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
    // reg Branch_reg, MemRead_reg, MemtoReg_reg, MemWrite_reg, ALUSrc_reg, RegWrite_reg;
    // reg [1:0] ALUOp;
    // reg [1:0] ALUOp_reg;
    // assign Branch = Branch_reg, MemRead = MemRead_reg, MemtoReg = MemtoReg_reg, ALUOp = ALUOp_reg, MemWrite = MemWrite_reg, ALUSrc = ALUSrc_reg, RegWrite = RegWrite_reg;

    // Register_file
    wire [31:0] ReadData1;
    wire [31:0] ReadData2;
    reg [31:0] ALUresult_nxt, ALUresult;
    reg [63:0] MULresult_nxt, MULresult;
    reg  [31:0] writeData;
    // reg [31:0] ALUresult_nxt_reg;
    // assign ALUresult_nxt = ALUresult_nxt_reg;
    wire zero, wen;

    //state
    reg [1:0] state, state_nxt;
    reg [4:0] counter, counter_nxt;

    // ALU contrl
    reg [6:0] func7;
    reg [2:0] func3;
    reg [3:0] det;
    reg [3:0] ALUcontrol;

    // Data Memory 
    reg[31:0] memData;


    // Instuction Fetch
    reg I_cen_nxt;
    reg I_cen;
    assign o_IMEM_cen = I_cen;
    // Data Fetch
    reg D_cen, D_wen;
    assign o_DMEM_cen = D_cen;
    assign o_DMEM_wen = D_wen;
    reg D_cen_nxt, D_wen_nxt;
    // Data Memory
    reg [31:0] Data_address, writeInData;
    assign o_DMEM_wdata = writeInData;
    assign o_DMEM_addr = Data_address;



    // Finish signal
    reg o_finish_reg;
    reg o_finish_nxt;
    assign o_finish = o_finish_reg;
    
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment
    assign o_IMEM_addr = PC;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (RegWrite),          
        .rs1    (i_IMEM_data[19:15]),                
        .rs2    (i_IMEM_data[24:20]),                
        .rd     (i_IMEM_data[11:7]),                 
        .wdata  (writeData),             
        .rdata1 (ReadData1),           
        .rdata2 (ReadData2)
    );


// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // Todo: any combinational/sequential circuit

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            I_cen <= 0;
            state <= IDLE;
            counter <= 5'd0;
            D_cen <= 0;
            D_wen <= 0;
            ALUresult <= 32'd0;
            MULresult <= 64'd0;
            o_finish_reg <= 0;
        end
        else begin
            PC <= next_PC;
            I_cen <= I_cen_nxt;
            D_cen <= D_cen_nxt;
            D_wen <= D_wen_nxt;
            ALUresult <= ALUresult_nxt;
            MULresult <= MULresult_nxt;
            o_finish_reg <= o_finish_nxt;
            state <= state_nxt;
        end
    end

    // FSM
    always@(*) begin
        case(state)
            IDLE : state_nxt = IF;
            IF: state_nxt = (i_IMEM_data[6:0] == 7'b0110011 && I_cen == 1)? MULTI : SINGLE;
            SINGLE: begin
                if(i_DMEM_stall == 1) begin
                    state_nxt = state;
                end
                else begin
                    state_nxt = IF;
                end
            end
            MULTI: state_nxt = (counter == 31)? IF : MULTI;
            default: state_nxt = state;
        endcase
    end

    always @(*)
    begin
        if (state == IDLE) counter_nxt = 7'd0;
        else if (state == MULTI) counter_nxt = counter + 1;
        else counter_nxt = counter;
    end


    // Instuction fetch
    always@(*) begin
        if (state == IF) begin
            I_cen_nxt = 1;
        end
        else I_cen_nxt = 0;
    end

    

    // PC
    always@(*) begin
        func3 = i_IMEM_data[14:12];
        func7 = i_IMEM_data[31:25];
        if (state == SINGLE) begin
            if(i_IMEM_data[6:0] == B_op) begin
                
                if(func3 == 3'b000) begin
                    if(ReadData1 == ReadData2) next_PC = PC + {i_IMEM_data[31], i_IMEM_data[7], i_IMEM_data[30:25], i_IMEM_data[11:8],1'b0};
                    else next_PC = PC + 4;
                end
                else if(func3 == 3'b101) begin
                    if(ReadData1 >= ReadData2) next_PC = PC + {i_IMEM_data[31], i_IMEM_data[7], i_IMEM_data[30:25], i_IMEM_data[11:8],1'b0};
                    else next_PC = PC + 4;
                end
                else if(func3 == 3'b100) begin
                    if(ReadData1 < ReadData2) next_PC = PC + {i_IMEM_data[31], i_IMEM_data[7], i_IMEM_data[30:25], i_IMEM_data[11:8],1'b0};
                    else next_PC = PC + 4;
                end
                else begin
                    if(ReadData1 != ReadData2) next_PC = PC + {i_IMEM_data[31], i_IMEM_data[7], i_IMEM_data[30:25], i_IMEM_data[11:8],1'b0};
                    else next_PC = PC + 4;
                end     
            end
            else if(i_IMEM_data[6:0] == 7'b1101111) begin
                next_PC = PC + {i_IMEM_data[31], i_IMEM_data[19:12], i_IMEM_data[20], i_IMEM_data[30:21], 1'b0};
            end
            else if(i_IMEM_data[6:0] == 7'b1100111) begin
                next_PC = ReadData1 + i_IMEM_data[31:20];
            end
            else begin
                next_PC = PC + 4;
            end
        end
        else if (state == MULTI) begin
            if (counter == 31) next_PC = PC + 4;
            else next_PC = PC;
        end
        else next_PC = PC;

    end

    

    // Control Signals control
    always@(*) begin
        case(i_IMEM_data[6:0])
            B_op : RegWrite = 0;
            R_op: begin // ADD, SUB, AND, XOR
                RegWrite = 1;
                if({func7, func3} == 4'b0000) begin// ADD
                    ALUresult_nxt = $signed(ReadData1) + $signed(ReadData2);
                end
                else if({func7, func3} == 4'b1000)begin// SUB
                    ALUresult_nxt = $signed(ReadData1) - $signed(ReadData2);
                end
                else if({func7, func3} == 4'b1000) begin// AND
                    ALUresult_nxt = ReadData1 & ReadData2;
                end
                else begin// XOR
                    ALUresult_nxt = ReadData1 ^ ReadData2;
                end
            end
            
            I_op: begin // ADDI, SLLI, SLTI, SRAI
                RegWrite = 1;
                if (func3 == 3'b000) ALUresult_nxt = ReadData1 + i_IMEM_data[31:20];
                else if (func3 == 3'b001) ALUresult_nxt = ReadData1 << i_IMEM_data[24:20];
                else if (func3 == 3'b010) ALUresult_nxt = (ReadData1 < i_IMEM_data[31:20])? 32'd1:32'd0;
                else ALUresult_nxt = ReadData1 >>> i_IMEM_data[24:20];
            end
            LW_op: begin
                RegWrite = 1;
                Data_address = ReadData1 + i_IMEM_data[31:20];
                D_cen_nxt = 1;
                D_wen_nxt = 0;
                if(i_DMEM_stall == 0) ALUresult_nxt = i_DMEM_rdata;
                else ALUresult_nxt = 0;
            end
            SW_op: begin
                RegWrite = 0;
                D_cen_nxt = 1;
                D_wen_nxt = 1;
                writeInData = ReadData2;
                Data_address = ReadData1 + {i_IMEM_data[31:25],i_IMEM_data[11:7]};
            end
            jalr_op: begin
                ALUresult_nxt = PC + 4;
            end
            auipc_op: begin
                ALUresult_nxt = PC + i_IMEM_data[31:12];
            end
            jal_op: begin
                ALUresult_nxt = PC + 4;
            end
            MUL_op: begin
                if (counter == 0)
                begin
                    MULresult_nxt[30:0] = ReadData1[31:1];
                    if (ReadData1[0] == 1) 
                    begin
                        MULresult_nxt[62:31] = ReadData2;
                        MULresult_nxt[63] = 1'd0;
                    end
                    else MULresult_nxt[63:31] = 33'd0;
                end
                else
                begin
                    MULresult_nxt[62:0] = MULresult[63:1];
                    if (MULresult[0] == 1) MULresult_nxt[63:31] = MULresult[63:32] + ReadData2;
                    else 
                    begin
                        MULresult_nxt[63] = 1'd0;
                        ALUresult = MULresult_nxt[31:0];
                    end
                end
            end
            ecall_op: begin
                o_finish_nxt = 1;
            end
            default: begin
                
            end
        endcase
    end
    
    // ALU
    

    // MUX @ write data
    always@(*) begin
        if (MemtoReg == 0) writeData = ALUresult;
        else writeData = memData;
    end

    always@(posedge i_clk or negedge i_rst_n) begin
        
    end

endmodule

    

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// module ALU(i_clk, i_rst_n, instruction, data1, data2, result, zero);
//     input i_clk, i_rst_n;
//     input [31:0] instruction;
//     input [31:0] data1, data2;
//     output [31:0] result;
//     reg [31:0] result_reg;
//     assign result = result_reg;
//     output zero;


//     // FSM state
//     parameter IDLE = 2'd0, SINGLE = 2'd1, MULTI = 2'd2;
    

//     // always@(*) begin
//     //     case(instruction)
            
//     //     endcase
//     // end
// endmodule
module Reg_file(i_clk, i_rst_n, wen, rs1, rs2, rd, wdata, rdata1, rdata2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input i_clk, i_rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] wdata;
    input [addr_width-1:0] rs1, rs2, rd;

    output [BITS-1:0] rdata1, rdata2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign rdata1 = mem[rs1];
    assign rdata2 = mem[rs2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (rd == i)) ? wdata : mem[i];
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module MULDIV_unit(
    // TODO: port declaration
    );
    // Todo: HW2
endmodule

module Cache#(
        parameter BIT_W = 32,
        parameter ADDR_W = 32
    )(
        input i_clk,
        input i_rst_n,
        // processor interface
            input i_proc_cen,
            input i_proc_wen,
            input [ADDR_W-1:0] i_proc_addr,
            input [BIT_W-1:0]  i_proc_wdata,
            output [BIT_W-1:0] o_proc_rdata,
            output o_proc_stall,
            input i_proc_finish,
            output o_cache_finish,
        // memory interface
            output o_mem_cen,
            output o_mem_wen,
            output [ADDR_W-1:0] o_mem_addr,
            output [BIT_W*4-1:0]  o_mem_wdata,
            input [BIT_W*4-1:0] i_mem_rdata,
            input i_mem_stall,
            output o_cache_available,
        // others
        input  [ADDR_W-1: 0] i_offset
    );

    assign o_cache_available = 0; // change this value to 1 if the cache is implemented

    //------------------------------------------//
    //          default connection              //
    assign o_mem_cen = i_proc_cen;              //
    assign o_mem_wen = i_proc_wen;              //
    assign o_mem_addr = i_proc_addr;            //
    assign o_mem_wdata = i_proc_wdata;          //
    assign o_proc_rdata = i_mem_rdata[0+:BIT_W];//
    assign o_proc_stall = i_mem_stall;          //
    //------------------------------------------//

    // Todo: BONUS

endmodule