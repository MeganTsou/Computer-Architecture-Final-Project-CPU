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

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
    reg [BIT_W-1:0] PC, PC_plus4, PC_plusimm, PC_branch, PC_nxt;
    reg [1:0] state, state_nxt;
    reg dcen,icen;
    reg [31:0] instruc_delay;
    reg fin_sig;
    
    
    wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
    wire aulsrc,memtoreg,regwrite,memread,memwrite,branch,jal,jalr,auipc;
    wire [1:0] aluop;
    wire [31:0] imm;
    wire [2:0] alu_control;
    wire [31:0] in_A,in_B;
    wire [31:0] rd1;
    wire [31:0] m2r,wd,m2r_nxt;
    wire jump;
    wire block;
    wire [4:0] counter;
    wire alu_mul;
    wire [63:0] o_data;
    wire alu_done;
    wire new;

    
    




// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
    assign in_A = (auipc)? PC : rd1;
    assign in_B = (aulsrc)? imm : o_DMEM_wdata;
    assign m2r = (memtoreg)? i_DMEM_rdata : o_DMEM_addr;
    assign wd = (jal|jalr)? PC_plus4 : m2r;
    assign o_IMEM_addr = PC;
    assign jump = (i_IMEM_data[14:12] == 3'b000 && branch ==1 && o_DMEM_addr ==0)? 1: //jump
                  (i_IMEM_data[14:12] == 3'b001 && branch ==1 && o_DMEM_addr !=0)? 1:
                  (i_IMEM_data[14:12] == 3'b100 && branch ==1 && o_DMEM_addr[31] ==1)? 1:
                  (i_IMEM_data[14:12] == 3'b101 && branch ==1 && o_DMEM_addr[31] ==0)? 1:0;
    assign o_DMEM_wen = memwrite;
    assign o_IMEM_cen = ~block&icen;
    assign o_DMEM_cen = dcen;
    assign block = (memwrite|memread)&(new);
    assign new = (instruc_delay != i_IMEM_data);
    assign o_finish = fin_sig;

    

// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
    Controller control_unit(
        .opcode(i_IMEM_data[6:0]), 
        .ALUSrc(aulsrc), 
        .MemtoReg(memtoreg), 
        .RegWrite(regwrite), 
        .MemRead(memread), 
        .MemWrite(memwrite), 
        .Branch(branch), 
        .Jal(jal), 
        .Jalr(jalr),
        .ALUOp(aluop),
        .Auipc(auipc));

    ImmGen imm_gen_unit(
        .instr(i_IMEM_data), 
        .ALUOp(aluop), 
        .imm(imm));


    ALUControl alu_control_unit(
        .ALUOp(aluop), 
        .funct7({i_IMEM_data[30],i_IMEM_data[25]}), 
        .funct3(i_IMEM_data[14:12]), 
        .opcode(i_IMEM_data[5]),
        .ALU_Control(alu_control));

    ALU alu_unit(
        .ALU_Control(alu_control), 
        .in_A(in_A), 
        .in_B(in_B), 
        .out(o_DMEM_addr),
        .counter(counter),
        .data(o_data),
        .done(alu_done),
        .ismul(alu_mul));


    MULDIV_unit mul(
        .i_clk(i_clk), 
        .i_rst_n(i_rst_n), 
        .i_valid(alu_mul), 
        .i_A(in_A), 
        .i_B(in_B), 
        .o_data(o_data), 
        .counter(counter));


// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (regwrite),          
        .rs1    (i_IMEM_data[19:15]),                
        .rs2    (i_IMEM_data[24:20]),                
        .rd     (i_IMEM_data[11:7]),                 
        .wdata  (wd),             
        .rdata1 (rd1),           
        .rdata2 (o_DMEM_wdata)
    );

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
    // FSM
    always @(*) begin//state handle for mem or not
        case(state)
            2'b00  : begin
                        icen = 1;
                        dcen = 0;
                        if(memwrite|memread) state_nxt = 2'b01;
                        else if(alu_mul) state_nxt = 2'b11;
                        else state_nxt = 2'b00;
                    end
            2'b01   : begin
                        icen = 0;
                        dcen = 1;
                        state_nxt = 2'b10;       
                    end
            2'b10   : begin
                        icen = 0;
                        dcen = 0;
                        if(i_DMEM_stall) state_nxt = 2'b10;
                        else state_nxt = 2'b00;  
                    end
            2'b11   : begin
                        icen = 0;
                        dcen = 0;
                        if(alu_done) state_nxt = 2'b00;
                        else state_nxt = 2'b11;   
                    end          
            default : begin
                        icen = 0;
                        dcen = 0;
                        state_nxt = 2'b00; 
                    end
        endcase
    end
    always @(*) begin//PC handle next PC and  branch PC
        if(o_IMEM_cen) begin // load instruction
            PC_plus4 = PC + 4;
            PC_plusimm = PC + $signed(imm);
            if(jal || jump) PC_branch = PC_plusimm;
            else PC_branch = PC_plus4;
            if(jalr) PC_nxt = $signed(rd1) + $signed(imm);
            else PC_nxt = PC_branch;
        end
        else begin // not load instruction
            PC_nxt = PC;
            PC_plus4 = PC + 4;
            PC_plusimm = PC + $signed(imm);
            if(jal || jump) PC_branch = PC_plusimm;
            else PC_branch = PC_plus4;
        end

    end

    always @(*) begin
        if (i_IMEM_data[6:0] == 7'b1110011) fin_sig = 1'd1;
        else fin_sig = 1'd0;
    end
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // Todo: any combinational/sequential circuit

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            state <= 2'b00;
        end
        else begin
            PC <= PC_nxt;
            state <= state_nxt;
        end
    end


    always @(posedge i_clk) begin
        instruc_delay <= i_IMEM_data;
    end


endmodule

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

module MULDIV_unit(i_clk, i_rst_n, i_valid, i_A, i_B, o_data, counter);
    input          i_clk;
    input          i_rst_n;
    input          i_valid;
    input  [31:0]  i_A;
    input  [31:0]  i_B;
    output [63:0]  o_data;
    output reg [4:0] counter;
    // Todo: HW2
    reg            state, state_nxt;
    reg    [4:0]   counter_nxt;
    reg    [63:0]  shift, next_shift, data, data_nxt;
    reg    [31:0]  alu_in, next_alu_in;
    reg    [32:0]  alu_out;

    reg [31:0] operand_a, operand_a_nxt;
    reg [31:0] operand_b, operand_b_nxt;


    assign o_data = data;
    
    // Todo: FSM
    always @(*) begin
        if (i_valid) begin
            operand_a_nxt = i_A;
            operand_b_nxt = i_B;
            data_nxt[32 - 1 : 0] = i_B;
            //inst_nxt      = i_inst;
        end
        else begin
            operand_a_nxt = operand_a;
            operand_b_nxt = operand_b;
            //inst_nxt      = inst;
        end
    end
    // Todo: FSM
    always @(*) begin
        case(state)
            1'b0 : if (!i_valid) state_nxt = 1'b0;
                   else state_nxt = 1'b1;
            1'b1 : if (counter == 31) state_nxt = 1'b0;
                   else state_nxt = 1'b1;
            default : state_nxt = state;
        endcase
    end

    // Todo: Counter
    always @(*) begin
        if (state == 1'b1) begin
            counter_nxt = counter + 1;
        end 
        else counter_nxt = 0;
    end

    // Todo: ALU output
    always @(*) begin
        // multiplication
        if (data[0] == 1'b0) begin  // if bit = 0
            data_nxt = data >> 1;
        end
        else begin  // if bit = 1
            data_nxt = data >> 1;
            data_nxt[63 : 32 - 1] = data_nxt[2*32 - 1 : 32 - 1] + operand_a;
        end
        
        // initialize
        // if (state == S_IDLE) begin
        //     data_nxt = 0;
        //     if (i_valid == 1) begin
        //         if (i_inst == 6) begin
        //             data_nxt[32 - 1 : 0] = i_B;
        //         end
        //         else if (i_inst == 7) begin
        //             data_nxt[32 : 0] = {i_A, 1'b0};
        //         end
        //     end
        // end
    end

    // // Todo: output valid signal
    // always @(*) begin
    //     if (state == S_MULTI_CYCLE_OP) begin
    //         if (counter == 31) begin
    //             done_nxt = 1;
    //         end
    //         else done_nxt = 0;
    //     end
        
    //     else done_nxt = 0;
    // end
 
    // Todo: Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state       <= 1'b0;
            operand_a   <= 0;
            operand_b   <= 0;
            counter         <= 0;
            data        <= 0;
        end
        else begin
            state       <= state_nxt;
            operand_a   <= operand_a_nxt;
            operand_b   <= operand_b_nxt;
            counter         <= counter_nxt;
            data        <= data_nxt;
        end
    end
    // 0: idle
    // 1: mul
    // always @(*) begin
    //     case(state)
    //         1'b0: begin
    //             // IDLE
    //             if(i_valid) begin
    //                 if(shift[0]) alu_out = shift[63:32] + alu_in;
    //                 else alu_out = {1'b0,shift[63:32]};
    //                 state_nxt = 1'b1;
    //             end
    //             else begin
    //                 state_nxt = 1'b0;
    //                 alu_out = 33'b0;
    //             end
    //         end
    //         1'b1: begin
    //             // MUL
    //             if(shift[0]) alu_out = shift[63:32] + alu_in;
    //             else alu_out = {1'b0,shift[63:32]};
    //             if(counter==31) state_nxt = 1'b0;
    //             else state_nxt = 1'b1;
    //         end
    //     endcase
    // end
    // // Todo: Counter
    // always @(*) begin
    //     case(state)
    //         1'b0: counter_nxt = 0;
    //         1'b1: begin
    //             if(counter==31) counter_nxt = 1;
    //             else counter_nxt = counter + 1;
    //         end
    //     endcase
    // end
    // // Todo: ALU output
    // always @(*) begin
    //     case(state)
    //         1'b0: begin
    //             if(i_valid) next_alu_in = {32'b0,i_B};
    //             else next_alu_in = 0;
    //         end
    //         1'b1: next_alu_in = alu_in;
    //     endcase
    // end
    // // Todo: Shift register
    // always @(*) begin
    //     case(state) 
    //         1'b0: begin
    //             data = 0;
    //             if(i_valid) next_shift = {32'b0,i_A};
    //             else next_shift = 0;
    //         end
    //         1'b1 : begin
    //             next_shift = {alu_out,shift[31:1]};
    //             if(counter == 31) data = next_shift;
    //             else data = 0;
    //         end
    //     endcase
    // end
    // // Todo: Sequential always block
    // always @(posedge i_clk or negedge i_rst_n) begin
    //     if (!i_rst_n) begin
    //         state <= 1'b0;
    //         counter <= 5'b0;
    //         alu_in <= 32'b0;
    //         shift <= 64'b0;
    //     end
    //     else begin
    //         state <= state_nxt;
    //         counter <= counter_nxt;
    //         alu_in <= next_alu_in;
    //         shift <= next_shift;
    //     end
    // end
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





module Controller(opcode, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, Jal, Jalr, ALUOp, Auipc);
    input  [6:0] opcode;
    output       ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, Jal, Jalr, Auipc;
    output [1:0] ALUOp;
    reg    [10:0] control;

    assign {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, Jal, Jalr, ALUOp, Auipc} = control;

    always @(*) begin

        control = 11'b00000000000;
        case(opcode)
            7'b0110011: control = 11'b00100000100; // R-type
            7'b0010011: control = 11'b10100000100; // I-type
            7'b0000011: control = 11'b11110000000; // lw
            7'b0100011: control = 11'b10001000000; // sw
            7'b1100011: control = 11'b00000100010; // beq, bne, blt, bge
            7'b1101111: control = 11'b10100010000; // jal
            7'b1100111: control = 11'b10100001000; // jalr
            7'b0010111: control = 11'b10100000001; // auipc
            // 7'b1110011: control = 11'b00000000000; // ecall
        endcase
    end
endmodule

module ALUControl(ALUOp, funct7, funct3, opcode, ALU_Control);
    input      [1:0] ALUOp, funct7;
    input      [2:0] funct3;
    input            opcode;
    output reg [2:0] ALU_Control;

    always @(*) begin
        case(ALUOp)
            2'b00: ALU_Control = 3'b000; // add
            2'b01: ALU_Control = 3'b001; // sub
            2'b10: begin
                if(opcode) begin
                    case(funct3)
                        3'b000: begin
                            ALU_Control = (funct7==2'b00) ? 3'b000 :          // add
                                          (funct7==2'b10) ? 3'b001 : 3'b011;  // sub, mul
                        end
                        3'b111: ALU_Control = 3'b010; // and
                        3'b100: ALU_Control = 3'b100; // xor
                        default: ALU_Control = 3'b000;
                    endcase
                end
                else begin
                    case(funct3)
                        3'b000: ALU_Control = 3'b000; // add
                        3'b001: ALU_Control = 3'b101; // sll
                        3'b101: ALU_Control = 3'b110; // sra
                        3'b010: ALU_Control = 3'b111; // slt
                        default: ALU_Control = 3'b000;
                    endcase

                end
            end
            default: ALU_Control = 3'b000;
        endcase
    end
endmodule

module ALU(ALU_Control, in_A, in_B, out, counter, data, done, ismul);
    input      [2:0]  ALU_Control;
    input      [31:0] in_A, in_B;
    input      [4:0]  counter;
    input      [63:0] data;
    output reg [31:0] out;
    output reg        done;
    output            ismul;  

    reg mul;

    assign ismul = mul; 
    always @(*) begin
        out = 32'b0;
        case(ALU_Control)
            3'b000: begin
                out = $signed(in_A) + $signed(in_B); // add
                done = 1'b1;
                mul = 1'b0;
            end
            3'b001: begin
                out = $signed(in_A) - $signed(in_B); // sub
                done = 1'b1;
                mul = 1'b0;
            end
            3'b010: begin
                out = in_A & in_B; // and
                done = 1'b1;
                mul = 1'b0;
            end
            3'b011: begin
                out = data[31:0]; // mul
                if(counter==31) done = 1'b1;
                else done = 1'b0;  
                mul = 1'b1;    
            end
            3'b100: begin
                out = in_A ^ in_B; // xor
                done = 1'b1;
                mul = 1'b0;
            end
            3'b101: begin
                out = in_A << $signed(in_B[4:0]); // slli
                done = 1'b1;
                mul = 1'b0;
            end
            3'b110: begin
                out = in_A >>> $signed(in_B[4:0]); // srai
                done = 1'b1;
                mul = 1'b0;
            end
            3'b111: begin
                out = ($signed(in_A) < $signed(in_B)) ? 1:0; // slti
                done = 1'b1;
                mul = 1'b0;
            end
        endcase
    end
endmodule

module ImmGen(instr, ALUOp, imm);
    input      [31:0] instr;
    input      [1:0]  ALUOp;
    output reg [31:0] imm;

    always @(*) begin      
        case(ALUOp)
            2'b10: imm = {{20{instr[31]}}, instr[31:20]}; // I-type
            2'b01: imm = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; // beq, bne, blt, bge
            2'b00: begin
                case(instr[6:0])
                    7'b0000011: imm = {{20{instr[31]}}, instr[31:20]}; // lw
                    7'b0100011: imm = {{20{instr[31]}}, instr[31:25], instr[11:7]}; // sw
                    7'b1101111: imm = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; // jal
                    7'b1100111: imm = {{20{instr[31]}}, instr[31:20]}; // jalr
                    7'b0010111: imm = {instr[31:12],12'b0}; // auipc
                    default: imm = 32'b0;
                endcase
            end
            default: imm = 32'b0;
        endcase
    end
endmodule









// module Cache#(
//         parameter BIT_W = 32,
//         parameter ADDR_W = 32
//     )(
//         input i_clk,
//         input i_rst_n,
//         // processor interface
//             input i_proc_cen,
//             input i_proc_wen,
//             input [ADDR_W-1:0] i_proc_addr,
//             input [BIT_W-1:0]  i_proc_wdata,
//             output [BIT_W-1:0] o_proc_rdata,
//             output o_proc_stall,
//         // memory interface
//             output o_mem_cen,
//             output o_mem_wen,
//             output [ADDR_W-1:0] o_mem_addr,
//             output [BIT_W-1:0]  o_mem_wdata,
//             input [BIT_W-1:0] i_mem_rdata,
//             input i_mem_stall
//     );


// /*    //---------------------------------------//
//     //          default connection           //
//     assign o_mem_cen = i_proc_cen;        //
//     assign o_mem_wen = i_proc_wen;        //
//     assign o_mem_addr = i_proc_addr;      //
//     assign o_mem_wdata = i_proc_wdata;    //
//     assign o_proc_rdata = i_mem_rdata;    //
//     assign o_proc_stall = i_mem_stall;    //
//     //---------------------------------------//

// */
//     // Todo: BONUS
 
//     //==== state definition ===================================
//         parameter READY = 2'b00; 
//         parameter MISS  = 2'b01;
//         parameter WRITE  = 2'b10; 
//     //==== wire/reg definition ================================
//         // 30 = 28 + 2
//         wire [29:0]  proc_addr;
//         reg          mem_cen,mem_wen,proc_stall;
//         reg          mem_write_w, mem_write_r;
//         reg  [31:0]  mem_addr_w, mem_addr_r;
//         reg  [31:0]  mem_wdata_w, mem_wdata_r;
//         reg  [1:0]   state_w, state_r;
//         reg  [63:0]  cache_w[0:3], cache_r[0:3];
//         reg  [55:0]  tag_w[0:3], tag_r[0:3];
//         reg          lru_w[0:3], lru_r[0:3];
//         wire [1:0]   index;
//         wire [63:0]  data;
//         wire [55:0]  tag;
//         wire         lru;
//         wire         hit1, hit0;
//         wire         hit;
//         integer i;

//     //==== combinational circuit ==============================
//         assign proc_addr = i_proc_addr[31:2];
//         assign index = proc_addr[1:0];
//         assign {tag, data, lru} = {tag_r[index], cache_r[index], lru_r[index]};
//         assign hit1 = (tag[55:28] == proc_addr[29:2]);
//         assign hit0 = (tag[27:0] == proc_addr[29:2]);
//         assign hit = hit1 || hit0;
//         assign o_mem_addr = mem_addr_r;
//         assign o_mem_wdata = mem_wdata_r;
//         assign o_mem_cen = mem_cen;
//         assign o_mem_wen = mem_wen;
//         assign o_proc_stall = proc_stall;
//         assign o_proc_rdata = (hit1)? data[63:32]: data[31:0];

//        always @(*) begin
//             case(state_r)
//                 READY: begin
//                     if(i_proc_cen) begin
//                         if(hit) begin
//                             if(!i_proc_wen) begin
//                                 // read hit
//                                 proc_stall = 1'b0;
//                                 state_w = state_r;
//                                 mem_cen = 1'b0;
//                                 mem_wen = 1'b0;
//                                 mem_addr_w = mem_addr_r;
//                                 mem_wdata_w = mem_wdata_r;
//                             end
//                             else begin
//                                 // write hit
//                                 proc_stall = 1'b1;
//                                 state_w = WRITE;
//                                 mem_cen = 1'b1;
//                                 mem_wen = 1'b1;
//                                 mem_addr_w = i_proc_addr;
//                                 mem_wdata_w = i_proc_wdata;
//                             end
//                         end
//                         else begin
//                             // miss
//                             proc_stall = 1'b1;
//                             state_w = MISS;
//                             mem_cen = 1'b1;
//                             mem_wen = 1'b0;
//                             mem_addr_w = i_proc_addr;
//                             mem_wdata_w = lru ? data[63:32] : data[31:0];
//                         end
//                     end
//                     else begin
//                         proc_stall = 1'b0;
//                         state_w = state_r;
//                         mem_cen = 1'b0;
//                         mem_wen = 1'b0;
//                         mem_addr_w = mem_addr_r;
//                         mem_wdata_w = mem_wdata_r;

//                     end
//                 end
//                 MISS: begin
//                     // read
//                     if(~i_mem_stall) begin// no stall
//                         if(!i_proc_wen) begin
//                             // read miss
//                             proc_stall = 1'b1;
//                             state_w = READY;
//                             mem_cen = 1'b0;
//                             mem_wen = 1'b0;
//                             mem_addr_w = i_proc_addr;
//                             mem_wdata_w = lru ? data[63:32] : data[31:0];
//                         end
//                         else begin
//                             // write miss
//                             proc_stall = 1'b1;
//                             state_w = WRITE;
//                             mem_cen = 1'b1;
//                             mem_wen = 1'b1;
//                             mem_addr_w = i_proc_addr;
//                             mem_wdata_w = i_proc_wdata;
//                         end
//                     end
//                     else begin//stall
//                         proc_stall = 1'b1;
//                         state_w = MISS;
//                         mem_cen = 1'b1;
//                         mem_wen = 1'b0;
//                         mem_addr_w = mem_addr_r;
//                         mem_wdata_w = lru ? data[63:32] : data[31:0];
//                     end
//                 end
//                 WRITE: begin
//                     if(~i_mem_stall) begin
//                             proc_stall = 1'b1;
//                             state_w = READY;
//                             mem_cen = 1'b0;
//                             mem_wen = 1'b0;
//                             mem_addr_w = i_proc_addr;
//                             mem_wdata_w = lru ? data[63:32] : data[31:0];
//                         end
//                     else begin
//                             proc_stall = 1'b1;
//                             state_w = WRITE;
//                             mem_cen = 1'b1;
//                             mem_wen = 1'b1;
//                             mem_addr_w = mem_addr_r;
//                             mem_wdata_w = mem_wdata_r;
//                         end
//                     end
//                 default: begin
//                     proc_stall = 1'b1;
//                     state_w = READY;
//                     mem_cen = 1'b0;
//                     mem_wen = 1'b1;
//                     mem_addr_w = mem_addr_r;
//                     mem_wdata_w = mem_wdata_r;
//                 end
//             endcase
//         end

//     //==== sequential circuit =================================
//     always@( posedge i_clk ) begin
//         if( !i_rst_n ) begin
//             state_r <= READY;
//             mem_write_r <= 1'b0;
//             mem_addr_r <= 32'b0;
//             mem_wdata_r <= 32'b0;
//             for(i=0;i<4;i=i+1) begin
//                 cache_r[i] <= 64'b0;
//                 tag_r[i] <= 56'hff_ffff_ffff_ffff;
//                 lru_r[i] <= 1'b0;
//             end
//         end
//         else begin
//             state_r <= state_w;
//             mem_write_r <= mem_write_w;
//             mem_addr_r <= mem_addr_w;
//             mem_wdata_r <= mem_wdata_w;
//             for(i=0;i<4;i=i+1) begin
//                 cache_r[i] <= cache_w[i];
//                 tag_r[i] <= tag_w[i];
//                 lru_r[i] <= lru_w[i];
//             end
//         end
//     end
    

// endmodule


