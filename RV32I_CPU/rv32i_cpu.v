//
//  Author: Prof. Taeweon Suh
//          Computer Science & Engineering
//          Korea University
//  Date: July 14, 2020
//  Description: Skeleton design of RV32I Single-cycle CPU
//
//  Edited by Seunggi Moon
//
//  Changes:
//  + Add additional instruction support
//  + Add pipelining
//  + Add control hazard detection & handling logic
//  * Code refactoring


`timescale 1ns/1ns
`define simdelay 1

module rv32i_cpu (
            input         clk, reset,
            output [31:0] pc,		  		// program counter for instruction fetch
            input  [31:0] inst, 			// incoming instruction
            output        Memwrite, 	// 'memory write' control signal
            output [31:0] Memaddr,  	// memory address 
            output [31:0] MemWdata, 	// data to write to memory
            input  [31:0] MemRdata); 	// data read from memory

  wire        auipc, lui;
  wire [31:0] IFID_inst; // [Seunggi Moon] Edit: Connect to IF/ID stage instr register
  wire        alusrc, regwrite;
  wire [4:0]  alucontrol;
  wire        memtoreg, memwrite;
  wire        branch, jal, jalr;

  // Instantiate Controller
  controller i_controller(
    // Controller Input
	 
	 // [Seunggi Moon] Edit: Feed IFID_inst instead of inst into the controller
		.opcode		(IFID_inst[6:0]), 
		.funct7		(IFID_inst[31:25]), 
		.funct3		(IFID_inst[14:12]), 

    // Controller Output
		.auipc		(auipc),
		.lui			(lui),
		.memtoreg	(memtoreg),
		.memwrite	(memwrite),
		.branch		(branch),
		.alusrc		(alusrc),
		.regwrite	(regwrite),
		.jal			(jal),
		.jalr			(jalr),
		.alucontrol	(alucontrol));

  // Instantiate Datapath
  datapath i_datapath(
    // Datapath Input
		.clk				(clk),
		.reset			(reset),
		.auipc			(auipc),
		.lui				(lui),
		.memtoreg		(memtoreg),
		.memwrite		(memwrite),
		.branch			(branch),
		.alusrc			(alusrc),
		.regwrite		(regwrite),
		.jal				(jal),
		.jalr				(jalr),
		.alucontrol (alucontrol),
		.inst				(inst),
		.MemRdata		(MemRdata),
		.opcode     (IFID_inst[6:0]), // [Seunggi Moon] Edit: add opcode as an input to datapath

    // Datapath Output
		.pc				  (pc),
		.MemWdata		(MemWdata),

    // [Seunggi Moon] Edit: Use MEM stage register to access memory properly
		.EXMEM_aluout (Memaddr),
		.EXMEM_memwrite (Memwrite),
		
		.IFID_inst  (IFID_inst));
endmodule

//
// Instruction Decoder 
// to generate control signals for datapath
//
module controller(input  [6:0] opcode,
                  input  [6:0] funct7,
                  input  [2:0] funct3,
                  output       auipc,
                  output       lui,
                  output       alusrc,
                  output [4:0] alucontrol,
                  output       branch,
                  output       jal,
                  output       jalr,
                  output       memtoreg,
                  output       memwrite,
                  output       regwrite);

	maindec i_maindec(
		.opcode		(opcode),
		.auipc		(auipc),
		.lui			(lui),
		.memtoreg	(memtoreg),
		.memwrite	(memwrite),
		.branch		(branch),
		.alusrc		(alusrc),
		.regwrite	(regwrite),
		.jal			(jal),
		.jalr			(jalr));

	aludec i_aludec( 
		.opcode     (opcode),
		.funct7     (funct7),
		.funct3     (funct3),
		.alucontrol (alucontrol));
endmodule

//
// RV32I Opcode map = Inst[6:0]
//
`define OP_R        7'b0110011
`define OP_I_ARITH	7'b0010011
`define OP_I_LOAD  	7'b0000011
`define OP_I_JALR  	7'b1100111
`define OP_S        7'b0100011
`define OP_B        7'b1100011
`define OP_U_LUI		7'b0110111
`define OP_U_AUIPC  7'b0010111 // [Seunggi Moon] Edit: Add opcode for AUIPC
`define OP_J_JAL    7'b1101111

//
// Main decoder generates all control signals except alucontrol 
//
//
module maindec(input  [6:0] opcode,
               output       auipc,
               output       lui,
               output       regwrite,
               output       alusrc,
               output       memtoreg, memwrite,
               output       branch, 
               output       jal,
               output       jalr);

  reg [8:0] controls;

  assign {auipc, lui, regwrite, alusrc, 
			 memtoreg, memwrite, branch, jal, 
			 jalr} = controls;

  always @(*)
  begin
    case(opcode)
      `OP_R: 			  controls <= #`simdelay 9'b0010_0000_0; // R-type
      `OP_I_ARITH: 	controls <= #`simdelay 9'b0011_0000_0; // I-type Arithmetic
      `OP_I_LOAD: 	controls <= #`simdelay 9'b0011_1000_0; // I-type Load

      `OP_I_JALR: 	controls <= #`simdelay 9'b0011_0000_1; // [Seunggi Moon] Edit: Add case for JALR

      `OP_S: 			  controls <= #`simdelay 9'b0001_0100_0; // S-type Store
      `OP_B: 			  controls <= #`simdelay 9'b0000_0010_0; // B-type Branch
      `OP_U_LUI: 		controls <= #`simdelay 9'b0111_0000_0; // LUI

      `OP_U_AUIPC:  controls <= #`simdelay 9'b0111_0000_0; // [Seunggi Moon] Edit: Add case for AUIPC

      `OP_J_JAL: 		controls <= #`simdelay 9'b0011_0001_0; // JAL
      default:    	controls <= #`simdelay 9'b0000_0000_0; // ???
    endcase
  end
endmodule

//
// ALU decoder generates ALU control signal (alucontrol)
//
module aludec(input      [6:0] opcode,
              input      [6:0] funct7,
              input      [2:0] funct3,
              output reg [4:0] alucontrol);

  always @(*)
    case(opcode)
      `OP_R:   		// R-type
        begin
          case({funct7,funct3})
            10'b0000000_000: alucontrol <= #`simdelay 5'b00000; // addition (add)
            10'b0100000_000: alucontrol <= #`simdelay 5'b10000; // subtraction (sub)
            10'b0000000_111: alucontrol <= #`simdelay 5'b00001; // and (and)
            10'b0000000_110: alucontrol <= #`simdelay 5'b00010; // or (or)
            default:         alucontrol <= #`simdelay 5'bxxxxx; // ???
          endcase
        end

      `OP_I_ARITH:   // I-type Arithmetic
        begin
          casez({funct7, funct3}) // [Seunggi Moon] Edit: Capture funct7
            10'b???????_000:  alucontrol <= #`simdelay 5'b00000; // addition (addi)

            // [Seunggi Moon] Edit: Add case for XORI
            10'b???????_100:  alucontrol <= #`simdelay 5'b00011; 

            10'b???????_110:  alucontrol <= #`simdelay 5'b00010; // or (ori)
            10'b???????_111:  alucontrol <= #`simdelay 5'b00001; // and (andi)

            // [Seunggi Moon] Edit: Add case for SLLI
            10'b0000000_001:  alucontrol <= #`simdelay 5'b00100;
            
            default: alucontrol <= #`simdelay 5'bxxxxx; // ???
          endcase
        end

      `OP_I_LOAD:   // I-type Load (LW, LH, LB...)
      	alucontrol <= #`simdelay 5'b00000;  // addition 

      `OP_U_AUIPC:  // [Seunggi Moon] Edit: Add case for AUIPC
        alucontrol <= #`simdelay 5'b00000;  // addition 

      `OP_B:        // B-type Branch (BEQ, BNE, ...)
      	alucontrol <= #`simdelay 5'b10000;  // subtraction 
      
      `OP_I_JALR:   // [Seunggi Moon] Edit: Add case for JALR
        alucontrol <= #`simdelay 5'b00000;  // addition

      `OP_S:        // S-type Store (SW, SH, SB)
      	alucontrol <= #`simdelay 5'b00000;  // addition 

      `OP_U_LUI: 		// U-type (LUI)
      	alucontrol <= #`simdelay 5'b00000;  // addition

      default: 
      	alucontrol <= #`simdelay 5'b00000;  // 
    endcase
endmodule

//
// CPU datapath
//
module datapath(input         clk, reset,
                input  [31:0] inst,
                input         auipc,
                input         lui,
                input         regwrite,
                input         memtoreg,
                input         memwrite,
                input         alusrc, 
                input  [4:0]  alucontrol,
                input         branch,
                input         jal,
                input         jalr,
                input  [6:0]  opcode, // [Seunggi Moon] Edit: add opcode as an input to datapath (to detect whether the previous instr read memory or not)
                input  [31:0] MemRdata,

                output reg [31:0] pc,
                output [31:0]     MemWdata,

                // [Seunggi Moon] Edit: Use MEM stage register to access memory properly
                output reg [31:0] EXMEM_aluout,
                output reg        EXMEM_memwrite,
					 
                output reg [31:0] IFID_inst);

  wire [4:0]  rs1, rs2, rd;
  wire [2:0]  funct3;
  reg  [31:0] rs1_data, rs2_data;
  reg  [31:0] rd_data;
  wire [20:1] jal_imm;
  wire [31:0] se_jal_imm;
  wire [12:1] br_imm;
  wire [31:0] se_br_imm;
  wire [31:0] se_imm_itype;
  wire [31:0] se_imm_stype;
  wire [31:0] auipc_lui_imm;
  reg  [31:0] alusrc1;
  reg  [31:0] alusrc2;
  wire [31:0] branch_dest, jal_dest, jalr_dest; // [Seunggi Moon] Edit: Add jalr destination
  wire		    Nflag, Zflag, Cflag, Vflag;
  wire		    f3beq, f3blt, f3bgeu, f3bne; // [Seunggi Moon] Edit: Add additional wire for bgeu, bne
  wire		    beq_taken;
  wire		    blt_taken;
  wire		    bgeu_taken; // [Seunggi Moon] Edit: Add bgeu_taken
  wire		    bne_taken; // [Seunggi Moon] Edit: Add bne_taken

  // [Seunggi Moon] Edit: Add additional wires/flip-flops for pipelining

  // IF -> ID
  reg  [31:0] IFID_pc;

  // ID -> EX
  reg  [31:0] IDEX_pc;
  reg  [31:0] IDEX_rs1_data, IDEX_rs2_data;
  reg  [31:0] IDEX_se_jal_imm;
  reg  [31:0] IDEX_se_br_imm;
  reg  [31:0] IDEX_se_imm_itype;
  reg  [31:0] IDEX_se_imm_stype;
  reg  [31:0] IDEX_auipc_lui_imm;
  reg  [6:0]  IDEX_opcode;
  reg  [4:0]  IDEX_alucontrol;
  reg  [4:0]  IDEX_rs1;
  reg  [4:0]  IDEX_rs2;
  reg  [4:0]  IDEX_rd;
  reg  [2:0]  IDEX_funct3;
  reg         IDEX_auipc, IDEX_lui, IDEX_alusrc, IDEX_branch, IDEX_jal, IDEX_jalr;
  reg         IDEX_memtoreg;
  reg         IDEX_memwrite;
  reg         IDEX_regwrite;

  // EX -> MEM
  reg  [31:0] EXMEM_pc;
  reg  [31:0] EXMEM_rs2_data;
  reg  [6:0]  EXMEM_opcode;
  reg  [4:0]  EXMEM_rd;
  reg         EXMEM_jal, EXMEM_jalr;
  reg         EXMEM_memtoreg;
  reg         EXMEM_regwrite;

  // MEM -> WB
  reg  [31:0] MEMWB_pc;
  reg  [31:0] MEMWB_MemRdata;
  reg  [31:0] MEMWB_aluout;
  reg  [4:0]  MEMWB_rd;
  reg         MEMWB_jal, MEMWB_jalr;
  reg         MEMWB_memtoreg;
  reg         MEMWB_regwrite;

  // MISC
  wire [31:0] aluout;
  wire [31:0] rfrs1_data;
  wire [31:0] rfrs2_data;
  
  // Stall signal
  wire        stall;

  // Flush signal
  wire        flush;



  /////////////////////////////////////////
  // IF STAGE
  /////////////////////////////////////////

  //
  // PC (Program Counter) logic
  //

  // [Seunggi Moon] Edit: Use EX stage registers to calculate branch destinations
  assign branch_dest = (IDEX_pc + IDEX_se_br_imm);
  assign jal_dest 	= (IDEX_pc + IDEX_se_jal_imm);
  assign jalr_dest = (IDEX_rs1_data + IDEX_se_imm_itype);
  
  // assign jalr_dest = {aluout[31:1],1'b0}; // [Seunggi Moon] Edit: Multiply byte distance by 2 (setting LSB as 0), according to RISC-V design

  always @(posedge clk, posedge reset)
  begin
    if (reset)  pc <= 32'b0;
	  else
	  begin
      if (stall) // [Seunggi Moon] Edit: add stall condition
        pc <= #`simdelay pc;
      // [Seunggi Moon] Edit: add condition for bgeu, use EX stage registers
	    else if (beq_taken | blt_taken | bgeu_taken | bne_taken) // branch_taken
				pc <= #`simdelay branch_dest;
		  else if (IDEX_jal) // jal
				pc <= #`simdelay jal_dest;
      else if (IDEX_jalr)  // jalr
				pc <= #`simdelay jalr_dest; // [Seunggi Moon] Edit: handle JALR
		  else 
				pc <= #`simdelay (pc + 4);
	  end
  end
  
  // [Seunggi Moon] Edit: Pipeline register update logic (IF -> ID)
  always @(posedge clk)
  begin
    if (stall)
    begin
      IFID_inst <= #`simdelay IFID_inst;
      IFID_pc <= #`simdelay IFID_pc;
    end
    else if (flush) // Add flushing
    begin
      IFID_inst <= #`simdelay 32'b0;
      IFID_pc <= #`simdelay 32'b0;
    end
    else
    begin
      IFID_inst <= #`simdelay inst;
      IFID_pc <= #`simdelay pc;
    end
  end



  /////////////////////////////////////////
  // ID STAGE
  /////////////////////////////////////////

  // [Seunggi Moon] Edit: use IFID_inst instead of inst
  assign rs1 = IFID_inst[19:15];
  assign rs2 = IFID_inst[24:20];
  assign funct3 = IFID_inst[14:12];

  // JAL immediate
  assign jal_imm[20:1] = {IFID_inst[31],IFID_inst[19:12],IFID_inst[20],IFID_inst[30:21]};
  assign se_jal_imm[31:0] = {{11{jal_imm[20]}},jal_imm[20:1],1'b0};

  // Branch immediate
  assign br_imm[12:1] = {IFID_inst[31],IFID_inst[7],IFID_inst[30:25],IFID_inst[11:8]};
  assign se_br_imm[31:0] = {{19{br_imm[12]}},br_imm[12:1],1'b0};

  // [Seunggi Moon] Edit: use IFID_inst instead of inst
  assign se_imm_itype[31:0] = {{20{IFID_inst[31]}},IFID_inst[31:20]};
  assign se_imm_stype[31:0] = {{20{IFID_inst[31]}},IFID_inst[31:25],IFID_inst[11:7]};
  assign auipc_lui_imm[31:0] = {IFID_inst[31:12],12'b0};

  // 
  // Register File 
  //
  regfile i_regfile(
    .clk			(clk),
    .we			  (MEMWB_regwrite), // [Seunggi Moon] Edit: use MEMWB_regwrite instead of regwrite
    .rs1			(rs1),
    .rs2			(rs2),
    .rd			  (MEMWB_rd), // [Seunggi Moon] Edit: use MEMWB_rd instead of rd
    .rd_data	(rd_data),
    .rs1_data	(rfrs1_data),
    .rs2_data	(rfrs2_data));

  // [Seunggi Moon] Edit: Forwarding Unit
  always @(*)
  begin
    // if current rs1 == previous rd (!= zero), and previous instr didn't write anything to memory
    if      ((IDEX_rd != 5'b0) && (rs1 == IDEX_rd) & ~IDEX_memwrite)                   rs1_data = aluout; 
    else if ((EXMEM_rd != 5'b0) && (rs1 == EXMEM_rd) && (EXMEM_opcode == `OP_I_LOAD))  rs1_data = MemRdata[31:0];
    else if ((EXMEM_rd != 5'b0) && (rs1 == EXMEM_rd) && (EXMEM_opcode != `OP_I_LOAD))  rs1_data = EXMEM_aluout[31:0];
    else if ((MEMWB_rd != 5'b0) && (rs1 == MEMWB_rd))                                  rs1_data = rd_data[31:0];
    else                                                                            	 rs1_data = rfrs1_data;
  end
  always @(*)
  begin
    // if current rs2 == previous rd (!= zero), and previous instr didn't write anything to memory
    if      ((IDEX_rd != 5'b0) && (rs2 == IDEX_rd) & ~IDEX_memwrite)                   rs2_data = aluout; 
    else if ((EXMEM_rd != 5'b0) && (rs2 == EXMEM_rd) && (EXMEM_opcode == `OP_I_LOAD))  rs2_data = MemRdata[31:0];
    else if ((EXMEM_rd != 5'b0) && (rs2 == EXMEM_rd) && (EXMEM_opcode != `OP_I_LOAD))  rs2_data = EXMEM_aluout[31:0];
    else if ((MEMWB_rd != 5'b0) && (rs2 == MEMWB_rd))                                  rs2_data = rd_data[31:0];
    else                                                                            	 rs2_data = rfrs2_data;
  end

  // [Seunggi Moon] Edit: Pipeline register update logic (ID -> EX)
  always @(posedge clk)
  begin
    if (stall | flush) // Setting register values to zero to prevent executing same instr again
    begin
      IDEX_opcode <= #`simdelay 7'b0;

      IDEX_rs1 <= #`simdelay 5'b0;
      IDEX_rs2 <= #`simdelay 5'b0;
      IDEX_rd <= #`simdelay 5'b0;
      IDEX_rs1_data <= #`simdelay 32'b0;
      IDEX_rs2_data <= #`simdelay 32'b0;

      IDEX_jal <= #`simdelay 1'b0;
      IDEX_jalr <= #`simdelay 1'b0;
      IDEX_auipc <= #`simdelay 1'b0;
      IDEX_lui <= #`simdelay 1'b0;

      IDEX_branch <= #`simdelay 1'b0;

      IDEX_regwrite <= #`simdelay 1'b0;
      IDEX_memtoreg <= #`simdelay 1'b0;
      IDEX_memwrite <= #`simdelay 1'b0;

      IDEX_alusrc <= #`simdelay 1'b0;
      IDEX_alucontrol <= #`simdelay 5'b0;

      IDEX_funct3 <= #`simdelay 3'b0;
    end
    else
    begin
      IDEX_pc <= #`simdelay IFID_pc;
      IDEX_opcode <= #`simdelay opcode;

      IDEX_rs1 <= #`simdelay rs1;
      IDEX_rs2 <= #`simdelay rs2;
      IDEX_rd <= #`simdelay IFID_inst[11:7];
      IDEX_rs1_data <= #`simdelay rs1_data;
      IDEX_rs2_data <= #`simdelay rs2_data;
      
      IDEX_auipc_lui_imm <= #`simdelay auipc_lui_imm;
      IDEX_se_imm_itype <= #`simdelay se_imm_itype;
      IDEX_se_imm_stype <= #`simdelay se_imm_stype;
      IDEX_se_br_imm <= #`simdelay se_br_imm;
      IDEX_se_jal_imm <= #`simdelay se_jal_imm;

      IDEX_jal <= #`simdelay jal;
      IDEX_jalr <= #`simdelay jalr;
      IDEX_auipc<= #`simdelay auipc;
      IDEX_lui <= #`simdelay lui;

      IDEX_branch <= #`simdelay branch;

      IDEX_regwrite <= #`simdelay regwrite;
      IDEX_memtoreg <= #`simdelay memtoreg;
      IDEX_memwrite <= #`simdelay memwrite;

      IDEX_alusrc <= #`simdelay alusrc;
      IDEX_alucontrol <= #`simdelay alucontrol;

      IDEX_funct3 <= #`simdelay funct3;
    end
  end



  /////////////////////////////////////////
  // EX STAGE
  /////////////////////////////////////////
	
  assign MemWdata = EXMEM_rs2_data;
  
  // 1st source to ALU (alusrc1)
	always@(*)
	begin
		// [Seunggi Moon] Edit: Bypassing coniditions
		if      ((EXMEM_rd != 5'b0) &&(EXMEM_rd == IDEX_rs1) && (EXMEM_regwrite == 1'b1))	alusrc1[31:0] = EXMEM_aluout;
		else if ((MEMWB_rd != 5'b0) && (MEMWB_rd == IDEX_rs1) && (MEMWB_regwrite == 1'b1)) 	alusrc1[31:0] = rd_data;
    
		else if (IDEX_auipc)  alusrc1[31:0] = IDEX_pc;
		else if (IDEX_lui)    alusrc1[31:0] = 32'b0;
		else                  alusrc1[31:0] = IDEX_rs1_data[31:0];
	end
	
	// 2nd source to ALU (alusrc2)
	always@(*)
	begin
		// [Seunggi Moon] Edit: Bypassing conditions
		if      ((EXMEM_rd != 5'b0) && (EXMEM_rd == IDEX_rs2) && (EXMEM_regwrite == 1) & ~IDEX_alusrc)  alusrc2[31:0] = EXMEM_aluout;
		else if ((MEMWB_rd != 5'b0) && (MEMWB_rd == IDEX_rs2) && (MEMWB_regwrite == 1) & ~IDEX_alusrc)  alusrc2[31:0] = rd_data;
		
		else if	(IDEX_auipc | IDEX_lui)       alusrc2[31:0] = IDEX_auipc_lui_imm[31:0];
		else if (IDEX_alusrc & IDEX_memwrite)	alusrc2[31:0] = IDEX_se_imm_stype[31:0];
		else if (IDEX_alusrc)                 alusrc2[31:0] = IDEX_se_imm_itype[31:0];
		else                                  alusrc2[31:0] = IDEX_rs2_data[31:0];
	end

	//
	// ALU 
	//
	alu i_alu(
    // ALU input
		.a			(alusrc1),
		.b			(alusrc2),
		.alucont	(IDEX_alucontrol), // [Seunggi Moon] Edit: use IDEX_alucontrol instead of alucontrol
    // ALU output
		.result	(aluout),
		.N			(Nflag),
		.Z			(Zflag),
		.C			(Cflag),
		.V			(Vflag));

  // [Seunggi Moon] Edit: Implement stall logic (Data hazard detection)
  assign stall = ((rs1 == IDEX_rd) && (IDEX_opcode == `OP_I_LOAD) && (IDEX_rd != 5'b00000))
               + ((rs2 == IDEX_rd) && (IDEX_opcode == `OP_I_LOAD) && (IDEX_rd != 5'b00000));
  
  // [Seunggi Moon] Edit: use IDEX_funct3 instead of funct3
  assign f3beq  = (IDEX_funct3 == 3'b000);
  assign f3blt  = (IDEX_funct3 == 3'b100);
  assign f3bgeu = (IDEX_funct3 == 3'b111); // [Seunggi Moon] Edit: to detect whether IDEX_funct3 is BGEU
  assign f3bne  = (IDEX_funct3 == 3'b001); // [Seunggi Moon] Edit: to detect whether IDEX_funct3 is BNE

  assign beq_taken  =  IDEX_branch & f3beq & Zflag;
  assign blt_taken  =  IDEX_branch & f3blt & (Nflag != Vflag);
  assign bgeu_taken =  IDEX_branch & f3bgeu & Cflag; // [Seunggi Moon] Edit: to detect whether the condition of BGEU is met
  assign bne_taken  =  IDEX_branch & f3bne & ~Zflag; // [Seunggi Moon] Edit: to detect whether the condition of BNE is met

  // [Seunggi Moon] Edit: Implement flush logic (Control hazard detection)
  assign flush = (beq_taken | blt_taken | bgeu_taken | bne_taken | IDEX_jal | IDEX_jalr);

  // [Seunggi Moon] Edit: Pipeline register update logic (EX -> MEM)
  always@(posedge clk)
  begin 
    EXMEM_pc <= #`simdelay IDEX_pc;
    EXMEM_opcode <= #`simdelay IDEX_opcode;

    EXMEM_memwrite <= #`simdelay IDEX_memwrite;
    EXMEM_memtoreg <= #`simdelay IDEX_memtoreg;
    EXMEM_regwrite <= #`simdelay IDEX_regwrite;
    
    EXMEM_rd <= #`simdelay IDEX_rd;
    EXMEM_rs2_data <= #`simdelay IDEX_rs2_data;

    EXMEM_jal <= #`simdelay IDEX_jal;
    EXMEM_jalr <= #`simdelay IDEX_jalr;

    EXMEM_aluout <= #`simdelay aluout;
  end



  /////////////////////////////////////////
  // MEM STAGE
  /////////////////////////////////////////
  
  //
  // Assume that memory access happens at this point
  //

  // [Seunggi Moon] Edit: Pipeline register update logic (MEM -> WB)
  always@(posedge clk)
  begin
    MEMWB_pc <= #`simdelay EXMEM_pc;

    MEMWB_rd <= #`simdelay EXMEM_rd;

    MEMWB_regwrite <= #`simdelay EXMEM_regwrite;
    MEMWB_memtoreg <= #`simdelay EXMEM_memtoreg;

    MEMWB_jal <= #`simdelay EXMEM_jal;
    MEMWB_jalr <= #`simdelay EXMEM_jalr;

    MEMWB_aluout <= #`simdelay EXMEM_aluout;

    MEMWB_MemRdata <= #`simdelay MemRdata;
  end
  
  
  
  /////////////////////////////////////////
  // WB STAGE
  /////////////////////////////////////////

  // Data selection for writing to RF
  always@(*)
  begin
		if      (MEMWB_jal | MEMWB_jalr)  rd_data[31:0] = MEMWB_pc + 4; // [Seunggi Moon] Edit: handle JALR
		else if (MEMWB_memtoreg)          rd_data[31:0] = MEMWB_MemRdata; // [Seunggi Moon] Edit: use MEMWB_memtoreg instead of memtoreg
		else                              rd_data[31:0] = MEMWB_aluout; // [Seunggi Moon] Edit: use MEMWB_aluout instead of aluout
  end
	
endmodule
