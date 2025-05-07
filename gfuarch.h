/// ======================================================================= ///
/// This file is part of the GameFU Station project                         ///
///   GFUARCH - GameFU Station Architecture Library                         ///
/// ----------------------------------------------------------------------- ///
/// Copyright (C) 2025  Local Atticus <contact@nashiora.com>                ///
///                                                                         ///
/// This software is provided 'as-is', without any express or implied       ///
/// warranty. In no event will the authors be held liable for any damages   ///
/// arising from the use of this software.                                  ///
///                                                                         ///
/// Permission is granted to anyone to use this software for any purpose,   ///
/// including commercial applications, and to alter it and redistribute it  ///
/// freely, subject to the following restrictions:                          ///
///                                                                         ///
/// 1. The origin of this software must not be misrepresented; you must not ///
///    claim that you wrote the original software. If you use this software ///
///    in a product, an acknowledgment in the product documentation would   ///
///    be appreciated but is not required.                                  ///
///                                                                         ///
/// 2. Altered source versions must be plainly marked as such, and must not ///
///    be misrepresented as being the original software.                    ///
///                                                                         ///
/// 3. This notice may not be removed or altered from any source            ///
///    distribution.                                                        ///
/// ======================================================================= ///

#ifndef GFUARCH_H_
#define GFUARCH_H_

#include <stdint.h>

#define GFU_STACK_SIZE 1024

#define GFU_MEM_OFFSET_MAIN_RAM 0x00000000
#define GFU_MEM_SIZE_MAIN_RAM 0x00200000 // 2048 * 1024

#define GFU_MEM_OFFSET_ROM 0x00200000
#define GFU_MEM_SIZE_ROM 0x00800000 // 8192 * 1024

#define GFU_MEM_SIZE (GFU_MEM_SIZE_MAIN_RAM + GFU_MEM_SIZE_ROM)

/// Core set instruction format:
///
/// Type |   31..26   | 25..21 | 20..16 | 15..11 |   10..6   |   5..0
/// -----+------------+--------+--------+--------+-----------+-----------
///   R  | opcode (6) | rs (5) | rt (5) | rd (5) | shamt (5) | funct (6)
///   I  | opcode (6) | rs (5) | rt (5) |       immediate (16)
///   J  | opcode (6) |               address (26)
///
/// - R-type (register) instructions specify three registers, a shift amount
///   field, and a function field.
/// - I-type (immediate) instructions specify two registers and a 16-bit
///   immediate value.
/// - J-type (jump) instructions follow the opcode with a 26-bit jump target.


#define GFU_GET_OPCODE(Inst) (((Inst) >> 26) & 0x0000003F)
#define GFU_GET_RS(Inst)     (((Inst) >> 21) & 0x0000001F)
#define GFU_GET_RT(Inst)     (((Inst) >> 16) & 0x0000001F)
#define GFU_GET_RD(Inst)     (((Inst) >> 11) & 0x0000001F)
#define GFU_GET_SHAMT(Inst)  (((Inst) >>  6) & 0x0000001F)
#define GFU_GET_FUNCT(Inst)  (((Inst) >>  0) & 0x0000003F)
#define GFU_GET_IMM(Inst)    (((Inst) >>  0) & 0x0000FFFF)
#define GFU_GET_ADDR(Inst)   (((Inst) >>  0) & 0x03FFFFFF)
#define GFU_GET_CODE(Inst)   (((Inst) >>  6) & 0x000FFFFF)
#define GFU_GET_SEL(Inst)    (((Inst) >>  0) & 0x00000007)

#define GFU_ENC_OPCODE(Val)  (((Val) & 0x0000003F) << 26)
#define GFU_ENC_RS(Val)      (((Val) & 0x0000001F) << 21)
#define GFU_ENC_RT(Val)      (((Val) & 0x0000001F) << 16)
#define GFU_ENC_RD(Val)      (((Val) & 0x0000001F) << 11)
#define GFU_ENC_SHAMT(Val)   (((Val) & 0x0000001F) <<  6)
#define GFU_ENC_FUNCT(Val)   (((Val) & 0x0000003F) <<  0)
#define GFU_ENC_IMM(Val)     (((Val) & 0x0000FFFF) <<  0)
#define GFU_ENC_ADDR(Val)    (((Val) & 0x03FFFFFF) <<  0)
#define GFU_ENC_CODE(Val)    (((Val) & 0x000FFFFF) <<  6)
#define GFU_ENC_SEL(Val)     (((Val) & 0x00000007) <<  0)

typedef union gfu_inst {
    struct {
        uint32_t funct  : 6;
        uint32_t shamt  : 5;
        uint32_t rd     : 5;
        uint32_t rt     : 5;
        uint32_t rs     : 5;
        uint32_t opcode : 6;
    };
    struct {
        uint32_t _code_padding : 6;
        uint32_t code : 20;
    };
    struct {
        uint32_t cofun : 25;
        uint32_t co : 1;
    };
    uint32_t sel  : 3;
    uint32_t imm  : 16;
    uint32_t addr : 26;
    uint32_t raw  : 32;
} gfu_inst;

/// Primary opcode field.
typedef enum gfu_opcode {
    GFU_OPCODE_SPECIAL = 0x00,
    GFU_OPCODE_REGIMM = 0x01,
    GFU_OPCODE_J = 0x02,
    GFU_OPCODE_JAL = 0x03,
    GFU_OPCODE_BEQ = 0x04,
    GFU_OPCODE_BNE = 0x05,
    GFU_OPCODE_BLEZ = 0x06,
    GFU_OPCODE_BGTZ = 0x07,

    GFU_OPCODE_ADDI = 0x08,
    GFU_OPCODE_ADDIU = 0x09,
    GFU_OPCODE_SLTI = 0x0A,
    GFU_OPCODE_SLTIU = 0x0B,
    GFU_OPCODE_ANDI = 0x0C,
    GFU_OPCODE_ORI = 0x0D,
    GFU_OPCODE_XORI = 0x0E,
    GFU_OPCODE_LUI = 0x0F,

    GFU_OPCODE_COP0 = 0x10,
    GFU_OPCODE_COP1 = 0x11,
    GFU_OPCODE_COP2 = 0x12,
    GFU_OPCODE_COP1X = 0x13,

    GFU_OPCODE_SPECIAL2 = 0x1C,

    GFU_OPCODE_LB = 0x20,
    GFU_OPCODE_LH = 0x21,
    GFU_OPCODE_LWL = 0x22,
    GFU_OPCODE_LW = 0x23,
    GFU_OPCODE_LBU = 0x24,
    GFU_OPCODE_LHU = 0x25,
    GFU_OPCODE_LWR = 0x26,

    GFU_OPCODE_SB = 0x28,
    GFU_OPCODE_SH = 0x29,
    GFU_OPCODE_SWL = 0x2A,
    GFU_OPCODE_SW = 0x2B,
    GFU_OPCODE_SWR = 0x2E,
    GFU_OPCODE_CACHE = 0x2F,

    GFU_OPCODE_LL = 0x30,
    GFU_OPCODE_LWC1 = 0x31,
    GFU_OPCODE_LWC2 = 0x32,
    GFU_OPCODE_PREF = 0x33,
    GFU_OPCODE_LDC1 = 0x35,
    GFU_OPCODE_LDC2 = 0x36,

    GFU_OPCODE_SC = 0x38,
    GFU_OPCODE_SWC1 = 0x39,
    GFU_OPCODE_SWC2 = 0x3A,
    GFU_OPCODE_SDC1 = 0x3D,
    GFU_OPCODE_SDC2 = 0x3E,

    GFU_OPCODE_MAX = 0x3F,
} gfu_opcode;

/// Secondary opcode field.
typedef enum gfu_funct {
    GFU_FUNCT_SLL = 0x00,
    GFU_FUNCT_MOVCI = 0x01,
    GFU_FUNCT_SRL = 0x02,
    GFU_FUNCT_SRA = 0x03,
    GFU_FUNCT_SLLV = 0x04,
    GFU_FUNCT_SRLV = 0x06,
    GFU_FUNCT_SRAV = 0x07,

    GFU_FUNCT_JR = 0x08,
    GFU_FUNCT_JALR = 0x09,
    GFU_FUNCT_MOVZ = 0x0A,
    GFU_FUNCT_MOVN = 0x0B,
    GFU_FUNCT_SYSCALL = 0x0C,
    GFU_FUNCT_BREAK = 0x0D,
    GFU_FUNCT_SYNC = 0x0F,

    GFU_FUNCT_MFHI = 0x10,
    GFU_FUNCT_MTHI = 0x11,
    GFU_FUNCT_MFLO = 0x12,
    GFU_FUNCT_MTLO = 0x13,

    GFU_FUNCT_MULT = 0x18,
    GFU_FUNCT_MULTU = 0x19,
    GFU_FUNCT_DIV = 0x1A,
    GFU_FUNCT_DIVU = 0x1B,

    GFU_FUNCT_ADD = 0x20,
    GFU_FUNCT_ADDU = 0x21,
    GFU_FUNCT_SUB = 0x22,
    GFU_FUNCT_SUBU = 0x23,
    GFU_FUNCT_AND = 0x24,
    GFU_FUNCT_OR = 0x25,
    GFU_FUNCT_XOR = 0x26,
    GFU_FUNCT_NOR = 0x27,

    GFU_FUNCT_SLT = 0x2A,
    GFU_FUNCT_SLTU = 0x2B,

    GFU_FUNCT_TGE = 0x30,
    GFU_FUNCT_TGEU = 0x31,
    GFU_FUNCT_TLT = 0x32,
    GFU_FUNCT_TLTU = 0x33,
    GFU_FUNCT_TEQ = 0x34,
    GFU_FUNCT_TNE = 0x36,

    GFU_FUNCT_MAX = 0x3F,
} gfu_funct;

typedef enum gfu_funct2 {
    GFU_FUNCT2_MADD = 0x00,
    GFU_FUNCT2_MADDU = 0x01,
    GFU_FUNCT2_MUL = 0x02,
    GFU_FUNCT2_MSUB = 0x04,
    GFU_FUNCT2_MSUBU = 0x05,

    GFU_FUNCT2_CLZ = 0x20,
    GFU_FUNCT2_CLO = 0x21,

    GFU_FUNCT2_SDBBP = 0x3F,

    GFU_FUNCT2_MAX = 0x3F,
} gfu_funct2;

typedef enum gfu_funct_cop0 {
    GFU_FUNCTC0_TLBR = 0x01,
    GFU_FUNCTC0_TLBWI = 0x02,
    GFU_FUNCTC0_TLBWR = 0x06,

    GFU_FUNCTC0_TLBP = 0x08,

    GFU_FUNCTC0_ERET = 0x18,
    GFU_FUNCTC0_DERET = 0x1F,

    GFU_FUNCTC0_WAIT = 0x20,

    GFU_FUNCTC0_MAX = 0x3F,
} gfu_funct_cop0;

typedef enum gfu_rs_cop0 {
    GFU_RSC0_MFC0 = 0x00,
    GFU_RSC0_MTC0 = 0x04,

    GFU_RSC0_C0 = 0x10,

    GFU_RSC0_MAX = 0x3F,
} gfu_rs_cop0;

typedef enum gfu_rs_cop2 {
    GFU_RSC2_MFC2 = 0x00,
    GFU_RSC2_CFC2 = 0x02,
    GFU_RSC2_MTC2 = 0x04,
    GFU_RSC2_CTC2 = 0x06,

    GFU_RSC2_BC2 = 0x08,

    GFU_RSC2_C2 = 0x10,

    GFU_RSC2_MAX = 0x3F,
} gfu_rs_cop2;

typedef enum gfu_rt_regimm {
    GFU_RTRI_BLTZ = 0x00,
    GFU_RTRI_BGEZ = 0x01,

    GFU_RTRI_TGEI = 0x08,
    GFU_RTRI_TGEIU = 0x09,
    GFU_RTRI_TLTI = 0x0A,
    GFU_RTRI_TLTIU = 0x0B,
    GFU_RTRI_TEQI = 0x0C,
    GFU_RTRI_TNEI = 0x0E,

    GFU_RTRI_BLTZAL = 0x10,
    GFU_RTRI_BGEZAL = 0x11,

    GFU_RTRI_MAX = 0x1F,
} gfu_rt_regimm;

typedef enum gfu_register {
    // Constant 0
    GFU_REG_R0 = 0,
    GFU_REG_ZERO = GFU_REG_R0,

    // Assembler Temporary
    GFU_REG_R1 = 1,
    GFU_REG_AT = GFU_REG_R1,

    // Subroutine Return Values
    GFU_REG_R2 = 2,
    GFU_REG_R3 = 3,
    GFU_REG_V0 = GFU_REG_R2,
    GFU_REG_V1 = GFU_REG_R3,

    // Subroutine Arguments
    GFU_REG_R4 = 4,
    GFU_REG_R5 = 5,
    GFU_REG_R6 = 6,
    GFU_REG_R7 = 7,
    GFU_REG_A0 = GFU_REG_R4,
    GFU_REG_A1 = GFU_REG_R5,
    GFU_REG_A2 = GFU_REG_R6,
    GFU_REG_A3 = GFU_REG_R7,

    // Temporaries
    GFU_REG_R8 = 8,
    GFU_REG_R9 = 9,
    GFU_REG_R10 = 10,
    GFU_REG_R11 = 11,
    GFU_REG_R12 = 12,
    GFU_REG_R13 = 13,
    GFU_REG_R14 = 14,
    GFU_REG_R15 = 15,
    GFU_REG_T0 = GFU_REG_R8,
    GFU_REG_T1 = GFU_REG_R9,
    GFU_REG_T2 = GFU_REG_R10,
    GFU_REG_T3 = GFU_REG_R11,
    GFU_REG_T4 = GFU_REG_R12,
    GFU_REG_T5 = GFU_REG_R13,
    GFU_REG_T6 = GFU_REG_R14,
    GFU_REG_T7 = GFU_REG_R15,

    // Static Variables
    GFU_REG_R16 = 16,
    GFU_REG_R17 = 17,
    GFU_REG_R18 = 18,
    GFU_REG_R19 = 19,
    GFU_REG_R20 = 20,
    GFU_REG_R21 = 21,
    GFU_REG_R22 = 22,
    GFU_REG_R23 = 23,
    GFU_REG_S0 = GFU_REG_R16,
    GFU_REG_S1 = GFU_REG_R17,
    GFU_REG_S2 = GFU_REG_R18,
    GFU_REG_S3 = GFU_REG_R19,
    GFU_REG_S4 = GFU_REG_R20,
    GFU_REG_S5 = GFU_REG_R21,
    GFU_REG_S6 = GFU_REG_R22,
    GFU_REG_S7 = GFU_REG_R23,

    // Temporaries
    GFU_REG_R24 = 24,
    GFU_REG_R25 = 25,
    GFU_REG_T8 = GFU_REG_R24,
    GFU_REG_T9 = GFU_REG_R25,

    // Kernel Reserved
    GFU_REG_R26 = 26,
    GFU_REG_R27 = 27,
    GFU_REG_K0 = GFU_REG_R26,
    GFU_REG_K1 = GFU_REG_R27,

    // Global Pointer
    GFU_REG_R28 = 28,
    GFU_REG_GP = GFU_REG_R28,

    // Stack Pointer
    GFU_REG_R29 = 29,
    GFU_REG_SP = GFU_REG_R29,

    // Frame Pointer OR Static Variable
    GFU_REG_R30 = 30,
    GFU_REG_FP = GFU_REG_R30,
    GFU_REG_S8 = GFU_REG_R30,

    // Return Address
    GFU_REG_R31 = 31,
    GFU_REG_RA = GFU_REG_R31,

    // Program Counter
    GFU_REG_PC = 32,
    // Multiply/divide results
    GFU_REG_HI = 33,
    GFU_REG_LO = 34,

    GFU_REG_COUNT
} gfu_register;

/// ======================================================================= ///
/// CPU Arithmetic Instructions.                                            ///
/// ======================================================================= ///

#define GFU_INST_ADD(Rd, Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RD(Rd) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_ADD))
#define GFU_INST_ADDI(Rt, Rs, Imm) (GFU_ENC_OPCODE(GFU_OPCODE_ADDI) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Imm))
#define GFU_INST_ADDIU(Rt, Rs, Imm) (GFU_ENC_OPCODE(GFU_OPCODE_ADDIU) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Imm))
#define GFU_INST_ADDU(Rd, Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RD(Rd) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_ADDU))
#define GFU_INST_CLO(Rd, Rs) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL2) | GFU_ENC_RD(Rd) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rd) | GFU_ENC_FUNCT(GFU_FUNCT2_CLO))
#define GFU_INST_CLZ (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL2) | GFU_ENC_RD(Rd) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rd) | GFU_ENC_FUNCT(GFU_FUNCT2_CLZ))
#define GFU_INST_DIV(Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_DIV))
#define GFU_INST_DIVU(Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_DIVU))
#define GFU_INST_MADD(Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL2) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT2_MADD))
#define GFU_INST_MADDU(Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL2) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT2_MADDU))
#define GFU_INST_MSUB(Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL2) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT2_MSUB))
#define GFU_INST_MSUBU(Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL2) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT2_MSUBU))
#define GFU_INST_MUL(Rd, Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL2) | GFU_ENC_RD(Rd) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT2_MUL))
#define GFU_INST_MULT(Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_MULT))
#define GFU_INST_MULTU(Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_MULTU))
#define GFU_INST_SLT(Rd, Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RD(Rd) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_SLT))
#define GFU_INST_SLTI(Rt, Rs, Imm) (GFU_ENC_OPCODE(GFU_OPCODE_SLTI) | GFU_ENC_RT(Rt) | GFU_ENC_RS(Rs) | GFU_ENC_IMM(Imm))
#define GFU_INST_SLTIU(Rt, Rs, Imm) (GFU_ENC_OPCODE(GFU_OPCODE_SLTIU) | GFU_ENC_RT(Rt) | GFU_ENC_RS(Rs) | GFU_ENC_IMM(Imm))
#define GFU_INST_SLTU(Rd, Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RD(Rd) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_SLTU))
#define GFU_INST_SUB(Rd, Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RD(Rd) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_SUB))
#define GFU_INST_SUBU(Rd, Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RD(Rd) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_SUBU))

/// ======================================================================= ///
/// CPU Branch and Jump Instructions.                                       ///
/// ======================================================================= ///

#define GFU_INST_B(Offset) (GFU_INST_BEQ(GFU_REG_R0, GFU_REG_R0, (Offset)))
#define GFU_INST_BAL(Offset) (GFU_INST_BGEZAL(GFU_REG_R0, (Offset)))
#define GFU_INST_BEQ(Rt, Rs, Offset) (GFU_ENC_OPCODE(GFU_OPCODE_BEQ) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_BGEZ(Rs, Offset) (GFU_ENC_OPCODE(GFU_OPCODE_REGIMM) | GFU_ENC_RS(Rs) | GFU_ENC_RT(GFU_RTRI_BGEZ) | GFU_ENC_IMM(Offset))
#define GFU_INST_BGEZAL(Rs, Offset) (GFU_ENC_OPCODE(GFU_OPCODE_REGIMM) | GFU_ENC_RS(Rs) | GFU_ENC_RT(GFU_RTRI_BGEZAL) | GFU_ENC_IMM(Offset))
#define GFU_INST_BGTZ(Rs, Offset) (GFU_ENC_OPCODE(GFU_OPCODE_BGTZ) | GFU_ENC_RS(Rs) | GFU_ENC_IMM(Offset))
#define GFU_INST_BLEZ(Rs, Offset) (GFU_ENC_OPCODE(GFU_OPCODE_BLEZ) | GFU_ENC_RS(Rs) | GFU_ENC_IMM(Offset))
#define GFU_INST_BLTZ(Rs, Offset) (GFU_ENC_OPCODE(GFU_OPCODE_REGIMM) | GFU_ENC_RS(Rs) | GFU_ENC_RT(GFU_RTRI_BLTZ) | GFU_ENC_IMM(Offset))
#define GFU_INST_BLTZAL(Rs, Offset) (GFU_ENC_OPCODE(GFU_OPCODE_REGIMM) | GFU_ENC_RS(Rs) | GFU_ENC_RT(GFU_RTRI_BLTZAL) | GFU_ENC_IMM(Offset))
#define GFU_INST_BNE(Rs, Rt, Offset) (GFU_ENC_OPCODE(GFU_OPCODE_BNE) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_J(Addr) (GFU_ENC_OPCODE(GFU_OPCODE_J) | GFU_ENC_ADDR(Addr))
#define GFU_INST_JAL(Addr) (GFU_ENC_OPCODE(GFU_OPCODE_JAL) | GFU_ENC_ADDR(Addr))
#define GFU_INST_JALR(Rd, Rs) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RD(Rd) | GFU_ENC_RS(Rs) | GFU_ENC_FUNCT(GFU_FUNCT_JALR))
#define GFU_INST_JR(Rs) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_FUNCT(GFU_FUNCT_JR))

/// ======================================================================= ///
/// CPU Instruction Control Instructions.                                   ///
/// ======================================================================= ///

#define GFU_INST_NOP() (0)
#define GFU_INST_SSNOP() (0x40)

/// ======================================================================= ///
/// CPU Load, Store and Memory Control Instructions.                        ///
/// ======================================================================= ///

#define GFU_INST_LB(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_LB) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_LBU(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_LBU) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_LH(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_LH) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_LHU(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_LHU) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_LL(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_LL) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_LW(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_LW) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_LWL(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_LWL) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_LWR(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_LWR) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_PREF(Hint, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_PREF) | GFU_ENC_RS(Base) | GFU_ENC_RT(Hint) | GFU_ENC_IMM(Offset))
#define GFU_INST_SB(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_SB) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_SC(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_SC) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
//#define GFU_INST_SD
#define GFU_INST_SH(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_SH) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_SW(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_SW) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_SWL(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_SWL) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_SWR(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_SWR) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_SYNC() (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_FUNCT(GFU_FUNCT_SYNC))

/// ======================================================================= ///
/// CPU Logical Instructions.                                               ///
/// ======================================================================= ///

#define GFU_INST_AND(Rd, Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RD(Rd) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_AND))
#define GFU_INST_ANDI(Rt, Rs, Imm) (GFU_ENC_OPCODE(GFU_OPCODE_ANDI) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Imm))
#define GFU_INST_LUI(Rt, Imm) (GFU_ENC_OPCODE(GFU_OPCODE_LUI) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Imm))
#define GFU_INST_NOR(Rd, Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RD(Rd) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_NOR))
#define GFU_INST_OR(Rd, Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RD(Rd) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_OR))
#define GFU_INST_ORI(Rt, Rs, Imm) (GFU_ENC_OPCODE(GFU_OPCODE_ORI) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Imm))
#define GFU_INST_XOR(Rd, Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RD(Rd) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_XOR))
#define GFU_INST_XORI(Rt, Rs, Imm) (GFU_ENC_OPCODE(GFU_OPCODE_XORI) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Imm))

/// ======================================================================= ///
/// CPU Move Instructions.                                                  ///
/// ======================================================================= ///

#define GFU_INST_MFHI(Rd) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RD(Rd) | GFU_ENC_FUNCT(GFU_FUNCT_MFHI))
#define GFU_INST_MFLO(Rd) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RD(Rd) | GFU_ENC_FUNCT(GFU_FUNCT_MFLO))
#define GFU_INST_MOVN(Rd, Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_RD(Rd) | GFU_ENC_FUNCT(GFU_FUNCT_MOVN))
#define GFU_INST_MOVZ(Rd, Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_RD(Rd) | GFU_ENC_FUNCT(GFU_FUNCT_MOVZ))
#define GFU_INST_MTHI(Rs) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_FUNCT(GFU_FUNCT_MTHI))
#define GFU_INST_MTLO(Rs) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_FUNCT(GFU_FUNCT_MTLO))

/// ======================================================================= ///
/// CPU Shift Instructions.                                                 ///
/// ======================================================================= ///

#define GFU_INST_SLL(Rd, Rt, Sa) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RT(Rt) | GFU_ENC_RD(Rd) | GFU_ENC_SHAMT(Sa) | GFU_ENC_FUNCT(GFU_FUNCT_SLL))
#define GFU_INST_SLLV(Rd, Rt, Rs) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_RD(Rd) | GFU_ENC_FUNCT(GFU_FUNCT_SLLV))
#define GFU_INST_SRA(Rd, Rt, Sa) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RT(Rt) | GFU_ENC_RD(Rd) | GFU_ENC_SHAMT(Sa) | GFU_ENC_FUNCT(GFU_FUNCT_SRA))
#define GFU_INST_SRAV(Rd, Rt, Rs) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_RD(Rd) | GFU_ENC_FUNCT(GFU_FUNCT_SRAV))
#define GFU_INST_SRL(Rd, Rt, Sa) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RT(Rt) | GFU_ENC_RD(Rd) | GFU_ENC_SHAMT(Sa) | GFU_ENC_FUNCT(GFU_FUNCT_SRL))
#define GFU_INST_SRLV(Rd, Rt, Rs) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_RD(Rd) | GFU_ENC_FUNCT(GFU_FUNCT_SRLV))

/// ======================================================================= ///
/// CPU Trap Instructions.                                                  ///
/// ======================================================================= ///

#define GFU_INST_BREAK(Code) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_CODE(Code) | GFU_ENC_FUNCT(GFU_FUNCT_BREAK))
#define GFU_INST_SYSCALL(Code) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_CODE(Code) | GFU_ENC_FUNCT(GFU_FUNCT_SYSCALL))
#define GFU_INST_TEQ(Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_TEQ))
#define GFU_INST_TEQI(Rs, Imm) (GFU_ENC_OPCODE(GFU_OPCODE_REGIMM) | GFU_ENC_RS(Rs) | GFU_ENC_RT(GFU_RTRI_TEQI) | GFU_ENC_IMM(Imm))
#define GFU_INST_TGE(Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_TGE))
#define GFU_INST_TGEI(Rs, Imm) (GFU_ENC_OPCODE(GFU_OPCODE_REGIMM) | GFU_ENC_RS(Rs) | GFU_ENC_RT(GFU_RTRI_TGEI) | GFU_ENC_IMM(Imm))
#define GFU_INST_TGEIU(Rs, Imm) (GFU_ENC_OPCODE(GFU_OPCODE_REGIMM) | GFU_ENC_RS(Rs) | GFU_ENC_RT(GFU_RTRI_TGEIU) | GFU_ENC_IMM(Imm))
#define GFU_INST_TGEU(Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_TGEU))
#define GFU_INST_TLT(Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_TLT))
#define GFU_INST_TLTI(Rs, Imm) (GFU_ENC_OPCODE(GFU_OPCODE_REGIMM) | GFU_ENC_RS(Rs) | GFU_ENC_RT(GFU_RTRI_TLTI) | GFU_ENC_IMM(Imm))
#define GFU_INST_TLTIU(Rs, Imm) (GFU_ENC_OPCODE(GFU_OPCODE_REGIMM) | GFU_ENC_RS(Rs) | GFU_ENC_RT(GFU_RTRI_TLTIU) | GFU_ENC_IMM(Imm))
#define GFU_INST_TLTU(Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_TLTU))
#define GFU_INST_TNE(Rs, Rt) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL) | GFU_ENC_RS(Rs) | GFU_ENC_RT(Rt) | GFU_ENC_FUNCT(GFU_FUNCT_TNE))
#define GFU_INST_TNEI(Rs, Imm) (GFU_ENC_OPCODE(GFU_OPCODE_REGIMM) | GFU_ENC_RS(Rs) | GFU_ENC_RT(GFU_RTRI_TNEI) | GFU_ENC_IMM(Imm))

/// ======================================================================= ///
/// Coprocessor Branch Instructions.                                        ///
/// ======================================================================= ///

#define GFU_INST_BC2F(Cc, Offset) (GFU_ENC_OPCODE(GFU_OPCODE_COP2) | GFU_ENC_RS(GFU_RSC2_BC2) | GFU_ENC_RT(Cc << 2) | GFU_ENC_IMM(Offset))
#define GFU_INST_BC2T(Cc, Offset) (GFU_ENC_OPCODE(GFU_OPCODE_COP2) | GFU_ENC_RS(GFU_RSC2_BC2) | GFU_ENC_RT((Cc << 2) | 0x01) | GFU_ENC_IMM(Offset))

/// ======================================================================= ///
/// Coprocessor Execute Instructions.                                       ///
/// ======================================================================= ///

#define GFU_INST_COP2(Func) (GFU_ENC_OPCODE(GFU_OPCODE_COP2) | GFU_ENC_ADDR((Func) | 0x02000000))

/// ======================================================================= ///
/// Coprocessor Load and Store Instructions.                                ///
/// ======================================================================= ///

#define GFU_INST_LDC2(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_LDC2) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_LWC2(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_LWC2) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_SDC2(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_SDC2) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))
#define GFU_INST_SWC2(Rt, Offset, Base) (GFU_ENC_OPCODE(GFU_OPCODE_LWC2) | GFU_ENC_RS(Base) | GFU_ENC_RT(Rt) | GFU_ENC_IMM(Offset))

/// ======================================================================= ///
/// Coprocessor Move Instructions.                                          ///
/// ======================================================================= ///

#define GFU_INST_CFC2(Rt, Rd) (GFU_ENC_OPCODE(GFU_OPCODE_COP2) | GFU_ENC_RS(GFU_RSC2_CFC2) | GFU_ENC_RT(Rt) | GFU_ENC_RD(Rd))
#define GFU_INST_CTC2(Rt, Rd) (GFU_ENC_OPCODE(GFU_OPCODE_COP2) | GFU_ENC_RS(GFU_RSC2_CTC2) | GFU_ENC_RT(Rt) | GFU_ENC_RD(Rd))
#define GFU_INST_MFC2(Rt, Rd, Sel) (GFU_ENC_OPCODE(GFU_OPCODE_COP2) | GFU_ENC_RS(GFU_RSC2_MFC2) | GFU_ENC_RT(Rt) | GFU_ENC_RD(Rd) | GFU_ENC_SEL(Sel))
#define GFU_INST_MTC2(Rt, Rd, Sel) (GFU_ENC_OPCODE(GFU_OPCODE_COP2) | GFU_ENC_RS(GFU_RSC2_MTC2) | GFU_ENC_RT(Rt) | GFU_ENC_RD(Rd) | GFU_ENC_SEL(Sel))

/// ======================================================================= ///
/// Privileged Instructions.                                                ///
/// ======================================================================= ///

#define GFU_INST_CACHE(Base, Op, Offset) (GFU_ENC_OPCODE(GFU_OPCODE_CACHE) | GFU_ENC_RS(Base) | GFU_ENC_RT(Op) | GFU_ENC_IMM(Offset))
#define GFU_INST_ERET() (GFU_ENC_OPCODE(GFU_OPCODE_COP0) | GFU_ENC_RS(GFU_RSC0_C0) | GFU_ENC_FUNCT(GFU_FUNCTC0_ERET))
#define GFU_INST_MFC0(Rt, Rd, Sel) (GFU_ENC_OPCODE(GFU_OPCODE_COP0) | GFU_ENC_RS(GFU_RSC0_MFC0) | GFU_ENC_RT(Rt) | GFU_ENC_RD(Rd) | GFU_ENC_SEL(Sel))
#define GFU_INST_MTC0(Rt, Rd, Sel) (GFU_ENC_OPCODE(GFU_OPCODE_COP0) | GFU_ENC_RS(GFU_RSC0_MTC0) | GFU_ENC_RT(Rt) | GFU_ENC_RD(Rd) | GFU_ENC_SEL(Sel))
#define GFU_INST_TLBP() (GFU_ENC_OPCODE(GFU_OPCODE_COP0) | 0x02000000 | GFU_ENC_FUNCT(GFU_FUNCTC0_TLBP))
#define GFU_INST_TLBR() (GFU_ENC_OPCODE(GFU_OPCODE_COP0) | 0x02000000 | GFU_ENC_FUNCT(GFU_FUNCTC0_TLBR))
#define GFU_INST_TLBWI() (GFU_ENC_OPCODE(GFU_OPCODE_COP0) | 0x02000000 | GFU_ENC_FUNCT(GFU_FUNCTC0_TLBWI))
#define GFU_INST_TLBWR() (GFU_ENC_OPCODE(GFU_OPCODE_COP0) | 0x02000000 | GFU_ENC_FUNCT(GFU_FUNCTC0_TLBWR))
#define GFU_INST_WAIT() (GFU_ENC_OPCODE(GFU_OPCODE_COP0) | 0x02000000 | GFU_ENC_FUNCT(GFU_FUNCTC0_WAIT))

/// ======================================================================= ///
/// EJTAG Instructions.                                                     ///
/// ======================================================================= ///

#define GFU_INST_DERET() (GFU_ENC_OPCODE(GFU_OPCODE_COP0) | 0x02000000 | GFU_ENC_FUNCT(GFU_FUNCTC0_DERET))
#define GFU_INST_SDBBP(Code) (GFU_ENC_OPCODE(GFU_OPCODE_SPECIAL2) | GFU_ENC_CODE(Code) | GFU_ENC_FUNCT(GFU_FUNCT2_SDBBP))

#endif /* GFUARCH_H_ */
