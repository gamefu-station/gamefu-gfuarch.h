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

#ifndef GFUARCH_INLINE
#define GFUARCH_INLINE inline
#endif /* GFUARCH_INLINE */

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


#define GFU_INST_OPCODE(Inst) (((Inst) >> 26) & 0x0000003F)
#define GFU_INST_RS(Inst)     (((Inst) >> 21) & 0x0000001F)
#define GFU_INST_RT(Inst)     (((Inst) >> 16) & 0x0000001F)
#define GFU_INST_RD(Inst)     (((Inst) >> 11) & 0x0000001F)
#define GFU_INST_SHAMT(Inst)  (((Inst) >>  6) & 0x0000001F)
#define GFU_INST_FUNCT(Inst)  (((Inst) >>  0) & 0x0000003F)
#define GFU_INST_IMM(Inst)    (((Inst) >>  0) & 0x0000FFFF)
#define GFU_INST_ADDR(Inst)   (((Inst) >>  0) & 0x03FFFFFF)

#define GFU_INST_OPCODE_ENC(Val) (((Val) & 0x0000003F) << 26)
#define GFU_INST_RS_ENC(Val)     (((Val) & 0x0000001F) << 21)
#define GFU_INST_RT_ENC(Val)     (((Val) & 0x0000001F) << 16)
#define GFU_INST_RD_ENC(Val)     (((Val) & 0x0000001F) << 11)
#define GFU_INST_SHAMT_ENC(Val)  (((Val) & 0x0000001F) <<  6)
#define GFU_INST_FUNCT_ENC(Val)  (((Val) & 0x0000003F) <<  0)
#define GFU_INST_IMM_ENC(Val)    (((Val) & 0x0000FFFF) <<  0)
#define GFU_INST_ADDR_ENC(Val)   (((Val) & 0x03FFFFFF) <<  0)

#define GFU_INST_OPCODE_SET(Inst, Val) (((Inst) & ~0xFC000000) | GFU_INST_OPCODE_ENC(Val))
#define GFU_INST_RS_SET(Inst, Val)     (((Inst) & ~0x03E00000) | GFU_INST_RS_ENC(Val))
#define GFU_INST_RT_SET(Inst, Val)     (((Inst) & ~0x001F0000) | GFU_INST_RT_ENC(Val))
#define GFU_INST_RD_SET(Inst, Val)     (((Inst) & ~0x0000F800) | GFU_INST_RD_ENC(Val))
#define GFU_INST_SHAMT_SET(Inst, Val)  (((Inst) & ~0x000007C0) | GFU_INST_SHAMT_ENC(Val))
#define GFU_INST_FUNCT_SET(Inst, Val)  (((Inst) & ~0x0000003F) | GFU_INST_FUNCT_ENC(Val))
#define GFU_INST_IMM_SET(Inst, Val)    (((Inst) & ~0x0000FFFF) | GFU_INST_IMM_ENC(Val))
#define GFU_INST_ADDR_SET(Inst, Val)   (((Inst) & ~0x03FFFFFF) | GFU_INST_ADDR_ENC(Val))

typedef union gfu_inst {
    struct{
        uint8_t funct  : 6;
        uint8_t shamt  : 5;
        uint8_t rd     : 5;
        uint8_t rt     : 5;
        uint8_t rs     : 5;
        uint8_t opcode : 6;
    };
    uint16_t imm  : 16;
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
    GFU_OPCODE_0x14 = 0x14,
    GFU_OPCODE_0x15 = 0x15,
    GFU_OPCODE_0x16 = 0x16,
    GFU_OPCODE_0x17 = 0x17,

    GFU_OPCODE_0x18 = 0x18,
    GFU_OPCODE_0x19 = 0x19,
    GFU_OPCODE_0x1A = 0x1A,
    GFU_OPCODE_0x = 0x1B,
    GFU_OPCODE_SPECIAL2 = 0x1C,
    GFU_OPCODE_0x1D = 0x1D,
    GFU_OPCODE_0x1E = 0x1E,
    GFU_OPCODE_SPECIAL3 = 0x1F,

    GFU_OPCODE_LB = 0x20,
    GFU_OPCODE_LH = 0x21,
    GFU_OPCODE_LWL = 0x22,
    GFU_OPCODE_LW = 0x23,
    GFU_OPCODE_LBU = 0x24,
    GFU_OPCODE_LHU = 0x25,
    GFU_OPCODE_LWR = 0x26,
    GFU_OPCODE_0x27 = 0x27,

    GFU_OPCODE_SB = 0x28,
    GFU_OPCODE_SH = 0x29,
    GFU_OPCODE_SWL = 0x2A,
    GFU_OPCODE_SW = 0x2B,
    GFU_OPCODE_0x2C = 0x2C,
    GFU_OPCODE_0x2D = 0x2D,
    GFU_OPCODE_SWR = 0x2E,
    GFU_OPCODE_CACHE = 0x2F,

    GFU_OPCODE_LL = 0x30,
    GFU_OPCODE_LWC1 = 0x31,
    GFU_OPCODE_LWC2 = 0x32,
    GFU_OPCODE_PREF = 0x33,
    GFU_OPCODE_0x34 = 0x34,
    GFU_OPCODE_LDC1 = 0x35,
    GFU_OPCODE_LDC2 = 0x36,
    GFU_OPCODE_0x37 = 0x37,

    GFU_OPCODE_SC = 0x38,
    GFU_OPCODE_SWC1 = 0x39,
    GFU_OPCODE_SWC2 = 0x3A,
    GFU_OPCODE_0x3B = 0x3B,
    GFU_OPCODE_0x3C = 0x3C,
    GFU_OPCODE_SDC1 = 0x3D,
    GFU_OPCODE_SDC2 = 0x3E,
    GFU_OPCODE_0x3F = 0x3F,

    GFU_OPCODE_MAX = 0x3F,
} gfu_opcode;

/// Secondary opcode field.
typedef enum gfu_funct {
    GFU_FUNCT_SLL = 0x00,
    GFU_FUNCT_MOVCI = 0x01,
    GFU_FUNCT_SRL = 0x02,
    GFU_FUNCT_SRA = 0x03,
    GFU_FUNCT_SLLV = 0x04,
    GFU_FUNCT_0x05 = 0x05,
    GFU_FUNCT_SRLV = 0x06,
    GFU_FUNCT_SRAV = 0x07,

    GFU_FUNCT_JR = 0x08,
    GFU_FUNCT_JALR = 0x09,
    GFU_FUNCT_MOVZ = 0x0A,
    GFU_FUNCT_MOVN = 0x0B,
    GFU_FUNCT_SYSCALL = 0x0C,
    GFU_FUNCT_BREAK = 0x0D,
    GFU_FUNCT_0x0E = 0x0E,
    GFU_FUNCT_SYNC = 0x0F,

    GFU_FUNCT_MFHI = 0x10,
    GFU_FUNCT_MTHI = 0x11,
    GFU_FUNCT_MFLO = 0x12,
    GFU_FUNCT_MTLO = 0x13,
    GFU_FUNCT_0x14 = 0x14,
    GFU_FUNCT_0x15 = 0x15,
    GFU_FUNCT_0x16 = 0x16,
    GFU_FUNCT_0x17 = 0x17,

    GFU_FUNCT_MULT = 0x18,
    GFU_FUNCT_MULTU = 0x19,
    GFU_FUNCT_DIV = 0x1A,
    GFU_FUNCT_DIVU = 0x1B,
    GFU_FUNCT_0x1C = 0x1C,
    GFU_FUNCT_0x1D = 0x1D,
    GFU_FUNCT_0x1E = 0x1E,
    GFU_FUNCT_0x1F = 0x1F,

    GFU_FUNCT_ADD = 0x20,
    GFU_FUNCT_ADDU = 0x21,
    GFU_FUNCT_SUB = 0x22,
    GFU_FUNCT_SUBU = 0x23,
    GFU_FUNCT_AND = 0x24,
    GFU_FUNCT_OR = 0x25,
    GFU_FUNCT_XOR = 0x26,
    GFU_FUNCT_NOR = 0x27,

    GFU_FUNCT_0x28 = 0x28,
    GFU_FUNCT_0x29 = 0x29,
    GFU_FUNCT_SLT = 0x2A,
    GFU_FUNCT_SLTU = 0x2B,
    GFU_FUNCT_0x2C = 0x2C,
    GFU_FUNCT_0x2D = 0x2D,
    GFU_FUNCT_0x2E = 0x2E,
    GFU_FUNCT_0x2F = 0x2F,

    GFU_FUNCT_TGE = 0x30,
    GFU_FUNCT_TGEU = 0x31,
    GFU_FUNCT_TLT = 0x32,
    GFU_FUNCT_TLTU = 0x33,
    GFU_FUNCT_TEQ = 0x34,
    GFU_FUNCT_0x35 = 0x35,
    GFU_FUNCT_TNE = 0x36,
    GFU_FUNCT_0x37 = 0x37,

    GFU_FUNCT_0x38 = 0x38,
    GFU_FUNCT_0x39 = 0x39,
    GFU_FUNCT_0x3A = 0x3A,
    GFU_FUNCT_0x3B = 0x3B,
    GFU_FUNCT_0x3C = 0x3C,
    GFU_FUNCT_0x3D = 0x3D,
    GFU_FUNCT_0x3E = 0x3E,
    GFU_FUNCT_0x3F = 0x3F,

    GFU_FUNCT_MAX = 0x3F,
} gfu_funct;

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

GFUARCH_INLINE uint32_t gfu_inst_add(gfu_register rd, gfu_register rs, gfu_register rt);
GFUARCH_INLINE uint32_t gfu_inst_addi(gfu_register rt, gfu_register rs, int32_t imm);
GFUARCH_INLINE uint32_t gfu_inst_addiu(gfu_register rt, gfu_register rs, int32_t imm);
GFUARCH_INLINE uint32_t gfu_inst_addu(gfu_register rd, gfu_register rs, gfu_register rt);
GFUARCH_INLINE uint32_t gfu_inst_div(gfu_register rs, gfu_register rt);
GFUARCH_INLINE uint32_t gfu_inst_divu(gfu_register rs, gfu_register rt);
GFUARCH_INLINE uint32_t gfu_inst_mult(gfu_register rs, gfu_register rt);
GFUARCH_INLINE uint32_t gfu_inst_multu(gfu_register rs, gfu_register rt);
GFUARCH_INLINE uint32_t gfu_inst_sub(gfu_register rd, gfu_register rs, gfu_register rt);
GFUARCH_INLINE uint32_t gfu_inst_subu(gfu_register rd, gfu_register rs, gfu_register rt);

/// ======================================================================= ///
/// CPU Branch and Jump Instructions.                                       ///
/// ======================================================================= ///

/// ======================================================================= ///
/// CPU Instruction Control Instructions.                                   ///
/// ======================================================================= ///

/// ======================================================================= ///
/// CPU Load, Store and Memory Control Instructions.                        ///
/// ======================================================================= ///

/// ======================================================================= ///
/// CPU Logical Instructions.                                               ///
/// ======================================================================= ///

GFUARCH_INLINE uint32_t gfu_inst_and(gfu_register rd, gfu_register rs, gfu_register rt);
GFUARCH_INLINE uint32_t gfu_inst_andi(gfu_register rt, gfu_register rs, int32_t imm);
GFUARCH_INLINE uint32_t gfu_inst_lui(gfu_register rt, int32_t immediate);
GFUARCH_INLINE uint32_t gfu_inst_nor(gfu_register rd, gfu_register rs, gfu_register rt);
GFUARCH_INLINE uint32_t gfu_inst_or(gfu_register rd, gfu_register rs, gfu_register rt);
GFUARCH_INLINE uint32_t gfu_inst_ori(gfu_register rt, gfu_register rs, int32_t imm);
GFUARCH_INLINE uint32_t gfu_inst_xor(gfu_register rd, gfu_register rs, gfu_register rt);
GFUARCH_INLINE uint32_t gfu_inst_xori(gfu_register rt, gfu_register rs, int32_t imm);

/// ======================================================================= ///
/// CPU Insert/Extract Instructions.                                        ///
/// ======================================================================= ///

/// ======================================================================= ///
/// CPU Move Instructions.                                                  ///
/// ======================================================================= ///

GFUARCH_INLINE uint32_t gfu_inst_movn(gfu_register rd, gfu_register rs, gfu_register rt);
GFUARCH_INLINE uint32_t gfu_inst_movz(gfu_register rd, gfu_register rs, gfu_register rt);

/// ======================================================================= ///
/// CPU Shift Instructions.                                                 ///
/// ======================================================================= ///

/// ======================================================================= ///
/// CPU Trap Instructions.                                                  ///
/// ======================================================================= ///

/// ======================================================================= ///
/// CPU Branch Instructions.                                                ///
/// ======================================================================= ///

/// ======================================================================= ///
/// CPU Coprocessor Branch Instructions.                                    ///
/// ======================================================================= ///

/// ======================================================================= ///
/// CPU Coprocessor Execute Instructions.                                   ///
/// ======================================================================= ///

/// ======================================================================= ///
/// CPU Coprocessor Load and Store Instructions.                            ///
/// ======================================================================= ///

/// ======================================================================= ///
/// CPU Coprocessor Move Instructions.                                      ///
/// ======================================================================= ///

#endif /* GFUARCH_H_ */


#ifdef GFUARCH_IMPLEMENTATION
#undef GFUARCH_IMPLEMENTATION

/// ======================================================================= ///
/// CPU Arithmetic Instructions.                                            ///
/// ======================================================================= ///

GFUARCH_INLINE uint32_t gfu_inst_add(gfu_register rd, gfu_register rs, gfu_register rt) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_SPECIAL) |
           GFU_INST_RD_ENC(rd) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_FUNCT_ENC(GFU_FUNCT_ADD);
}

GFUARCH_INLINE uint32_t gfu_inst_addi(gfu_register rt, gfu_register rs, int32_t imm) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_ADDI) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_IMM_ENC(imm);
}

GFUARCH_INLINE uint32_t gfu_inst_addiu(gfu_register rt, gfu_register rs, int32_t imm) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_ADDIU) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_IMM_ENC(imm);
}

GFUARCH_INLINE uint32_t gfu_inst_addu(gfu_register rd, gfu_register rs, gfu_register rt) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_SPECIAL) |
           GFU_INST_RD_ENC(rd) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_FUNCT_ENC(GFU_FUNCT_ADDU);
}

GFUARCH_INLINE uint32_t gfu_inst_div(gfu_register rs, gfu_register rt) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_SPECIAL) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_FUNCT_ENC(GFU_FUNCT_DIV);
}

GFUARCH_INLINE uint32_t gfu_inst_divu(gfu_register rs, gfu_register rt) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_SPECIAL) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_FUNCT_ENC(GFU_FUNCT_DIVU);
}

GFUARCH_INLINE uint32_t gfu_inst_mult(gfu_register rs, gfu_register rt) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_SPECIAL) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_FUNCT_ENC(GFU_FUNCT_MULT);
}

GFUARCH_INLINE uint32_t gfu_inst_multu(gfu_register rs, gfu_register rt) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_SPECIAL) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_FUNCT_ENC(GFU_FUNCT_MULTU);
}

GFUARCH_INLINE uint32_t gfu_inst_sub(gfu_register rd, gfu_register rs, gfu_register rt) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_SPECIAL) |
           GFU_INST_RD_ENC(rd) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_FUNCT_ENC(GFU_FUNCT_SUB);
}

GFUARCH_INLINE uint32_t gfu_inst_subu(gfu_register rd, gfu_register rs, gfu_register rt) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_SPECIAL) |
           GFU_INST_RD_ENC(rd) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_FUNCT_ENC(GFU_FUNCT_SUBU);
}

/// ======================================================================= ///
/// CPU Logical Instructions.                                               ///
/// ======================================================================= ///

GFUARCH_INLINE uint32_t gfu_inst_and(gfu_register rd, gfu_register rs, gfu_register rt) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_SPECIAL) |
           GFU_INST_RD_ENC(rd) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_FUNCT_ENC(GFU_FUNCT_AND);
}

GFUARCH_INLINE uint32_t gfu_inst_andi(gfu_register rt, gfu_register rs, int32_t imm) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_ANDI) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_IMM_ENC(imm);
}

GFUARCH_INLINE uint32_t gfu_inst_nor(gfu_register rd, gfu_register rs, gfu_register rt) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_SPECIAL) |
           GFU_INST_RD_ENC(rd) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_FUNCT_ENC(GFU_FUNCT_NOR);
}

GFUARCH_INLINE uint32_t gfu_inst_or(gfu_register rd, gfu_register rs, gfu_register rt) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_SPECIAL) |
           GFU_INST_RD_ENC(rd) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_FUNCT_ENC(GFU_FUNCT_OR);
}

GFUARCH_INLINE uint32_t gfu_inst_ori(gfu_register rt, gfu_register rs, int32_t imm) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_ORI) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_IMM_ENC(imm);
}

GFUARCH_INLINE uint32_t gfu_inst_xor(gfu_register rd, gfu_register rs, gfu_register rt) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_SPECIAL) |
           GFU_INST_RD_ENC(rd) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_FUNCT_ENC(GFU_FUNCT_XOR);
}

GFUARCH_INLINE uint32_t gfu_inst_xori(gfu_register rt, gfu_register rs, int32_t imm) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_XORI) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_IMM_ENC(imm);
}

/// ======================================================================= ///
/// CPU Move Instructions.                                                  ///
/// ======================================================================= ///

GFUARCH_INLINE uint32_t gfu_inst_movn(gfu_register rd, gfu_register rs, gfu_register rt) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_SPECIAL) |
           GFU_INST_RD_ENC(rd) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_FUNCT_ENC(GFU_FUNCT_MOVN);
}

GFUARCH_INLINE uint32_t gfu_inst_movz(gfu_register rd, gfu_register rs, gfu_register rt) {
    return GFU_INST_OPCODE_ENC(GFU_OPCODE_SPECIAL) |
           GFU_INST_RD_ENC(rd) |
           GFU_INST_RS_ENC(rs) |
           GFU_INST_RT_ENC(rt) |
           GFU_INST_FUNCT_ENC(GFU_FUNCT_MOVZ);
}

#endif /* GFUARCH_IMPLEMENTATION */
