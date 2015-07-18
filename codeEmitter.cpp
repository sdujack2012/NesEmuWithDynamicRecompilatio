/* 
 * File:   codeEmitter.cpp
 * Author: 凯
 * 
 * Created on 2015年1月3日, 下午12:59
 */

#include "codeEmitter.h"
#include <sys/mman.h>
#include <memory.h>
#include <cstdlib>
#include <stdio.h>

codeEmitter::codeEmitter() {
    currentBlock = 0;
    startPointer = 0;
}



codeEmitter::~codeEmitter() {
}

void codeEmitter::BeginEmit() {
    currentBlock = 0;
    startPointer = 0;

    unsigned int codeBytes = 4096;
    startPointer = currentBlock = (int8*) mmap(
            NULL,
            codeBytes,
            PROT_READ | PROT_WRITE | PROT_EXEC,
            MAP_ANONYMOUS | MAP_SHARED,
            0,
            0);
}

int8* codeEmitter::EndEmit() {

    return currentBlock;
}

void codeEmitter::Ret() {
    write8(0xC3);
}

void codeEmitter::ModRM(unint8 mod, unint8 rm, unint8 reg) {
    write8((mod << 6) | (rm << 3) | (reg));
}

void codeEmitter::SibSB(unint8 ss, unint8 rm, unint8 index) {
    write8((ss << 6) | (rm << 3) | (index));
}

void codeEmitter::write8(unint8 Val8) {

    *(unint8*) currentBlock = Val8;
    currentBlock++;

}

////////////////////////////////////////////////////

void codeEmitter::write16(unint16 Val16) {
    *(unint16*) currentBlock = Val16;
    currentBlock += 2;

}

void codeEmitter::write32(unint32 Val32) {

    *(unint32*) currentBlock = Val32;

    currentBlock += 4;

}

void codeEmitter::write64(unint64 val64) {

    *(unint64*) currentBlock = val64;
    currentBlock += 8;

}

void codeEmitter::Mov16RtoM(unint32 to, x86RegType from) {
    write8(0x66);
    write8(0x89);
    ModRM(0, from, DISP32);
    write32(to);
}

void codeEmitter::Mov16MtoR(x86RegType to, unint32 from) {
    write8(0x66);
    write8(0x8B);
    ModRM(0, to, DISP32);
    write32(from);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void codeEmitter::Add32ItoR(x86RegType to, unint32 from) {
    if (to == EAX) {
        write8(0x05);
    } else {
        write8(0x81);
        ModRM(3, 0, to);
    }

    write32(from);
}

void codeEmitter::ExecuteBlock() {
    if (currentBlock > 0) {
        X86Fun BlockFunction = (X86Fun) startPointer;
        BlockFunction();
    }
}

void codeEmitter::SET8R(unint8 cc, unint8 to) {
    write8(0x0F);
    write8(cc);
    write8(0xC0 | (to));
}

void codeEmitter::J8Rel(unint8 cc, unint8 to) {
    write8(cc);
    write8(to);

}

void codeEmitter::J32Rel(unint8 cc, unint32 to) {
    write8(0x0F);
    write8(cc);
    write32(to);

}

void codeEmitter::CMOV32RtoR(unint8 cc, unint8 to, unint8 from) {
    write8(0x0F);
    write8(cc);
    ModRM(3, to, from);
}

void codeEmitter::CMOV32MtoR(unint8 cc, unint8 to, unint32 from) {
    write8(0x0F);
    write8(cc);
    ModRM(0, to, DISP32);
    write32(from);
}

////////////////////////////////////////////////////

void codeEmitter::x86SetPtr(char* ptr) {
    currentBlock = ptr;
}

////////////////////////////////////////////////////

void codeEmitter::x86Shutdown(void) {
}

////////////////////////////////////////////////////

void codeEmitter::x86SetJ8(unint8* j8) {
    unint32 jump = (currentBlock - (int8*) j8) - 1;

    if (jump > 0x7f) {
        printf("j8 greater than 0x7f!!\n");
    }
    *j8 = (unint8) jump;
}

////////////////////////////////////////////////////

void codeEmitter::x86SetJ32(unint32* j32) {
    *j32 = (currentBlock - (int8*) j32) - 4;
}

////////////////////////////////////////////////////

void codeEmitter::x86Align(int bytes) {
    // fordward align
    currentBlock = (int8*) (((unint32) currentBlock + bytes) & ~(bytes - 1));
}

/********************/
/* IX86 intructions */

/********************/

void codeEmitter::STC(void) {
    write8(0xF9);
}

void codeEmitter::CLC(void) {
    write8(0xF8);
}

////////////////////////////////////
// mov instructions                /
////////////////////////////////////

/* mov r32 to r32 */
void codeEmitter::MOV32RtoR(x86RegType to, x86RegType from) {
    write8(0x89);
    ModRM(3, from, to);
}

/* mov r32 to m32 */
void codeEmitter::MOV32RtoM(unint32 to, x86RegType from) {
    write8(0x89);
    ModRM(0, from, DISP32);
    write32(to);
}

/* mov m32 to r32 */
void codeEmitter::MOV32MtoR(x86RegType to, unint32 from) {
    write8(0x8B);
    ModRM(0, to, DISP32);
    write32(from);
}

/* mov [r32] to r32 */
void codeEmitter::MOV32RmtoR(x86RegType to, x86RegType from) {
    write8(0x8B);
    ModRM(0, to, from);
}

/* mov [r32][r32*scale] to r32 */
void codeEmitter::MOV32RmStoR(x86RegType to, x86RegType from, x86RegType from2, int scale) {
    write8(0x8B);
    ModRM(0, to, 0x4);
    SibSB(scale, from2, from);
}

/* mov r32 to [r32] */
void codeEmitter::MOV32RtoRm(x86RegType to, x86RegType from) {
    write8(0x89);
    ModRM(0, from, to);
}

/* mov r32 to [r32][r32*scale] */
void codeEmitter::MOV32RtoRmS(x86RegType to, x86RegType from, x86RegType from2, int scale) {
    write8(0x89);
    ModRM(0, to, 0x4);
    SibSB(scale, from2, from);
}

/* mov imm32 to r32 */
void codeEmitter::MOV32ItoR(x86RegType to, unint32 from) {
    write8(0xB8 | to);
    write32(from);
}

/* mov imm32 to m32 */
void codeEmitter::MOV32ItoM(unint32 to, unint32 from) {
    write8(0xC7);
    ModRM(0, 0, DISP32);
    write32(to);
    write32(from);
}

/* mov r16 to m16 */
void codeEmitter::MOV16RtoM(unint32 to, x86RegType from) {
    write8(0x66);
    write8(0x89);
    ModRM(0, from, DISP32);
    write32(to);
}

/* mov m16 to r16 */
void codeEmitter::MOV16MtoR(x86RegType to, unint32 from) {
    write8(0x66);
    write8(0x8B);
    ModRM(0, to, DISP32);
    write32(from);
}

/* mov imm16 to m16 */
void codeEmitter::MOV16ItoM(unint32 to, unint16 from) {
    write8(0x66);
    write8(0xC7);
    ModRM(0, 0, DISP32);
    write32(to);
    write16(from);
}

/* movsx r8 to r32 */
void codeEmitter::MOVSX32R8toR(x86RegType to, x86RegType from) {
    write16(0xBE0F);
    ModRM(3, to, from);
}

/* movsx m8 to r32 */
void codeEmitter::MOVSX32M8toR(x86RegType to, unint32 from) {
    write16(0xBE0F);
    ModRM(0, to, DISP32);
    write32(from);
}

/* movsx r16 to r32 */
void codeEmitter::MOVSX32R16toR(x86RegType to, x86RegType from) {
    write16(0xBF0F);
    ModRM(3, to, from);
}

/* movsx m16 to r32 */
void codeEmitter::MOVSX32M16toR(x86RegType to, unint32 from) {
    write16(0xBF0F);
    ModRM(0, to, DISP32);
    write32(from);
}

/* movzx r8 to r32 */
void codeEmitter::MOVZX32R8toR(x86RegType to, x86RegType from) {
    write16(0xB60F);
    ModRM(3, to, from);
}

/* movzx m8 to r32 */
void codeEmitter::MOVZX32M8toR(x86RegType to, unint32 from) {
    write16(0xB60F);
    ModRM(0, to, DISP32);
    write32(from);
}

/* movzx r16 to r32 */
void codeEmitter::MOVZX32R16toR(x86RegType to, x86RegType from) {
    write16(0xB70F);
    ModRM(3, to, from);
}

/* movzx m16 to r32 */
void codeEmitter::MOVZX32M16toR(x86RegType to, unint32 from) {
    write16(0xB70F);
    ModRM(0, to, DISP32);
    write32(from);
}

/* cmovbe r32 to r32 */
void codeEmitter::CMOVBE32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x46, to, from);
}

/* cmovbe m32 to r32*/
void codeEmitter::CMOVBE32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x46, to, from);
}

/* cmovb r32 to r32 */
void codeEmitter::CMOVB32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x42, to, from);
}

/* cmovb m32 to r32*/
void codeEmitter::CMOVB32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x42, to, from);
}

/* cmovae r32 to r32 */
void codeEmitter::CMOVAE32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x43, to, from);
}

/* cmovae m32 to r32*/
void codeEmitter::CMOVAE32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x43, to, from);
}

/* cmova r32 to r32 */
void codeEmitter::CMOVA32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x47, to, from);
}

/* cmova m32 to r32*/
void codeEmitter::CMOVA32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x47, to, from);
}

/* cmovo r32 to r32 */
void codeEmitter::CMOVO32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x40, to, from);
}

/* cmovo m32 to r32 */
void codeEmitter::CMOVO32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x40, to, from);
}

/* cmovp r32 to r32 */
void codeEmitter::CMOVP32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x4A, to, from);
}

/* cmovp m32 to r32 */
void codeEmitter::CMOVP32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x4A, to, from);
}

/* cmovs r32 to r32 */
void codeEmitter::CMOVS32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x48, to, from);
}

/* cmovs m32 to r32 */
void codeEmitter::CMOVS32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x48, to, from);
}

/* cmovno r32 to r32 */
void codeEmitter::CMOVNO32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x41, to, from);
}

/* cmovno m32 to r32 */
void codeEmitter::CMOVNO32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x41, to, from);
}

/* cmovnp r32 to r32 */
void codeEmitter::CMOVNP32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x4B, to, from);
}

/* cmovnp m32 to r32 */
void codeEmitter::CMOVNP32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x4B, to, from);
}

/* cmovns r32 to r32 */
void codeEmitter::CMOVNS32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x49, to, from);
}

/* cmovns m32 to r32 */
void codeEmitter::CMOVNS32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x49, to, from);
}

/* cmovne r32 to r32 */
void codeEmitter::CMOVNE32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x45, to, from);
}

/* cmovne m32 to r32*/
void codeEmitter::CMOVNE32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x45, to, from);
}

/* cmove r32 to r32*/
void codeEmitter::CMOVE32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x44, to, from);
}

/* cmove m32 to r32*/
void codeEmitter::CMOVE32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x44, to, from);
}

/* cmovg r32 to r32*/
void codeEmitter::CMOVG32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x4F, to, from);
}

/* cmovg m32 to r32*/
void codeEmitter::CMOVG32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x4F, to, from);
}

/* cmovge r32 to r32*/
void codeEmitter::CMOVGE32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x4D, to, from);
}

/* cmovge m32 to r32*/
void codeEmitter::CMOVGE32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x4D, to, from);
}

/* cmovl r32 to r32*/
void codeEmitter::CMOVL32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x4C, to, from);
}

/* cmovl m32 to r32*/
void codeEmitter::CMOVL32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x4C, to, from);
}

/* cmovle r32 to r32*/
void codeEmitter::CMOVLE32RtoR(x86RegType to, x86RegType from) {
    CMOV32RtoR(0x4E, to, from);
}

/* cmovle m32 to r32*/
void codeEmitter::CMOVLE32MtoR(x86RegType to, unint32 from) {
    CMOV32MtoR(0x4E, to, from);
}

////////////////////////////////////
// arithmetic instructions         /
////////////////////////////////////

/* add imm32 to r32 */
void codeEmitter::ADD32ItoR(x86RegType to, unint32 from) {
    write8(0x81);
    ModRM(3, 0, to);


    write32(from);
}

/* add imm32 to m32 */
void codeEmitter::ADD32ItoM(unint32 to, unint32 from) {
    write8(0x81);
    ModRM(0, 0, DISP32);
    write32(to);
    write32(from);
}

/* add r32 to r32 */
void codeEmitter::ADD32RtoR(x86RegType to, x86RegType from) {
    write8(0x01);
    ModRM(3, from, to);
}

/* add r32 to m32 */
void codeEmitter::ADD32RtoM(unint32 to, x86RegType from) {
    write8(0x01);
    ModRM(0, from, DISP32);
    write32(to);
}

/* add m32 to r32 */
void codeEmitter::ADD32MtoR(x86RegType to, unint32 from) {
    write8(0x03);
    ModRM(0, to, DISP32);
    write32(from);
}

/* add imm16 to r16 */
void codeEmitter::ADD16ItoR(x86RegType to, unint16 from) {
    write8(0x66);
    write8(0x81);
    ModRM(3, 0, to);
    write16(from);
}

/* add r16 to m16 */
void codeEmitter::ADD16RtoM(unint32 to, x86RegType from) {
    write8(0x66);
    write8(0x01);
    ModRM(0, from, DISP32);
    write32(to);
}

/* add m16 to r16 */
void codeEmitter::ADD16MtoR(x86RegType to, unint32 from) {
    write8(0x66);
    write8(0x03);
    ModRM(0, to, DISP32);
    write32(from);
}

/* adc imm32 to r32 */
void codeEmitter::ADC32ItoR(x86RegType to, unint32 from) {

    write8(0x81);
    ModRM(3, 2, to);

    write32(from);
}

/* adc imm32 to m32 */
void codeEmitter::ADC32ItoM(unint32 to, unint32 from) {
    write8(0x81);
    ModRM(0, 2, DISP32);
    write32(to);
    write32(from);
}

/* adc r32 to r32 */
void codeEmitter::ADC32RtoR(x86RegType to, x86RegType from) {
    write8(0x11);
    ModRM(3, from, to);
}

/* adc m32 to r32 */
void codeEmitter::ADC32MtoR(x86RegType to, unint32 from) {
    write8(0x13);
    ModRM(0, to, DISP32);
    write32(from);
}

/* inc r32 */
void codeEmitter::INC32R(x86RegType to) {
    write8(0x40 + to);
}

/* inc m32 */
void codeEmitter::INC32M(unint32 to) {
    write8(0xFF);
    ModRM(0, 0, DISP32);
    write32(to);
}

/* inc r16 */
void codeEmitter::INC16R(x86RegType to) {
    write8(0x66);
    write8(0x40 + to);
}

/* inc m16 */
void codeEmitter::INC16M(unint32 to) {
    write8(0x66);
    write8(0xFF);
    ModRM(0, 0, DISP32);
    write32(to);
}

/* sub imm32 to r32 */
void codeEmitter::SUB32ItoR(x86RegType to, unint32 from) {

    write8(0x81);
    ModRM(3, 5, to);

    write32(from);
}

/* sub imm32 to m32 */
void codeEmitter::SUB32ItoM(unint32 to, unint32 from) {
    write8(0x81);
    ModRM(0, 5, DISP32);
    write32(to);
    write32(from);
}

/* sub r32 to r32 */
void codeEmitter::SUB32RtoR(x86RegType to, x86RegType from) {
    write8(0x29);
    ModRM(3, from, to);
}

/* sub m32 to r32 */
void codeEmitter::SUB32MtoR(x86RegType to, unint32 from) {
    write8(0x2B);
    ModRM(0, to, DISP32);
    write32(from);
}

/* sub m16 to r16 */
void codeEmitter::SUB16MtoR(x86RegType to, unint32 from) {
    write8(0x66);
    write8(0x2B);
    ModRM(0, to, DISP32);
    write32(from);
}

/* sbb imm32 to r32 */
void codeEmitter::SBB32ItoR(x86RegType to, unint32 from) {

    write8(0x81);
    ModRM(3, 3, to);

    write32(from);
}

/* sbb imm32 to m32 */
void codeEmitter::SBB32ItoM(unint32 to, unint32 from) {
    write8(0x81);
    ModRM(0, 3, DISP32);
    write32(to);
    write32(from);
}

/* sbb r32 to r32 */
void codeEmitter::SBB32RtoR(x86RegType to, x86RegType from) {
    write8(0x19);
    ModRM(3, from, to);
}

/* sbb m32 to r32 */
void codeEmitter::SBB32MtoR(x86RegType to, unint32 from) {
    write8(0x1B);
    ModRM(0, to, DISP32);
    write32(from);
}

/* dec r32 */
void codeEmitter::DEC32R(x86RegType to) {
    write8(0x48 + to);
}

/* dec m32 */
void codeEmitter::DEC32M(unint32 to) {
    write8(0xFF);
    ModRM(0, 1, DISP32);
    write32(to);
}

/* dec r16 */
void codeEmitter::DEC16R(x86RegType to) {
    write8(0x66);
    write8(0x48 + to);
}

/* dec m16 */
void codeEmitter::DEC16M(unint32 to) {
    write8(0x66);
    write8(0xFF);
    ModRM(0, 1, DISP32);
    write32(to);
}

/* mul eax by r32 to edx:eax */
void codeEmitter::MUL32R(x86RegType from) {
    write8(0xF7);
    ModRM(3, 4, from);
}

/* imul eax by r32 to edx:eax */
void codeEmitter::IMUL32R(x86RegType from) {
    write8(0xF7);
    ModRM(3, 5, from);
}

/* mul eax by m32 to edx:eax */
void codeEmitter::MUL32M(unint32 from) {
    write8(0xF7);
    ModRM(0, 4, DISP32);
    write32(from);
}

/* imul eax by m32 to edx:eax */
void codeEmitter::IMUL32M(unint32 from) {
    write8(0xF7);
    ModRM(0, 5, DISP32);
    write32(from);
}

/* imul r32 by r32 to r32 */
void codeEmitter::IMUL32RtoR(x86RegType to, x86RegType from) {
    write16(0xAF0F);
    ModRM(3, to, from);
}

/* div eax by r32 to edx:eax */
void codeEmitter::DIV32R(x86RegType from) {
    write8(0xF7);
    ModRM(3, 6, from);
}

/* idiv eax by r32 to edx:eax */
void codeEmitter::IDIV32R(x86RegType from) {
    write8(0xF7);
    ModRM(3, 7, from);
}

/* div eax by m32 to edx:eax */
void codeEmitter::DIV32M(unint32 from) {
    write8(0xF7);
    ModRM(0, 6, DISP32);
    write32(from);
}

/* idiv eax by m32 to edx:eax */
void codeEmitter::IDIV32M(unint32 from) {
    write8(0xF7);
    ModRM(0, 7, DISP32);
    write32(from);
}

////////////////////////////////////
// shifting instructions           /
////////////////////////////////////

/* shl imm8 to r32 */
void codeEmitter::SHL32ItoR(x86RegType to, unint8 from) {
    if (from == 1) {
        write8(0xD1);
        write8(0xE0 | to);
        return;
    }
    write8(0xC1);
    ModRM(3, 4, to);
    write8(from);
}

/* shl imm8 to m32 */
void codeEmitter::SHL32ItoM(unint32 to, unint8 from) {
    if (from == 1) {
        write8(0xD1);
        ModRM(0, 4, to);
    } else {
        write8(0xC1);
        ModRM(0, 4, to);
        write8(from);
    }
}

/* shl cl to r32 */
void codeEmitter::SHL32CLtoR(x86RegType to) {
    write8(0xD3);
    ModRM(3, 4, to);
}

/* shr imm8 to r32 */
void codeEmitter::SHR32ItoR(x86RegType to, unint8 from) {
    if (from == 1) {
        write8(0xD1);
        write8(0xE8 | to);
    } else {
        write8(0xC1);
        ModRM(3, 5, to);
        write8(from);
    }
}

/* shr imm8 to m32 */
void codeEmitter::SHR32ItoM(unint32 to, unint8 from) {
    if (from == 1) {
        write8(0xD1);
        ModRM(0, 5, to);
    } else {
        write8(0xC1);
        ModRM(0, 5, to);
        write8(from);
    }
}

/* shr cl to r32 */
void codeEmitter::SHR32CLtoR(x86RegType to) {
    write8(0xD3);
    ModRM(3, 5, to);
}

/* sar imm8 to r32 */
void codeEmitter::SAR32ItoR(x86RegType to, unint8 from) {
    write8(0xC1);
    ModRM(3, 7, to);
    write8(from);
}

/* sar imm8 to m32 */
void codeEmitter::SAR32ItoM(unint32 to, unint8 from) {
    write8(0xC1);
    ModRM(0, 7, DISP32);
    write32(to);
    write8(from);
}

/* sar cl to r32 */
void codeEmitter::SAR32CLtoR(x86RegType to) {
    write8(0xD3);
    ModRM(3, 7, to);
}

void codeEmitter::RCR32ItoR(x86RegType to, unint8 from) {
    if (from == 1) {
        write8(0xd1);
        write8(0xd8 | to);
    } else {
        write8(0xc1);
        write8(0xd8 | to);
        write8(from);
    }
}

// shld imm8 to r32

void codeEmitter::SHLD32ItoR(unint32 to, unint32 from, unint8 shift) {
    write8(0x0F);
    write8(0xA4);
    ModRM(3, to, from);
    write8(shift);
}

// shrd imm8 to r32

void codeEmitter::SHRD32ItoR(unint32 to, unint32 from, unint8 shift) {
    write8(0x0F);
    write8(0xAC);
    ModRM(3, to, from);
    write8(shift);
}

////////////////////////////////////
// logical instructions            /
////////////////////////////////////

/* or imm32 to r32 */
void codeEmitter::OR32ItoR(x86RegType to, unint32 from) {
    write8(0x81);
    ModRM(3, 1, to);

    write32(from);
}

/* or imm32 to m32 */
void codeEmitter::OR32ItoM(unint32 to, unint32 from) {
    write8(0x81);
    ModRM(0, 1, DISP32);
    write32(to);
    write32(from);
}

/* or r32 to r32 */
void codeEmitter::OR32RtoR(x86RegType to, x86RegType from) {
    write8(0x09);
    ModRM(3, from, to);
}

/* or r32 to m32 */
void codeEmitter::OR32RtoM(unint32 to, x86RegType from) {
    write8(0x09);
    ModRM(0, from, DISP32);
    write32(to);
}

/* or m32 to r32 */
void codeEmitter::OR32MtoR(x86RegType to, unint32 from) {
    write8(0x0B);
    ModRM(0, to, DISP32);
    write32(from);
}

/* or m16 to r16 */
void codeEmitter::OR16MtoR(x86RegType to, unint32 from) {
    write8(0x66);
    write8(0x0B);
    ModRM(0, to, DISP32);
    write32(from);
}

/* xor imm32 to r32 */
void codeEmitter::XOR32ItoR(x86RegType to, unint32 from) {
    write8(0x81);
    ModRM(3, 6, to);

    write32(from);
}

/* xor imm32 to m32 */
void codeEmitter::XOR32ItoM(unint32 to, unint32 from) {
    write8(0x81);
    ModRM(0, 6, DISP32);
    write32(to);
    write32(from);
}

/* xor r32 to r32 */
void codeEmitter::XOR32RtoR(x86RegType to, x86RegType from) {
    write8(0x31);
    ModRM(3, from, to);
}

/* xor r16 to r16 */
void codeEmitter::XOR16RtoR(x86RegType to, x86RegType from) {
    write8(0x66);
    write8(0x31);
    ModRM(3, from, to);
}

/* xor r32 to m32 */
void codeEmitter::XOR32RtoM(unint32 to, x86RegType from) {
    write8(0x31);
    ModRM(0, from, DISP32);
    write32(to);
}

/* xor m32 to r32 */
void codeEmitter::XOR32MtoR(x86RegType to, unint32 from) {
    write8(0x33);
    ModRM(0, to, DISP32);
    write32(from);
}

/* and imm32 to r32 */
void codeEmitter::AND32ItoR(x86RegType to, unint32 from) {

    write8(0x81);
    ModRM(3, 0x4, to);

    write32(from);
}

/* and imm32 to m32 */
void codeEmitter::AND32ItoM(unint32 to, unint32 from) {
    write8(0x81);
    ModRM(0, 0x4, DISP32);
    write32(to);
    write32(from);
}

/* and r32 to r32 */
void codeEmitter::AND32RtoR(x86RegType to, x86RegType from) {
    write8(0x21);
    ModRM(3, from, to);
}

/* and r32 to m32 */
void codeEmitter::AND32RtoM(unint32 to, x86RegType from) {
    write8(0x21);
    ModRM(0, from, DISP32);
    write32(to);
}

/* and m32 to r32 */
void codeEmitter::AND32MtoR(x86RegType to, unint32 from) {
    write8(0x23);
    ModRM(0, to, DISP32);
    write32(from);
}

/* and m16 to r16 */
void codeEmitter::AND16MtoR(x86RegType to, unint32 from) {
    write8(0x66);
    write8(0x23);
    ModRM(0, to, DISP32);
    write32(from);
}

/* not r32 */
void codeEmitter::NOT32R(x86RegType from) {
    write8(0xF7);
    ModRM(3, 2, from);
}

/* neg r32 */
void codeEmitter::NEG32R(x86RegType from) {
    write8(0xF7);
    ModRM(3, 3, from);
}

/* neg r16 */
void codeEmitter::NEG16R(x86RegType from) {
    write8(0x66);
    write8(0xF7);
    ModRM(3, 3, from);
}

////////////////////////////////////
// jump instructions               /
////////////////////////////////////

void codeEmitter::JMP(unint32 to) {
    unint32 jump = (currentBlock - (int8*) to) - 1;

    if (jump > 0x7f) {
        JMP32(to);
    } else {
        JMP8(to);
    }
}

/* jmp rel8 */
void codeEmitter::JMP8(unint8 to) {
    write8(0xEB);
    write8(to);

}

/* jmp rel32 */
void codeEmitter::JMP32(unint32 to) {
    write8(0xE9);
    write32(to);

}

/* jmp r32 */
void codeEmitter::JMP32R(x86RegType to) {
    write8(0xFF);
    ModRM(3, 4, to);
}

/* jp rel8 */
void codeEmitter::JP8(unint8 to) {
    J8Rel(0x7A, to);
}

/* jnp rel8 */
void codeEmitter::JNP8(unint8 to) {
    J8Rel(0x7B, to);
}

/* je rel8 */
void codeEmitter::JE8(unint8 to) {
    J8Rel(0x74, to);
}

/* jz rel8 */
void codeEmitter::JZ8(unint8 to) {
    J8Rel(0x74, to);
}

/* js rel8 */
void codeEmitter::JS8(unint8 to) {
    J8Rel(0x78, to);
}

/* jns rel8 */
void codeEmitter::JNS8(unint8 to) {
    J8Rel(0x79, to);
}

/* jg rel8 */
void codeEmitter::JG8(unint8 to) {
    J8Rel(0x7F, to);
}

/* jge rel8 */
void codeEmitter::JGE8(unint8 to) {
    J8Rel(0x7D, to);
}

/* jl rel8 */
void codeEmitter::JL8(unint8 to) {
    J8Rel(0x7C, to);
}

/* ja rel8 */
void codeEmitter::JA8(unint8 to) {
    J8Rel(0x77, to);
}

void codeEmitter::JAE8(unint8 to) {
    J8Rel(0x73, to);
}

/* jb rel8 */
void codeEmitter::JB8(unint8 to) {
    J8Rel(0x72, to);
}

/* jbe rel8 */
void codeEmitter::JBE8(unint8 to) {
    J8Rel(0x76, to);
}

/* jle rel8 */
void codeEmitter::JLE8(unint8 to) {
    J8Rel(0x7E, to);
}

/* jne rel8 */
void codeEmitter::JNE8(unint8 to) {
    J8Rel(0x75, to);
}

/* jnz rel8 */
void codeEmitter::JNZ8(unint8 to) {
    J8Rel(0x75, to);
}

/* jng rel8 */
void codeEmitter::JNG8(unint8 to) {
    J8Rel(0x7E, to);
}

/* jnge rel8 */
void codeEmitter::JNGE8(unint8 to) {
    J8Rel(0x7C, to);
}

/* jnl rel8 */
void codeEmitter::JNL8(unint8 to) {
    J8Rel(0x7D, to);
}

/* jnle rel8 */
void codeEmitter::JNLE8(unint8 to) {
    J8Rel(0x7F, to);
}

/* jo rel8 */
void codeEmitter::JO8(unint8 to) {
    J8Rel(0x70, to);
}

/* jno rel8 */
void codeEmitter::JNO8(unint8 to) {
    J8Rel(0x71, to);
}

/* je rel32 */
void codeEmitter::JE32(unint32 to) {
    J32Rel(0x84, to);
}

/* jz rel32 */
void codeEmitter::JZ32(unint32 to) {
    J32Rel(0x84, to);
}

/* jg rel32 */
void codeEmitter::JG32(unint32 to) {
    J32Rel(0x8F, to);
}

/* jge rel32 */
void codeEmitter::JGE32(unint32 to) {
    J32Rel(0x8D, to);
}

/* jl rel32 */
void codeEmitter::JL32(unint32 to) {
    J32Rel(0x8C, to);
}

/* jle rel32 */
void codeEmitter::JLE32(unint32 to) {
    J32Rel(0x8E, to);
}

/* jne rel32 */
void codeEmitter::JNE32(unint32 to) {
    J32Rel(0x85, to);
}

/* jnz rel32 */
void codeEmitter::JNZ32(unint32 to) {
    J32Rel(0x85, to);
}

/* jng rel32 */
void codeEmitter::JNG32(unint32 to) {
    J32Rel(0x8E, to);
}

/* jnge rel32 */
void codeEmitter::JNGE32(unint32 to) {
    J32Rel(0x8C, to);
}

/* jnl rel32 */
void codeEmitter::JNL32(unint32 to) {
    J32Rel(0x8D, to);
}

/* jnle rel32 */
void codeEmitter::JNLE32(unint32 to) {
    J32Rel(0x8F, to);
}

/* jo rel32 */
void codeEmitter::JO32(unint32 to) {
    J32Rel(0x80, to);
}

/* jno rel32 */
void codeEmitter::JNO32(unint32 to) {
    J32Rel(0x81, to);
}

/* call func */
void codeEmitter::CALLFunc(unint32 func) {
    CALL32(func - ((unint32) currentBlock + 5));
}

/* call rel32 */
void codeEmitter::CALL32(unint32 to) {
    write8(0xE8);
    write32(to);
}

/* call r32 */
void codeEmitter::CALL32R(x86RegType to) {
    write8(0xFF);
    ModRM(3, 2, to);
}

/* call m32 */
void codeEmitter::CALL32M(unint32 to) {
    write8(0xFF);
    ModRM(0, 2, DISP32);
    write32(to);
}

////////////////////////////////////
// misc instructions               /
////////////////////////////////////

/* cmp imm32 to r32 */
void codeEmitter::CMP32ItoR(x86RegType to, unint32 from) {

    write8(0x81);
    ModRM(3, 7, to);

    write32(from);
}

/* cmp imm32 to m32 */
void codeEmitter::CMP32ItoM(unint32 to, unint32 from) {
    write8(0x81);
    ModRM(0, 7, DISP32);
    write32(to);
    write32(from);
}

/* cmp r32 to r32 */
void codeEmitter::CMP32RtoR(x86RegType to, x86RegType from) {
    write8(0x39);
    ModRM(3, from, to);
}

/* cmp m32 to r32 */
void codeEmitter::CMP32MtoR(x86RegType to, unint32 from) {
    write8(0x3B);
    ModRM(0, to, DISP32);
    write32(from);
}

/* test imm32 to r32 */
void codeEmitter::TEST32ItoR(x86RegType to, unint32 from) {

    write8(0xF7);
    ModRM(3, 0, to);

    write32(from);
}

/* test r32 to r32 */
void codeEmitter::TEST32RtoR(x86RegType to, x86RegType from) {
    write8(0x85);
    ModRM(3, from, to);
}

/* sets r8 */
void codeEmitter::SETS8R(x86RegType to) {
    SET8R(0x98, to);
}

/* setl r8 */
void codeEmitter::SETL8R(x86RegType to) {
    SET8R(0x9C, to);
}

/* setb r8 */
void codeEmitter::SETB8R(x86RegType to) {
    SET8R(0x92, to);
}

/* setb r8 */
void codeEmitter::SETNZ8R(x86RegType to) {
    SET8R(0x95, to);
}

/* cbw */
void codeEmitter::CBW(void) {
    write16(0x9866);
}

/* cwd */
void codeEmitter::CWD(void) {
    write8(0x98);
}

/* cdq */
void codeEmitter::CDQ(void) {
    write8(0x99);
}

/* push r32 */
void codeEmitter::PUSH32R(x86RegType from) {
    write8(0x50 | from);
}

/* push m32 */
void codeEmitter::PUSH32M(unint32 from) {
    write8(0xFF);
    ModRM(0, 6, DISP32);
    write32(from);
}

/* push imm32 */
void codeEmitter::PUSH32I(unint32 from) {
    write8(0x68);
    write32(from);
}

/* pop r32 */
void codeEmitter::POP32R(x86RegType from) {
    write8(0x58 | from);
}

/* pushad */
void codeEmitter::PUSHA32(void) {
    write8(0x60);
}

/* popad */
void codeEmitter::POPA32(void) {
    write8(0x61);
}

/* pushfd */
void codeEmitter::PUSHFD(void) {
    write8(0x9C);
}

/* popfd */
void codeEmitter::POPFD(void) {
    write8(0x9D);
}

/* ret */
void codeEmitter::RET(void) {
    write8(0xC3);
}

void codeEmitter::BT32ItoR(x86RegType to, x86RegType from) {
    write16(0xBA0F);
    write8(0xE0 | to);
    write8(from);
}

/********************/
/* FPU instructions */
/********************/

/* fild m32 to fpu reg stack */
void codeEmitter::FILD32(unint32 from) {
    write8(0xDB);
    ModRM(0, 0x0, DISP32);
    write32(from);
}

/* fistp m32 from fpu reg stack */
void codeEmitter::FISTP32(unint32 from) {
    write8(0xDB);
    ModRM(0, 0x3, DISP32);
    write32(from);
}

/* fld m32 to fpu reg stack */
void codeEmitter::FLD32(unint32 from) {
    write8(0xD9);
    ModRM(0, 0x0, DISP32);
    write32(from);
}

/* fst m32 from fpu reg stack */
void codeEmitter::FST32(unint32 to) {
    write8(0xD9);
    ModRM(0, 0x2, DISP32);
    write32(to);
}

/* fstp m32 from fpu reg stack */
void codeEmitter::FSTP32(unint32 to) {
    write8(0xD9);
    ModRM(0, 0x3, DISP32);
    write32(to);
}

/* fldcw fpu control word from m16 */
void codeEmitter::FLDCW(unint32 from) {
    write8(0xD9);
    ModRM(0, 0x5, DISP32);
    write32(from);
}

/* fnstcw fpu control word to m16 */
void codeEmitter::FNSTCW(unint32 to) {
    write8(0xD9);
    ModRM(0, 0x7, DISP32);
    write32(to);
}

void codeEmitter::FNSTSWtoAX(void) {
    write16(0xE0DF);
}

/* fadd ST(src) to fpu reg stack ST(0) */
void codeEmitter::FADD32Rto0(x86RegType src) {
    write8(0xD8);
    write8(0xC0 + src);
}

/* fadd ST(0) to fpu reg stack ST(src) */
void codeEmitter::FADD320toR(x86RegType src) {
    write8(0xDC);
    write8(0xC0 + src);
}

/* fsub ST(src) to fpu reg stack ST(0) */
void codeEmitter::FSUB32Rto0(x86RegType src) {
    write8(0xD8);
    write8(0xE0 + src);
}

/* fsub ST(0) to fpu reg stack ST(src) */
void codeEmitter::FSUB320toR(x86RegType src) {
    write8(0xDC);
    write8(0xE8 + src);
}

/* fsubp -> substract ST(0) from ST(1), store in ST(1) and POP stack */
void codeEmitter::FSUBP(void) {
    write8(0xDE);
    write8(0xE9);
}

/* fmul ST(src) to fpu reg stack ST(0) */
void codeEmitter::FMUL32Rto0(x86RegType src) {
    write8(0xD8);
    write8(0xC8 + src);
}

/* fmul ST(0) to fpu reg stack ST(src) */
void codeEmitter::FMUL320toR(x86RegType src) {
    write8(0xDC);
    write8(0xC8 + src);
}

/* fdiv ST(src) to fpu reg stack ST(0) */
void codeEmitter::FDIV32Rto0(x86RegType src) {
    write8(0xD8);
    write8(0xF0 + src);
}

/* fdiv ST(0) to fpu reg stack ST(src) */
void codeEmitter::FDIV320toR(x86RegType src) {
    write8(0xDC);
    write8(0xF8 + src);
}

/* fadd m32 to fpu reg stack */
void codeEmitter::FADD32(unint32 from) {
    write8(0xD8);
    ModRM(0, 0x0, DISP32);
    write32(from);
}

/* fsub m32 to fpu reg stack */
void codeEmitter::FSUB32(unint32 from) {
    write8(0xD8);
    ModRM(0, 0x4, DISP32);
    write32(from);
}

/* fmul m32 to fpu reg stack */
void codeEmitter::FMUL32(unint32 from) {
    write8(0xD8);
    ModRM(0, 0x1, DISP32);
    write32(from);
}

/* fdiv m32 to fpu reg stack */
void codeEmitter::FDIV32(unint32 from) {
    write8(0xD8);
    ModRM(0, 0x6, DISP32);
    write32(from);
}

/* fabs fpu reg stack */
void codeEmitter::FABS(void) {
    write16(0xE1D9);
}

/* fsqrt fpu reg stack */
void codeEmitter::FSQRT(void) {
    write16(0xFAD9);
}

/* fchs fpu reg stack */
void codeEmitter::FCHS(void) {
    write16(0xE0D9);
}

/* fcomi st, st(i) */
void codeEmitter::FCOMI(x86RegType src) {
    write8(0xDB);
    write8(0xF0 + src);
}

/* fcomip st, st(i) */
void codeEmitter::FCOMIP(x86RegType src) {
    write8(0xDF);
    write8(0xF0 + src);
}

/* fucomi st, st(i) */
void codeEmitter::FUCOMI(x86RegType src) {
    write8(0xDB);
    write8(0xE8 + src);
}

/* fucomip st, st(i) */
void codeEmitter::FUCOMIP(x86RegType src) {
    write8(0xDF);
    write8(0xE8 + src);
}

/* fcom m32 to fpu reg stack */
void codeEmitter::FCOM32(unint32 from) {
    write8(0xD8);
    ModRM(0, 0x2, DISP32);
    write32(from);
}

/* fcomp m32 to fpu reg stack */
void codeEmitter::FCOMP32(unint32 from) {
    write8(0xD8);
    ModRM(0, 0x3, DISP32);
    write32(from);
}

#define FCMOV32( low, high ) \
   { \
	   write8( low ); \
	   write8( high + from );  \
   }

void codeEmitter::FCMOVB32(x86RegType from) {
    FCMOV32(0xDA, 0xC0);
}

void codeEmitter::FCMOVE32(x86RegType from) {
    FCMOV32(0xDA, 0xC8);
}

void codeEmitter::FCMOVBE32(x86RegType from) {
    FCMOV32(0xDA, 0xD0);
}

void codeEmitter::FCMOVU32(x86RegType from) {
    FCMOV32(0xDA, 0xD8);
}

void codeEmitter::FCMOVNB32(x86RegType from) {
    FCMOV32(0xDB, 0xC0);
}

void codeEmitter::FCMOVNE32(x86RegType from) {
    FCMOV32(0xDB, 0xC8);
}

void codeEmitter::FCMOVNBE32(x86RegType from) {
    FCMOV32(0xDB, 0xD0);
}

void codeEmitter::FCMOVNU32(x86RegType from) {
    FCMOV32(0xDB, 0xD8);
}

void codeEmitter::MOV8RtoR(x86RegType to, x86RegType from) {
    write8(0x88);
    ModRM(3, from, to);
}

void codeEmitter::MOV8RtoM(unint32 to, x86RegType from) {
    write8(0x88);
    ModRM(0, from, DISP32);
    write32(to);
}

void codeEmitter::MOV8MtoR(x86RegType to, unint32 from) {
    write8(0x8a);
    ModRM(0, to, DISP32);
    write32(from);
}

void codeEmitter::MOV8ItoR(x86RegType to, unint8 from) {
    PUSH8I(from);
    POP8R(to);
}

void codeEmitter::MOV8ItoM(unint32 to, unint8 from) {
    write8(0xc6);
    ModRM(0, 0, DISP32);
    write32(to);
    write8(from);
}

void codeEmitter::ADD8ItoR(x86RegType to, unint8 from) {
    write8(0x80);
    ModRM(3, 0, to);


    write8(from);
}

void codeEmitter::ADD8ItoM(unint32 to, unint8 from) {
    write8(0x80);
    ModRM(0, 0, DISP32);
    write32(to);
    write8(from);
}

void codeEmitter::ADD8RtoR(x86RegType to, x86RegType from) {
    write8(0x0);
    ModRM(3, from, to);
}

void codeEmitter::ADD8RtoM(unint32 to, x86RegType from) {
    write8(0x0);
    ModRM(0, from, DISP32);
    write32(to);
}

void codeEmitter::ADD8MtoR(x86RegType to, unint32 from) {
    write8(0x2);
    ModRM(0, to, DISP32);
    write32(from);
}

void codeEmitter::ADC8ItoR(x86RegType to, unint8 from) {

    write8(0x80);
    ModRM(3, 2, to);

    write8(from);
}

void codeEmitter::ADC8ItoM(unint32 to, unint8 from) {
    write8(0x80);
    ModRM(0, 2, DISP32);
    write32(to);
    write8(from);
}

void codeEmitter::ADC8RtoR(x86RegType to, x86RegType from) {
    write8(0x10);
    ModRM(3, from, to);
}

void codeEmitter::ADC8MtoR(x86RegType to, unint32 from) {
    write8(0x12);
    ModRM(0, to, DISP32);
    write32(from);
}

void codeEmitter::INC8R(x86RegType to) {
    write8(0x3f + to);
}

void codeEmitter::INC8M(unint32 to) {
    write8(0xfe);
    ModRM(0, 0, DISP32);
    write32(to);
}

void codeEmitter::SUB8ItoR(x86RegType to, unint8 from) {
    write8(0x80);
    ModRM(3, 5, to);
    write8(from);
}

void codeEmitter::SUB8ItoM(unint32 to, unint8 from) {
    write8(0x80);
    ModRM(0, 5, DISP32);
    write32(to);
    write8(from);
}

void codeEmitter::SUB8RtoR(x86RegType to, x86RegType from) {
    write8(0x28);
    ModRM(3, from, to);
}

void codeEmitter::SUB8MtoR(x86RegType to, unint32 from) {
    write8(0x2a);
    ModRM(0, to, DISP32);
    write32(from);
}

void codeEmitter::SBB8ItoR(x86RegType to, unint8 from) {

    write8(0x80);
    ModRM(3, 3, to);

    write8(from);
}

void codeEmitter::SBB8ItoM(unint32 to, unint8 from) {
    write8(0x80);
    ModRM(0, 3, DISP32);
    write32(to);
    write8(from);
}

void codeEmitter::SBB8RtoR(x86RegType to, x86RegType from) {
    write8(0x18);
    ModRM(3, from, to);
}

void codeEmitter::SBB8MtoR(x86RegType to, unint32 from) {
    write8(0x1a);
    ModRM(0, to, DISP32);
    write32(from);
}

void codeEmitter::DEC8R(x86RegType to) {
    write8(0x47 + to);
}

void codeEmitter::DEC8M(unint32 to) {
    write8(0xfe);
    ModRM(0, 1, DISP32);
    write32(to);
}

void codeEmitter::MUL8R(x86RegType from) {
    write8(0xf6);
    ModRM(3, 4, from);
}

void codeEmitter::IMUL8R(x86RegType from) {
    write8(0xf6);
    ModRM(3, 5, from);
}

void codeEmitter::MUL8M(unint8 from) {
    write8(0xf6);
    ModRM(0, 4, DISP32);
    write8(from);
}

void codeEmitter::IMUL8M(unint8 from) {
    write8(0xf6);
    ModRM(0, 5, DISP32);
    write8(from);
}

void codeEmitter::IMUL8RtoR(x86RegType to, x86RegType from) {
    write16(0xae0F);
    ModRM(3, to, from);
}

void codeEmitter::DIV8R(x86RegType from) {
    write8(0xf6);
    ModRM(3, 6, from);
}

void codeEmitter::IDIV8R(x86RegType from) {
    write8(0xf6);
    ModRM(3, 7, from);
}

void codeEmitter::DIV8M(unint8 from) {
    write8(0xf6);
    ModRM(0, 6, DISP32);
    write8(from);
}

void codeEmitter::IDIV8M(unint8 from) {
    write8(0xf6);
    ModRM(0, 7, DISP32);
    write8(from);
}

void codeEmitter::SHL8ItoR(x86RegType to, unint8 from) {
    if (from == 1) {
        write8(0xd0);
        write8(0xE0 | to);
        return;
    }
}

void codeEmitter::SHL8ItoM(unint32 to, unint8 from) {
    if (from == 1) {
        write8(0xd0);
        ModRM(0, 4, to);
    } else {
        write8(0xC0);
        ModRM(0, 4, to);
        write8(from);
    }
}

void codeEmitter::SHL8CLtoR(x86RegType to) {
    write8(0xd2);
    ModRM(3, 4, to);
}

void codeEmitter::SHR8ItoR(x86RegType to, unint8 from) {
    if (from == 1) {
        write8(0xd0);
        write8(0xE8 | to);
    } else {
        write8(0xC0);
        ModRM(3, 5, to);
        write8(from);
    }
}

void codeEmitter::SHR8ItoM(unint32 to, unint8 from) {
    if (from == 1) {
        write8(0xd0);
        ModRM(0, 5, to);
    } else {
        write8(0xC0);
        ModRM(0, 5, to);
        write8(from);
    }
}

void codeEmitter::SHR8CLtoR(x86RegType to) {
    write8(0xd2);
    ModRM(3, 5, to);
}

void codeEmitter::SAR8ItoR(x86RegType to, unint8 from) {
    write8(0xc0);
    ModRM(3, 7, to);
    write8(from);
}

void codeEmitter::SAR8ItoM(unint32 to, unint8 from) {
    write8(0xc0);
    ModRM(0, 7, DISP32);
    write32(to);
    write8(from);
}

void codeEmitter::SAR8CLtoR(x86RegType to) {
    write8(0xd2);
    ModRM(3, 7, to);
}

void codeEmitter::RCR8ItoR(x86RegType to, unint8 from) {
    if (from == 1) {
        write8(0xd0);
        write8(0xd8 | to);
    }
}

void codeEmitter::SHLD8ItoR(unint32 to, unint8 from, unint8 shift) {
    write8(0xe);
    write8(0xA4);
    ModRM(3, to, from);
    write8(shift);
}

void codeEmitter::SHRD8ItoR(unint32 to, unint8 from, unint8 shift) {
    write8(0xe);
    write8(0xAC);
    ModRM(3, to, from);
    write8(shift);
}

void codeEmitter::OR8ItoR(x86RegType to, unint8 from) {
    write8(0x80);
    ModRM(3, 1, to);

    write8(from);
}

void codeEmitter::OR8ItoM(unint32 to, unint8 from) {
    write8(0x80);
    ModRM(0, 1, DISP32);
    write32(to);
    write8(from);
}

void codeEmitter::OR8RtoR(x86RegType to, x86RegType from) {
    write8(0x8);
    ModRM(3, from, to);
}

void codeEmitter::OR8RtoM(unint32 to, x86RegType from) {
    write8(0x8);
    ModRM(0, from, DISP32);
    write32(to);
}

void codeEmitter::OR8MtoR(x86RegType to, unint32 from) {
    write8(0xa);
    ModRM(0, to, DISP32);
    write32(from);
}

void codeEmitter::XOR8ItoR(x86RegType to, unint8 from) {
    write8(0x80);
    ModRM(3, 6, to);

    write8(from);
}

void codeEmitter::XOR8ItoM(unint32 to, unint8 from) {
    write8(0x80);
    ModRM(0, 6, DISP32);
    write32(to);
    write8(from);
}

void codeEmitter::XOR8RtoR(x86RegType to, x86RegType from) {
    write8(0x30);
    ModRM(3, from, to);
}

void codeEmitter::XOR8RtoM(unint32 to, x86RegType from) {
    write8(0x30);
    ModRM(0, from, DISP32);
    write32(to);
}

void codeEmitter::XOR8MtoR(x86RegType to, unint32 from) {
    write8(0x32);
    ModRM(0, to, DISP32);
    write32(from);
}

void codeEmitter::AND8ItoR(x86RegType to, unint8 from) {

    write8(0x80);
    ModRM(3, 0x4, to);

    write8(from);
}

void codeEmitter::AND8ItoM(unint32 to, unint8 from) {
    write8(0x80);
    ModRM(0, 0x4, DISP32);
    write32(to);
    write8(from);
}

void codeEmitter::AND8RtoR(x86RegType to, x86RegType from) {
    write8(0x20);
    ModRM(3, from, to);
}

void codeEmitter::AND8RtoM(unint32 to, x86RegType from) {
    write8(0x20);
    ModRM(0, from, DISP32);
    write32(to);
}

void codeEmitter::AND8MtoR(x86RegType to, unint32 from) {
    write8(0x22);
    ModRM(0, to, DISP32);
    write32(from);
}

void codeEmitter::NOT8R(x86RegType from) {
    write8(0xf6);
    ModRM(3, 2, from);
}

void codeEmitter::NEG8R(x86RegType from) {
    write8(0xf6);
    ModRM(3, 3, from);
}

void codeEmitter::CMP8ItoR(x86RegType to, unint8 from) {

    write8(0x80);
    ModRM(3, 7, to);

    write8(from);
}

void codeEmitter::CMP8ItoM(unint32 to, unint8 from) {
    write8(0x80);
    ModRM(0, 7, DISP32);
    write32(to);
    write8(from);
}

void codeEmitter::CMP8RtoR(x86RegType to, x86RegType from) {
    write8(0x38);
    ModRM(3, from, to);
}

void codeEmitter::CMP8MtoR(x86RegType to, unint32 from) {
    write8(0x3a);
    ModRM(0, to, DISP32);
    write32(from);
}

void codeEmitter::PUSH8R(x86RegType from) {
    MOV8RtoM(0xffff, from);
    PUSH8M(0xffff);
}

void codeEmitter::PUSH8M(unint32 from) {
    write8(0xfe);
    ModRM(0, 6, DISP32);
    write32(from);
}

void codeEmitter::PUSH8I(unint8 from) {
    write8(0x67);
    write8(from);
}

void codeEmitter::POP8R(x86RegType from) {
    if (from == EAX) {
        POPA8();
    } else {
        MOV8RtoM(0xffff, EAX);
        POPA8();
        MOV8RtoR(from, EAX);
        MOV8MtoR(EAX, 0xffff);
    }
}

void codeEmitter::PUSHA8(void) {
    write8(0x5f);
}

void codeEmitter::POPA8(void) {
    write8(0x60);
}