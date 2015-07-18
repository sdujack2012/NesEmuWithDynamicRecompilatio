/* 
 * File:   codeEmitter.h
 * Author: 凯
 *
 * Created on 2015年1月3日, 下午12:59
 */

#ifndef CODEEMITTER_H
#define	CODEEMITTER_H
#define SIB 4
#define DISP32 5

#define MaxBlockSize 2000

// general types

typedef enum {
    EAX = 0,
    EBX = 3,
    ECX = 1,
    EDX = 2,
    ESI = 6,
    EDI = 7,
    EBP = 5,
    ESP = 4
} x86RegType;

typedef char int8; // 8 bit signed integer
typedef short int16; // 16 bit signed integer
typedef long int32; // 32 bit signed integer
typedef long long int64; // 64 bit signed integer

typedef unsigned char unint8; // 8 bit unsigned integer
typedef unsigned short unint16; // 16 bit unsigned integer
typedef unsigned long unint32; // 32 bit unsigned integer
typedef unsigned long long unint64; // 64 bit unsigned integer

typedef void (*X86Fun)();

class codeEmitter {
public:
    codeEmitter();
    codeEmitter(const codeEmitter& orig);
    virtual ~codeEmitter();

    void BeginEmit();
    int8* EndEmit();
    void SibSB(unint8 ss, unint8 rm, unint8 index);
    void write8(unint8 Val8);
    void write16(unint16 Val16);
    void write32(unint32 Val32);
    void write64(unint64 val64);
    void ModRM(unint8 mod, unint8 rm, unint8 reg);
    void Ret();
    void Mov16RtoM(unint32 to, x86RegType from);
    void STC(void);
    void CLC(void);
    void SET8R(unint8 cc, unint8 to);
    void J8Rel(unint8 cc, unint8 to);
    void J32Rel(unint8 cc, unint32 to);
    void CMOV32RtoR(unint8 cc, unint8 to, unint8 from);
    void CMOV32MtoR(unint8 cc, unint8 to, unint32 from);
    void x86SetPtr(char* ptr);
    void x86Shutdown(void);
    void x86SetJ8(unint8* j8);
    void x86SetJ32(unint32* j32);
    void x86Align(int bytes);
    ////////////////////////////////////
    // mov instructions                /
    ////////////////////////////////////
    /* mov r32 to r32 */
    void MOV32RtoR(x86RegType to, x86RegType from);
    /* mov r32 to m32 */
    void MOV32RtoM(unint32 to, x86RegType from);
    /* mov m32 to r32 */
    void MOV32MtoR(x86RegType to, unint32 from);
    /* mov [r32] to r32 */
    void MOV32RmtoR(x86RegType to, x86RegType from);
    /* mov [r32][r32*scale] to r32 */
    void MOV32RmStoR(x86RegType to, x86RegType from, x86RegType from2, int scale);
    /* mov r32 to [r32] */
    void MOV32RtoRm(x86RegType to, x86RegType from);
    /* mov r32 to [r32][r32*scale] */
    void MOV32RtoRmS(x86RegType to, x86RegType from, x86RegType from2, int scale);
    /* mov imm32 to r32 */
    void MOV32ItoR(x86RegType to, unint32 from);
    /* mov imm32 to m32 */
    void MOV32ItoM(unint32 to, unint32 from);
    /* mov r16 to m16 */
    void MOV16RtoM(unint32 to, x86RegType from);
    /* mov m16 to r16 */
    void MOV16MtoR(x86RegType to, unint32 from);
    /* mov imm16 to m16 */
    void MOV16ItoM(unint32 to, unint16 from);

    void Mov16MtoR(x86RegType to, unint32 from);

    ///////////////////////////////////////////////////////////////////////////////////////////////////

    void Add32ItoR(x86RegType to, unint32 from);
    /* movsx r8 to r32 */
    void MOVSX32R8toR(x86RegType to, x86RegType from);
    /* movsx m8 to r32 */
    void MOVSX32M8toR(x86RegType to, unint32 from);
    /* movsx r16 to r32 */
    void MOVSX32R16toR(x86RegType to, x86RegType from);
    /* movsx m16 to r32 */
    void MOVSX32M16toR(x86RegType to, unint32 from);
    /* movzx r8 to r32 */
    void MOVZX32R8toR(x86RegType to, x86RegType from);
    /* movzx m8 to r32 */
    void MOVZX32M8toR(x86RegType to, unint32 from);
    /* movzx r16 to r32 */
    void MOVZX32R16toR(x86RegType to, x86RegType from);
    /* movzx m16 to r32 */
    void MOVZX32M16toR(x86RegType to, unint32 from);
    /* cmovbe r32 to r32 */
    void CMOVBE32RtoR(x86RegType to, x86RegType from);
    /* cmovbe m32 to r32*/
    void CMOVBE32MtoR(x86RegType to, unint32 from);
    /* cmovb r32 to r32 */
    void CMOVB32RtoR(x86RegType to, x86RegType from);
    /* cmovb m32 to r32*/
    void CMOVB32MtoR(x86RegType to, unint32 from);
    /* cmovae r32 to r32 */
    void CMOVAE32RtoR(x86RegType to, x86RegType from);
    /* cmovae m32 to r32*/
    void CMOVAE32MtoR(x86RegType to, unint32 from);
    /* cmova r32 to r32 */
    void CMOVA32RtoR(x86RegType to, x86RegType from);
    /* cmova m32 to r32*/
    void CMOVA32MtoR(x86RegType to, unint32 from);
    /* cmovo r32 to r32 */
    void CMOVO32RtoR(x86RegType to, x86RegType from);
    /* cmovo m32 to r32 */
    void CMOVO32MtoR(x86RegType to, unint32 from);
    /* cmovp r32 to r32 */
    void CMOVP32RtoR(x86RegType to, x86RegType from);
    /* cmovp m32 to r32 */
    void CMOVP32MtoR(x86RegType to, unint32 from);
    /* cmovs r32 to r32 */
    void CMOVS32RtoR(x86RegType to, x86RegType from);
    /* cmovs m32 to r32 */
    void CMOVS32MtoR(x86RegType to, unint32 from);
    /* cmovno r32 to r32 */
    void CMOVNO32RtoR(x86RegType to, x86RegType from);
    /* cmovno m32 to r32 */
    void CMOVNO32MtoR(x86RegType to, unint32 from);
    /* cmovnp r32 to r32 */
    void CMOVNP32RtoR(x86RegType to, x86RegType from);
    /* cmovnp m32 to r32 */
    void CMOVNP32MtoR(x86RegType to, unint32 from);
    /* cmovns r32 to r32 */
    void CMOVNS32RtoR(x86RegType to, x86RegType from);
    /* cmovns m32 to r32 */
    void CMOVNS32MtoR(x86RegType to, unint32 from);
    /* cmovne r32 to r32 */
    void CMOVNE32RtoR(x86RegType to, x86RegType from);
    /* cmovne m32 to r32*/
    void CMOVNE32MtoR(x86RegType to, unint32 from);
    /* cmove r32 to r32*/
    void CMOVE32RtoR(x86RegType to, x86RegType from);
    /* cmove m32 to r32*/
    void CMOVE32MtoR(x86RegType to, unint32 from);
    /* cmovg r32 to r32*/
    void CMOVG32RtoR(x86RegType to, x86RegType from);
    /* cmovg m32 to r32*/
    void CMOVG32MtoR(x86RegType to, unint32 from);
    /* cmovge r32 to r32*/
    void CMOVGE32RtoR(x86RegType to, x86RegType from);
    /* cmovge m32 to r32*/
    void CMOVGE32MtoR(x86RegType to, unint32 from);
    /* cmovl r32 to r32*/
    void CMOVL32RtoR(x86RegType to, x86RegType from);
    /* cmovl m32 to r32*/
    void CMOVL32MtoR(x86RegType to, unint32 from);
    /* cmovle r32 to r32*/
    void CMOVLE32RtoR(x86RegType to, x86RegType from);
    /* cmovle m32 to r32*/
    void CMOVLE32MtoR(x86RegType to, unint32 from);
    ////////////////////////////////////
    // arithmetic instructions         /
    ////////////////////////////////////
    /* add imm32 to r32 */
    void ADD32ItoR(x86RegType to, unint32 from);
    /* add imm32 to m32 */
    void ADD32ItoM(unint32 to, unint32 from);
    /* add r32 to r32 */
    void ADD32RtoR(x86RegType to, x86RegType from);
    /* add r32 to m32 */
    void ADD32RtoM(unint32 to, x86RegType from);
    /* add m32 to r32 */
    void ADD32MtoR(x86RegType to, unint32 from);
    /* add imm16 to r16 */
    void ADD16ItoR(x86RegType to, unint16 from);
    /* add r16 to m16 */
    void ADD16RtoM(unint32 to, x86RegType from);
    /* add m16 to r16 */
    void ADD16MtoR(x86RegType to, unint32 from);
    /* adc imm32 to r32 */
    void ADC32ItoR(x86RegType to, unint32 from);
    /* adc imm32 to m32 */
    void ADC32ItoM(unint32 to, unint32 from);
    /* adc r32 to r32 */
    void ADC32RtoR(x86RegType to, x86RegType from);
    /* adc m32 to r32 */
    void ADC32MtoR(x86RegType to, unint32 from);
    /* inc r32 */
    void INC32R(x86RegType to);
    /* inc m32 */
    void INC32M(unint32 to);
    /* inc r16 */
    void INC16R(x86RegType to);
    /* inc m16 */
    void INC16M(unint32 to);
    /* sub imm32 to r32 */
    void SUB32ItoR(x86RegType to, unint32 from);
    /* sub imm32 to m32 */
    void SUB32ItoM(unint32 to, unint32 from);
    /* sub r32 to r32 */
    void SUB32RtoR(x86RegType to, x86RegType from);
    /* sub m32 to r32 */
    void SUB32MtoR(x86RegType to, unint32 from);
    /* sub m16 to r16 */
    void SUB16MtoR(x86RegType to, unint32 from);
    /* sbb imm32 to r32 */
    void SBB32ItoR(x86RegType to, unint32 from);
    /* sbb imm32 to m32 */
    void SBB32ItoM(unint32 to, unint32 from);
    /* sbb r32 to r32 */
    void SBB32RtoR(x86RegType to, x86RegType from);
    /* sbb m32 to r32 */
    void SBB32MtoR(x86RegType to, unint32 from);
    /* dec r32 */
    void DEC32R(x86RegType to);
    /* dec m32 */
    void DEC32M(unint32 to);
    /* dec r16 */
    void DEC16R(x86RegType to);
    /* dec m16 */
    void DEC16M(unint32 to);
    /* mul eax by r32 to edx:eax */
    void MUL32R(x86RegType from);
    /* imul eax by r32 to edx:eax */
    void IMUL32R(x86RegType from);
    /* mul eax by m32 to edx:eax */
    void MUL32M(unint32 from);
    /* imul eax by m32 to edx:eax */
    void IMUL32M(unint32 from);
    /* imul r32 by r32 to r32 */
    void IMUL32RtoR(x86RegType to, x86RegType from);
    /* div eax by r32 to edx:eax */
    void DIV32R(x86RegType from);
    /* idiv eax by r32 to edx:eax */
    void IDIV32R(x86RegType from);
    /* div eax by m32 to edx:eax */
    void DIV32M(unint32 from);
    /* idiv eax by m32 to edx:eax */
    void IDIV32M(unint32 from);
    ////////////////////////////////////
    // shifting instructions           /
    ////////////////////////////////////
    /* shl imm8 to r32 */
    void SHL32ItoR(x86RegType to, unint8 from);
    /* shl imm8 to m32 */
    void SHL32ItoM(unint32 to, unint8 from);
    /* shl cl to r32 */
    void SHL32CLtoR(x86RegType to);
    /* shr imm8 to r32 */
    void SHR32ItoR(x86RegType to, unint8 from);
    /* shr imm8 to m32 */
    void SHR32ItoM(unint32 to, unint8 from);
    /* shr cl to r32 */
    void SHR32CLtoR(x86RegType to);
    /* sar imm8 to r32 */
    void SAR32ItoR(x86RegType to, unint8 from);
    /* sar imm8 to m32 */
    void SAR32ItoM(unint32 to, unint8 from);
    /* sar cl to r32 */
    void SAR32CLtoR(x86RegType to);
    void RCR32ItoR(x86RegType to, unint8 from);
    // shld imm8 to r32
    void SHLD32ItoR(unint32 to, unint32 from, unint8 shift);
    // shrd imm8 to r32
    void SHRD32ItoR(unint32 to, unint32 from, unint8 shift);
    ////////////////////////////////////
    // logical instructions            /
    ////////////////////////////////////
    /* or imm32 to r32 */
    void OR32ItoR(x86RegType to, unint32 from);
    /* or imm32 to m32 */
    void OR32ItoM(unint32 to, unint32 from);
    /* or r32 to r32 */
    void OR32RtoR(x86RegType to, x86RegType from);
    /* or r32 to m32 */
    void OR32RtoM(unint32 to, x86RegType from);
    /* or m32 to r32 */
    void OR32MtoR(x86RegType to, unint32 from);
    /* or m16 to r16 */
    void OR16MtoR(x86RegType to, unint32 from);
    /* xor imm32 to r32 */
    void XOR32ItoR(x86RegType to, unint32 from);
    /* xor imm32 to m32 */
    void XOR32ItoM(unint32 to, unint32 from);
    /* xor r32 to r32 */
    void XOR32RtoR(x86RegType to, x86RegType from);
    /* xor r16 to r16 */
    void XOR16RtoR(x86RegType to, x86RegType from);
    /* xor r32 to m32 */
    void XOR32RtoM(unint32 to, x86RegType from);
    /* xor m32 to r32 */
    void XOR32MtoR(x86RegType to, unint32 from);
    /* and imm32 to r32 */
    void AND32ItoR(x86RegType to, unint32 from);
    /* and imm32 to m32 */
    void AND32ItoM(unint32 to, unint32 from);
    /* and r32 to r32 */
    void AND32RtoR(x86RegType to, x86RegType from);
    /* and r32 to m32 */
    void AND32RtoM(unint32 to, x86RegType from);
    /* and m32 to r32 */
    void AND32MtoR(x86RegType to, unint32 from);
    /* and m16 to r16 */
    void AND16MtoR(x86RegType to, unint32 from);
    /* not r32 */
    void NOT32R(x86RegType from);
    /* neg r32 */
    void NEG32R(x86RegType from);
    /* neg r16 */
    void NEG16R(x86RegType from);
    ////////////////////////////////////
    // jump instructions               /
    ////////////////////////////////////
    void JMP(unint32 to);
    /* jmp rel8 */
    void JMP8(unint8 to);
    /* jmp rel32 */
    void JMP32(unint32 to);
    /* jmp r32 */
    void JMP32R(x86RegType to);
    /* jp rel8 */
    void JP8(unint8 to);
    /* jnp rel8 */
    void JNP8(unint8 to);
    /* je rel8 */
    void JE8(unint8 to);
    /* jz rel8 */
    void JZ8(unint8 to);
    /* js rel8 */
    void JS8(unint8 to);
    /* jns rel8 */
    void JNS8(unint8 to);
    /* jg rel8 */
    void JG8(unint8 to);
    /* jge rel8 */
    void JGE8(unint8 to);
    /* jl rel8 */
    void JL8(unint8 to);
    /* ja rel8 */
    void JA8(unint8 to);
    void JAE8(unint8 to);
    /* jb rel8 */
    void JB8(unint8 to);
    /* jbe rel8 */
    void JBE8(unint8 to);
    /* jle rel8 */
    void JLE8(unint8 to);
    /* jne rel8 */
    void JNE8(unint8 to);
    /* jnz rel8 */
    void JNZ8(unint8 to);
    /* jng rel8 */
    void JNG8(unint8 to);
    /* jnge rel8 */
    void JNGE8(unint8 to);
    /* jnl rel8 */
    void JNL8(unint8 to);
    /* jnle rel8 */
    void JNLE8(unint8 to);
    /* jo rel8 */
    void JO8(unint8 to);
    /* jno rel8 */
    void JNO8(unint8 to);
    /* je rel32 */
    void JE32(unint32 to);
    /* jz rel32 */
    void JZ32(unint32 to);
    /* jg rel32 */
    void JG32(unint32 to);
    /* jge rel32 */
    void JGE32(unint32 to);
    /* jl rel32 */
    void JL32(unint32 to);
    /* jle rel32 */
    void JLE32(unint32 to);
    /* jne rel32 */
    void JNE32(unint32 to);
    /* jnz rel32 */
    void JNZ32(unint32 to);
    /* jng rel32 */
    void JNG32(unint32 to);
    /* jnge rel32 */
    void JNGE32(unint32 to);
    /* jnl rel32 */
    void JNL32(unint32 to);
    /* jnle rel32 */
    void JNLE32(unint32 to);
    /* jo rel32 */
    void JO32(unint32 to);
    /* jno rel32 */
    void JNO32(unint32 to);
    /* call func */
    void CALLFunc(unint32 func);
    /* call rel32 */
    void CALL32(unint32 to);
    /* call r32 */
    void CALL32R(x86RegType to);
    /* call m32 */
    void CALL32M(unint32 to);
    ////////////////////////////////////
    // misc instructions               /
    ////////////////////////////////////
    /* cmp imm32 to r32 */
    void CMP32ItoR(x86RegType to, unint32 from);
    /* cmp imm32 to m32 */
    void CMP32ItoM(unint32 to, unint32 from);
    /* cmp r32 to r32 */
    void CMP32RtoR(x86RegType to, x86RegType from);
    /* cmp m32 to r32 */
    void CMP32MtoR(x86RegType to, unint32 from);
    /* test imm32 to r32 */
    void TEST32ItoR(x86RegType to, unint32 from);
    /* test r32 to r32 */
    void TEST32RtoR(x86RegType to, x86RegType from);
    /* sets r8 */
    void SETS8R(x86RegType to);
    /* setl r8 */
    void SETL8R(x86RegType to);
    /* setb r8 */
    void SETB8R(x86RegType to);
    /* setb r8 */
    void SETNZ8R(x86RegType to);
    /* cbw */
    void CBW(void);
    /* cwd */
    void CWD(void);
    /* cdq */
    void CDQ(void);
    /* push r32 */
    void PUSH32R(x86RegType from);
    /* push m32 */
    void PUSH32M(unint32 from);
    /* push imm32 */
    void PUSH32I(unint32 from);
    /* pop r32 */
    void POP32R(x86RegType from);
    /* pushad */
    void PUSHA32(void);
    /* popad */
    void POPA32(void);
    /* pushfd */
    void PUSHFD(void);
    /* popfd */
    void POPFD(void);
    /* ret */
    void RET(void);
    void BT32ItoR(x86RegType to, x86RegType from);
    /********************/
    /* FPU instructions */
    /********************/
    /* fild m32 to fpu reg stack */
    void FILD32(unint32 from);
    /* fistp m32 from fpu reg stack */
    void FISTP32(unint32 from);
    /* fld m32 to fpu reg stack */
    void FLD32(unint32 from);
    /* fst m32 from fpu reg stack */
    void FST32(unint32 to);
    /* fstp m32 from fpu reg stack */
    void FSTP32(unint32 to);
    /* fldcw fpu control word from m16 */
    void FLDCW(unint32 from);
    /* fnstcw fpu control word to m16 */
    void FNSTCW(unint32 to);
    void FNSTSWtoAX(void);
    /* fadd ST(src) to fpu reg stack ST(0) */
    void FADD32Rto0(x86RegType src);
    /* fadd ST(0) to fpu reg stack ST(src) */
    void FADD320toR(x86RegType src);
    /* fsub ST(src) to fpu reg stack ST(0) */
    void FSUB32Rto0(x86RegType src);
    /* fsub ST(0) to fpu reg stack ST(src) */
    void FSUB320toR(x86RegType src);
    /* fsubp -> substract ST(0) from ST(1), store in ST(1) and POP stack */
    void FSUBP(void);
    /* fmul ST(src) to fpu reg stack ST(0) */
    void FMUL32Rto0(x86RegType src);
    /* fmul ST(0) to fpu reg stack ST(src) */
    void FMUL320toR(x86RegType src);
    /* fdiv ST(src) to fpu reg stack ST(0) */
    void FDIV32Rto0(x86RegType src);
    /* fdiv ST(0) to fpu reg stack ST(src) */
    void FDIV320toR(x86RegType src);
    /* fadd m32 to fpu reg stack */
    void FADD32(unint32 from);
    /* fsub m32 to fpu reg stack */
    void FSUB32(unint32 from);
    /* fmul m32 to fpu reg stack */
    void FMUL32(unint32 from);
    /* fdiv m32 to fpu reg stack */
    void FDIV32(unint32 from);
    /* fabs fpu reg stack */
    void FABS(void);
    /* fsqrt fpu reg stack */
    void FSQRT(void);
    /* fchs fpu reg stack */
    void FCHS(void);
    /* fcomi st, st(i) */
    void FCOMI(x86RegType src);
    /* fcomip st, st(i) */
    void FCOMIP(x86RegType src);
    /* fucomi st, st(i) */
    void FUCOMI(x86RegType src);
    /* fucomip st, st(i) */
    void FUCOMIP(x86RegType src);
    /* fcom m32 to fpu reg stack */
    void FCOM32(unint32 from);
    /* fcomp m32 to fpu reg stack */
    void FCOMP32(unint32 from);

    void FCMOVB32(x86RegType from);
    void FCMOVE32(x86RegType from);
    void FCMOVBE32(x86RegType from);
    void FCMOVU32(x86RegType from);
    void FCMOVNB32(x86RegType from);
    void FCMOVNE32(x86RegType from);
    void FCMOVNBE32(x86RegType from);
    void FCMOVNU32(x86RegType from);
    void MOV8RtoR(x86RegType to, x86RegType from);
    void MOV8RtoM(unint32 to, x86RegType from);
    void MOV8MtoR(x86RegType to, unint32 from);
    void MOV8ItoR(x86RegType to, unint8 from);
    void MOV8ItoM(unint32 to, unint8 from);
    void ADD8ItoR(x86RegType to, unint8 from);
    void ADD8ItoM(unint32 to, unint8 from);
    void ADD8RtoR(x86RegType to, x86RegType from);
    void ADD8RtoM(unint32 to, x86RegType from);
    void ADD8MtoR(x86RegType to, unint32 from);
    void ADC8ItoR(x86RegType to, unint8 from);
    void ADC8ItoM(unint32 to, unint8 from);
    void ADC8RtoR(x86RegType to, x86RegType from);
    void ADC8MtoR(x86RegType to, unint32 from);
    void INC8R(x86RegType to);
    void INC8M(unint32 to);
    void SUB8ItoR(x86RegType to, unint8 from);
    void SUB8ItoM(unint32 to, unint8 from);
    void SUB8RtoR(x86RegType to, x86RegType from);
    void SUB8MtoR(x86RegType to, unint32 from);
    void SBB8ItoR(x86RegType to, unint8 from);
    void SBB8ItoM(unint32 to, unint8 from);
    void SBB8RtoR(x86RegType to, x86RegType from);
    void SBB8MtoR(x86RegType to, unint32 from);
    void DEC8R(x86RegType to);
    void DEC8M(unint32 to);
    void MUL8R(x86RegType from);
    void IMUL8R(x86RegType from);
    void MUL8M(unint8 from);
    void IMUL8M(unint8 from);
    void IMUL8RtoR(x86RegType to, x86RegType from);
    void DIV8R(x86RegType from);
    void IDIV8R(x86RegType from);
    void DIV8M(unint8 from);
    void IDIV8M(unint8 from);
    void SHL8ItoR(x86RegType to, unint8 from);
    void SHL8ItoM(unint32 to, unint8 from);
    void SHL8CLtoR(x86RegType to);
    void SHR8ItoR(x86RegType to, unint8 from);
    void SHR8ItoM(unint32 to, unint8 from);
    void SHR8CLtoR(x86RegType to);
    void SAR8ItoR(x86RegType to, unint8 from);
    void SAR8ItoM(unint32 to, unint8 from);
    void SAR8CLtoR(x86RegType to);
    void RCR8ItoR(x86RegType to, unint8 from);
    void SHLD8ItoR(unint32 to, unint8 from, unint8 shift);
    void SHRD8ItoR(unint32 to, unint8 from, unint8 shift);
    void OR8ItoR(x86RegType to, unint8 from);
    void OR8ItoM(unint32 to, unint8 from);
    void OR8RtoR(x86RegType to, x86RegType from);
    void OR8RtoM(unint32 to, x86RegType from);
    void OR8MtoR(x86RegType to, unint32 from);
    void XOR8ItoR(x86RegType to, unint8 from);
    void XOR8ItoM(unint32 to, unint8 from);
    void XOR8RtoR(x86RegType to, x86RegType from);
    void XOR8RtoM(unint32 to, x86RegType from);
    void XOR8MtoR(x86RegType to, unint32 from);
    void AND8ItoR(x86RegType to, unint8 from);
    void AND8ItoM(unint32 to, unint8 from);
    void AND8RtoR(x86RegType to, x86RegType from);
    void AND8RtoM(unint32 to, x86RegType from);
    void AND8MtoR(x86RegType to, unint32 from);
    void NOT8R(x86RegType from);
    void NEG8R(x86RegType from);
    void CMP8ItoR(x86RegType to, unint8 from);
    void CMP8ItoM(unint32 to, unint8 from);
    void CMP8RtoR(x86RegType to, x86RegType from);
    void CMP8MtoR(x86RegType to, unint32 from);
    void PUSH8R(x86RegType from);
    void PUSH8M(unint32 from);
    void PUSH8I(unint8 from);
    void POP8R(x86RegType from);
    void PUSHA8(void);
    void POPA8(void);
    void ExecuteBlock();
private:

    int8* currentBlock;
    int8* startPointer;

};

#endif	/* CODEEMITTER_H */

