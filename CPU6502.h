/* 
 * File:   CPU6502.h
 * Author: 凯
 *
 * Created on 2015年1月17日, 上午7:09
 */

#ifndef CPU6502_H
#define	CPU6502_H
#include "codeEmitter.h"
#define A EDX




class CPU6502 {
public:
    CPU6502();
    CPU6502(const CPU6502& orig);
    virtual ~CPU6502();
    void tranlateBlock();
    int32* getBlock();
    static void resumeRegisters();
    static void getZeroPage();
    static void getZeroPageX();
    static void getAbsolute();
    static void getAbsoluteX();
    static void getAbsoluteY();
    static void getIndirectX();
    static void getIndirectY();
    static void getImmediate() ;
    static void saveRegisters();

private:
    static int8 AB;
    static int8 I;
    static int8 D;
    static int32 PB;
    static int8 X;
    static int8 Y;
    static int8 *S;
    static int16 PC;
    static unint32 temp;
    static unint8* Memory;
    static int32** tanslatedBlock;
    static int32 Cycles;
    static codeEmitter Emitter;
};

#endif	/* CPU6502_H */

