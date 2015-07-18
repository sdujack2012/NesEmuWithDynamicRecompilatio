/* 
 * File:   CPU6502.cpp
 * Author: 凯
 * 
 * Created on 2015年1月17日, 上午7:09
 */

/*#include "CPU6502.h"
#include <iostream>

using namespace std;

CPU6502::CPU6502() {

}

CPU6502::CPU6502(const CPU6502& orig) {
}

CPU6502::~CPU6502() {
}

int32* CPU6502::getBlock() {
    if (tanslatedBlock[PC] == 0) {
        tranlateBlock();
    }
    return tanslatedBlock[PC];
}

void CPU6502::tranlateBlock() {
    bool finished = false;
    Emitter.BeginEmit();
    Emitter.PUSHA32();
    resumeRegisters();
    while (!finished) {
        unint8 op = Memory[PC++];
        switch (op) {
                //ADC
            case 0x69:
                Emitter.CALLFunc((unint32) & getImmediate);
                Emitter.ADC8ItoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 2);
                break;
            case 0x65:
                Emitter.CALLFunc((unint32) & getZeroPage);
                Emitter.ADC8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 3);
                break;
            case 0x75:
                Emitter.CALLFunc((unint32) & getZeroPageX);
                Emitter.ADC8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0x6D:
                Emitter.CALLFunc((unint32) & getAbsolute);
                Emitter.ADC8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0x7D:
                Emitter.CALLFunc((unint32) & getAbsoluteX);
                Emitter.ADC8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0x79:
                Emitter.CALLFunc((unint32) & getAbsoluteY);
                Emitter.ADC8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0x61:
                Emitter.CALLFunc((unint32) & getIndirectX);
                Emitter.ADC8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 6);
                break;
            case 0x71:
                Emitter.CALLFunc((unint32) & getIndirectY);
                Emitter.ADC8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 5);
                break;


                //AND    
            case 0x29:
                Emitter.CALLFunc((unint32) & getImmediate);
                Emitter.AND8ItoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 2);
                break;
            case 0x25:
                Emitter.CALLFunc((unint32) & getZeroPage);
                Emitter.AND8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 3);
                break;
            case 0x35:
                Emitter.CALLFunc((unint32) & getZeroPageX);
                Emitter.AND8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0x2D:
                Emitter.CALLFunc((unint32) & getAbsolute);
                Emitter.AND8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0x3D:
                Emitter.CALLFunc((unint32) & getAbsoluteX);
                Emitter.AND8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0x39:
                Emitter.CALLFunc((unint32) & getAbsoluteY);
                Emitter.AND8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0x21:
                Emitter.CALLFunc((unint32) & getIndirectX);
                Emitter.AND8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 6);
                break;
            case 0x31:
                Emitter.CALLFunc((unint32) & getIndirectY);
                Emitter.AND8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 5);
                break;



              


                //ASL   
            //case 0x0A:
                Emitter.SHL8ItoR(A, 1);
                Emitter.ADD32ItoM((unint32) & Cycles, 2);
                break;
            case 0x06:
                Emitter.CALLFunc((unint32) & getZeroPage);
                Emitter.SHL8ItoM(temp, 1);
                Emitter.ADD32ItoM((unint32) & Cycles, 5);
                break;
            case 0x16:
                Emitter.CALLFunc((unint32) & getZeroPageX);
                Emitter.SHL8ItoM(temp, 1);
                Emitter.ADD32ItoM((unint32) & Cycles, 6);
                break;
            case 0x0E:
                Emitter.CALLFunc((unint32) & getAbsolute);
                Emitter.SHL8ItoM(temp, 1);
                Emitter.ADD32ItoM((unint32) & Cycles, 6);
                break;
            case 0x1E:
                Emitter.CALLFunc((unint32) & getAbsoluteX);
                Emitter.SHL8ItoM(temp, 1);
                Emitter.ADD32ItoM((unint32) & Cycles, 7);
                break;


                //BCC   
            case 0x1A:
                Emitter.CALLFunc((unint32) & getImmediate);
                Emitter.JB8(4);
               // Emitter.ADD16ItoR((unint32) & PC, temp - 1);
                Emitter.RET();
                break;

                //RTI
            case 0x4D:
                Emitter.MOV32MtoR(EAX, (unint32) & S);
                Emitter.SUB32ItoM((unint32) & S, 4);
                Emitter.PUSH32R(EAX);
                Emitter.POPFD();
               // Emitter.MOV8MtoR(EAX, S);
                Emitter.MOV8RtoM((unint32) & PC, EAX);
                Emitter.DEC8M((unint32) & S);
                Emitter.ADD32ItoM((unint32) & Cycles, 6);
                break;
                //RTS
            case 0x60:
               // Emitter.MOV8MtoR(EAX, S);
                Emitter.MOV8RtoM((unint32) & PC, EAX);
                Emitter.INC8M((unint32) & PC);
                Emitter.DEC8M((unint32) & S);
                Emitter.ADD32ItoM((unint32) & Cycles, 6);
                break;
                //SBC
            case 0xE9:
                Emitter.SBB8ItoR(A, Memory[PC++]);
                Emitter.ADD32ItoM((unint32) & Cycles, 2);
                break;
            case 0xE5:
                Emitter.CALLFunc((unint32) & CPU6502::getZeroPage);
                Emitter.SBB8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 3);
                break;
            case 0xF5:
                Emitter.CALLFunc((unint32) & getZeroPageX);
                Emitter.SBB8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0xED:
                Emitter.CALLFunc((unint32) & getAbsolute);
                Emitter.SBB8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0xFD:
                Emitter.CALLFunc((unint32) & getAbsoluteX);
                Emitter.SBB8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0xF9:
                Emitter.CALLFunc((unint32) & getAbsoluteY);
                Emitter.SBB8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0xE1:
                Emitter.CALLFunc((unint32) & getIndirectX);
                Emitter.SBB8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 6);
                break;
            case 0xF1:
                Emitter.CALLFunc((unint32) & getIndirectY);
                Emitter.SBB8MtoR(A, temp);
                Emitter.ADD32ItoM((unint32) & Cycles, 5);
                break;

                //SEC  
            case 0xF8:
                Emitter.STC();
                Emitter.ADD32ItoM((unint32) & Cycles, 2);
                break;
                //SED  
            case 0x18:
                Emitter.MOV8ItoM((unint32) & D, 1);
                Emitter.ADD32ItoM((unint32) & Cycles, 2);
                break;
                //SEI   
            case 0x78:
                Emitter.MOV8ItoM((unint32) & I, 1);
                Emitter.ADD32ItoM((unint32) & Cycles, 2);
                break;



                //STA    

            case 0x85:
                Emitter.CALLFunc((unint32) & getZeroPage);
                Emitter.MOV8RtoM(temp, A);
                Emitter.ADD32ItoM((unint32) & Cycles, 3);
                break;
            case 0x95:
                Emitter.CALLFunc((unint32) & getZeroPageX);
                Emitter.MOV8RtoM(temp, A);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0x80:
                Emitter.CALLFunc((unint32) & getAbsolute);
                Emitter.MOV8RtoM(temp, A);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);

                break;
            case 0x90:
                Emitter.CALLFunc((unint32) & getAbsoluteX);
                Emitter.MOV8RtoM(temp, A);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);

                break;
            case 0x99:
                Emitter.CALLFunc((unint32) & getAbsoluteY);
                Emitter.MOV8RtoM(temp, A);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);


                break;
            case 0x81:
                Emitter.CALLFunc((unint32) & getIndirectX);
                Emitter.MOV8RtoM(temp, A);
                Emitter.ADD32ItoM((unint32) & Cycles, 6);
                break;
            case 0x91:
                Emitter.CALLFunc((unint32) & getIndirectY);
                Emitter.MOV8RtoM(temp, A);
                Emitter.ADD32ItoM((unint32) & Cycles, 5);

                break;


                //STX

            case 0x86:
                Emitter.CALLFunc((unint32) & getZeroPage);
                Emitter.MOV8MtoR(EAX, (unint32) & X);
                Emitter.MOV8RtoM(temp, EAX);
                Emitter.ADD32ItoM((unint32) & Cycles, 3);
                break;
            case 0x96:
                Emitter.CALLFunc((unint32) & getZeroPageX);
                Emitter.MOV8MtoR(EAX, (unint32) & X);
                Emitter.MOV8RtoM(temp, EAX);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0x8E:
                Emitter.CALLFunc((unint32) & getAbsolute);
                Emitter.MOV8MtoR(EAX, (unint32) & X);
                Emitter.MOV8RtoM(temp, EAX);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
                //STY

            case 0x84:
                Emitter.CALLFunc((unint32) & getZeroPage);
                Emitter.MOV8MtoR(EAX, (unint32) & Y);
                Emitter.MOV8RtoM(temp, EAX);
                Emitter.ADD32ItoM((unint32) & Cycles, 3);
                break;
            case 0x94:
                Emitter.CALLFunc((unint32) & getZeroPageX);
                Emitter.MOV8MtoR(EAX, (unint32) & Y);
                Emitter.MOV8RtoM(temp, EAX);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);
                break;
            case 0x8C:
                Emitter.CALLFunc((unint32) & getAbsolute);
                Emitter.MOV8MtoR(EAX, (unint32) & Y);
                Emitter.MOV8RtoM(temp, EAX);
                Emitter.ADD32ItoM((unint32) & Cycles, 4);

                break;




                //TAY  
            case 0xA8:
                Emitter.MOV8RtoM((unint32) & Y, A);
                Emitter.ADD32ItoM((unint32) & Cycles, 2);
                break;
                //TAX
            case 0xAA:
                Emitter.MOV8RtoM((unint32) & X, A);
                Emitter.ADD32ItoM((unint32) & Cycles, 2);
                break;
                //TSX  
            case 0xBA:
                Emitter.MOV8MtoR(EDX, (unint32) & S);
                Emitter.MOV8RtoM((unint32) & X, EAX);
                Emitter.ADD32ItoM((unint32) & Cycles, 2);
                break;
                //TXA  
            case 0x8A:
                Emitter.MOV8MtoR(A, (unint32) & X);
                Emitter.ADD32ItoM((unint32) & Cycles, 2);
                break;
                //TXS  
            case 0x9A:
                Emitter.MOV8MtoR(EDX, (unint32) & S);
                Emitter.MOV8RtoM((unint32) & X, EAX);
                Emitter.ADD32ItoM((unint32) & Cycles, 2);
                break;
                //TYA  
            case 0x98:
                Emitter.MOV8MtoR(A, (unint32) & Y);
                Emitter.ADD32ItoM((unint32) & Cycles, 2);
                break;







            default:
                cout << "Unknown Instruction" << endl;
        }
    }
    saveRegisters();
    Emitter.POPA32();
}

void CPU6502::resumeRegisters() {
    Emitter.MOV8MtoR(A, (unint32) & AB);
    Emitter.PUSH32M((unint32) & PB);
    Emitter.POPFD();
}

void CPU6502::saveRegisters() {
    Emitter.MOV8RtoM((unint32) & AB, A);
    Emitter.PUSHFD();
    Emitter.POP32R(EAX);
    Emitter.MOV32RtoM((unint32) & PB, EAX);
}

void CPU6502::getImmediate() {
    temp = (unint32) & Memory[Memory[PC++]];
}

void CPU6502::getZeroPage() {
    temp = (unint32) & Memory[Memory[PC++]];
}

void CPU6502::getZeroPageX() {
    temp = (unint32) & Memory[Memory[PC++] + X];
}

void CPU6502::getAbsolute() {
    temp = (unint32) & Memory[Memory[PC++] + Memory[PC++] << 8];
}

void CPU6502::getAbsoluteX() {

    temp = (unint32) & Memory[Memory[PC++] + Memory[PC++] << 8 + X];
}

void CPU6502::getAbsoluteY() {

    temp = (unint32) & Memory[Memory[PC++] + Memory[PC++] << 8 + Y];
}

void CPU6502::getIndirectX() {
    unint8 byte = 0;
    byte = Memory[PC++] + X;
    temp = (unint32) & Memory[Memory[byte] + Memory[byte + 1] << 8];
}

void CPU6502::getIndirectY() {
    unint8 byte = 0;
    byte = Memory[PC++];
    temp = (unint32) & Memory[Memory[byte] + Memory[byte + 1] << 8 + Y];
}
*/