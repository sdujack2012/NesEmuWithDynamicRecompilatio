#include <stdio.h>
#include <sys/mman.h>
#include "codeEmitter.h"
#include "CPU6502.h"
#include <iostream>
using namespace std;
void abc(){
    
    cout<<111<<endl;
}
int main(int argc, char *argv[]) {
    int8* ii=new int8[2];
    ii[0]=0;
    ii[1]=2;
    int16* ij = (int16*)ii; 
    cout<<(*ij>>8)<<endl;
    codeEmitter emitter;
    emitter.BeginEmit();
    int i=500;
    emitter.MOV32MtoR(EAX, (unint32)&i);
   
    emitter.Ret();
    emitter.ExecuteBlock();
    cout<<i<<endl;

     
    

    
    
    
    return 0;
}
