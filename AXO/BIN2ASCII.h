// Bin to ASCII
// IN = value
// A B C = ascii code of digits
#define BIN8toASCII3(a_,b_,c_,in_) 						\
do										\
{										\
    asm volatile(									\
    "LDI %A[RA],-1+'0'" 				"\n\t" 		\
    "bcd1%=:"	"INC %A[RA]"					"\n\t"		\
    "SUBI %A[RC],100"				"\n\t"		\
    "BRCC bcd1%="					"\n\t"		\
    "LDI %A[RB],10+'0'"				"\n\t"		\
    "bcd2%=:"   	"DEC %A[RB]"					"\n\t"		\
    "SUBI %A[RC],-10"				"\n\t"		\
    "BRCS bcd2%="					"\n\t"		\
    "SBCI %A[RC],-'0'"				"\n\t"		\
    \
    : [RA]"=a" (a_), [RB]"=a" (b_), [RC]"=a" (c_)			\
    : "[RC]" (in_)							\
    );									\
    \
}										\
while(0)