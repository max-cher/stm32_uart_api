#define STACK_TOP 0x20002000											// This can move quite a lot !

void nmi_handler(void);
void hardfault_handler(void);
void delay(void);
void ms_delay(int);
int main(void);

//	Four vectors - the starting stack pointer value, code entry point and NMI and Hard-Fault handlers

unsigned int * myvectors[4] 
__attribute__ ((section("vectors")))= {
    (unsigned int *)	STACK_TOP,         
    (unsigned int *) 	main,              
    (unsigned int *)	nmi_handler,       
    (unsigned int *)	hardfault_handler  
};

