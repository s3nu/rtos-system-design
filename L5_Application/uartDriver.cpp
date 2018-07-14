#include "uartDriver.hpp"
void LabUART::init_my_uart2(int portValue)
{
    if(portValue == 2){
        LPC_SC->PCONP |= (1<<24); //init uart2 and uart3
        //zero bits 18 and 19 of PCLKSEL1 register for UART2
        LPC_SC->PCLKSEL1 &= ~(0x3 << 16);
        //set CPU clk of UART2  and UART3
        LPC_SC->PCLKSEL1 |= (1 << 16);
        //enable DLAB
        LPC_UART2->LCR |= (1<<7);
        //set DLL and DLM
        LPC_UART2->DLL |= (0x38<<0);
        LPC_UART2->DLM |= (0x01<<0);
        //set to 8 bit char length
        LPC_UART2->LCR |= (3<<0);
        //disable DLAB
        LPC_UART2->LCR &= ~(1<<7);
        //set pins 2.8 TX and 2.9 RX
        LPC_PINCON->PINSEL4 &= ~(0xF<<16);
        LPC_PINCON->PINSEL4 |= (0xA<<16);
        //init FIFO
        LPC_UART2->FCR |= (1<<0);
        // Init UART Rx interrupt (TX interrupt is optional)
        //init interrupts
        LPC_UART2->IER |= (1<<0);
        NVIC_EnableIRQ(UART2_IRQn);
    }
    else if(portValue == 3){
        LPC_SC->PCONP |= (1<<25);
        //init uart3
        //zero bits 18 and 19 of PCLKSEL1 register for UART2
        LPC_SC->PCLKSEL1 &= ~(0x3 << 18);
        //enable DLAB
        LPC_UART3->LCR |= (1<<7);
        //set DLL and DLM
        LPC_UART3->DLL |= (0x38<<0);
        LPC_UART3->DLM |= (0x01<<0);
        //set to 8 bit char length
        LPC_UART3->LCR |= (3<<0);
        //disable DLAB
        LPC_UART3->LCR &= ~(1<<7);
        //set pins 0.0 TX and 0.1 RX
        LPC_PINCON->PINSEL0 &= ~(0xF<<0);
        LPC_PINCON->PINSEL0 |= (0xA<<16);
        //set pins 4.28 TX and 4.29 RX
        LPC_PINCON->PINSEL9 &= ~(0xF<<24);
        LPC_PINCON->PINSEL9 |= (0xF<<24);
        //init FIFO
        LPC_UART3->FCR |= (1<<0);
        // Init UART Rx interrupt (TX interrupt is optional)
        //init interrupts
        LPC_UART3->IER |= (1<<0);
        NVIC_EnableIRQ(UART3_IRQn);
    }
}
void LabUART::transfer(uint8_t value){
    LPC_UART2->THR |= (value<<0);
    while(!(LPC_UART2->LSR & (1 << 6))); //wait for THR to be empty
}
LabUART::LabUART(){
}
LabUART::~LabUART(){

}
