#include "tasks.hpp"
#include "examples/examples.hpp"
#include "periodic_scheduler/periodic_callback.h"
#include "uart2.hpp"
#include "uart3.hpp"
#include "utilities.h"
#include "uart0_min.h"
#include "handlers.hpp"
#include "printf_lib.h"
#include "stdio.h"
#include "LabGPIO.hpp"
#include "interrupts.h"
#include "ADCDriver.hpp"
#include "PWMDriver.hpp"
#include "LabSPI.hpp"
#include "uartDriver.hpp"
#include "io.hpp"
#include "i2c2.hpp"


ADCDriver light_sensor;

class vReadLightSensor : public scheduler_task {

public:
    vReadLightSensor():scheduler_task("vReadLightSensor", 2048, 1, NULL) {

    }
    bool init(void) {
        light_sensor.adcInitBurstMode();
        light_sensor.adcSelectPin(ADCDriver::ADC_PIN_0_26);
        return true;
    }

    bool run( void *p) {
        uint16_t rvalue = light_sensor.readADCVoltageByChannel(3);
        printf("rvalue %d\n", rvalue);
        vTaskDelay(1000);
        return true;
    }

private:
};

bool switch0bool;
SemaphoreHandle_t xSemaphore;

int LEDptr = 1;
int ptrSwitchParam = 0;


void vControlLED(void * pvParameters )
{

    uint32_t paramLED = *((uint32_t*)pvParameters);
    uint32_t portParamLED = 100;
    if(paramLED<100){
        portParamLED = 0;
    }
    else if((paramLED>= 100)&& (paramLED <200)){
        portParamLED = 1;
        paramLED = paramLED -100;
    }
    else{
        portParamLED = 2;
        paramLED = paramLED -200;
    }
    LabGPIO_0 LED0 (1,0);

    LED0.setAsOutput();
    while(1)
    {
        if(xSemaphoreTake(xSemaphore, 500)){
            LED0.setLow();
            vTaskDelay(500);
        }
        else{
            LED0.setHigh();
        }
    }
    vTaskDelete(NULL);
}

LabGPIOInterrupts gpio_intr_instance;
void c_eint3_handler(void)
{
    gpio_intr_instance.handle_interrupt();
}
//Init things once
void user_callback(void)
{
    xSemaphoreGive(xSemaphore);
    u0_dbg_printf("Calling port 0, pin 0\n");
}

TaskHandle_t xhandler;

//SPI Lab2
class lab2: public scheduler_task {
public:
    lab2() :
        scheduler_task("lab2", 2000, PRIORITY_LOW) {
    }

    void spi1_Init() {
        LPC_SC->PCONP |= (1 << 10);             // SPI1 Power Enable
        LPC_SC->PCLKSEL0 &= ~(3 << 20);         // Clear clock Bits
        LPC_SC->PCLKSEL0 |= (1 << 20);      // CLK / 1

        // Select MISO, MOSI, and SCK pin-select functionality
        LPC_PINCON->PINSEL0 &= ~((3 << 14) | (3 << 16) | (3 << 18));
        LPC_PINCON->PINSEL0 |= ((2 << 14) | (2 << 16) | (2 << 18));
        LPC_SSP1->CR0 = 7;                      // 8-bit mode
        LPC_SSP1->CR1 = (1 << 1);               // Enable SSP as Master
        LPC_SSP1->CPSR = 8;                     // SCK speed = CPU / 8
        //      LPC_PINCON->PINSEL0 &= ~( (1 << 13)|(1 << 12) ); //Select CS
        LPC_PINCON->PINSEL0 &= ~(3 << 13); //Select CS
        LPC_GPIO0->FIODIR |= (1 << 6);      //CS output
        LPC_GPIO0->FIOSET = (1 << 6);           //CS high (not selected)
    }

    bool run(void *p) {
        /*
         * Main task for Lab2 handler
         */
        vTaskDelay(1000);                       //Delay

        printf("\nDevice ID\n");
        LPC_GPIO0->FIOCLR = (1 << 6);           //CS low
        spi1_ExchangeByte(0x9F);                //Opcode for Device ID
        for (int i = 0; i < 5; i++) {
            printf("\n %d\t %x", i, spi1_ExchangeByte(0x00));
        }
        LPC_GPIO0->FIOSET = (1 << 6);               //CS high
        vTaskDelay(1000);                       //Delay
        printf(
                "\n******************************************************************************\n");

        printf("Memory's Status Register\n");
        LPC_GPIO0->FIOCLR = (1 << 6);           //CS low
        spi1_ExchangeByte(0xD7);                //Opcode for Status Register
        uint16_t temp, temp1;
        temp = spi1_ExchangeByte(0x00);
        printf("\n %x\t", temp);
        temp1 = spi1_ExchangeByte(0x00);
        printf("\n %x\t", temp1);
        LPC_GPIO0->FIOSET = (1 << 6);               //CS high
        vTaskDelay(1000);                       //Delay
        return true;
    }

    char spi1_ExchangeByte(char out) {
        LPC_SSP1->DR = out;
        while (LPC_SSP1->SR & (1 << 4))
            ; // Wait until SSP is busy
        return LPC_SSP1->DR;
    }
};

//LabSPI A;
uint8_t transfer(uint8_t out)
{
    LPC_SSP1->DR = out;
    while (LPC_SSP1->SR & (1 << 4)); // Wait for SSP to be busy
    return LPC_SSP1->DR;

}

uint8_t d[3];
SemaphoreHandle_t spi_bus_lock = xSemaphoreCreateMutex();
void task_sig_reader(void *p)
{
    while (1)
    {

        if(xSemaphoreTake(spi_bus_lock, 1000)) {
            // The simplest test is to try to read the signature of the Adesto flash and print it out
            LPC_GPIO0->FIOCLR = (1 << 6);     //adesto_cs();
            {
                d[0] = transfer(0x9F); // TODO: Find what to send to read Adesto flash signature
                d[1] = transfer(0x00);
                d[2] = transfer(0x00);
            }
            LPC_GPIO0->FIOSET = (1 << 6);     //adesto_ds();
        }
        xSemaphoreGive(spi_bus_lock);


        if (d[0] != 0xFF || d[1] != 0x1F || d[2] != 0x26) {
            printf("Ooops... race condition");
            vTaskSuspend(NULL); // Suspend this task
        }
    }
}

void task_page_reader(void *p)
{
    while (1)
    {
        if(xSemaphoreTake(spi_bus_lock, 1000)) {
            LPC_GPIO0->FIOCLR = (1 << 6);     //adesto_cs();
            {
                transfer(0xD2); // TODO: Find what to send to read Adesto flash signature
                transfer(0x00);
                transfer(0x00);
                transfer(0x00);
                transfer(0x00);
                transfer(0x00);
                transfer(0x00);
                transfer(0x00);
                d[0] = transfer(0xD2); // TODO: Find what to send to read Adesto flash signature
                d[1] = transfer(0x00);
                d[2] = transfer(0x00);
            }
            LPC_GPIO0->FIOSET = (1 << 6);     //adesto_ds();
        }
        xSemaphoreGive(spi_bus_lock);
        vTaskDelay(1);
    }
}

LabUART A;
uint8_t received[3], final, received1, MSB, LSB, total;
int i = 0;
int j = 0;
QueueHandle_t my_queue;
void my_uart2_rx_intr(void)
{
    if(LPC_UART2->IIR & (4<<0)){
        received1 = LPC_UART2->RBR;
        xQueueSendFromISR(my_queue, &received1, NULL);
    }
}
uint8_t calculate(uint8_t receivedVal1, uint8_t receivedVal2,
        uint8_t receivedChar){
    switch(receivedChar){
        case(0x2B):
                                                                                                                                return final = receivedVal1 + receivedVal2;
        break;
        case(0x2D):
                                                                                                                                return final = receivedVal1 - receivedVal2;
        break;
        case(0x2A):
                                                                                                                                return final = receivedVal1 * receivedVal2;
        break; }
}
void master(void *p)
{
    A.transfer(6);
    A.transfer(6);
    A.transfer('+');
    while (1) {
        {
            if (xQueueReceive(my_queue, &received1, portMAX_DELAY))
                received[j] = received1;
            printf("received from slave: %x\n",
                    received[j]);
            j++; }
        if(j==2){
            total = received[0]*10  + received[1];
            LD.setNumber(total);
            printf("value from master: %i\n", total);
        } }
}
void slave(void *p)
{
    while (1) {
        if (xQueueReceive(my_queue, &received1,portMAX_DELAY)) {
            received[i] = received1;
            printf("value from master %x\n", received[i]);
            i++;
        }
        if(i==3){
            int myFinal = calculate(received[0],received[1],received[2]);
            printf("final values %x ", myFinal);
            MSB = myFinal/10;
            LSB = myFinal%10;
            LD.setNumber((int)myFinal);
            A.transfer(MSB);
            A.transfer(LSB);
        }
    }
}

//UART Lab 3
class lab3: public scheduler_task {
public:
    lab3() :
        scheduler_task("lab3", 2000, PRIORITY_LOW) {
    }

    bool init(void) {
        //UART driver
        LPC_SC->PCONP |= (1 << 25); //UART3 Power Enable
        LPC_SC->PCLKSEL1 &= ~(3 << 19); //CLR CLK
        LPC_SC->PCLKSEL1 |= (1 << 18); // SET CLK
        LPC_UART3->LCR = 0x03; //bits 7:0 -- parity
        LPC_UART3->LCR |= (1 << 7); //DLAB is enabled
        uint16_t dll = 48000000 / (16 * 4800); //calculating the baud
        LPC_UART3->DLL = dll; // setting baud rate
        LPC_UART3->LCR = 3; //DLAB is disable -- set to 8bit transfer
        LPC_PINCON->PINSEL9 |= ((1 << 25) | (1 << 24)); //set bit 25:24 11 for TXD3
        LPC_PINCON->PINSEL9 |= ((1 << 27) | (1 << 26)); //set bit 27:26 11 for RXD3
        LPC_PINCON->PINMODE9 &= ~((1 << 13) | (1 << 12)); //set bit 25:24 for 00 pull up -- disable pull-down resistor
        LPC_PINCON->PINMODE9 &= ~((1 << 13) | (1 << 12)); //set bit 27:26 for 00 pull up

        return true;
    }

    bool run(void *p) {
        char userinput = '0';
        printf("\n 1: board send");
        printf("\n 2: board receive");
        scanf("%c", &userinput);
        uart3_putchar('q');
        vTaskDelay(1000);
        printf("\n********** %c", uart3_getchar());
        if (userinput == '1') {
            for (int i = 0; i < 10; i++) {
                uart3_putchar('a');
                uart3_putchar('n');
                uart3_putchar('a');
                uart3_putchar('h');
                uart3_putchar('i');
                uart3_putchar('t');
            }
            printf(
                    "\n******************************************************************************\n");
        }

        if (userinput == '2') {
            for (int i = 0; i < 10; i++) {
                printf("\n %c", uart3_getchar());
            }
            printf(
                    "\n******************************************************************************\n");
        }
        return true;
    }

    char uart3_getchar() {
        while (!(LPC_UART3->LSR & (1 << 6)))
            ;
        return LPC_UART3->RBR;
    }

    char uart3_putchar(char out) {
        while (!(LPC_UART3->LSR & (1 << 6)))
            ;
        LPC_UART3->THR = out;
        return 1;
    }
};

//I2c LAB
class lab5: public scheduler_task {
public:
    lab5() :
        scheduler_task("lab5", 2000, PRIORITY_LOW) {
        printf("lab5 task started");
    }

    bool init(void) {
        i2c.slave(slaveadder, buffer, sizeof(buffer));

        return true;
    }

    bool run(void *p) {
        //          I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance
        //          const uint8_t slaveadder = 0x60;  // Pick any address other than the used used at i2c2.hpp
        //          uint8_t buffer[256] = { 0 };     // Our slave read/write buffer
        uint8_t prev = buffer[0];
        while (1) {
            if (prev != buffer[0]) {
                prev = buffer[0];
                u0_dbg_printf("buffer[0] changed to %#x\n", buffer[0]);
            }
        }
        return true;
    }
private:
    I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance
    const uint8_t slaveadder = 0x66; // Pick any address other than the used used at i2c2.hpp
    uint8_t buffer[256] = { 0 };
};

void exam(void *p){


}



int main(void)
{

    //    xSemaphore = xSemaphoreCreateBinary();
    //    xTaskCreate(vControlLED, (const char*)"control external LED", STACK_BYTES(2048), (void*)&LEDptr, 1, 0);
    //    gpio_intr_instance.init();
    //    gpio_intr_instance.attachInterruptHandler(2, 3, user_callback, rising_edge);
    //    gpio_intr_instance.attachInterruptHandler(0, 0, user_callback, both_edges);
    //    isr_register(EINT3_IRQn, c_eint3_handler);
    //    xTaskCreate(ReadLED, "Read light", 2048, (void *) 0, PRIORITY_MEDIUM, &xhandler);
    //    scheduler_add_task(new vReadLightSensor());

    //    scheduler_add_task(new lab2());

    //        A.init(SSP1, 8, SPI, 0x80);
    //        while(1){
    //        A.read_sig();
    //        delay_ms(1000);
    //        }

    //    A.init_my_uart2(2);
    //    isr_register(UART2_IRQn, my_uart2_rx_intr);
    //    my_queue = xQueueCreate(3, sizeof(int));
    //    xTaskCreate(master, (const char*)"trc", STACK_BYTES(2048), 0, 1, 0);
    //    xTaskCreate(slave, (const char*)"trc", STACK_BYTES(2048), 0, 1, 0);

    //    scheduler_add_task(new lab3());
    //
//        I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance
//        const uint8_t slaveAddr = 0xC0;  // Pick any address other than an existing one at i2c2.hpp
//        volatile uint8_t buffer[256] = { 0 }; // Our slave read/write buffer (This is the memory your other master board will read/write)
//
//         I2C is already initialized before main(), so you will have to add initSlave() to i2c base class for your slave driver
//        i2c.init_slave(slaveAddr, &buffer[0], sizeof(buffer));
//
//         I2C interrupt will (should) modify our buffer.
//         So just monitor our buffer, and print and/or light up LEDs
//         ie: If buffer[0] == 0, then LED ON, else LED OFF
//            scheduler_add_task(new lab5());
//            uint8_t prev = buffer[0];
//            while(1)
//            {
//                if (prev != buffer[0]) {
//                    prev = buffer[0];
//                    printf("buffer[0] changed to %#x by the other Master Board\n", buffer[0]);
//                }
//        scheduler_add_task(new terminalTask(PRIORITY_HIGH));
//        scheduler_start();

    //    uint8_t num = 0xff;
    //    for (int i2=0;i2<8;i2++) {
    //        delay_ms(1000);
    //        if(num & (1 << i2)){
    //            u0_dbg_printf("\nbit number %d is set", (i2));
    //        }
    //        else u0_dbg_printf("\nbit number %d is not set",(i2));
    //    }

    uint32_t value = 0xabcdefff;
    uint8_t result[4];

    result[0] = (value & 0x000000ff);
    result[1] = (value & 0x0000ff00) >> 8;
    result[2] = (value & 0x00ff0000) >> 16;
    result[3] = (value & 0xff000000) >> 24;
    u0_dbg_printf("\nbit number %x",(result[0]));
    u0_dbg_printf("\nbit number %x",(result[1]));
    u0_dbg_printf("\nbit number %x",(result[2]));
    u0_dbg_printf("\nbit number %x",(result[3]));

    vTaskStartScheduler();
    return 0;
}
