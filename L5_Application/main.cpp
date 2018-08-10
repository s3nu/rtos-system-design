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
#include "i2c_base.hpp"
#include "ff.h"
#include "string.h"
#include "command_handler.hpp"
#include "event_groups.h"
#include "io.hpp"
#include "storage.hpp"
#include "LCD.hpp"
#include "event_groups.h"
#include "handlers.hpp"

ADCDriver light_sensor;
PWMDriver foo;
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
        if (rvalue < 50) {
            foo.pwmSelectPin(PWMDriver::PWM_PIN_2_0);
            foo.pwmInitSingleEdgeMode(4);
            foo.setDutyCycle(PWMDriver::PWM_PIN_2_0, 45);
        }
        else if (rvalue > 50 && rvalue < 100) {
            foo.pwmSelectPin(PWMDriver::PWM_PIN_2_1);
            foo.pwmInitSingleEdgeMode(4);
            foo.setDutyCycle(PWMDriver::PWM_PIN_2_1, 60);
        }
        else if (rvalue > 1000 && rvalue < 2000) {
            foo.pwmSelectPin(PWMDriver::PWM_PIN_2_1);
            foo.pwmInitSingleEdgeMode(4);
            foo.setDutyCycle(PWMDriver::PWM_PIN_2_1, 23);
        }
        else if (rvalue > 2200 && rvalue < 3300) {
            foo.pwmSelectPin(PWMDriver::PWM_PIN_2_1);
            foo.pwmInitSingleEdgeMode(4);
            foo.setDutyCycle(PWMDriver::PWM_PIN_2_1, 78);
        }
        else if (rvalue > 3000 && rvalue < 8000) {
            foo.pwmSelectPin(PWMDriver::PWM_PIN_2_2);
            foo.pwmInitSingleEdgeMode(4);
            foo.setDutyCycle(PWMDriver::PWM_PIN_2_2, 100);

        } else {
            foo.pwmSelectPin(PWMDriver::PWM_PIN_2_2);
            foo.pwmInitSingleEdgeMode(4);
            foo.setDutyCycle(PWMDriver::PWM_PIN_2_2, 78);
            foo.pwmSelectPin(PWMDriver::PWM_PIN_2_1);
            foo.pwmInitSingleEdgeMode(4);
            foo.setDutyCycle(PWMDriver::PWM_PIN_2_1, 56);
            foo.pwmSelectPin(PWMDriver::PWM_PIN_2_0);
            foo.pwmInitSingleEdgeMode(4);
            foo.setDutyCycle(PWMDriver::PWM_PIN_2_0, 45);

        }
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

LabSPI A;
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

//LabUART A;
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
        scheduler_task("I2C", 2000, PRIORITY_LOW) {
        printf("I2C task started");
    }

    bool init(void) {
        i2c.slave(slaveadder, buffer, sizeof(buffer));
        return true;
    }

    bool run(void *p) {
        uint8_t prev = buffer[0];
        while (1) {
            if (prev != buffer[0]) {
                prev = buffer[0];
                u0_dbg_printf("buffer[0] changed to %#x\n", buffer[0]);
                uint16_t turn_it_on = i2c.readReg(slaveadder,0x04);
                if (turn_it_on == 10) {
                    scheduler_add_task(new vReadLightSensor());
                }
                else {
                    scheduler_add_task(new vReadLightSensor());
                }
            }
        }
        return true;
    }
private:
    I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance
    const uint8_t slaveadder = 0x66; // Pick any address other than the used used at i2c2.hpp
    uint8_t buffer[256] = { 0 };
};

//typedef enum {
//    invalid,
//    left,
//    right,
//    up,
//    down,
//} ornt_t;
//QueueHandle_t ornt_queue;
//ornt_t ornt;
//
//void producer(void *p)
//{
//    while (1) {
//        int xaxis = AS.getX();
//        int yaxis = AS.getY();
//        ornt = invalid;
//        if((xaxis < -300) && (yaxis < 300) && (yaxis > -300)){
//            ornt = right;
//        }
//        else if((xaxis > 300) && (yaxis < 300) && (yaxis > -300)){
//            ornt = left;
//        }
//        if((yaxis < -300) && (xaxis < 300) && (xaxis > -300)){
//            ornt = up;
//        }
//        else if((yaxis > 300) && (xaxis < 300) && (xaxis > -300)){
//            ornt = down;
//        }
//        u0_dbg_printf("sending to queue\n");
//        xQueueSend(ornt_queue, &ornt, portMAX_DELAY);
//        u0_dbg_printf("sent to queue\n");
//        vTaskDelay(1000);
//    }
//}
//
//void consumer(void *p)
//{
//    while (1) {
//        if(xQueueReceive(ornt_queue, &ornt, portMAX_DELAY)){
//            switch(ornt){
//                case 0:
//                    u0_dbg_printf("invalid\n");
//                    LD.setNumber(0);
//                    break;
//                case 1:
//                    u0_dbg_printf("left\n");
//                    LD.setNumber(1);
//                    break;
//                case 2:
//                    u0_dbg_printf("right\n");
//                    LD.setNumber(2);
//                    break;
//                case 3:
//                    u0_dbg_printf("up\n");
//                    LD.setNumber(3);
//                    break;
//                case 4:
//                    u0_dbg_printf("down\n");
//                    LD.setNumber(4);
//                    break;
//            }
//        }
//    }
//}

//QueueHandle_t xQueue1;
//EventGroupHandle_t xEventGroup;
//TaskHandle_t consumerTask;
//TaskHandle_t producerTask;
//TaskHandle_t watchdogTask;


//CMD_HANDLER_FUNC(taskSuspendResume)
//{
//    scheduler_task* pTask = scheduler_task::getTaskPtrByName("producer");
//    scheduler_task* cTask = scheduler_task::getTaskPtrByName("consumer_task");
//    if(cmdParams == "resume producer task") {
//        vTaskResume(producerTask);
//        output.printf("resumed prodcuer task\n");
//    }
//    else if(cmdParams == "resume consumer task") {
//        vTaskResume(consumerTask);
//        output.printf("resumed consumer task \n");
//    }
//    else if(cmdParams == "suspend producer task") {
//        vTaskSuspend(producerTask);
//        output.printf("suspended producer task \n");
//    }
//    else if(cmdParams == "suspend consumer task"){
//        vTaskSuspend(consumerTask);
//        output.printf("suspended consumer task \n");
//    }
//    return true;
//}

//void producer_task(void * param)
//{
//    uint32_t light_value = 0;
//    uint32_t avg = 0;
//    while(1)
//    {
//        uint32_t sum = 0;
//        for(int i3 = 0; i3 < 99; i3++)
//        {
//            light_value = LS.getRawValue();
//            sum = sum + light_value;
//            //u0_dbg_printf("The light sensor value is:  %d \n", light_value);
//            //u0_dbg_printf("The sum of the light sensor values is:  %d \n", sum);
//        }
//        avg = sum/100;
//        //u0_dbg_printf("The average of the light sensor values is:  %d \n", avg);
//        if(xQueueSend(xQueue1, &avg, NULL))
//        {
//            u0_dbg_printf("data sent to Queue %d \n", avg);
//        }
//        xEventGroupSetBits(xEventGroup, 1 << 1);
//        vTaskDelay(1000);
//    }
//}
//
//void consumer_task(void * pVParam)
//{
//    uint32_t light = 0;
//    while(1)
//    {
//        if( xQueueReceive(xQueue1, &light, portMAX_DELAY ) )
//        {
//            //uint64_t time = sys_get_uptime_ms();
//            u0_dbg_printf("data received from Queue %d \n", light);
//            //u0_dbg_printf("Time from systime %u \n", time);
//
//            // The object in the queue has been received
//            char string1[12];// = ("light: %i", light);
//            char string2[12];
//            uint64_t time = sys_get_uptime_ms();
//            sprintf(string1, "%u, ", time);
//            sprintf(string2, "%d\n", light);
//            strcat(string1, string2);
//           Storage::append("1:sensor.txt", string1, sizeof(string1), 0);
//        }
//        xEventGroupSetBits(xEventGroup, 1 << 2);
//        vTaskDelay(1000);
//    }
//}
//void watchdog_task(void * pParam)
//{
//    while(1)
//    {
//        uint32_t bit = xEventGroupWaitBits(xEventGroup, ((1 << 1) | (1 << 2)), pdTRUE, pdTRUE, 1000);
//        if (bit == 0) {
//            char data1[30] = "stuck at producer task\n";
//            Storage::append("1:stuck.txt", data1, sizeof(data1) - 1, 0);
//        } else if (bit == 4) {
//            char data2[30] = "stuck at consumer task\n";
//            Storage::append("1:stuck.txt", data2, sizeof(data2) - 1, 0);
//        }
//        vTaskDelay(1000);
//    }
//}


//QueueHandle_t q;
//TaskHandle_t xproducer;
//TaskHandle_t xconsumer;
//EventGroupHandle_t xEventGroupCreate( void );
//EventGroupHandle_t task_watchdog;
//const uint32_t producer_task_id = (1 << 0);
//const uint32_t consumer_task_id = (1 << 1);
//const uint32_t both_task_id = (producer_task_id | consumer_task_id);
//CMD_HANDLER_FUNC(myCmdHandler) {
//    /* cmdParams is a str passed to you after user's command.
//     * If command was "newcmd test 123" then cmdParams will be "test 123". *
//     * output is a CharDev class where the command came from, so
//     * we can use this to output a reply message.
//     * See "handlers.cpp" for more examples */
//    bool suspend = cmdParams.beginsWithIgnoreCase("suspend");
//    bool resume = cmdParams.beginsWithIgnoreCase("resume");
//    char taskname[64];
//    if(suspend) {
//        cmdParams.scanf("%*s %s", taskname);
//        if(strcmp(taskname, "producer")){
//            vTaskSuspend( xproducer);
//            output.printf("Producer task suspended \n");}
//        else if(strcmp(taskname, "consumer")){
//            vTaskSuspend( xconsumer);
//            output.printf("Consumer task suspended \n");}
//        else output.printf("Invalid Task\n");
//    }
//    else if(resume)
//    {
//        cmdParams.scanf("%*s %s", taskname);
//        if(strcmp(taskname, "producer")){
//            vTaskResume( xproducer);
//            output.printf("Producer task resumed \n");}
//        else if(strcmp(taskname, "consumer")){
//            vTaskResume( xconsumer);
//            output.printf("Consumer task resumed \n");}
//        else output.printf("Invalid Task\n");
//    }
//    else {
//        output.printf("ERROR for my command\n");}
//    /* return false will display command's help to the user */
//
//    return true; /* return true if command was successful */ }
//
//void vWatchDog(void *p) {
//    while(1){
//        uint32_t result = xEventGroupWaitBits(task_watchdog, both_task_id, pdTRUE, pdTRUE, 2000);
//        if((result & both_task_id) == both_task_id) {
//            printf("Both task is running...\n"); }
//        else {
//            if (!(result & producer_task_id)) {
//                printf ("Producer task stopped responding...\n"); }
//            if (!(result & consumer_task_id)) {
//                printf ("Consumer task stopped responding...\n"); }
//        }
//        vTaskDelay(1000);
//    }
//}
//void vProducer(void *p) {
//    while(1){
//        int ls = 0;
//        int average = 0;
//        int lightsensor = LS.getRawValue();
//        for(int i3 = 0; i3 <100; i3 ++) {
//            ls = ls + lightsensor;
//            vTaskDelay(1);
//        }
//        average = ls/100;
//        // printf("LS data value sent: %i \n", average); xQueueSend(q, &average, 0); //send to queue
//        xEventGroupSetBits(task_watchdog, producer_task_id);
//        vTaskDelay(1000);
//    }
//}
//void vConsumer(void *p) {
//    int LSvalues;
//    while(1){
//        vTaskDelay(1000);
//        xQueueReceive(q, &LSvalues, portMAX_DELAY);
//        //printf("LS data value received: %i \n", LSvalues );
//        FILE *fd = fopen("1:sensor.txt", "a+");
//        fprintf(fd, "%d \n", LSvalues);
//        fclose(fd);
//        xEventGroupSetBits(task_watchdog, consumer_task_id);
//    }
//}

EventGroupHandle_t tevent = xEventGroupCreate();
typedef enum {
    shared_SensorQueueId,
} sharedHandleId_t;

class consumertask: public scheduler_task {
public:
    consumertask(uint8_t priority) :
        scheduler_task("consumer", 4096, priority) {
    }

    bool run(void *p) {
        QueueHandle_t qid = getSharedObject(shared_SensorQueueId);
        int light;
        if (xQueueReceive(qid, &light, portMAX_DELAY)) {
            char string1[12];
            char string2[12];
            uint32_t time = sys_get_uptime_ms()/1000;
            sprintf(string1, "%lu, ", time);
            sprintf(string2, "%i\n", light);
            strcat(string1, string2);
            Storage::append("1:sensor.txt", string1, sizeof(string1), 0);
        }
        xEventGroupSetBits(tevent, 1 << 2);
        vTaskDelay(1000);
        return true;
    }
};

class producertask: public scheduler_task {
public:
    producertask(uint8_t priority) :
        scheduler_task("producer", 4096, priority) {
        QueueHandle_t my_queue2 = xQueueCreate(1, 4);
        addSharedObject(shared_SensorQueueId, my_queue2);
    }
    bool run(void *p) {
        int light_value[100];
        int sum = 0;
        int average;
        TickType_t xLastWakeTime;
        const TickType_t xFrequency = 1;
        xLastWakeTime = xTaskGetTickCount();
        for (int i3 = 0; i3 < 100; i3++) {
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            light_value[i] = LS.getRawValue();
            sum += light_value[i];
        }
        average = sum / 100;
        xQueueSend(getSharedObject(shared_SensorQueueId), &average, portMAX_DELAY);
        xEventGroupSetBits(tevent, 1 << 1);
        return true;
    }
};

class watchdogtask: public scheduler_task {
public:
    watchdogtask(uint8_t priority) :
        scheduler_task("watchdog", 2048, priority) {
    }
    bool run(void *p) {
        uint32_t bit = xEventGroupWaitBits(tevent, ((1 << 1) | (1 << 2)), 1, 1, 1000);
        if (bit == 0) {
            char data1[30] = "stuck at producer task\n";
            Storage::append("1:stuck.txt", data1, sizeof(data1) - 1, 0);
        } else if (bit == 4) {
            char data2[30] = "stuck at consumer task\n";
            Storage::append("1:stuck.txt", data2, sizeof(data2) - 1, 0);
        }
        vTaskDelay(1000);
        return true;
    }
};


uint8_t getcount (uint32_t max) {
    uint8_t counter = 0;
    for (int var = 0; var <= max; var++) {
        if ((var % 4 == 0) | (var % 6 == 0)){
            counter++;
        }
    }
    u0_dbg_printf("value $d", counter);
    return counter;
}

uint8_t getcountrec (uint32_t max) {
    uint8_t counter = 0;
    if (max > 1) {
        if ((max % 4 == 0) | (max % 6 == 0)){
            counter++;
        }
        return counter + (getcountrec(max-1));
    }
    return 0;
}

void prac ( void *p){
    while(1){
//        uint8_t testing = getcountrec(20);
//        u0_dbg_printf("value %i", testing);
        vTaskDelay(3000);
    }
}


QueueHandle_t qh = 0;

void rx(void *p)
{
    int item = 0;

    puts("rx task\n");
    if (xQueueReceive(qh, &item, portMAX_DELAY))
    {
        puts("Rx received an item!\n");
    }

    vTaskSuspend(0);
    puts("Rx is suspended!\n");
}

void tx(void *p)
{
    int item = 0;
    while(1)
    {
        puts("Yield\n");
        taskYIELD();

        xQueueSend(qh, &item, 0);
        puts("Did I send an item?\n");

        xQueueSend(qh, &item, portMAX_DELAY);
        puts("I must have sent an item\n");
    }
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

    //      scheduler_add_task(new lab2());
    //    A.init(SSP1, 8, SPI, 0x80);
    //    while(1){
    //        A.read_sig();
    //        delay_ms(1000);
    //
    //    }

    //    A.init_my_uart2(2);
    //    isr_register(UART2_IRQn, my_uart2_rx_intr);
    //    my_queue = xQueueCreate(5, sizeof(int));
    //    xTaskCreate(master, (const char*)"trc", STACK_BYTES(2048), 0, PRIORITY_HIGH, 0);
    //    xTaskCreate(slave, (const char*)"trc", STACK_BYTES(2048), 0, PRIORITY_HIGH, 0);

    //    scheduler_add_task(new lab3());
    //      scheduler_add_task(new lab5());
    //    I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance
    //    const uint8_t slaveAddr = 0xC0;  // Pick any address other than an existing one at i2c2.hpp
    //    unsigned char slave = 0XC0;
    //    volatile uint8_t *buffer[256] = { 0 }; // Our slave read/write buffer (This is the memory your other master board will read/write)
    //    uint8_t prev = buffer[0];
    //
    //    i2c.slave(slave, buffer, sizeof(buffer));
    //    while(1)
    //    {
    //        if (prev != buffer[0]) {
    //            prev = buffer[0];
    //            printf("buffer[0] changed to %#x by the other Master Board\n", buffer[0]);
    //        }
    //    }

    //    ornt_queue = xQueueCreate(5, sizeof(int));
    //    xTaskCreate(producer, (const char*)"producer", STACK_BYTES(2048), 0, PRIORITY_HIGH, 0);
    //    xTaskCreate(consumer, (const char*)"consumer", STACK_BYTES(2048), 0, PRIORITY_HIGH, 0);

    //
    //scheduler_add_task(new terminalTask(PRIORITY_HIGH));
    //    scheduler_add_task(new consumertask(PRIORITY_MEDIUM));
    //    scheduler_add_task(new producertask(PRIORITY_MEDIUM));
    //    scheduler_add_task(new watchdogtask(PRIORITY_HIGH));
    //xTaskCreate(prac, (const char*)"prac", STACK_BYTES(2048), 0, PRIORITY_HIGH, 0);


    /*
     * MIDTERM
     */
    //        uint8_t num = 0xff;
    //        for (int i2=0;i2<8;i2++) {
    //            delay_ms(1000);
    //            if(num & (1 << i2)){
    //                u0_dbg_printf("\nbit number %d is set", (i2));
    //            }
    //            else u0_dbg_printf("\nbit number %d is not set",(i2));
    //        }
    //
    //        uint32_t value = 0xabcdefff;
    //        uint8_t result[4];
    //
    //        result[0] = (value & 0x000000ff);
    //        result[1] = (value & 0x0000ff00) >> 8;
    //        result[2] = (value & 0x00ff0000) >> 16;
    //        result[3] = (value & 0xff000000) >> 24;
    //        u0_dbg_printf("\nbit number %x",(result[0]));
    //        u0_dbg_printf("\nbit number %x",(result[1]));
    //        u0_dbg_printf("\nbit number %x",(result[2]));
    //        u0_dbg_printf("\nbit number %x",(result[3]));



    /*
     * FINAL
     */

    qh = xQueueCreate(3, sizeof(int));
    xTaskCreate(rx, "rx", 1024, NULL, PRIORITY_LOW, NULL);
    xTaskCreate(tx, "tx", 1024, NULL, PRIORITY_LOW, NULL);
    uint32_t *x;
    *x = 0x4000;
    uint8_t *y;
    *y = 0x4000;
    u0_dbg_printf("%x %x", (x+1), (y+1)); //4004 and 4001 jumps by pointer type




















        //scheduler_start();
        vTaskStartScheduler();


    return 0;
}
