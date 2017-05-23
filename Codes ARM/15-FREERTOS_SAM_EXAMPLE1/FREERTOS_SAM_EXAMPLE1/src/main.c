/**
 * \file
 *
 * \brief FreeRTOS Real Time Kernel example.
 *
 * Copyright (c) 2012-2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage FreeRTOS Real Time Kernel example
 *
 * \section Purpose
 *
 * The FreeRTOS example will help users how to use FreeRTOS in SAM boards.
 * This basic application shows how to create task and get information of
 * created task.
 *
 * \section Requirements
 *
 * This package can be used with SAM boards.
 *
 * \section Description
 *
 * The demonstration program create two task, one is make LED on the board
 * blink at a fixed rate, and another is monitor status of task.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *    GNU-Based Software Development</a>
 *    application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>,
 *    depending on your chosen solution.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# LED should start blinking on the board. In the terminal window, the
 *    following text should appear (values depend on the board and chip used):
 *    \code
	-- Freertos Example xxx --
	-- xxxxxx-xx
	-- Compiled: xxx xx xxxx xx:xx:xx --
\endcode
 *
 */

#include <asf.h>
#include "conf_board.h"

#define TASK_MONITOR_STACK_SIZE            (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY        (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE                (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY            (tskIDLE_PRIORITY)
/**
 * LEDs do SAME70
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		    8
#define LED_PIN_MASK    (1<<LED_PIN)
//definindo os leds que serão controlados pelos botões criados
//LED1
#define LED1_PIO_ID		ID_PIOA
#define LED1_PIO         PIOA
#define LED1_PIN		    0
#define LED1_PIN_MASK    (1<<LED1_PIN)
//LED2 
#define LED2_PIO_ID		ID_PIOC
#define LED2_PIO         PIOC
#define LED2_PIN		    30
#define LED2_PIN_MASK    (1<<LED2_PIN)
//LED3
#define LED3_PIO_ID		ID_PIOB
#define LED3_PIO         PIOB
#define LED3_PIN		    2
#define LED3_PIN_MASK    (1<<LED3_PIN)


/**
 * Botão do SAME70
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79
//definindo os botões que irão controlar os leds e seus PIOs
//BOTÃO 1
#define BOTAO1_PIO_ID	ID_PIOD
#define BUT1_PIO		PIOD
#define BUT1_PIN		28
#define BUT1_PIN_MASK	(1<<BUT1_PIN)
//BOTÃO 2
#define BOTAO2_PIO_ID	ID_PIOC
#define BUT2_PIO		PIOC
#define BUT2_PIN		31
#define BUT2_PIN_MASK	(1<<BUT2_PIN)
//BOTÃO 3
#define BOTAO3_PIO_ID	ID_PIOA
#define BUT3_PIO		PIOA
#define BUT3_PIN		19
#define BUT3_PIN_MASK	(1<<BUT3_PIN)
SemaphoreHandle_t xSemaphore = NULL;
//INICIALIZANDO
void led_init(int estado);

void but_init(void);
void but1_init(void);
void but2_init(void);
void but3_init(void);

void but_Handler();
void but_Handler1();
void but_Handler2();
void but_Handler3();
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

#if !(defined(SAMV71) || defined(SAME70))
/**
 * \brief Handler for Sytem Tick interrupt.
 */
void SysTick_Handler(void)
{
	xPortSysTickHandler();
}
#endif

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */

void but_Handler(){
    /*
     *  limpa interrupcao do PIO
     */
    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT_PIO);
    
   /**
    *  Toggle status led
    */
   if(pio_get_output_data_status(LED_PIO, LED_PIN_MASK))
    pio_clear(LED_PIO, LED_PIN_MASK);
   else
    pio_set(LED_PIO,LED_PIN_MASK);
    
}
//Atuação do botão 1 no led 1
void but_Handler1(){
    /*
     *  limpa interrupcao do PIO
     */
    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT1_PIO);
    
   /**
    *  Toggle status led
    */
   if(pio_get_output_data_status(LED1_PIO, LED1_PIN_MASK))
    pio_clear(LED1_PIO, LED1_PIN_MASK);
   else
    pio_set(LED1_PIO,LED1_PIN_MASK);
    
}

//Atuação do botão 2 no led 2
void but_Handler2(){
    /*
     *  limpa interrupcao do PIO
     */
    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT2_PIO);
    
   /**
    *  Toggle status led
    */
   if(pio_get_output_data_status(LED2_PIO, LED2_PIN_MASK))
    pio_clear(LED2_PIO, LED2_PIN_MASK);
   else
    pio_set(LED2_PIO,LED2_PIN_MASK);
    
}

//Atuação do botão 3 no led 3
void but_Handler3(){
    /*
     *  limpa interrupcao do PIO
     */
    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT3_PIO);
    
   /**
    *  Toggle status led
    */
   if(pio_get_output_data_status(LED3_PIO, LED3_PIN_MASK))
    pio_clear(LED3_PIO, LED3_PIN_MASK);
   else
    pio_set(LED3_PIO,LED3_PIN_MASK);
    
}
/**
 * @Brief Inicializa o pino do LED
 */
void led_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, 1, 0, 0 );
	//inicializando os 3 LEDs
	//LED1 inicializado
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIN_MASK, 1, 0, 0 );
	
	//LED2 inicializado
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_PIN_MASK, 1, 0, 0 );
	
	//LED3 inicializado
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIN_MASK, 1, 0, 0 );
};

/**
 * @Brief Inicializa o pino do BUT
 *  config. botao em modo entrada enquanto 
 *  ativa e configura sua interrupcao.
 */
void but_init(void){
    /* config. pino botao em modo de entrada */
    pmc_enable_periph_clk(BUT_PIO_ID);
    pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    
    /* config. interrupcao em borda de descida no botao do kit */
    /* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
    //pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
    //pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, but_Handler);
 	/* habilita interrupçcão do PIO que controla o botao */
 	/* e configura sua prioridade                        */
 	NVIC_EnableIRQ(BUT_PIO_ID);
 	NVIC_SetPriority(BUT_PIO_ID, 1);
 };	

 
//Inicializando os três botões
//BOTAO1 inicializado
void but1_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BOTAO1_PIO_ID);
	pio_set_input(BUT1_PIO, BUT1_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
	//pio_enable_interrupt(BUT1_PIO, BUT1_PIN_MASK);
	//pio_handler_set(BUT1_PIO, BOTAO1_PIO_ID, BUT1_PIN_MASK, PIO_IT_FALL_EDGE, but_Handler1); ;//aciona na borda de descida   PIO_IT_FALL_EDGE
 	/* habilita interrupçcão do PIO que controla o botao */
 	/* e configura sua prioridade                        */
 	NVIC_EnableIRQ(BOTAO1_PIO_ID);
 	NVIC_SetPriority(BOTAO1_PIO_ID, 1);
 }; 
//BOTAO2 inicializado
void but2_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BOTAO2_PIO_ID);
	pio_set_input(BUT2_PIO, BUT2_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
	//pio_enable_interrupt(BUT2_PIO, BUT2_PIN_MASK); 
	//pio_handler_set(BUT2_PIO, BOTAO2_PIO_ID, BUT2_PIN_MASK, PIO_IT_RISE_EDGE, but_Handler2);;//aciona na borda de subida &PIO_IT_RISE_EDGE
 	/* habilita interrupçcão do PIO que controla o botao */
 	/* e configura sua prioridade                        */
 	NVIC_EnableIRQ(BOTAO2_PIO_ID);
 	NVIC_SetPriority(BOTAO2_PIO_ID, 1);
 };
 //BOTAO3 inicializado
 void but3_init(void){
	 /* config. pino botao em modo de entrada */
	 pmc_enable_periph_clk(BOTAO3_PIO_ID);
	 pio_set_input(BUT3_PIO, BUT3_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	 
	 /* config. interrupcao em borda de descida no botao do kit */
	 /* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
	 //pio_enable_interrupt(BUT3_PIO, BUT3_PIN_MASK);
	 //pio_handler_set(BUT3_PIO, BOTAO3_PIO_ID, BUT3_PIN_MASK,  PIO_IT_RISE_EDGE, but_Handler3);//aciona na borda de subida e descida   PIO_IT_FALL_EDGE&&PIO_IT_RISE_EDGE
    
 	/* habilita interrupçcão do PIO que controla o botao */
 	/* e configura sua prioridade                        */
 	NVIC_EnableIRQ(BOTAO3_PIO_ID);
 	NVIC_SetPriority(BOTAO3_PIO_ID, 1);
 };
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);

	for (;;) {
		printf("--- task ## %u", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(1000);
	}
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */
static void task_led(void *pvParameters)
{
	int delay = 100;
	UNUSED(pvParameters);
	for (;;) {
		if(pio_get_output_data_status(LED_PIO, LED_PIN_MASK))
	   pio_clear(LED_PIO, LED_PIN_MASK);
	   else
	   pio_set(LED_PIO,LED_PIN_MASK);
		vTaskDelay(delay);
	if (xSemaphoreTake(xSemaphore,( TickType_t ) 10)==pdTRUE){//segure o botão 1 da placa OLED e ele fica mais rapido
		delay = 50;
	}else{
		delay = 100;
	}
	}
}
static void task_led_1(void *pvParameters)
{
	UNUSED(pvParameters);
	for (;;) {
	   if(pio_get_output_data_status(LED1_PIO, LED1_PIN_MASK))
	   pio_clear(LED1_PIO, LED1_PIN_MASK);
	   else
	   pio_set(LED1_PIO,LED1_PIN_MASK);
	   vTaskDelay(100);
	}
}

static void task_led_2(void *pvParameters)
{
	UNUSED(pvParameters);
	for (;;) {
		if(pio_get_output_data_status(LED2_PIO, LED2_PIN_MASK))
		pio_clear(LED2_PIO, LED2_PIN_MASK);
		else
		pio_set(LED2_PIO,LED2_PIN_MASK);
		vTaskDelay(100);
	}
}

static void task_led_3(void *pvParameters)
{
	UNUSED(pvParameters);
	for (;;) {
		if(pio_get_output_data_status(LED3_PIO, LED3_PIN_MASK))
		pio_clear(LED3_PIO, LED3_PIN_MASK);
		else
		pio_set(LED3_PIO,LED3_PIN_MASK);
		//if (xSemaphoreTake(xSemaphore,( TickType_t ) 10)==pdTRUE){
			//xSemaphoreTake(xSemaphore,( TickType_t ) 10)==pdFALSE;
		//}
		vTaskDelay(100);
	}
}

/**
 * \brief Configure the console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}

static void task_but0(void *pvParameters)
{
	UNUSED(pvParameters);
	for (;;) {
		//Mudando o semaphore
		if(!pio_get(BUT1_PIO,PIO_INPUT,BUT1_PIN_MASK)){
			xSemaphoreGive( xSemaphore);//We have finished accessing the shared resource.  Release the semaphore.
			vTaskDelay(100);
		}
	}
}


/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
		led_init(1);
		but_init();
		but1_init();
		but2_init();
		but3_init();
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	 xSemaphore = xSemaphoreCreateCounting(1,0);
	 xSemaphoreTake( xSemaphore, ( TickType_t ) 0 );
	 
	/* Output demo infomation. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);


	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,
			TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
				printf("Failed to create test led task\r\n");
			}
	
	/* Create task to make led blink */
	if (xTaskCreate(task_led_1, "Led1", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to make led blink */
	if (xTaskCreate(task_led_2, "Led2", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	/* Create task to make led blink */
	if (xTaskCreate(task_led_3, "Led3", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	

	  
	/* Create task to make check btn value */
	if (xTaskCreate(task_but0, "Botao", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test button task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;

}
