/************************************************************************
* Rafael Corsi   - Insper
* rafael.corsi@insper.edu.br
*
* Computação Embarcada
*
* 10-PIO-INTERRUPCAO
*
* [ref] http://www.atmel.com/Images/Atmel-42142-SAM-AT03258-Using-Low-Power-Mode-in-SAM4E-Microcontroller_Application-Note.pdf
* [ref] https://www.eecs.umich.edu/courses/eecs373/labs/refs/M3%20Guide.pdf
************************************************************************/


#include "asf.h"
#include "conf_clock.h"
#include "Driver/pio_insper.h"
#include "Driver/pmc_insper.h"


/************************************************************************/
/* Defines                                                              */
/************************************************************************/

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

/************************************************************************/
/* prototype                                                             */
/************************************************************************/
void led_init(int estado);

void but_init(void);
void but1_init(void);
void but2_init(void);
void but3_init(void);

void but_Handler();
void but_Handler1();
void but_Handler2();
void but_Handler3();

/************************************************************************/
/* Interrupçcões                                                        */
/************************************************************************/

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
/************************************************************************/
/* Funções	                                                            */
/************************************************************************/

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
    pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
    pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, but_Handler);
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
	pio_enable_interrupt(BUT1_PIO, BUT1_PIN_MASK);
	pio_handler_set(BUT1_PIO, BOTAO1_PIO_ID, BUT1_PIN_MASK, PIO_IT_FALL_EDGE, but_Handler1); ;//aciona na borda de descida   PIO_IT_FALL_EDGE
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
	pio_enable_interrupt(BUT2_PIO, BUT2_PIN_MASK); 
	pio_handler_set(BUT2_PIO, BOTAO2_PIO_ID, BUT2_PIN_MASK, PIO_IT_RISE_EDGE, but_Handler2);;//aciona na borda de subida &PIO_IT_RISE_EDGE
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
	 pio_enable_interrupt(BUT3_PIO, BUT3_PIN_MASK);
	 pio_handler_set(BUT3_PIO, BOTAO3_PIO_ID, BUT3_PIN_MASK, PIO_IT_FALL_EDGE&&PIO_IT_RISE_EDGE, but_Handler3);//aciona na borda de subida e descida   PIO_IT_FALL_EDGE&&PIO_IT_RISE_EDGE
    
 	/* habilita interrupçcão do PIO que controla o botao */
 	/* e configura sua prioridade                        */
 	NVIC_EnableIRQ(BOTAO3_PIO_ID);
 	NVIC_SetPriority(BOTAO3_PIO_ID, 1);
 };

/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{
	/************************************************************************/
	/* Inicialização básica do uC                                           */
	/************************************************************************/
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;

	/************************************************************************/
	/* Inicializao I/OS                                                     */
	/************************************************************************/
	led_init(1);
    but_init();
	but1_init();
	but2_init();
	but3_init();
	

	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
       /* entra em modo sleep */
       pmc_sleep(SLEEPMGR_SLEEP_WFI);
	   
	   for (int i=0; i<6;i++ ){//o led da placa pisca 6 vezes toda vez que um botão externo é clicado
	   delay_ms(50);//atrasa 50 miliseconds
	      if(pio_get_output_data_status(LED_PIO, LED_PIN_MASK))//checa o status do LED da placa
	      pio_clear(LED_PIO, LED_PIN_MASK);//se for zero ele coloca 1
	      else
	      pio_set(LED_PIO,LED_PIN_MASK); //se for 1 ele coloca 0
	   }
	};
}


