#include "asf.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
#define YEAR        2017
#define MOUNTH      3
#define DAY         27
#define WEEK        13
#define HOUR        9
#define MINUTE      5
#define SECOND      0

/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		    8
#define LED_PIN_MASK    (1<<LED_PIN)
//definindo os leds que ser�o controlados pelos bot�es criados
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
 * Bot�o
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

//definindo os bot�es que ir�o controlar os leds e seus PIOs
//BOT�O 1
#define BOTAO1_PIO_ID	ID_PIOD
#define BUT1_PIO		PIOD
#define BUT1_PIN		28
#define BUT1_PIN_MASK	(1<<BUT1_PIN)
//BOT�O 2
#define BOTAO2_PIO_ID	ID_PIOC
#define BUT2_PIO		PIOC
#define BUT2_PIN		31
#define BUT2_PIN_MASK	(1<<BUT2_PIN)
//BOT�O 3
#define BOTAO3_PIO_ID	ID_PIOA
#define BUT3_PIO		PIOA
#define BUT3_PIN		19
#define BUT3_PIN_MASK	(1<<BUT3_PIN)

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/
volatile uint8_t flag_led0 = 1;


/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(void);
void LED_init(int estado);
void led_init(int estado);
//declarando as funcoes dos  botoes e leds criados
void but_init(void);
void but1_init(void);
void but2_init(void);
void but3_init(void);

void but_Handler();
void but_Handler1();
void but_Handler2();
void but_Handler3();
void TC0_init(void);
void TC1_init(void);
void TC2_init(void);
void TC3_init(void);

void RTC_init(void);
void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
 *  Handle Interrupcao botao 1
 */
static void Button1_Handler(uint32_t id, uint32_t mask)
{
	
}

/**
 *  Interrupt handler for TC1 interrupt. 
 */
void TC0_Handler(void){
	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
    if(flag_led0)
        pin_toggle(LED_PIO, LED_PIN_MASK);
}

//Criando os TCs para os Leds que eu defini
void TC1_Handler(void){
	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
    if(flag_led0)
        pin_toggle(LED1_PIO, LED1_PIN_MASK);
}

void TC2_Handler(void){
	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
    if(flag_led0)
        pin_toggle(LED2_PIO, LED2_PIN_MASK);
}

void TC3_Handler(void){
	volatile uint32_t ul_dummy;

    /****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC1, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
    if(flag_led0)
        pin_toggle(LED3_PIO, LED3_PIN_MASK);
}


/**
 * \brief Interrupt handler for the RTC. Refresh the display.
 */
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/* Second increment interrupt */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);

		} else {
		/* Time or date alarm */
		if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			
			/*Atualiza hora */
			uint32_t ano,mes,dia,hora,minuto,segundo;
			
			rtc_get_date(RTC, &ano, &mes, &dia, NULL);
			rtc_get_time(RTC, &hora, &minuto, &segundo);
			
			/* incrementa minuto (Para casos de mudan�a de hora ou de mes ou de ano) */ 
			if(minuto>=59){
				if(hora>=23){
					if(dia>=30){
						if(mes>=11){
							ano++;
						}
						else{
							mes++;
						}
						
					}
					else{
						dia++;
					}
				}
				else{
					hora++;
				}
			}
			else{
				minuto++;
			}
			
			
			/* configura novo alarme do RTC */
			rtc_set_date_alarm(RTC, 1, mes, 1, dia);
			rtc_set_time_alarm(RTC, 1, hora, 1, minuto, 1, segundo);
			
			/* inverte status led */
			flag_led0 ^= 1;
			
			/* Ativa/desativa o TimerCounter */
			if(flag_led0){
				tc_start(TC0, 0);
				tc_start(TC0, 1);
				tc_start(TC0, 2);
				tc_start(TC1, 0);
			}
			else{
				tc_stop(TC0, 0);
				tc_stop(TC0, 1);
				tc_stop(TC0, 2);
				tc_stop(TC1, 0);
			}
			
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
		}
	}
}

//Atua��o do bot�o 1 no led 1
void but_Handler1(){
    /*
     *  limpa interrupcao do PIO
     */
    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT1_PIO);
    
   /**
    *  Toggle status led
    */
   if(pio_get_output_data_status(LED1_PIO, LED1_PIN_MASK)){
    pio_clear(LED1_PIO, LED1_PIN_MASK);
	tc_start(TC0,1);
   }
   else{
    pio_set(LED1_PIO,LED1_PIN_MASK);
	tc_stop(TC0,1);
   }
    
}

//Atua��o do bot�o 2 no led 2
void but_Handler2(){
    /*
     *  limpa interrupcao do PIO
     */
    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT2_PIO);
    
   /**
    *  Toggle status led
    */
   if(pio_get_output_data_status(LED2_PIO, LED2_PIN_MASK)){
    pio_clear(LED2_PIO, LED2_PIN_MASK);
	tc_start(TC0,2);
   }
   else{
    pio_set(LED2_PIO,LED2_PIN_MASK);
    tc_stop(TC0,2);
   
}
}

//Atua��o do bot�o 3 no led 3
void but_Handler3(){
    /*
     *  limpa interrupcao do PIO
     */
    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT3_PIO);
    
   /**
    *  Toggle status led
    */
   if(pio_get_output_data_status(LED3_PIO, LED3_PIN_MASK)){
    pio_clear(LED3_PIO, LED3_PIN_MASK);
	tc_start(TC1,0);
   }
   else{
    pio_set(LED3_PIO,LED3_PIN_MASK);
	tc_stop(TC1,0);
   }
    
}


/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/** 
 *  Toggle pin controlado pelo PIO (out)
 */
void pin_toggle(Pio *pio, uint32_t mask){
   if(pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
   else
    pio_set(pio,mask);
}

/**
 * @Brief Inicializa o pino do BUT
 */
void BUT_init(void){
    /* config. pino botao em modo de entrada */
    pmc_enable_periph_clk(BUT_PIO_ID);
    pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    
    /* config. interrupcao em borda de descida no botao do kit */
    /* indica funcao (but_Handler) a ser chamada quando houver uma interrup��o */
    pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
    pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);
    
    /* habilita interrup�c�o do PIO que controla o botao */
    /* e configura sua prioridade                        */
    NVIC_EnableIRQ(BUT_PIO_ID);
    NVIC_SetPriority(BUT_PIO_ID, 1);
};

/**
 * @Brief Inicializa o pino do LED
 */
void LED_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
	
	//inicializando os 3 LEDs
	//LED1 inicializado
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIN_MASK, 1, 0, 0 );
		
	//LED2 inicializado
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_PIN_MASK, 1, 0, 0 );
		
	//LED3 inicializado
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIN_MASK, 0, 0, 0 );

};

/**
 * Configura TimerCounter (TC0) para gerar uma interrupcao no canal 0-(ID_TC1) 
 * a cada 250 ms (4Hz)
 */
void TC0_init(void){   
    uint32_t ul_div;
    uint32_t ul_tcclks;
    uint32_t ul_sysclk = sysclk_get_cpu_hz();
    
    uint32_t channel = 0;
    
    /* Configura o PMC */
    pmc_enable_periph_clk(ID_TC0);    

    /** Configura o TC para operar em  4Mhz e interrup�c�o no RC compare */
    tc_find_mck_divisor(4, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
    tc_init(TC0, channel, ul_tcclks | TC_CMR_CPCTRG);
    tc_write_rc(TC0, channel, (ul_sysclk / ul_div) / 4);

    /* Configura e ativa interrup�c�o no TC canal 0 */
    NVIC_EnableIRQ((IRQn_Type) ID_TC0);
    tc_enable_interrupt(TC0, channel, TC_IER_CPCS);

    /* Inicializa o canal 0 do TC */
    tc_start(TC0, channel);
}


void TC1_init(void){
	    uint32_t ul_div;
	    uint32_t ul_tcclks;
	    uint32_t ul_sysclk = sysclk_get_cpu_hz();
	    
	    uint32_t channel = 1;
	    
	    /* Configura o PMC */
	    pmc_enable_periph_clk(ID_TC1);

	    /** Configura o TC para operar em  4Mhz e interrup�c�o no RC compare */
	    tc_find_mck_divisor(8, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	    tc_init(TC0, channel, ul_tcclks | TC_CMR_CPCTRG);
	    tc_write_rc(TC0, channel, (ul_sysclk / ul_div) / 8);

	    /* Configura e ativa interrup�c�o no TC canal 0 */
	    NVIC_EnableIRQ((IRQn_Type) ID_TC1);
	    tc_enable_interrupt(TC0, channel, TC_IER_CPCS);

	    /* Inicializa o canal 0 do TC */
	    tc_start(TC0, channel);
    }



void TC2_init(void){
	    uint32_t ul_div;
	    uint32_t ul_tcclks;
	    uint32_t ul_sysclk = sysclk_get_cpu_hz();
	    
	    uint32_t channel = 2;
	    
	    /* Configura o PMC */
	    pmc_enable_periph_clk(ID_TC2);

	    /** Configura o TC para operar em  4Mhz e interrup�c�o no RC compare */
	    tc_find_mck_divisor(11, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	    tc_init(TC0, channel, ul_tcclks | TC_CMR_CPCTRG);
	    tc_write_rc(TC0, channel, (ul_sysclk / ul_div) / 11);

	    /* Configura e ativa interrup�c�o no TC canal 0 */
	    NVIC_EnableIRQ((IRQn_Type) ID_TC2);
	    tc_enable_interrupt(TC0, channel, TC_IER_CPCS);

	    /* Inicializa o canal 0 do TC */
	    tc_start(TC0, channel);
    }



void TC3_init(void){
		    uint32_t ul_div;
		    uint32_t ul_tcclks;
		    uint32_t ul_sysclk = sysclk_get_cpu_hz();
		    
		    uint32_t channel = 0;
		    
		    /* Configura o PMC */
		    pmc_enable_periph_clk(ID_TC3);

		    /** Configura o TC para operar em  4Mhz e interrup�c�o no RC compare */
		    tc_find_mck_divisor(17, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
		    tc_init(TC1, channel, ul_tcclks | TC_CMR_CPCTRG);
		    tc_write_rc(TC1, channel, (ul_sysclk / ul_div) / 17);

		    /* Configura e ativa interrup�c�o no TC canal 0 */
		    NVIC_EnableIRQ((IRQn_Type) ID_TC3);
		    tc_enable_interrupt(TC1, channel, TC_IER_CPCS);

		    /* Inicializa o canal 0 do TC */
		    tc_start(TC1, channel);
	    }

/**
 * Configura o RTC para funcionar com interrupcao de alarme
 */
void RTC_init(){
    /* Configura o PMC */
    pmc_enable_periph_clk(ID_RTC);
        
    /* Default RTC configuration, 24-hour mode */
    rtc_set_hour_mode(RTC, 0);

    /* Configura data e hora manualmente */
    rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
    rtc_set_time(RTC, HOUR, MINUTE, SECOND);

    /* Configure RTC interrupts */
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 0);
    NVIC_EnableIRQ(RTC_IRQn);
    
    /* Ativa interrupcao via alarme */
    rtc_enable_interrupt(RTC,  RTC_IER_ALREN); 
    
}

//Inicializando os tr�s bot�es
//BOTAO1 inicializado
void but1_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BOTAO1_PIO_ID);
	pio_set_input(BUT1_PIO, BUT1_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrup��o */
	pio_enable_interrupt(BUT1_PIO, BUT1_PIN_MASK);
	pio_handler_set(BUT1_PIO, BOTAO1_PIO_ID, BUT1_PIN_MASK, PIO_IT_FALL_EDGE, but_Handler1); ;//aciona na borda de descida   PIO_IT_FALL_EDGE
	/* habilita interrup�c�o do PIO que controla o botao */
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
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrup��o */
	pio_enable_interrupt(BUT2_PIO, BUT2_PIN_MASK);
	pio_handler_set(BUT2_PIO, BOTAO2_PIO_ID, BUT2_PIN_MASK, PIO_IT_RISE_EDGE, but_Handler2);;//aciona na borda de subida &PIO_IT_RISE_EDGE
	/* habilita interrup�c�o do PIO que controla o botao */
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
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrup��o */
	pio_enable_interrupt(BUT3_PIO, BUT3_PIN_MASK);
	pio_handler_set(BUT3_PIO, BOTAO3_PIO_ID, BUT3_PIN_MASK, PIO_IT_RISE_EDGE, but_Handler3);//aciona na borda de subida PIO_IT_RISE_EDGE
	
	/* habilita interrup�c�o do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BOTAO3_PIO_ID);
	NVIC_SetPriority(BOTAO3_PIO_ID, 1);
};


/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

    /* Configura Leds */
    LED_init(0);

	
	
	/* Configura os bot�es */
	BUT_init(); 	
	but1_init();
	but2_init();
	but3_init();  
    
    /** Configura timer 0 */
    TC0_init();
	TC1_init();
    TC2_init();
	TC3_init();
	
    /** Configura RTC */
    RTC_init();

   /* configura alarme do RTC */    
   rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
   rtc_set_time_alarm(RTC, 1, HOUR, 1, MINUTE+1, 1, SECOND);

          
	while (1) {
		/* Entra em modo sleep */
		
	}
}

