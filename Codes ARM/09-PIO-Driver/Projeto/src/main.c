/************************************************************************
* Rafael Corsi   - Insper
* rafael.corsi@insper.edu.br
*
* Computa��o Embarcada
*
* 09-PIO-DRIVER
************************************************************************/


#include "asf.h"
#include "conf_clock.h"
//Importando os Headers da pasta Driver
#include "Driver/pmc_insper.h"
#include "Driver/pio_insper.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		    8
#define LED_PIN_MASK    (1<<LED_PIN)

/**
 * Bot�o
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

/************************************************************************/
/* prototype                                                             */
/************************************************************************/
void led_init(int estado);
void but_init(void);

/************************************************************************/
/* Fun��es	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void led_init(int estado){
	_pmc_enable_periph_clock(LED_PIO_ID);	    // Ativa clock do perif�rico no PMC(PMC->PMC_PCER0    = (1<<LED_PIO_ID);)
	_pio_set_output(LED_PIO_ID,LED_PIN_MASK,0, 1); // Ativa sa�da  
	//C�digo original
	/*PMC->PMC_PCER0    = (1<<LED_PIO_ID);	    // Ativa clock do perif�rico no PMC
	LED_PIO->PIO_PER  = LED_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LED_PIO->PIO_OER  = LED_PIN_MASK;           // Ativa sa�da                      (Output ENABLE register)
    if(!estado)                                 // Checa pela inicializa��o desejada
	    LED_PIO->PIO_CODR = LED_PIN_MASK;       // Coloca 0 na sa�da                (CLEAR Output Data register)
    else
        LED_PIO->PIO_SODR = LED_PIN_MASK;       // Coloca 1 na sa�da  (SET Output Data register)
		*/
}

/**
 * @Brief Inicializa o pino do BUT
 */
void but_init(void){
	_pmc_enable_periph_clock(BUT_PIO_ID);     // Ativa clock do perif�rico no PMC(PMC->PMC_PCER0       = (1<<BUT_PIO_ID);)
	_pio_set_input(BUT_PIO, BUT_PIN_MASK, 0);
	_pio_pull_up(BUT_PIO, BUT_PIN_MASK, 1);
	
	//C�digo original
	/*PMC->PMC_PCER0       = (1<<BUT_PIO_ID);     // Ativa clock do perif�rico no PMC
	BUT_PIO->PIO_ODR	 = BUT_PIN_MASK;        // Desativa sa�da                   (Output DISABLE register)
	BUT_PIO->PIO_PER	 = BUT_PIN_MASK;        // Ativa controle do pino no PIO    (PIO ENABLE register)
	BUT_PIO->PIO_PUER	 = BUT_PIN_MASK;        // Ativa pull-up no PIO             (PullUp ENABLE register)
	BUT_PIO->PIO_IFER	 = BUT_PIN_MASK;        // Ativa debouncing
	BUT_PIO->PIO_IFSCER  = BUT_PIN_MASK;        // Ativa clock periferico
	BUT_PIO->PIO_SCDR	 = BUT_DEBOUNCING_VALUE;// Configura a frequencia do debouncing
};



/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{
	/************************************************************************/
	/* Inicializa��o b�sica do uC                                           */
	/************************************************************************/

	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;

	/************************************************************************/
	/* Inicializao I/OS                                                     */
	/************************************************************************/

	led_init(1);
    but_init();

	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/

	while(1){
		/**
		* @Brief Verifica constantemente o status do bot�o
		* 1 : n�o apertado
		* 0 : apertado
		*/
	   	   
		if(BUT_PIO->PIO_PDSR & (BUT_PIN_MASK)){
			_pio_set(BUT_PIO, BUT_PIN_MASK);
			} else {
			_pio_clear(BUT_PIO, BUT_PIN_MASK);
		}
	   
	    //C�digo original
		/*if(BUT_PIO->PIO_PDSR & (BUT_PIN_MASK)){
			LED_PIO->PIO_CODR = LED_PIN_MASK;
        }
		else{
			LED_PIO->PIO_SODR = LED_PIN_MASK;
        }*/
		}
	}
}


