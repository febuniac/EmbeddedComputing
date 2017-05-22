/**************************************************************************
* Rafael Corsi   - Insper 
* rafael.corsi@insper.edu.br        
*
* Computação Embarcada
*
* 08-PIO-ENTRADA
************************************************************************/


#include "asf.h"
#include "conf_clock.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC 
#define LED_PIO         PIOC
#define LED_PIN			8
#define LED_PIN_MASK	(1<<LED_PIN) 

/**
 * Botão
 */ 
#define BUT_PIO_ID	 10    //ID
#define BUT_PIO		 PIOA //PIO   
#define BUT_PIN		 11	 //PINO
#define BUT_PIN_MASK (1 << BUT_PIN)//criando a mascara	

/************************************************************************/
/* Prototipação                                                        */
/************************************************************************/
void ledConfig();

/************************************************************************/
/* Funções	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
//Mesma configuração da aula 7
void ledConfig(){
		PMC->PMC_PCER0 = (1<<LED_PIO_ID);
		PIOC->PIO_OER  = (1 << 8);
		PIOC->PIO_PER  = (1 << 8);
		PIOC->PIO_CODR = (1 << 8);
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
	/* Inicializa perifericos                                               */
	/************************************************************************/
	// Configura LED em modo saída
	ledConfig();

	// Configura botao
		PMC->PMC_PCER0= (1<<10);
		PIOA->PIO_PER = (1<<11);
		PIOA->PIO_ODR = (1<<11);
		PIOA->PIO_PUER= (1<<11);
		PIOA->PIO_IFER= (1<<11);

	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
		int estado =0;
		int i =0;
		if( PIOA->PIO_PDSR & (1<<11)){
			PIOC->PIO_SODR = (1 << 8);
			
		}
		else{
			PIOC->PIO_CODR = (1 << 8);
			}
	}
}
		/*if (estado){
			
			if(i > 100000000){
				if(estado){
					PIOC->PIO_CODR = (1 << 8);
					estado = 0;
					}else{
					PIOC->PIO_SODR = (1 << 8);
					estado = 1;
				}
				i = 0;
			}
			i++;
		}
	}*/


