/**
 * Computaçao Embarcada
 * Mutirão C
 * Aula 1
 * Rafael Corsi e Fábio Ayres
 *
 */

#include <stdio.h>

void main(){
    int PV;
    int i;
    int n;
    printf("Qual Valor Presente?");
    scanf(%d, PV);
    printf("Qual a Taxa de Juros?");
    scanf(%d, i);
    printf("Qual Numer de Periodos?");
    scanf(%d, n);

    FV = PV * (1+i)**n;
    
	printf("Valor Futuro:" FV);
}
	
