/**
 * Computaçao Embarcada
 * Mutirão C
 * Aula 1
 * Rafael Corsi e Fábio Ayres
 * Entrega do Exercicio 4 do Aluno Felipe Frid Buniac
 */

#include <stdio.h>
#include <math.h>

void main(){
    int numprodutos;
    char nome[100];
    int preco;
    int quantidade;
    int precototal;
    char nomes[50][100];
    int precos[50];
    int quantidades[50];
    int precostotais[50];
    int total =0 ;


    
    numprodutos = 0;
    

    printf("Quantos Produtos tem sua lista?");
    scanf("%d", &numprodutos);
    
    for (int i=1; i<=numprodutos ;i++){


    printf("Qual Nome do Produto?");
    scanf("%s", nomes[i-1]);

    printf("Qual a preco do Produto?");
    scanf("%d", &precos[i-1]);

    printf("Qual a Quantidade do Produto?");
    scanf("%d", &quantidades[i-1]);

    precostotais[i-1] = ((precos[i-1]) * (quantidades[i-1]));
    
    total += precostotais[i-1];

    
}
    for(int i =1; i<=numprodutos; i++){
    printf("Nome:%s ", nomes[i-1]);
    printf("Quantidade:%d ", quantidades[i-1]);
    printf("Preco:%d ", precos[i-1]);
    printf("Preco Total:%d \n " , precostotais[i-1]);
}
printf("Preco Total dos Produtos:%d \n" , total);
}
	