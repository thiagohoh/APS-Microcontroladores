/*
 * interface.c
 *
 *  Created on: 6 de jun de 2017
 *      Author: Thiago
 */


#ifndef _DEF_PRINCIPAIS_H
#define _DEF_PRINCIPAIS_H
//Definições de macros para o trabalho com bits

#define set_bit(y,bit)  (y|=(1<<bit)) //coloca em 1 o bit x da variável Y
#define clr_bit(y,bit)  (y&=~(1<<bit))  //coloca em 0 o bit x da variável Y
#define cpl_bit(y,bit)  (y^=(1<<bit)) //troca o estado lógico do bit x da variável Y
#define tst_bit(y,bit)  (y&(1<<bit))  //retorna 0 ou 1 conforme leitura do bit

#endif


#ifndef _LCD_H
#define _LCD_H

//#include "def_principais.h"

//Definições para facilitar a troca dos pinos do hardware e facilitar a re-programação

#define DADOS_LCD      PORTD   //4 bits de dados do LCD no PORTD
#define nibble_dados  1   //0 para via de dados do LCD nos 4 LSBs do PORT empregado (Px0-D4, Px1-D5, Px2-D6, Px3-D7)
                //1 para via de dados do LCD nos 4 MSBs do PORT empregado (Px4-D4, Px5-D5, Px6-D6, Px7-D7)
#define CONTR_LCD     PORTB   //PORT com os pinos de controle do LCD (pino R/W em 0).
#define E         PB1     //pino de habilitação do LCD (enable)
#define RS        PB0     //pino para informar se o dado é uma instrução ou caractere

#define tam_vetor 5 //número de digitos individuais para a conversão por ident_num()
#define conv_ascii  48  //48 se ident_num() deve retornar um número no formato ASCII (0 para formato normal)

//sinal de habilitação para o LCD
#define pulso_enable()  _delay_us(1); set_bit(CONTR_LCD,E); _delay_us(1); clr_bit(CONTR_LCD,E); _delay_us(45)

//protótipo das funções
void cmd_LCD(unsigned char c, char cd);
void inic_LCD_4bits();
void escreve_LCD(char *c);
void escreve_LCD_Flash(const char *c);

void ident_num(unsigned int valor, unsigned char *disp);
const unsigned char msg1[] PROGMEM = "*";
#endif

//---------------------------------------------------------------------------------------------
// Sub-rotina para enviar caracteres e comandos ao LCD com via de dados de 4 bits
//---------------------------------------------------------------------------------------------
void cmd_LCD(unsigned char c, char cd){      //c é o dado  e cd indica se é instrução ou caractere

  if(cd==0)
    clr_bit(CONTR_LCD,RS);
  else
    set_bit(CONTR_LCD,RS);

  //primeiro nibble de dados - 4 MSB
  #if (nibble_dados)                //compila código para os pinos de dados do LCD nos 4 MSB do PORT
    DADOS_LCD = (DADOS_LCD & 0x0F)|(0xF0 & c);
  #else                     //compila código para os pinos de dados do LCD nos 4 LSB do PORT
    DADOS_LCD = (DADOS_LCD & 0xF0)|(c>>4);
  #endif

  pulso_enable();

  //segundo nibble de dados - 4 LSB
  #if (nibble_dados)                //compila código para os pinos de dados do LCD nos 4 MSB do PORT
    DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & (c<<4));
  #else                     //compila código para os pinos de dados do LCD nos 4 LSB do PORT
    DADOS_LCD = (DADOS_LCD & 0xF0) | (0x0F & c);
  #endif

  pulso_enable();

  if((cd==0) && (c<4))        //se for instrução de retorno ou limpeza espera LCD estar pronto
    _delay_ms(2);
}
//---------------------------------------------------------------------------------------------
//Sub-rotina para inicialização do LCD com via de dados de 4 bits
//---------------------------------------------------------------------------------------------
void inic_LCD_4bits(){   //sequência ditada pelo fabricando do circuito integrado HD44780
            //o LCD será só escrito. Então, R/W é sempre zero.

  clr_bit(CONTR_LCD,RS);  //RS em zero indicando que o dado para o LCD será uma instrução
  clr_bit(CONTR_LCD,E); //pino de habilitação em zero

  _delay_ms(20);      //tempo para estabilizar a tensão do LCD, após VCC ultrapassar 4.5 V (na prática pode
              //ser maior).
  //interface de 8 bits
  #if (nibble_dados)
    DADOS_LCD = (DADOS_LCD & 0x0F) | 0x30;
  #else
    DADOS_LCD = (DADOS_LCD & 0xF0) | 0x03;
  #endif

  pulso_enable();     //habilitação respeitando os tempos de resposta do LCD
  _delay_ms(5);
  pulso_enable();
  _delay_us(200);
  pulso_enable();

  //interface de 4 bits, deve ser enviado duas vezes (a outra está abaixo)
  #if (nibble_dados)
    DADOS_LCD = (DADOS_LCD & 0x0F) | 0x20;
  #else
    DADOS_LCD = (DADOS_LCD & 0xF0) | 0x02;
  #endif

  pulso_enable();
    cmd_LCD(0x28,0);    //interface de 4 bits 2 linhas (aqui se habilita as 2 linhas)
              //são enviados os 2 nibbles (0x2 e 0x8)
    cmd_LCD(0x08,0);    //desliga o display
    cmd_LCD(0x01,0);    //limpa tod o display
    cmd_LCD(0x0C,0);    //mensagem aparente cursor inativo não piscando
    cmd_LCD(0x80,0);    //inicializa cursor na primeira posição a esquerda - 1a linha
}
//---------------------------------------------------------------------------------------------
//Sub-rotina de escrita no LCD -  dados armazenados na RAM
//---------------------------------------------------------------------------------------------
void escreve_LCD(char *c){
   for (; *c!=0;c++) cmd_LCD(*c,1);
}
//---------------------------------------------------------------------------------------------
//Sub-rotina de escrita no LCD - dados armazenados na FLASH
//---------------------------------------------------------------------------------------------
void escreve_LCD_Flash(const char *c){
   for (;pgm_read_byte(&(*c))!=0;c++) cmd_LCD(pgm_read_byte(&(*c)),1);
}
//---------------------------------------------------------------------------------------------
//Conversão de um número em seus digitos individuais
//---------------------------------------------------------------------------------------------
void ident_num(unsigned int valor, unsigned char *disp){
  unsigned char n;

  for(n=0; n<tam_vetor; n++)
    disp[n] = 0 + conv_ascii;   //limpa vetor para armazenagem do digitos

  do
  {
       *disp = (valor%10) + conv_ascii; //pega o resto da divisao por 10
     valor /=10;            //pega o inteiro da divisão por 10
     disp++;

  }while (valor!=0);
}


int main(){
	unsigned char digitos[tam_vetor];	//declaração da variável para armazenagem dos digitos
    //unsigned int cont;
  	unsigned char cont;
    unsigned char i;
	unsigned char jota;
	unsigned char epa= 0;

	DDRD = 0xFF;						//PORTD como saída
	DDRB = 0xFF;

	inic_LCD_4bits();					//inicializa o LCD

  for(i=0;i<1;i++){	//enviando caractere por caractere
		cmd_LCD(pgm_read_byte(&msg1[i]),1);	//l na memria flash e usa cmd_LCD
    	_delay_us(600);
	}
  cmd_LCD(0xC0,0);

	for(;;){

		while(epa<15){
			cmd_LCD(0x1c,0);
			_delay_us(1200);
			epa++;
		}

		while(epa>1){
			cmd_LCD(0x18,0);
			_delay_us(1200);
			epa--;
		}


			//la�o infinito
	}

}
