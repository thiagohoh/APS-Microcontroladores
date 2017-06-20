


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
#define BAUD   9600    //taxa de 9600 bps
#define MYUBRR  F_CPU/16/BAUD-1
#define tam_vetor 6 //número de digitos individuais para a conversão por ident_num()
#define conv_ascii  48  //48 se ident_num() deve retornar um número no formato ASCII (0 para formato normal)

void USART_Inic(unsigned int ubbr0);
void USART_Transmite(unsigned char dado);
unsigned char USART_Recebe();
void escreve_USART(char *c);
void escreve_USART_Flash(const char *c);

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

#endif

void USART_Inic(unsigned int ubrr0) {
	UBRR0H = (unsigned char) (ubrr0 >> 8); //Ajusta a taxa de transmissão
	UBRR0L = (unsigned char) ubrr0;

	UCSR0A = 0; //desabilitar velocidade dupla (no Arduino é habilitado por padrão)
	UCSR0B = (1 << RXEN0) | (1 << TXEN0); //Habilita a transmissão e a recepção
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);/*modo assíncrono, 8 bits de dados, 1 bit de parada, sem paridade*/
}
//---------------------------------------------------------------------------
void USART_Transmite(unsigned char dado) {
	while (!(UCSR0A & (1 << UDRE0)))
		;
	//espera o dado ser enviado
	UDR0 = dado;          //envia o dado
}
//---------------------------------------------------------------------------
unsigned char USART_Recebe() {
	while (!(UCSR0A & (1 << RXC0)))
		;
	//espera o dado ser recebido
	return UDR0;        //retorna o dado recebido
}
//---------------------------------------------------------------------------
void escreve_USART(char *c)   //escreve String (RAM)
{
	for (; *c != 0; c++)
		USART_Transmite(*c);
}
//---------------------------------------------------------------------------
void escreve_USART_Flash(const char *c) //escreve String (Flash)
{
	for (; pgm_read_byte(&(*c)) != 0; c++)
		USART_Transmite(pgm_read_byte(&(*c)));
}
//---------------------------------------------------------------------------
//Conversão de um número em seus digitos individuais
//---------------------------------------------------------------------------
void ident_num(unsigned int valor, unsigned char *disp) {
	unsigned char n;

	for (n = 0; n < tam_vetor; n++)
		disp[n] = 0 + conv_ascii; //limpa vetor para armazenagem dos digitos

	do {
		*disp = (valor % 10) + conv_ascii; //pega o resto da divisao por 10
		valor /= 10;            //pega o inteiro da divisão por 10
		disp++;

	} while (valor != 0);
}
// Sub-rotina para enviar caracteres e comandos ao LCD com via de dados de 4 bits
//---------------------------------------------------------------------------------------------
void cmd_LCD(unsigned char c, char cd) //c é o dado  e cd indica se é instrução ou caractere
{
  if (cd == 0)
    clr_bit(CONTR_LCD, RS);
  else
    set_bit(CONTR_LCD, RS);

  //primeiro nibble de dados - 4 MSB
#if (nibble_dados)                //compila código para os pinos de dados do LCD nos 4 MSB do PORT
  DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & c);
#else                     //compila código para os pinos de dados do LCD nos 4 LSB do PORT
  DADOS_LCD = (DADOS_LCD & 0xF0) | (c >> 4);
#endif

  pulso_enable()
  ;

  //segundo nibble de dados - 4 LSB
#if (nibble_dados)                //compila código para os pinos de dados do LCD nos 4 MSB do PORT
  DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & (c << 4));
#else                     //compila código para os pinos de dados do LCD nos 4 LSB do PORT
  DADOS_LCD = (DADOS_LCD & 0xF0) | (0x0F & c);
#endif

  pulso_enable()
  ;

  if ((cd == 0) && (c < 4)) //se for instrução de retorno ou limpeza espera LCD estar pronto
    _delay_ms(2);
}
//---------------------------------------------------------------------------------------------
//Sub-rotina para inicialização do LCD com via de dados de 4 bits
//---------------------------------------------------------------------------------------------
void inic_LCD_4bits() //sequência ditada pelo fabricando do circuito integrado HD44780
{ //o LCD será só escrito. Então, R/W é sempre zero.

  clr_bit(CONTR_LCD, RS); //RS em zero indicando que o dado para o LCD será uma instrução
  clr_bit(CONTR_LCD, E); //pino de habilitação em zero

  _delay_ms(20); //tempo para estabilizar a tensão do LCD, após VCC ultrapassar 4.5 V (na prática pode
  //ser maior).
  //interface de 8 bits
#if (nibble_dados)
  DADOS_LCD = (DADOS_LCD & 0x0F) | 0x30;
#else
  DADOS_LCD = (DADOS_LCD & 0xF0) | 0x03;
#endif

  pulso_enable()
  ;     //habilitação respeitando os tempos de resposta do LCD
  _delay_ms(5);
  pulso_enable()
  ;
  _delay_us(200);
  pulso_enable()
  ; /*até aqui ainda é uma interface de 8 bits.
   Muitos programadores desprezam os comandos acima, respeitando apenas o tempo de
   estabilização da tensão (geralmente funciona). Se o LCD não for inicializado primeiro no
   modo de 8 bits, haverá problemas se o microcontrolador for inicializado e o display já o tiver sido.*/

  //interface de 4 bits, deve ser enviado duas vezes (a outra está abaixo)
#if (nibble_dados)
  DADOS_LCD = (DADOS_LCD & 0x0F) | 0x20;
#else
  DADOS_LCD = (DADOS_LCD & 0xF0) | 0x02;
#endif

  pulso_enable()
  ;
  cmd_LCD(0x28, 0); //interface de 4 bits 2 linhas (aqui se habilita as 2 linhas)
  //são enviados os 2 nibbles (0x2 e 0x8)
  cmd_LCD(0x08, 0);    //desliga o display
  cmd_LCD(0x01, 0);    //limpa todo o display
  cmd_LCD(0x0C, 0);    //mensagem aparente cursor inativo não piscando
  cmd_LCD(0x02, 0);
  cmd_LCD(0x80, 0); //inicializa cursor na primeira posição a esquerda - 1a linha
}
//---------------------------------------------------------------------------------------------
//Sub-rotina de escrita no LCD -  dados armazenados na RAM
//---------------------------------------------------------------------------------------------
void escreve_LCD(char *c) {
  for (; *c != 0; c++)
    cmd_LCD(*c, 1);
}
//---------------------------------------------------------------------------------------------
//Sub-rotina de escrita no LCD - dados armazenados na FLASH
//---------------------------------------------------------------------------------------------
void escreve_LCD_Flash(const char *c) {
  for (; pgm_read_byte(&(*c)) != 0; c++)
    cmd_LCD(pgm_read_byte(&(*c)), 1);
}
//---------------------------------------------------------------------------------------------
//Conversão de um número em seus digitos individuais
//---------------------------------------------------------------------------------------------
void ident_num(unsigned int valor, unsigned char *disp) {
  unsigned char n;

  for (n = 0; n < tam_vetor; n++)
    disp[n] = 0 + conv_ascii;   //limpa vetor para armazenagem do digitos

  do {
    *disp = (valor % 10) + conv_ascii; //pega o resto da divisao por 10
    valor /= 10;            //pega o inteiro da divisão por 10
    disp++;

  } while (valor != 0);
}

const unsigned char msg[] PROGMEM = "OPA";
volatile unsigned char flag = 0;
//--------------------------------------------------------------------------------------

ISR(PCINT0_vect);
int main() {
  unsigned char k;

  DDRD = 0xFF;          //PORTD como saída
  //DDRB = 0xFF;          //PORTB como saída
  DDRB = 0b11000011;  // pb0 e pb1 como saida o resto como entrada
  PORTB = 0b00111100; // pullup
  PCICR = 1 << PCIE0; // habilita porb
  PCMSK0 = (1 << PCINT2) | (1 << PCINT3) | (1 << PCINT4) | (1 << PCINT5);//habilita os pinos
  sei();

  inic_LCD_4bits();       //inicializa o LCD

  cmd_LCD(0x80, 0);     //endereça a posição para escrita dos caracteres
  //cmd_LCD(0x00,1);        //apresenta primeiro caractere 0x00
  //cmd_LCD(0x01,1);        //apresenta segundo  caractere 0x01
  escreve_LCD("a");
  _delay_ms(200);
	
  for (;;) {



    if (flag) {

      if (!tst_bit(PINB, PB2)) {
	escreve_LCD("?");
	
        flag=0;
        
      } else if (!tst_bit(PINB, PB3)) {
        escreve_LCD("L");
	
        flag=0;
       
      } else if (!tst_bit(PINB, PB4)) {
        escreve_LCD("O");
	
        flag=0;
        
      } else if (!tst_bit(PINB, PB5)) {
        escreve_LCD("!");
	
        flag=0;
        
      }

      
      
    }
  }

}

ISR(PCINT0_vect) {
  flag = 1;
  _delay_ms(200);
}

