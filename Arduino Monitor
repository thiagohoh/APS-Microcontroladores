#ifndef _DEF_PRINCIPAIS_H
#define _DEF_PRINCIPAIS_H

//Definições de macros para o trabalho com bits

#define set_bit(y,bit)  (y|=(1<<bit)) //coloca em 1 o bit x da variável Y
#define clr_bit(y,bit)  (y&=~(1<<bit))  //coloca em 0 o bit x da variável Y
#define cpl_bit(y,bit)  (y^=(1<<bit)) //troca o estado lógico do bit x da variável Y
#define tst_bit(y,bit)  (y&(1<<bit))  //retorna 0 ou 1 conforme leitura do bit
#define LED PB1
#define TOP 39999          
#endif

//def_principais.h

//USART.h

#ifndef _USART_H
#define _USART_H

//#include "def_principais.h"

#define BAUD   9600    //taxa de 9600 bps
#define MYUBRR  F_CPU/16/BAUD-1

#define tam_vetor 6 //número de digitos individuais para a conversão por ident_num()   
#define conv_ascii  48  //48 se ident_num() deve retornar um número no formato ASCII (0 para formato normal)

void USART_Inic(unsigned int ubbr0);
void USART_Transmite(unsigned char dado);
unsigned char USART_Recebe();
void escreve_USART(char *c);
void escreve_USART_Flash(const char *c);

void ident_num(unsigned int valor, unsigned char *disp);

#endif

//USART.h

//USART.c


//#include "USART.h"

//---------------------------------------------------------------------------
void USART_Inic(unsigned int ubrr0)
{
  UBRR0H = (unsigned char)(ubrr0>>8); //Ajusta a taxa de transmissão
  UBRR0L = (unsigned char)ubrr0;

  UCSR0A = 0;//desabilitar velocidade dupla (no Arduino é habilitado por padrão)
  UCSR0B = (1<<RXEN0)|(1<<TXEN0); //Habilita a transmissão e a recepção
  UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);/*modo assíncrono, 8 bits de dados, 1 bit de parada, sem paridade*/
}
//---------------------------------------------------------------------------
void USART_Transmite(unsigned char dado)
{
  while (!( UCSR0A & (1<<UDRE0)) ); //espera o dado ser enviado
  UDR0 = dado;          //envia o dado
}
//---------------------------------------------------------------------------
unsigned char USART_Recebe()
{
  while (!(UCSR0A & (1<<RXC0)));  //espera o dado ser recebido
  return UDR0;        //retorna o dado recebido
}
//---------------------------------------------------------------------------
void escreve_USART(char *c)   //escreve String (RAM)
{
   for (; *c!=0;c++) USART_Transmite(*c);
}
//---------------------------------------------------------------------------
void escreve_USART_Flash(const char *c) //escreve String (Flash)
{
   for (;pgm_read_byte(&(*c))!=0;c++) USART_Transmite(pgm_read_byte(&(*c)));
}
//--------------------------------------------------------------------------- 
//Conversão de um número em seus digitos individuais
//---------------------------------------------------------------------------
void ident_num(unsigned int valor, unsigned char *disp)
{   
  unsigned char n;

  for(n=0; n<tam_vetor; n++)
    disp[n] = 0 + conv_ascii; //limpa vetor para armazenagem dos digitos 

  do
  {
       *disp = (valor%10) + conv_ascii; //pega o resto da divisao por 10 
     valor /=10;            //pega o inteiro da divisão por 10
     disp++;

  }while (valor!=0);
}
//------------------------------------------------------------------------------

//USART.c

//LM35.c

//#include "def_principais.h"
//#include "USART.h"

const char msg1[] PROGMEM = "Sensor de Temperatura LM35 - canal 0\n\0";
const char msg2[] PROGMEM = "Potenciometro - canal 2\n\0";
const char msg3[] PROGMEM = "Sensor de Luz - canal 1\n\0";
const char msg4[] PROGMEM = "servo\n\0";
unsigned int temp;
unsigned char digitos[tam_vetor];
//--------------------------------------------------------------------------
int main()
{ 
  
  USART_Inic(MYUBRR);
  
  //configura ADC
  //ADMUX  = 0b11000000;  //Tensão interna de ref (1.1V), canal 0 
  //ADMUX =  (1<<REFS1)|(1<<REFS0); 
  
  DDRB  = 0b00000010;        //pino pb1 PB2  como saída
  PORTB = 0b11111101;       //zera saídas e habilita pull-ups nos pinos não utilizados  
  //ADCSRA = 0b10000111;  //habilita o AD, prescaler = 128
  ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
  
  
  
  while(1)
    {
      //escreve_USART_Flash(msg1);
      ident_num((unsigned int)le_temp(0),digitos);   //leitura de temperatura, sem sinal
      escreve_USART_Flash(msg1);
      USART_Transmite(digitos[3]);
      USART_Transmite(digitos[2]);
      USART_Transmite(digitos[1]);
      USART_Transmite(',');
      USART_Transmite(digitos[0]);
      USART_Transmite('*');//simbolo '*'
      USART_Transmite('C');
      USART_Transmite('\n');
      _delay_ms(1000);
   
    
    
    ident_num((unsigned int)le_luz(1),digitos);
   escreve_USART_Flash(msg3);
   USART_Transmite(digitos[4]);
   USART_Transmite(digitos[3]);
   USART_Transmite(digitos[2]);
   USART_Transmite(digitos[1]);
   USART_Transmite(digitos[0]);
    USART_Transmite('\n');
   _delay_ms(1000);
    
    
     ident_num((unsigned int)le_pote(2),digitos);
   escreve_USART_Flash(msg2);
   USART_Transmite(digitos[4]);
   USART_Transmite(digitos[3]);
     USART_Transmite(digitos[2]);
     USART_Transmite(digitos[1]);
     USART_Transmite(digitos[0]);
     USART_Transmite('\n');
     _delay_ms(1000);
    
     

    escreve_USART_Flash(msg4);
     servo_controler((unsigned int)le_pote(2));
 
  
    
    
    }
}
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------------
signed int le_temp(unsigned char canal)
{
 // ADMUX =  (1<<REFS1)|(1<<REFS0); 
  ADMUX = 0xF0; //Limpar o canal lido anteriormente
  ADMUX  = 0b11000000;
  ADMUX |= canal; //Define o novo canal a ser lido
  //ADCSRA |= (1<<ADSC);
  set_bit(ADCSRA, ADSC);                //inicia a conversão
  //while(ADCSRA & (1<<ADSC));
  while(tst_bit(ADCSRA,ADSC));          //espera a conversão ser finalizada
  return (ADC + (ADC*19)/256);             //fator k de divisão = 1
  
}

signed int le_pote(unsigned char canal){
  //ADMUX =  (0<<REFS1)|(0<<REFS0); 
  ADMUX = 0xF0; //Limpar o canal lido anteriormente
  
  //ADMUX  = 0b00000011;
  //ADMUX |= canal; //Define o novo canal a ser lido
  //ADMUX =  (0<<REFS1)|(0<<REFS0) | canal;
  //ADCSRA |= (1<<ADSC);
  ADMUX = canal;// seleciona o canal
  ADMUX = ADMUX|0x40;// seta voltagem para VCC padrao 5v
  set_bit(ADCSRA, ADSC);                //inicia a conversão
  //while(ADCSRA & (1<<ADSC));
  while(tst_bit(ADCSRA,ADSC));          //espera a conversão ser finalizada
  //  return (ADC + (ADC*19)/256);             //fator k de divisão = 1
  ADMUX  = 0b11000000;
  return ADC;
}
signed int le_luz(unsigned char canal){
  //ADMUX =  (1<<REFS1)|(1<<REFS0); 
  ADMUX = 0xF0; //Limpar o canal lido anteriormente
  //ADMUX  = 0b11000101;//seta a tensao certa  refs1 refs0 para o canal 1
  ADMUX = canal; //Define o novo canal a ser lido
  ADMUX = ADMUX|0x40;
  //ADMUX = 0b11110000;
  //ADMUX =  (1<<REFS1)|(1<<REFS0) | canal;
  //ADCSRA |= (1<<ADSC);
  set_bit(ADCSRA, ADSC);                //inicia a conversão
  //while(ADCSRA & (1<<ADSC));
  while(tst_bit(ADCSRA,ADSC));          //espera a conversão ser finalizada
  //  return (ADC + (ADC*19)/256);             //fator k de divisão = 1
  return ADC;
}




void servo_controler(unsigned int valor){
ADMUX = ADMUX|0x40;
  ICR1 = TOP;           //configura o período do PWM (20 ms)
  // Configura o TC1 para o modo PWM rápido via ICR1, prescaler = 8
  
  TCCR1A = (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1<<WGM12) | (1 << CS11);
  set_bit(TCCR1A,COM1A1);   //ativa o PWM no OC1B, modo de comparação não-invertido
  set_bit(TCCR1A,COM1B1);                //para desabilitar empregar clr_bit(TCCR1A, COM1A1)
  
 
  if((valor >= 0) && (valor < 204)){
    
     OCR1A = 934; //0 grau
     _delay_ms(2000); 
  }
  
   if((valor >= 205) &&( valor < 409)){
    
     OCR1A = 2100; //45 graus
     _delay_ms(2000); 
  }
  
  if((valor >= 410) && (valor < 614)){
    
     OCR1A = 3000; //90 graus ajustar
     _delay_ms(2000); 
  }
  
  if((valor >= 615) && (valor < 819)){
    
    OCR1A = 4000; //135 grus ajustar
     _delay_ms(2000); 
  }
  
   if((valor >= 820) && (valor <= 1023)){
    
    OCR1A = 4500; //180 graus ajustar
     _delay_ms(2000); 
  }
  
  
  
}
