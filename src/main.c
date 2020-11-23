/*******************************************
 *  main.c
 *  Robot Seguidor de linha
 *  Created on: 11/2019
 *      Author: Tiago Vale e Anastasia Bezpalaya
 ********************************************/
#include <avr/io.h>        /* Register defs */
#include <avr/interrupt.h>
#include "serial_printf.h"
#include <avr/eeprom.h>
#include "LCD_AVR.h"
//#include "LiquidCrystal.h"

#define SENSOR2 PC0           /* Sensor 2 no pino PD0 */
#define SENSOR1 PC1           /* Sensor 1 no pino PD1 */
#define SENSOR0 PC2           /* Sensor 0 no pino PD2 */
#define SENSOR01 PC3          /* Sensor 01 no pino PD3 */
#define SENSOR02 PC4          /* Sensor 02 no pino PD4 */
#define SENSORIR PC5
#define MOTOR_ESQ PD5         // Saída Motor Esquerdo - OC0B
#define MOTOR_DIR PD6         // Saída Motor Direito - OC0A
#define BOT_LAP PD7
#define BOT_BAT PD4
#define LED_VERDE PB0
#define LED_AMAR PB4
#define LED_VERM PB5
#define AIN1 PD3 //Atrás
#define AIN2 PD2 //Frente
#define BIN1 PD7 //Trás
#define BIN2 PD4 //Frente
#define BOT_DESL 20 //Código do Botão Desligar IR
#define BOT_SEC 21 //Código de outro botão que quisermos configurar para dar reset ao número de voltas

volatile uint16_t sensor0 = 0, sensor1 = 0, sensor2 = 0, sensor3 = 0, sensor4 = 0, T1 = 0, T2 = 0, T3 = 0, IRsens;
volatile uint8_t nr_sensor = 0;
volatile uint8_t timer_pwm1 = 0;
volatile uint8_t timer_pwm2 = 0;
int8_t erro = 0, Kp=0, Ki=0, Kd=0, P, I, D, prev_erro, PID,mot_esq=0, mot_dir=0, curr_battery=0, curr_t_battery=0, laps=0, bat_display=0, stateLCD=0, stateLCD_V=0, voltas_display=0, stateCalibrate=0;
int8_t link=0, stateIR=0, aux=0, stateIR2=0;
uint16_t Vref=0;
uint16_t Vref0=0,Vref1=0,Vref2=0,Vref3=0,Vref4=0, Vrefmax0=0, Vrefmax1=0, Vrefmax2=0, Vrefmax3=0, Vrefmax4=0, Vrefmin0=0, Vrefmin1=0, Vrefmin2=0, Vrefmin3=0, Vrefmin4=0;
uint8_t EEMEM battery = 100;
uint8_t EEMEM t_battery = 0;
uint8_t EEMEM Vrefm0 = 0, Vrefm1=0, Vrefm2=0, Vrefm3=0, Vrefm4=0;
char text1[100]= "Bateria: ";
char text2[100]="";
char text3[100]="";
char text4[100]="";
char text5[100]="";


void io_init(void)
{
  DDRD |= (1 << MOTOR_ESQ) | (1 << MOTOR_DIR) | (1 << AIN1) | (1 << AIN2)|(1 << BIN1)| (1 << BIN2); //Atuadores como saídas
  PORTD &= ~((1 << MOTOR_ESQ) | (1 << MOTOR_DIR) | (1 << AIN1) | (1 << AIN2) | (1 << BIN1)| (1 << BIN2)| (1 << AIN1) | (1 << AIN2)|(1 << BIN1)| (1 << BIN2)); //MOTOR_ESQ e MOTOR_DIR start low
  /*DDRB |= (1 << BIN1)| (1 << BIN2)| (1 << LED_VERDE)| (1 << LED_AMAR)| (1 << LED_VERM);*/
  /*PORTB &= ~((1 << BIN1)| (1 << BIN2)| (1 << LED_VERDE)| (1 << LED_AMAR)| (1 << LED_VERM));*/
  
  DDRC &= ~((1 << SENSOR2)|(1 << SENSOR1)|(1 << SENSOR0)|(1 << SENSOR01)|(1 << SENSOR02)|(1 << SENSORIR)/*|(1<<BOT_LAP)|(1<<BOT_BAT)*/); // Sensor como input 
  PORTC |= (1 << SENSOR2)|(1 << SENSOR1)|(1 << SENSOR0)|(1 << SENSOR01)|(1 << SENSOR02)|(1 << SENSORIR)/*|(1<<BOT_LAP)|(1<<BOT_BAT)*/; // Pullup in SWITCH_PIN 
}

void init_timer1(void){  //Timers com 100ms
    // Para o timer1 e limpa os interrupts
    TCCR1B = 0;
    TIFR1 |= (7 << TOV1);
    //Modo CTC
    TCCR1A = 0;
    TCCR1B = (1 << WGM12);
    //Definir BOTTOM e TOP 
    TCNT1 = 0;
    OCR1A = 6250;
    //Ativar compare interrupts
    TIMSK1 = 0;
    TIMSK1 = (1 << OCIE1A);
    //Prescaler de 256
    TCCR1B |= (1 << CS12);

    //Ativar interrupts
    sei();
}

ISR(TIMER1_COMPA_vect){
  T1++;
  T2++;
  T3++;
}


void init_timerPWM(void){
  //Modo CTC, Compare // Modo FAST PWM
  TCCR0B = 0;
  TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00) | (1 << WGM01);
  TIMSK0 = 0; 

  sei();

  //Inicia Timer
  TCCR0B = (1 << CS01);
  
}

//Função que faz as configurações globais do módulo ADC
void init_ADC()
{
  sei();
   //Vref = AREF, deslocado para direita
  ADMUX =  ( 1 << REFS0 );
   
  ADCSRA = ( 1 << ADEN ) | (1 << ADIE) | ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 ) | (1 << ADSC);

  DIDR0 = (1 << ADC0D) | (1 << ADC1D) | (1 << ADC2D) | (1 << ADC3D) | (1 << ADC4D);

}


//Interrupt que executa sempre que acaba o ultimo ADC, lê cada sensor (5sensores + 1 sensor IR) de cada vez
ISR(ADC_vect)
{ 
  if(nr_sensor==0){
    sensor0 = ADCL | (ADCH << 8);
    nr_sensor++;
    ADMUX |= (1 << MUX0);
    ADCSRA |= (1 << ADSC);
  }
  else if(nr_sensor==1){
    sensor1 = ADCL | (ADCH << 8);
    nr_sensor++;
    ADMUX &= ~(1 << MUX0);
    ADMUX |= (1 << MUX1);
    ADCSRA |= (1 << ADSC);  
  }
  else if(nr_sensor==2){
    sensor2 = ADCL | (ADCH << 8);
    nr_sensor++;
    ADMUX |= (1 << MUX0);
    ADCSRA |= (1 << ADSC);
  }
  else if(nr_sensor==3){
    sensor3 = ADCL | (ADCH << 8);
    nr_sensor++;
    ADMUX |= (1 << MUX2);
    ADMUX &= ~(1 << MUX0);
    ADMUX &= ~(1 << MUX1);
    ADCSRA |= (1 << ADSC);
  }
  else if(nr_sensor==4){
    sensor4 = ADCL | (ADCH << 8);
    nr_sensor++;
    ADMUX |= (1 << MUX0);
    ADCSRA |= (1 << ADSC);
  }
  else if(nr_sensor==5){
    IRsens = ADCL | (ADCH << 8);
    ADMUX &= ~(1 << MUX0);
    ADMUX &= ~(1 << MUX1);
    ADMUX &= ~(1 << MUX2);
    ADMUX &= ~(1 << MUX3);
    nr_sensor=0;
    ADCSRA |= (1 << ADSC);
  }
}

/*
void calibrateSensor(){
  //Usar botões ou funções do comando para alternar entre as funções de calibração
  
  //Colocar Na supefície Branca
  if(stateCalibrate==1){
    Vrefmax0 = sensor0;
    Vrefmax1 = sensor1;
    Vrefmax2 = sensor2;
    Vrefmax3 = sensor3;
    Vrefmax4 = sensor4;
  }

  //Colocar na superfície Preta
  if(stateCalibrate==2){
    Vrefmin0 = sensor0;
    Vrefmin1 = sensor1;
    Vrefmin2 = sensor2;
    Vrefmin3 = sensor3;
    Vrefmin4 = sensor4;
  }

  //Com a calibração acabada:
  if(stateCalibrate==3){

    //Calcular os valores de referência
    Vref0 = Vrefmin0 + ((Vrefmax0 - Vrefmin0)/2);
    Vref1 = Vrefmin1 + ((Vrefmax1 - Vrefmin1)/2);
    Vref2 = Vrefmin2 + ((Vrefmax2 - Vrefmin2)/2);
    Vref3 = Vrefmin3 + ((Vrefmax3 - Vrefmin3)/2);
    Vref4 = Vrefmin4 + ((Vrefmax4 - Vrefmin4)/2);

    //Guardar na eprom os valores de referências dos sensores, para podermos usar estes valores mesmo depois de desligar
    //Cada sensor tem m valor de referência, aumentando a precisão
    eeprom_update_byte(&Vrefm0,Vref0);
    eeprom_update_byte(&Vrefm1,Vref1);
    eeprom_update_byte(&Vrefm2,Vref2);
    eeprom_update_byte(&Vrefm3,Vref3);
    eeprom_update_byte(&Vrefm4,Vref4);

  }
}
*/

//Função que calcula o PID consoante o erro
void calculatePID(){
   P = erro;

   I = I + erro;

   D = erro - prev_erro;

   PID = (Kp * P) + (Ki * I) + (Kd * D);

   prev_erro = erro;
}

//Função que retorna 1 se o sensor estiver na linha ou 0 se fora desta / <450 Preto >=Branco
int ISsensor(int i)
{
  if(i==0){
    if(sensor0 >= Vref)  return 0;
    else if(sensor0 < Vref) return 1;
  }
  else if(i==1){
    if(sensor1 >= Vref)  return 0;
    else if(sensor1 < Vref) return 1;
  }
  else if(i==2){
    if(sensor2 >= Vref)  return 0;
    else if(sensor2 < Vref) return 1;
  }
  else if(i==3){
    if(sensor3 >= Vref)  return 0;
    else if(sensor3 < Vref) return 1;
  }
  else if(i==4){
    if(sensor4 >= Vref)  return 0;
    else if(sensor4 < Vref) return 1;
  }
}

/*
int ISsensorIR(){
  if(!(PINC & (1<<5))){
    return 1;
  }
  else return 0;
}
*/

uint8_t isButtonLAP(){
  if ( !(PIND & (1<<7))) 
    return 1;
  else return 0;
}

uint8_t isButtonBAT(){
  if ( !(PIND & (1<<4))) 
    return 1;
  else return 0;
}

/* 
void set_LEDVERDE(uint8_t on){
  if (on) {
    PORTB |= (1<<0);
  }
  else{
    PORTB &= ~(1<<0);
  }
}
void set_LEDAMAR(uint8_t on){
  if (on) {
    PORTB |= (1<<4);
  }
  else{
    PORTB &= ~(1<<4);
  }
}
void set_LEDVERM(uint8_t on){
  if (on) {
    PORTB |= (1<<5);
  }
  else{
    PORTB &= ~(1<<5);
  }
}
*/

//Calcula a bateria e dá update à EEPROM - A cada 1min com os motores ligados perde 1% de bateria
//A cada 5seg incrementamos uma variavel. Quando essa variavel = 12 (60s) decrementa 1% da bateria na eeprom
void calculateBattery(){
  if(eeprom_read_byte(&battery)!=curr_battery){
    curr_battery = eeprom_read_byte(&battery);
  }
  if(mot_esq>0 && mot_dir>0){
    curr_t_battery = eeprom_read_byte(&t_battery);
  
    if(curr_t_battery>=12){
      curr_battery = eeprom_read_byte(&battery);
      curr_battery--;
      eeprom_update_byte(&battery,curr_battery);
      curr_t_battery=0;
      eeprom_update_byte(&t_battery,0);
    }
    
    if(T2>50){
      T2=0;
      curr_t_battery++;
      eeprom_update_byte(&t_battery,curr_t_battery);
    }
  }
}

/*
void displayBattery(){
  if(curr_battery>=70){
    set_LEDVERDE(1);
    set_LEDAMAR(0);
    set_LEDVERM(0);
  }
  else if(curr_battery>=30 && curr_battery<70){
    set_LEDVERDE(0);
    set_LEDAMAR(1);
    set_LEDVERM(0);
  }
  else if(curr_battery<30){
    set_LEDVERDE(0);
    set_LEDAMAR(0);
    set_LEDVERM(1);
  }
}


void finishRace(){
  if(laps==5){
    set_LEDAMAR(1);
    set_LEDVERDE(1);
    set_LEDVERM(1);
  }
}
*/

//Função que verifica e atualiza a bateria e o número de voltas no LCD
void LCD_battery(){
  if((bat_display!=curr_battery) || (stateLCD==0)){
    LCD_Line(0);
    bat_display = curr_battery;
    LCD_Init();
    link=1;
    //strcpy(text2, "");
    strcpy(text2, "");
    
    if(curr_battery>0){
      sprintf(text2, "Bateria: %d", curr_battery);
      strcat(text2,"%");
    }
    else{
      strcat(text2,"Eq. vai Desligar");
    }
  
    LCD_Message(text2);  
    stateLCD=1;
    bat_display = curr_battery;
  }
}

//Função que imprime no LCD o número de voltas. Atualiza sempre que o número de voltas muda ou sempre que a bateria é atualizada
void LCD_voltas(){
  if(stateLCD_V==0 || link==1 ||  voltas_display!=laps){
    strcpy(text4, "");
    LCD_Line(1);
    sprintf(text4, "Voltas:  %d", laps);

    LCD_Message(text4);  
    voltas_display=laps;
    stateLCD_V=1;
    link=0;  
  }
}


void IRcontrol(){
  
  //Máquina de Estados do controlo por Infravermelhos

  if(stateIR==0 && IRsens==BOT_DESL && T1>5){
    T1=0;
    stateIR=1;
  }
  else if(stateIR==1 && T1>5 && IRsens!=BOT_DESL){
    stateIR=2;
  }
  else if(stateIR==2 && IRsens==BOT_DESL){
    T1=0;
    stateIR=4;
  }
  else if(stateIR==4 && T1>5 && IRsens!=BOT_DESL){
    T1=0;
    stateIR=0;
  }
  else if(stateIR==1 && T1>50){
    aux=1;
    eeprom_update_byte(&battery,100);
    stateIR=5;
  }
  else if(stateIR==4 && T1>50){
    aux=0;
    eeprom_update_byte(&battery,100);
    stateIR=5;
  }
  else if(stateIR==5 && aux==0){
    stateIR=0;
  }
  else if(stateIR==5 && aux==1){
    stateIR=2;
  }

  //Máquina de estados do reset ao número de voltas
  if(stateIR2==0 && IRsens==BOT_SEC){
    laps=0;
    stateIR2=1;
    T3=0;
  }
  else if(stateIR2==1 && T3>5){
    stateIR2=0;
  }
}

void main(void) {

  io_init();
  printf_init();
  init_ADC();
  init_timerPWM();
  init_timer1();
  T1 = 0;
  uint8_t v_motor, stateL=0, stateBAT=0, stateLAP;
  uint8_t VS0=0, VS1=0, VS2=0, VS3=0, VS4=0;
  bat_display = curr_battery;

  Vref = 500;  


  PORTD |= (1<<4);
  PORTD |= (1<<2);
  OCR0A = 0; //Roda Esquerda
  OCR0B = 0; //Roda Direita

  
  SetupPorts();
  LCD_Init();
  LCD_Clear();

  while(1){

    //calibrateSensor();

  /*
    Valor normalizado: 450

    Erros:
    0 0 0 0 1 ==> erro = 4
    0 0 0 1 1 ==> erro = 3
    0 0 0 1 0 ==> erro = 2
    0 0 1 1 0 ==> erro = 1
    0 0 1 0 0 ==> erro = 0
    0 1 1 0 0 ==> erro = -1
    0 1 0 0 0 ==> erro = -2
    1 1 0 0 0 ==> erro = -3
    1 0 0 0 0 ==> erro = -4
*/

  //Guarda se os sensores estão no preto ou em branco - Assim só é preciso verificar os sensores 1 vez por iteração - Chamamos menos vezes a função
  VS0 = ISsensor(0);
  VS1 = ISsensor(1);
  VS2 = ISsensor(2);
  VS3 = ISsensor(3);
  VS4 = ISsensor(4);


  //Conta o número de voltas - VERIFICAR
  if(stateL==0 && VS0==1 && VS2==1 && VS4==1 & T1>5 && (erro!=50) && (stateIR!=0)){
    laps++;
    stateL=1;
  }
  else if(stateL==1 && (VS0==0 && VS4==0 && VS2==1)){
    stateL=2;
  }
  else if(stateL==2){
    stateL=0;
  }

  //Reset ao numero de voltas  COM BOTÃO
  if(stateLAP==0 && isButtonLAP()){
    laps=0;
    stateLAP=1;
  }
  else if(stateLAP==1 && !(isButtonLAP())){
    stateLAP=0;
  }

  //Reset ao contador de bateria COM BOTÃO
  if(stateBAT==0 && isButtonBAT()){
    eeprom_update_byte(&battery,100);
    stateBAT=1;
  }
  else if(stateBAT==1 && !(isButtonBAT())){
    stateBAT=0;
  }
  
 //Erros dependendo dos sensores que estão ativos
 
  if ((VS0==0) && (VS1==0) && (VS2==0) && (VS3==0) && (VS4==1)) erro = 4;
  
  else if ((VS0==0) && (VS1==0) && (VS2==0) && (VS3==1) && (VS4==1)) erro = 3;
  
  else if ((VS0==0) && (VS1==0) && (VS2==0) && (VS3==1) && (VS4==0)) erro = 2;

  else if ((VS0==0) && (VS1==0) && (VS2==1) && (VS3==1) && (VS4==0)) erro = 1;

  else if ((VS0==0) && (VS1==0) && (VS2==1) && (VS3==0) && (VS4==0)) erro = 0;

  else if ((VS0==0) && (VS1==1) && (VS2==1) && (VS3==0) && (VS4==0)) erro = -1;

  else if ((VS0==0) && (VS1==1) && (VS2==0) && (VS3==0) && (VS4==0)) erro = -2;

  else if ((VS0==1) && (VS1==1) && (VS2==0) && (VS3==0) && (VS4==0)) erro = -3;

  else if ((VS0==1) && (VS1==0) && (VS2==0) && (VS3==0) && (VS4==0)) erro = -4;

  else if ((VS0==0) && (VS1==0) && (VS2==0) && (VS3==0) && (VS4==0)) erro = 50;
  
  Kp=30; 
  Ki=0; 
  Kd=0;

  calculatePID();

  if(stateIR!=0){

  if(erro <5 && erro!=0){
    mot_esq = 120 + PID; 
    mot_dir = 120 - PID;
  }/*
  else if(erro==50){
    mot_esq = 100;
    mot_dir = 100;
  }*/
  else if(erro==0){
    mot_esq = 100;
    mot_dir = 100;
  }
 }
 else{
   mot_esq = 0;
   mot_dir = 0;
 }

  OCR0A= mot_esq;
  OCR0B= mot_dir;

  calculateBattery();
  IRcontrol();
  //displayBattery();
  //finishRace();
  
  LCD_battery();
  LCD_voltas();

  //printf("Sensor0: %d | Sensor1: %d | Sensor2: %d | Sensor3: %d | Sensor 4: %d | Erro: %d | PID: %d | M_ESQ: %d | M_DIR: %d | Voltas: %d | SensorIR: %d | Battery: %d | IR: %d\n", VS0, VS1, VS2, VS3, VS4, erro, PID, mot_esq, mot_dir, laps, IR, curr_battery, bat_display, IRsens);
  //printf("S0: %d |S1: %d |S2: %d |S3: %d |S4: %d | IR: %d |\n", sensor0, sensor1, sensor2, sensor3, sensor4, IRsens);
  }
}