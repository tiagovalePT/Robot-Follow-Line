#define LED 5
#define ClearBit(x,y) x &= ~_BV(y) 
#define SetBit(x,y) x |= _BV(y)
#define LCD_RS 0
#define LCD_E  1
#define DAT4   2
#define DAT5   3
#define DAT6   4
#define DAT7   5
#define CLEARDISPLAY 0x01
#define SETCURSOR    0x80
#define NUMCHARS 64

#include <avr/io.h> 
#include <util/delay.h> 
#include <string.h> 
#include <stdlib.h>

typedef uint8_t byte;
typedef int8_t sbyte;

void SetupPorts() {
  DDRB = 0x3F;                  // 0011.1111; set PB0-PB5 as outputs
  DDRC = 0x00;                  // 0000.0000; set PORTC as inputs
}

void msDelay(int delay) {
  for (int i=0;i<delay;i++)
    _delay_ms(1);
}

void FlashLED() {
  SetBit(PORTB,LED);
  msDelay(250);
  ClearBit(PORTB,LED);
  msDelay(250);
}

void PulseEnableLine () {
    SetBit(PORTB,LCD_E);
    _delay_us(40);
    ClearBit(PORTB,LCD_E);
}

void SendNibble(byte data) {
  PORTB &= 0xC3;
  if (data & _BV(4)) SetBit(PORTB,DAT4);
  if (data & _BV(5)) SetBit(PORTB,DAT5);
  if (data & _BV(6)) SetBit(PORTB,DAT6);
  if (data & _BV(7)) SetBit(PORTB,DAT7);
  PulseEnableLine();
}

void SendByte (byte data) {
    SendNibble(data);
    SendNibble(data<<4);
    ClearBit(PORTB,5);
}

//sends LCD controller command
void LCD_Cmd (byte cmd) {
    ClearBit(PORTB,LCD_RS);
    SendByte(cmd);
}

//send single ascii character to display
void LCD_Char (byte ch) {
    SetBit(PORTB,LCD_RS);
    SendByte(ch);
}

//initializes the LCD controller
void LCD_Init() {
    LCD_Cmd(0x33);
    LCD_Cmd(0x32);
    LCD_Cmd(0x28);
    LCD_Cmd(0x0C);
    LCD_Cmd(0x06);
    LCD_Cmd(0x01);
    msDelay(3);
}

//clears the LCD display and home cursors
void LCD_Clear() {
    LCD_Cmd(CLEARDISPLAY);
    msDelay(3);
}

//homes the LCD cursor
void LCD_Home() {
    LCD_Cmd(SETCURSOR);
}

//puts cursor at position (x,y)
void LCD_Goto(byte x, byte y) {
  byte addr = 0; 
  switch (y)
  {
  case 1: addr = 0x40; break; 
  case 2: addr = 0x14; break; 
  case 3: addr = 0x54; break;
  }
  LCD_Cmd(SETCURSOR+addr+x);
}

//puts cursor at start of line (x)
void LCD_Line(byte row) {
    LCD_Goto(0,row);
}

//displays a string
void LCD_Message(const char *text) {
  while (*text)
    LCD_Char(*text++);
}

//displays a hexadecimal value
void LCD_Hex(int data) {
  char st[8] = ""; 
  itoa(data,st,16);  
  LCD_Message(st);
}

//displays an integer value
void LCD_Integer(int data) {
  char st[8] = ""; 
  itoa(data,st,10); 
  LCD_Message(st);
}

//Just for help
void UpdateCursor (byte count) {
  switch(count) {
    case  0: LCD_Line(0); break;
    case 16: LCD_Line(1); break;
    case 32: LCD_Line(2); break;
    case 48: LCD_Line(3); break;
  } 
}

//Just for help
char GetNextChar(char ch) {
  if ((ch<0x20) | (ch>=0xFF)) return 0x20;
  if ((ch>=0x7F) & (ch<0xA0)) return 0xA0;
  return ++ch;
}

//Fill LCD with ascii characters
void FillScreen () {
  char ch = 'A'; 
  LCD_Clear();
  for (byte count=1;count<100;count++) {
    LCD_Goto(18,0);
    LCD_Integer(count);
    for (byte i=0;i<NUMCHARS;i++) {
      UpdateCursor(i); 
      LCD_Char(ch);
      ch = GetNextChar(ch); 
      msDelay(60);
    }
  }
}