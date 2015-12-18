
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <TimerOne.h>
#include <math.h>
#include "font.h"

#define DEBUG

#define CMD_KEY1               'w'    // change wave
#define CMD_KEY2               'f'    // VFL
#define CMD_KEY3               'd'    // dB->dBm
#define CMD_KEY4               'l'    // BackLight
#define CMD_KEY5               'z'    // zero
#define CMD_KEY6               'a'
#define CMD_CALCULATE_POWER    'p'    // power
#define CMD_UPDATE_SCREEN      'u'    // update screen
#define CMD_STORE_COEFF        's'    // save coef
#define CMD_READ_RESULT        'r'    // read result to Serial port 

#define MAX_DIAPAZON 5
#define NUM_AVERAGE 80

#define TIME_UPDATE 500 // ms

#define CS 10  // LCD Pins  PB2
#define CD 12  // PB4
//#define RESET 8

#define LCDSCK 13  // PB5
#define LCDMOSI 11  // PB3

#define buttonH1 14  // PC0
#define buttonH2 15  // PC1
#define buttonH3 16  // PC2

#define buttonV1 17  // PC3
#define buttonV2 18  // PC4

#define LIGHT  8  // PB0

#define kill 7  // PD7
#define inter 3   // PD3

#define SCLK 4  // ADC Pins  PD4
#define DOUT 2   // PD2

#define N2M  5  // PD5 Select range pins
#define N20K 6  // PD6

#define VFL  9 //  PB1

#define BEEP 19  // PC5
#define PIN_VOLTAGE A7 //   ADC7

#define SPI_PORTX PORTB
#define SPI_DDRX DDRB
 
//#define SPI_MISO 6 
#define SPI_MOSI 3
#define SPI_SCK 5
//#define SPI_SS 2


//////////////////////////////////
#define MAX_CMD_IN_STACK 10
static unsigned char posCurrent=0;
static unsigned char stackCmd[MAX_CMD_IN_STACK];

byte n; // number of range
unsigned long time2,time1;

double koef;
float vl;
double pw,pwr;   // pw - powe in Watt, pwr - Power in dBm
double pwr_db;   // 

String inputString = ""; 
boolean stringComplete = false;

boolean flag_dBm = false;
boolean fl_VFL = false; 
boolean fl_backLight = false;
boolean fl_key_pressed;

#define MAX_NUM_WAVE 5
byte num_wave;
String wave[]={"650","850","1310","1490","1550","1625"};
int calibr[]={1060,560,0,-30,-23,34};    // 10.6 dB

void SPI_begin()
{
  // SPI_DDRX |= (1<<SPI_MOSI)|(1<<SPI_SCK);
  // SPI_PORTX |= (1<<SPI_MOSI)|(1<<SPI_SCK);
 
   /*разрешение spi,старший бит вперед,мастер, режим 0*/
 //  SPCR = 0x53; //(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR1)|(0<<SPR0);
 //  SPSR = 0x01; //(0<<SPI2X); 
   SPI_DDRX |= (1<<SPI_MOSI)|(1<<SPI_SCK);
   SPI_PORTX |= (1<<SPI_MOSI)|(1<<SPI_SCK);
 
   /*разрешение spi,старший бит вперед,мастер, режим 0*/
   SPCR = (1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR1)|(0<<SPR0);
   SPSR = (0<<SPI2X);
 
}  
///////////////////////////////////
void SPI_transfer(unsigned char b)
{
 //  SPI_PORTX &= ~(1<<SPI_SS);
 //  SPDR = b;
 //  while(!(SPSR & (1<<SPIF)));
 //  SPI_PORTX |= (1<<SPI_SS); 
    /* 
  unsigned char i,j;
  j=0x80;
  for(i=0;i<8;i++)
  {
     if(b & j)
     {
       digitalWrite(LCDMOSI,HIGH);
     }
     else
     {
       digitalWrite(LCDMOSI,LOW);
     }  
     digitalWrite(LCDSCK,HIGH);
     digitalWrite(LCDSCK,LOW);
    
    j=j>>1; 
  }  
  */
  
 // shiftOut(LCDMOSI, LCDSCK, MSBFIRST, b);
  
}  

//////// Transfer Commmand    
void Write_Command(unsigned char command)
{
     digitalWrite(CS,LOW);
     digitalWrite(CD,LOW);

  //   SPI_transfer(command);
     shiftOut(LCDMOSI, LCDSCK, MSBFIRST, command);
     
     digitalWrite(CS,HIGH);
}

/**********************************/
/*      Transfer Data             */
/**********************************/
void Write_Data(unsigned char data1)
{

     digitalWrite(CS,LOW);
     digitalWrite(CD,HIGH);

    // SPI_transfer(data1);
     shiftOut(LCDMOSI, LCDSCK, MSBFIRST, data1);
     
     digitalWrite(CS,HIGH);
}
//-------------------------------------------------------------------
void Set_column_addr(unsigned char add)
{
     unsigned char temp;
     temp=add;
     add=add>>4;
     add=add&0x0f;
     add=add|0x10;
     Write_Command(add);	//Set upper addr;
     add=temp;
     add=add&0x0F;
     Write_Command(add);	//Set lower addr;
} 
//-----------------------------------------------------------------
void Set_row_addr(unsigned char row)
{
     row=row&0x0F;
     row=row|0x0B0;
     Write_Command(row);		//page addr set	
}	
/**********************************/
/*      LCD Initial Code          */
/**********************************/
void Lcd_Set()
{
  //   digitalWrite(RESET,LOW);
  //   delay(1); 
  //   digitalWrite(RESET,HIGH);
  //   delay(1);
         
     Write_Command(0xA2);          //Set Bias        0xA2
     Write_Command(0xA0);          //Segment Direction Select ,bit0=1,reverse;=0,normal;
     Write_Command(0xC8);          //Common Direction Select,bit3=1,reverse direction;=0,normal;
     Write_Command(0x24);          //set ra/rb 0x25
     Write_Command(0x81);          //Set Contrast     
     Write_Command(0x29);          //29    contrast
     Write_Command(0x2C);          //Power Control Set 
     delay(1); 
     Write_Command(0x2E);          //Power Control Set 
     delay(1); 
     Write_Command(0x2F);          //Power Control Set      
    
     delay(1);
     Write_Command(0xAF);          //Display ON

}
/**********************************/
/*      Clear Display             */
/**********************************/
void Display_Clear(unsigned char data1,unsigned char data2)
{
     int i,j,m;
     m=0xb0;
     for(i=0;i<8;i++)
        {
            Write_Command(m);
            Write_Command(0x10);
            Write_Command(0x00);
            for(j=0;j<64;j++)
               {    
                  Write_Data(data1);
                  Write_Data(data2);
                }
            m++;
         }
}
/////////////////////////////////////////////////////////////
void Show_string(unsigned char xs,unsigned char page,String p)
{
  byte i=0;
  while(p[i]!=0)
  {
    Show_one_char(xs,page,p[i++]);
    xs+=6;
  }  
}  

////////////////////////////////////////////////////////////////
void Show_big_one_char(unsigned char xs,unsigned char page,char p)
{
     byte j,y;
     char myChar;
     int i,iy;
     for(iy=0;iy<3;iy++)
     {
     Set_column_addr(xs);		//column addr set   
     Set_row_addr(page+iy);
     for(i=0;i<14;i++)		//font
        {
            myChar =  pgm_read_byte_near(big_font + ((int)p-'0')*42 + i + iy * 14);
         /*   
            y=0;
            for(j=0;j<8;j++)
            {
              y = y + (myChar & 0x01);
              y = y <<1;
              myChar = myChar>>1;
            }  
            */
            Write_Data(myChar);
 
         }
   //  Write_Data(0x00);  
     }  
}
////////////////////////////////////////////////////////////////
void Show_one_char(unsigned char xs,unsigned char page,char p)
{
     byte j,y;
     char myChar;
     int i;
     Set_column_addr(xs);		//column addr set   
     Set_row_addr(page);
     for(i=4;i>=0;i--)		//5X8 font
        {
            myChar =  pgm_read_byte_near(small_font + ((int)p-32)*5 + i);
            
            y=0;
            for(j=0;j<8;j++)
            {
              y = y + (myChar & 0x01);
              y = y <<1;
              myChar = myChar>>1;
            }  
            Write_Data(y);
 
         }
     Write_Data(0x00);    
}
//-------------------------------------------------------------------
void set_ch(byte nch)
{
  n = nch;
//  digitalWrite(NV,(nch & 0x01));
  digitalWrite(N20K,((nch >> 1)& 0x01));
  digitalWrite(N2M,((nch >> 2)& 0x01));
  
  if(n==0) koef=1.811;
  if(n==1) koef=1.811;
  
  if(n==2) koef=181.1;
  if(n==3) koef=181.1;
  
  if(n==4) koef=17600.0;
  if(n==5) koef=17600.0;

}
////////////////////////////////////
void next_range()
{
    n+=2;
    if(n>MAX_DIAPAZON)
    { 
      n=1;
    }
    set_ch(n);
} 
///////////////////////
void prev_range()
{
    if(n==1)
    { 
      n=MAX_DIAPAZON;
     }
     else n-=2;
     set_ch(n);
} 
//-------------------------------------------------------------------
volatile int num_aver;
boolean fl_event;
volatile boolean fl_int;   // flag interrupt
volatile boolean flag_result_ready;
volatile unsigned long dout,av_dout;
//-------------------------------------------------------------------
void serialEvent()
{
  boolean a = true;
  
  while (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    if ( (inChar == '\r') || (inChar == '\n')) {
      stringComplete = true;
      if(a)
      {
        pushCmd(inChar);
        a = false;
      }
    }
    else
    { 
      inputString += inChar;
    }
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
 
  }
  
  Serial.println("OK");  // !!!!!!
  
}
//-------------------------------------------------------------------
void power_off()
{
  
  cli();
      int i=0;
        while( (i<800))
       // for(int i=0;i<1600;i++)
        {
           delayMicroseconds(1000);
           i++;
           if(digitalRead(inter)==HIGH)
           {
             sei();
             return;
           }  
        }
  
     //  delay(2000);
      //  cli();
        if(i>=800)
        {
    
           Display_Clear(0x00,0x00);
         //  delay(200);
           Show_string(40,4,"Good bye");
           for(int j=0;j<1000;j++)
         { 
          delayMicroseconds(1000);
         }    
    
        digitalWrite(kill,LOW);
        }
}  


///// read from ADC ///////
void fready()
{
  cli();
  delayMicroseconds(10);
  
  if((digitalRead(DOUT)==LOW)  && (fl_int==true))
  {
    delayMicroseconds(2);
      fl_int=false;
      volatile unsigned long dt=0;
      
      for(byte i=0;i<24;i++)
      {
        digitalWrite(SCLK,HIGH);
        delayMicroseconds(1);
        if(digitalRead(DOUT)==HIGH)
        {
          dt+=1;
        }
        dt=dt<<1;
        digitalWrite(SCLK,LOW);
        delayMicroseconds(1);
      } 
       
    //  digitalWrite(8,HIGH);
      dout += dt;
      num_aver++;
      if(num_aver==NUM_AVERAGE)
      {
         av_dout = dout / NUM_AVERAGE;
         dout=0;
         num_aver=0;
         flag_result_ready = true;      /// !!! not need
         pushCmd(CMD_CALCULATE_POWER); 
      }  
      
  }
  else
  {
    fl_int=true;
  }  
 
 sei();

}  
//////////////////////
void updtateCoeffInEeprom(int addr)
{
  byte bw;  
  for(byte i=0;i<MAX_NUM_WAVE;i++)
  {
    bw = calibr[i] >> 8;
    EEPROM.write(addr + 2*i, bw);
    bw = calibr[i] & 0xFF; 
    EEPROM.write(addr + 2*i + 1, bw);  
  }
}
//////////////////////
void readCoeffFromEEPROM(int addr)
{
  for(byte i=0;i<MAX_NUM_WAVE;i++)
  {
    calibr[i] = (int)(EEPROM.read(addr + 2*i)) << 8 + (int)EEPROM.read(addr + 2*i + 1);
  }   
}
//////////////////////
void setup() {

   pinMode(kill,OUTPUT);
   digitalWrite(kill,HIGH);
   
   pinMode(BEEP,OUTPUT);
   digitalWrite(BEEP,HIGH);
   
   Serial.begin(57600);
  
//   SPI_begin();
  // reserve 20 bytes for the inputString:
  inputString.reserve(20); 
   
   pinMode(CS,OUTPUT);  // LCD
   pinMode(CD,OUTPUT); 
//   pinMode(RESET,OUTPUT);

   pinMode(LCDSCK,OUTPUT); 
   digitalWrite(LCDSCK,LOW);
   pinMode(LCDMOSI,OUTPUT); 

   pinMode(buttonH1,INPUT);
   pinMode(buttonH2,INPUT);
   pinMode(buttonH3,INPUT);
  
   pinMode(buttonV1,OUTPUT);
   pinMode(buttonV2,OUTPUT);
   digitalWrite(buttonV1,LOW);
   digitalWrite(buttonV2,LOW);
   
   pinMode(LIGHT,OUTPUT);
   digitalWrite(LIGHT,HIGH);
   
   pinMode(SCLK,OUTPUT);  // ADC
   pinMode(DOUT,INPUT);
   dout = 0;
   num_aver = 0;  
   
   pinMode(N2M,OUTPUT);   // range
   pinMode(N20K,OUTPUT);
 //  pinMode(NV,OUTPUT); //!!!!!
    
  pinMode(VFL,OUTPUT);
//  digitalWrite(VFL,LOW);  // TURN ON
 // delay(100);
  digitalWrite(VFL,HIGH);
  
  digitalWrite(BEEP,LOW);
  delay(100);
  digitalWrite(BEEP,HIGH);    
   
  set_ch(5);
  num_wave=2;   // 1310
  flag_dBm = false;
   
  Lcd_Set();
   
 //  delay(200);
   Display_Clear(0x00,0x00);
 //  delay(200);
 //  Display_Clear(0x55,0xaa);		
 //  delay(200);
 //  Display_Clear(0x00,0x00);
 //  delay(200);
   Show_string(0,0,"Power Meter v.1.0");
     
     
 #ifdef DEBUG    
   Show_string(0,0,"TEST 12345");
   while(1){}
   
   Show_string(0,0,"Rng="+String(n));
#endif   

   Show_string(36,0,"    ");
   Show_string(36,0,String(wave[num_wave]));
  
   attachInterrupt(digitalPinToInterrupt(DOUT), fready, RISING);
   pinMode(inter,INPUT);
   attachInterrupt(digitalPinToInterrupt(inter), power_off, LOW);  // !!!!
      
   Timer1.initialize(100000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
   Timer1.attachInterrupt( keyBoardCommand ); // attach the service routine here
   interrupts();
   
   time2 = 0;
   time1 = 0;

 #ifdef DEBUG 
  Serial.println("Coeff:");  
  for(byte i=0;i<MAX_NUM_WAVE;i++)
    Serial.println(calibr[i]);      
 #endif
   
 /*  
   byte pz=0;
   Show_big_one_char(0,0,'0');
   Show_big_one_char(pz+14*1,0,'1');
   Show_big_one_char(pz+14*2,0,'2');
   Show_big_one_char(pz+14*3,0,'3');
   Show_big_one_char(pz+14*4,0,'4');
   
   Show_big_one_char(pz+14*5,0,'5');
   Show_big_one_char(pz+14*6,0,'6');
   
   Show_big_one_char(pz+14*0,3,'7');
   Show_big_one_char(pz+14*1,3,'8');
   Show_big_one_char(pz+14*2,3,'9');
   
   Show_big_one_char(pz+14*3,3,'9'+2);
   Show_big_one_char(pz+14*4,3,'9'+3);
   Show_big_one_char(pz+14*5,3,'9'+4);
   
   while(1){};
   */
}

//////////////////////////////////////////////////////////////////////
char readKeyBoard()
{
   char key=0;
   digitalWrite(buttonV1,LOW);
   digitalWrite(buttonV2,HIGH);
 
  if(digitalRead(buttonH1)==LOW)  
     key=CMD_KEY1; 
  if(digitalRead(buttonH2)==LOW)  
     key = CMD_KEY2;
  if(digitalRead(buttonH3)==LOW)
     key = CMD_KEY3;
  
  if(key==0)
  {
     digitalWrite(buttonV1,HIGH);
     digitalWrite(buttonV2,LOW);
  
     if(digitalRead(buttonH1)==LOW)  
       key=CMD_KEY4;
     if(digitalRead(buttonH2)==LOW)  
       key = CMD_KEY5;
     if(digitalRead(buttonH3)==LOW)
       key = CMD_KEY6;
  }
  if( (key!=0) && (fl_key_pressed==false) )
  {
     digitalWrite(BEEP,LOW);
     delay(25);
     digitalWrite(BEEP,HIGH);
     fl_key_pressed = true;
    
  }
  else
  {
    fl_key_pressed = false;
    key = 0;
  }
  return key;
}  
//////////////////////////////////////////////////////////////////////
void pushCmd(unsigned char c)
{
  if(posCurrent<MAX_CMD_IN_STACK)
  {
     posCurrent++;
     stackCmd[posCurrent] = c;
  }    
}
//////////////////////////////////////////////////////////////////////
unsigned char popCmd()
{
  unsigned char c=0; 
  if(posCurrent>0)
  { 
    c = stackCmd[posCurrent--];
  }  
  return c;
}  
//////////////////////////////////////////////////////////////////////
// call auto from timer interruput 10Hz
void keyBoardCommand()
{
   unsigned char cmd;
   cmd = readKeyBoard();
   if(cmd!=0)
   {
      pushCmd(cmd);   
   }  
} 
//////////////////////////////////////////////////////////////////////
void changeWaveLenght()
{
   num_wave++;
   if(num_wave>MAX_NUM_WAVE) num_wave=0;
   pushCmd(CMD_UPDATE_SCREEN);
}  
//////////////////////////////////////////////////////////////////////
void changeFormat()
{
  flag_dBm = !flag_dBm;
  pwr_db = pwr;
  pushCmd(CMD_UPDATE_SCREEN);
}
//////////////////////////////////////////////////////////////////////    
void turnVFL()
{
    if(fl_VFL)
       digitalWrite(VFL,HIGH);  // TURN OFF        
    else
       digitalWrite(VFL,LOW); 
       
   fl_VFL = !fl_VFL;
} 

//////////////////////////////////////////////////////////////////////
void switchBackLight()
{
   if(fl_backLight)
       digitalWrite(LIGHT,LOW);  // TURN OFF        
    else
       digitalWrite(LIGHT,HIGH); 
       
   fl_backLight = !fl_backLight;  
}  

//////////////////////////////////////////////////////////////////////
void setZero()
{
  
}

//////////////////////////////////////////////////////////////////////
void updateScreen()
{
  #ifdef DEBUG   
     Show_string(0,0,"     ");
     Show_string(0,0,"Rng="+String(n));
     
     Show_string(0,1,"              ");
     Show_string(0,1,"ADC="+String(av_dout)); // output ADC      
     
  #endif    
       // output number wave
     Show_string(36,0,"    ");
     Show_string(36,0,String(wave[num_wave]));

#ifndef DEBUG
      if(flag_dBm)
      {
        Show_string(0,1,"Pwr="+String(pwr_db,2)+"dBm");
      }
      else
      {
        Show_string(0,1,"              ");
      }
#endif
     
  
     if(pw>1000.0) // show power in Watt
     {
       Show_string(0,6,"Pwr="+String(pw/1000.0,2)+" uW  ");
     }
     else
     {  
       Show_string(0,6,"Pwr="+String(pw,2)+" nW  ");
     }    
     
     
     String pwr_str;     
     pwr_str = String(pwr,2);
          
 // #ifdef DEBUG   
     Show_string(0,5,"               ");
     Show_string(0,5,"Pwr="+pwr_str+"dBm");
//  #endif     

     
       // show battery status
     int v;
     v = analogRead(PIN_VOLTAGE);
     vl = map(v,0,1023,0,6600);


     Show_string(100,0,"    ");
     Show_string(100,0,String(vl/1000,2));   
}  

//////////////////////////////////////////////////////////////////////
void calculatePower()
{
   pw=av_dout/koef; // pw - power in nWatt
   pwr = 10.0*log10(pw*1.0e-6);  // pwr - power in dBm
   pwr = pwr + calibr[num_wave]/100.0;

   pw = pow(10,pwr/10.0)*1.0e+6;  // power in nWatt
   if(flag_dBm)
   {
       pwr = pwr - pwr_db;
   } 
   
   if(av_dout>16777000)
   {
       prev_range();
   } 
   if((av_dout<7000) && (n!=MAX_DIAPAZON)) 
   {
       next_range();
   }    
   pushCmd(CMD_UPDATE_SCREEN);
} 
    
//////////////////////////////////////////////////////////////////////
void readResultToSerial()
{
  Serial.println("Result:");
  Serial.println(num_wave);
  Serial.println(pw);
  Serial.println(pwr);  
}


//////////////////////////////////////////////////////////////////////
void ciklReadCommand()
{
  unsigned char cmd;
  cmd = popCmd();
  if(cmd!=0)
  {
    switch (cmd)
     {
      case CMD_CALCULATE_POWER : { calculatePower(); break; } 
      case CMD_UPDATE_SCREEN : { updateScreen(); break; }
      case CMD_KEY1 : { changeWaveLenght(); break; }
      case CMD_KEY2 : { turnVFL(); break; }
      case CMD_KEY3 : { changeFormat(); break; }
      case CMD_KEY4 : { switchBackLight(); break; }
      case CMD_KEY5 : { setZero(); break; }
      case CMD_KEY6 : { break; }

      case CMD_STORE_COEFF : { updtateCoeffInEeprom(0); break;} 
      case CMD_READ_RESULT : { readResultToSerial; break;}
      
     } 
  }  
}  

//////////////////////////////////////////////////////////////////////
void loop()
{
   ciklReadCommand();
}  

///////////////////////////////////
void old_loop() {
  
   digitalWrite(buttonV1,LOW);
   digitalWrite(buttonV2,HIGH);
 
  if(digitalRead(buttonH1)==LOW)  // buton left
  {
     digitalWrite(BEEP,LOW);
     delay(50);
     digitalWrite(BEEP,HIGH);  
  #ifdef DEBUG     
     prev_range();
  #endif
    
     while(digitalRead(buttonH1)==LOW){}
     fl_event=true;
     
  }
  if(digitalRead(buttonH2)==LOW)  // buton right
  {
    
     digitalWrite(BEEP,LOW);
     delay(50);
     digitalWrite(BEEP,HIGH); 

  #ifdef DEBUG    
     next_range();
  #endif   
    
     while(digitalRead(buttonH2)==LOW){}
     fl_event=true;
     
  }
  if(digitalRead(buttonH3)==LOW)
  {
     digitalWrite(BEEP,LOW);
     delay(50);
     digitalWrite(BEEP,HIGH);
     flag_dBm = !flag_dBm;

     pwr_db = pwr;
     while(digitalRead(buttonH3)==LOW){}
     fl_event=true;
  }


  
  digitalWrite(buttonV1,HIGH);
  digitalWrite(buttonV2,LOW);

  if(digitalRead(buttonH3)==LOW)
  {
     digitalWrite(BEEP,LOW);
     delay(50);
     digitalWrite(BEEP,HIGH);
     num_wave++;
     while(digitalRead(buttonH3)==LOW){}
     
     if(num_wave>MAX_NUM_WAVE) num_wave=0;
     fl_event=true;
     
     if(num_wave==6)
     {
       digitalWrite(VFL,LOW);  // TURN ON
     } 
    else
     {
       digitalWrite(VFL,HIGH);
     }     
  }   
  ///////////////////////////////////////////////////////////
  
  if(fl_event) // key pressed
  {
    
  #ifdef DEBUG   
     Show_string(0,0,"     ");
     Show_string(0,0,"Rng="+String(n));
  #endif    
       // output number wave
     Show_string(36,0,"    ");
     Show_string(36,0,String(wave[num_wave]));
     fl_event = false;

#ifndef DEBUG
      if(flag_dBm)
      {
        Show_string(0,1,"Pwr="+String(pwr_db,2)+"dBm");
      }
      else
      {
        Show_string(0,1,"              ");
      }
#endif
     
     
   
  #ifdef DEBUG 
     Show_string(80,0,"pr");
  #endif   
     
  }
  else
  { 
  #ifdef DEBUG   
     Show_string(80,0,"nt"); 
  #endif   
  }
  
 /////// Receive from serial port /////////////
   if (stringComplete)
   {
    Show_string(0,7,"            ");  
    Show_string(0,7,inputString);  
   
    // clear the string:
    inputString = "";
    stringComplete = false;
    
    Serial.println("OK");
  }
 /////////////////////////// 
  
  
  
  

  time1 = millis();
  
  if( (flag_result_ready) && ((time1-time2)>TIME_UPDATE))  //flag_result_ready data from adc ready
  {
    
 #ifdef DEBUG    
     
     Show_string(0,0,"     ");
     Show_string(0,0,"Rng="+String(n)); // output range
     
     
     Show_string(0,1,"              ");
     Show_string(0,1,"ADC="+String(av_dout)); // output ADC
     
 #endif    

 
    
     
     pw=av_dout/koef; // pw - power in Watt
     
   //  Show_string(0,6,"                ");
     if(pw>1000.0)
     {
       Show_string(0,6,"Pwr="+String(pw/1000.0,2)+" uW  ");
     }
     else
     {  
       Show_string(0,6,"Pwr="+String(pw,2)+" nW  ");
     }
     
     pwr = 10.0*log10(pw*1.0e-6);  // pwr - power in dBm
     if(flag_dBm)
     {
       pwr = pwr - pwr_db;
     }
     
     String pwr_str;
     
     pwr_str = String(pwr,2);
     
     
  #ifdef DEBUG   
     Show_string(0,5,"               ");
     Show_string(0,5,"Pwr="+pwr_str+"dBm");
  #endif    
     
     
     
     int v;
     v = analogRead(PIN_VOLTAGE);
     vl=map(v,0,1023,0,6600);


     Show_string(100,0,"    ");
     Show_string(100,0,String(vl/1000,2));

   byte pz;
   if(pwr<=-10.0)
   {  
     Show_big_one_char(0,2,'9'+5); // minus
     pz=15;
     Show_big_one_char(pz+14*0,2,pwr_str[1]);
     Show_big_one_char(pz+14*1,2,pwr_str[2]);
     Show_big_one_char(pz+14*2,2,'9'+1);  // ,
   
     pz=50;
     Show_big_one_char(pz+14*0,2,pwr_str[4]);
     Show_big_one_char(pz+14*1,2,pwr_str[5]);  
     
   }
   
   if( (pwr<0.0) && (pwr>-10.0) )
   {
      Show_big_one_char(0,2,'9'+6);
      pz = 15;
      Show_big_one_char(pz+14*0,2,'9'+5); // minus
      Show_big_one_char(pz+14*1,2,pwr_str[1]);
      Show_big_one_char(pz+14*2,2,'9'+1);  // ,
      pz=50;
      Show_big_one_char(pz+14*0,2,pwr_str[3]);
      Show_big_one_char(pz+14*1,2,pwr_str[4]); 
      
   }
   if( (pwr>=0.0) && (pwr<=10.0)  )
   {
     Show_big_one_char(0,2,'9'+6); 
     pz=15;
     Show_big_one_char(pz+14*0,2,'9'+6);
     
     Show_big_one_char(pz+14*1,2,pwr_str[0]);
     Show_big_one_char(pz+14*2,2,'9'+1);  // ,
      pz=50;
      Show_big_one_char(pz+14*0,2,pwr_str[2]);
      Show_big_one_char(pz+14*1,2,pwr_str[3]);      
   }
   if( pwr>10.0)
   {
     Show_big_one_char(0,2,'9'+6); // minus
     pz=15;
     Show_big_one_char(pz+14*0,2,pwr_str[0]);
     Show_big_one_char(pz+14*1,2,pwr_str[1]);
     Show_big_one_char(pz+14*2,2,'9'+1);  // ,
   
     pz=50;
     Show_big_one_char(pz+14*0,2,pwr_str[3]);
     Show_big_one_char(pz+14*1,2,pwr_str[4]);  
         
   }
  
    
   Show_big_one_char(pz+14*2,2,'9'+2);  // dBm
   Show_big_one_char(pz+14*3,2,'9'+3);
   
   if(!flag_dBm)
   {
    Show_big_one_char(pz+14*4,2,'9'+4);
   }    
   else
   {
    Show_big_one_char(pz+14*4,2,'9'+6);
   }
    
    
     if(av_dout>16777000)
     {
       prev_range();
       fl_event=true;
     } 
    if((av_dout<7000) && (n!=MAX_DIAPAZON)) 
     {
       next_range();
       fl_event=true;
     }   
  //   if((av_dout<10000) && (n==3)) 
  //   {
  //     prev_range();
  //     fl_event=true;
  //   }
     
 
     flag_result_ready = false;
     digitalWrite(LIGHT,HIGH);
     delay(10);
     digitalWrite(LIGHT,LOW);
     
   //  Serial.println(tst_ser);
   //  tst_ser++;
     
     time2 = time1;
    
  
  }
 
}
