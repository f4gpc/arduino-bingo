
/********************************************************
 ** F4GPG le 08/04/202O                                **
 ** BINGO Q R P                                        **
 ** ARDUINO DUE                                        **
 **  pilote DDS AD9850                                 **
 ** TFT 3.5P                                           **
 ** Ver 3.30
 ********************************************************/


#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "Adafruit_GFX.h"// Hardware-specific library
#include <MCUFRIEND_kbv.h>
#include <Keypad.h>
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <AD9850.h>
#include <DueFlashStorage.h>


DueFlashStorage dueFlashStorage;
MCUFRIEND_kbv tft;
//#include <Adafruit_TFTLCD.h>
//Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// Assign human-readable names to some common 16-bit color values:
#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define WHITE2   0xEEEE

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif
void setup(void);
void loop(void);
unsigned int testFillScreen();
unsigned int testText();
unsigned int testLines(uint16_t color);
unsigned int testFastLines(uint16_t color1, uint16_t color2);
unsigned int testRects(uint16_t color);
unsigned int testFilledRects(uint16_t color1, uint16_t color2);
unsigned int testFilledCircles(uint8_t radius, uint16_t color);
unsigned int testCircles(uint8_t radius, uint16_t color);
unsigned int testTriangles();
unsigned int testFilledTriangles();
unsigned int testRoundRects();
unsigned int testFilledRoundRects();
void progmemPrint(const char *str);
void progmemPrintln(const char *str);

void runtests(void);

uint16_t g_identifier;

extern const uint8_t hanzi[];
void showhanzi(unsigned int x, unsigned int y, unsigned char index)
{
    uint8_t i, j, c, first = 1;
    uint8_t *temp = (uint8_t*)hanzi;
    uint16_t color;
    tft.setAddrWindow(x, y, x + 31, y + 31); //Zone de réglage
    temp += index * 128;
    for (j = 0; j < 128; j++)
    {
        c = pgm_read_byte(temp);
        for (i = 0; i < 8; i++)
        {
            if ((c & (1 << i)) != 0)
            {
                color = RED;
            }
            else
            {
                color = BLACK;
            }
            tft.pushColors(&color, 1, first);
            first = 0;
        }
        temp++;
    }
}


//CLK - D6, FQUP - D7,  BitData - D8, RESET - D9
//AH_AD9850(int CLK, int FQUP, int BitData, int RESET);
//AH_AD9850 AD9850(24, 25, 22, 23);// 24 BLAN  . 25 JONE . 22 VERT . 23 BLEU


//-------------------------------------------------------------------------------------------------------------------
// clavier ************************************

const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'0','1','2','3'},
  {'4','5','6','7'},
  {'8','9','A','B'},
  {'C','D','E','F'}
};
byte rowPins[ROWS] = {45, 44, 47, 46}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {41, 40, 43, 42}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 


//pins de UTILISSER
const int encoderPin1 = 53;
const int encoderPin2 = 52;
const int encoder2Pin1 = 50;
const int encoder2Pin2 = 51;
const int encoder2bout = 48;
const int PTT = 38;
const int sMetrePin = A8; 
const int r1tX = 39;
const int r80 = 37;
const int r60 = 35;
const int r40 = 33;
const int r20 = 31;
const int rG = 29;
const int rPA = 27;
// POUR DDS
const int W_CLK_PIN = 24;
const int FQ_UD_PIN = 25;
const int DATA_PIN = 22;
const int RESET_PIN = 23;


volatile int lastEncoded = 0;
volatile int encoderValue = 0;
volatile int lastEncoded2 = 0;
volatile int encoderValue2 = 0;
volatile char flagBoutCodeur = 0;


//****************************************************************************************************************************
//VARIABLE
//*********************************************************************************************************************************
long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;
int encoMemo = 0;
int encoMemo2 = 0;
int FRtx = 7100000; //DE DEMARAGE
int FRtx2 = 7090000; //DE DEMARAGE
int Fr = 0; 
int SSb =0; //SI 0 :LSB SI 1 :USB
int PAS =100; //PAS DE 100Hz au demarage
char Vfo =1 ; // VFO A OU B OU RIT (1=A 2=B 3=RTI)
int si =1;    //sinialle smetre ( MEMOIRE POSITION )
int siM =1;   //smetre
int siT =1;   //smetre
char siV =0;  //smetre
boolean PTTflag =0; //PTT ON OFF
boolean RITflag =0; //RIT ON OFF
int RITval =0;      //RIT VALEUR
boolean OrBandeflag =0;// VFO ORBANDE AMATEUR
unsigned char BandeF =3;  // BANDE UTILISE AU DEMARAGE pour 40M
int cotClavier = 6 ; // pour 5 digite
int FiLSB ;//     valeur FI en LSB
int FiUSB ;//     valeur FI en USB
int trimFreq ; // valeur quatz paur DDS1

//CONSTENTE
// -- mode par défaut ---
const int FiLSBm = 10236850 ;// la FI en LSB
const int FiUSBm = 10236850 ;// la FI en USB
const int trimFreqm = 124996620; // VALEUR QUATZ DDS1
const int PosiFR = 100 ;// POSITION CONTTEUR FRECANCE
const int PosiVFO2 = 190 ;// POSITION CONTTEUR FRECANCE2
const int Ba1 = 3600000;  // CHANGEMENT LE BANDE ..
const int Ba2 = 5360000;
const int Ba3 = 7100000;
const int Ba4 = 14200000;


void setup()
{ 
  
  // PTT
   pinMode(PTT, INPUT);
   digitalWrite(PTT, HIGH); //turn pullup resistor on
   // initialize serial communication
  Serial.begin(115200);
  EEPROMIniti();
  FiLSB = EEPROMReadInt(1);
  FiUSB = EEPROMReadInt(5);
  trimFreq = EEPROMReadInt(9);
  Serial.print("fr FI en LSB = ");
  Serial.println(FiLSBm);
  Serial.print("fr FI en USB = ");
  Serial.println(FiUSBm);
  Serial.print("fr oscilateur DDS = ");
  Serial.println(trimFreqm);
  // DDS
  DDS.begin(W_CLK_PIN, FQ_UD_PIN, DATA_PIN, RESET_PIN);
  DDS.calibrate(trimFreq);
  //TFT
  tft.begin(0x9486);
  delay (500);
 // Serial.print("TFT size is "); Serial.print(tft.width()); Serial.print("x"); Serial.println(tft.height());
  // pin 2 et 3 encodeur en enttre avec pullup
  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);
  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);

  tft.setRotation(1);
  ECRtrafique ();
  
 
  encoderValue = (FRtx / PAS);
  encoderValue2 = (FRtx2 / PAS);
  delay(1000);

  
  
  // pin 2 et 3 encodeur en enttre avec pullup
  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);
  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
  attachInterrupt(encoderPin1, updateEncoder, CHANGE);
  attachInterrupt(encoderPin2, updateEncoder, CHANGE);

// pin 2 et 3 encodeur2 en enttre avec pullup
  pinMode(encoder2Pin1, INPUT);
  pinMode(encoder2Pin2, INPUT);
  digitalWrite(encoder2Pin1, HIGH); //turn pullup resistor on
  digitalWrite(encoder2Pin2, HIGH); //turn pullup resistor on
  attachInterrupt(encoder2Pin1, updateEncoder2, CHANGE);
  attachInterrupt(encoder2Pin2, updateEncoder2, CHANGE);

// RELAIS
  pinMode(r1tX, OUTPUT); 
  pinMode(r80, OUTPUT);
  pinMode(r60, OUTPUT);
  pinMode(r40, OUTPUT);
  pinMode(r20, OUTPUT);
  pinMode(rG, OUTPUT);
  pinMode(rPA, OUTPUT);
  digitalWrite(r1tX, 0);
  digitalWrite(r80, 0);
  digitalWrite(r60, 0);
  digitalWrite(r40, 0);
  digitalWrite(r20, 0);
  digitalWrite(rG, 0);
  digitalWrite(rPA, 0);
  
 
 
//BOUT
  pinMode(encoder2bout, INPUT);
  digitalWrite(encoder2bout, HIGH); //turn pullup resistor on
  //attachInterrupt(encoder2bout, updateEncoder2BOUT, LOW);
  
 

  
}

// ------ vecteur d'interuption-------------------------------------------------------

void updateEncoder() {
  noInterrupts();
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  lastEncoded = encoded; //store this value for next time
  interrupts();
}

void updateEncoder2() {
  noInterrupts();
  int MSB = digitalRead(encoder2Pin1); //MSB = most significant bit
  int LSB = digitalRead(encoder2Pin2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded2 << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue2 ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue2 --;

  lastEncoded2 = encoded; //store this value for next time
  interrupts();
}

void updateEncoder2BOUT() { // BOUTON ENCODEUR 2
  
  detachInterrupt(encoder2bout) ;
  flagBoutCodeur ++ ;
  
}


void loop() {
 
  //----------------------------------------------------------------------------------------------
  //------ PTT
  PTTflag = digitalRead(PTT);
  if (OrBandeflag == 1 && PTTflag ==0 ){
   // tft.setTextColor(RED,BLACK);
   afVFOA (RED);
  }
  else if (OrBandeflag == 1 && PTTflag ==1 ){
   // tft.setTextColor(WHITE,BLACK);
   afVFOA (WHITE);
  }
  else{
   if (PTTflag ==0){
     digitalWrite(r1tX, 1);
     if (RITflag == 1){
       // tft.setTextColor(WHITE,BLACK);
        afVFOA (WHITE);
       if (Vfo == 1){
        VFOcamende ( FRtx );
       }
     }
    }
  
   else{
     digitalWrite(r1tX, 0);
   }

  }
   
  Time();
  int customKey = customKeypad.getKey();
   
   analogReadResolution(9);
   if (siV == 0) sMetre (analogRead(sMetrePin));
   if (siV == 1) sMetreSlo (analogRead(sMetrePin));
    for (int i=0; i <= 7; i++){ 
      tft.drawLine((i*68), 299, (i*68), 319, RED);
      tft.drawLine(((i*68)+34), 306, ((i*68)+34), 319, RED);
  }
  //------------------------------------------------------------
  // ----------- CALVIER
  if (customKey){ 
   // Serial.println(customKey);
   if (customKey > 47 && customKey < 58){
    ClavierFR (customKey);
   }
   if (customKey == 69){ // 69 POUR E CHANGRMEN DE BANDE
    BandeF ++  ;
    if (BandeF > 4) BandeF = 1;
 //   Serial.println(BandeF);
    if (BandeF == 1) {
      SSb =0;
        if (Vfo == 1){
         encoderValue = (Ba1 / PAS); // MODIF VFO A
       }
       else {
        encoderValue2 = (Ba1 / PAS); // MODIF VFO A
      }
    }
    else if (BandeF == 2) {
      SSb=1 ;
       if (Vfo == 1){
         encoderValue = (Ba2 / PAS); // MODIF VFO A
       }
      else {
        encoderValue2 = (Ba2 / PAS); // MODIF VFO A
      }
    }
   else if (BandeF == 3) {
    SSb=0 ;
    if (Vfo == 1){
       encoderValue = (Ba3 / PAS); // MODIF VFO A
     }
     else {
       encoderValue2 = (Ba3 / PAS); // MODIF VFO A
     }
   }
  else if (BandeF == 4) {
    SSb=1 ;
    if (Vfo == 1){
       encoderValue = (Ba4 / PAS); // MODIF VFO A
     }
     else {
       encoderValue2 = (Ba4 / PAS); // MODIF VFO A
     }
   
   }
  }
    if (customKey == 68){ // 68 POUR D COMENDE RIT
     if (RITflag == 1){
      RITflag = 0 ;
      tft.setTextColor(RED,BLACK);
      tft.setTextSize(2);
      tft.setCursor(280, PosiVFO2);
      tft.print("         ");
      encoderValue2 = RITval ;
      encoderValue2 = encoMemo2 ;
      afVFOA (WHITE);
      if (Vfo == 1){
        VFOcamende ( FRtx );
      }
     }
     else {
      RITflag ++ ;
      tft.setTextColor(RED,BLACK);
      tft.setTextSize(2);
      encoderValue2 = RITval ;   
     }
     
    }
    if (customKey == 67){ 
      ECRtmenu ();
      
    
    }
    if (customKey == 65){ // 65 POUR A COMENDE VFO A VFO B
      if (Vfo == 2){
      Vfo = 1 ;
      tft.setTextColor(GREEN,BLACK);
      tft.setTextSize(2);
      tft.setCursor(0, (PosiFR - 20));
      tft.println("VFO A");
      
      tft.setTextColor(WHITE,BLACK);
      tft.setCursor(0, PosiVFO2);
      tft.println("VFO");
      tft.println(" B ");
      VFOcamende ( FRtx );
      }
     else{
      Vfo = 2 ;
      tft.setTextColor(WHITE,BLACK);
      tft.setTextSize(2);
      tft.setCursor(0, (PosiFR - 20));
      tft.println("VFO A");
      
      tft.setTextColor(GREEN,BLACK);
      tft.setCursor(0, PosiVFO2);
      tft.println("VFO");
      tft.println(" B ");
      VFOcamende ( FRtx2 );   
     }
    }
    if (customKey == 66){ // 65 POUR B COMENDE USB LSB
     if (SSb == 0){
      SSb = 1;
     }
     else{
      SSb = 0;
     }
     if (Vfo == 1) VFOcamende ( FRtx );
     if (Vfo == 2) VFOcamende ( FRtx2 );
    }
    if (customKey == 70){ // 70 POUR F COMENDE PAS DES VFO
    PAScomende ();
    }
  }
  
  //long frx = 1;


// --- VFO A
  if (encoMemo != encoderValue)
  {
    encoMemo = encoderValue ;
    FRtx = (encoMemo * PAS);
   // tft.setTextColor(WHITE,BLACK);
    afVFOA (WHITE);
    if (Vfo == 1){
     VFOcamende ( FRtx );
    }
  
  }

// ------ VFO B  
if (RITflag == 0){
 if (encoMemo2 != encoderValue2)
  {
    encoMemo2 = encoderValue2 ;
   FRtx2 = (encoMemo2 * PAS);
  // tft.setTextColor(WHITE,BLACK);
  // tft.setTextSize(5);
  // tft.setCursor(40, PosiVFO2);
 //  tft.print(FRtx2);
   afVFOB (WHITE);
   if (Vfo == 2){
      VFOcamende ( FRtx2 );
     }
   }
  }
  //------- RIT ---------------------------------
  if (RITflag == 1 && PTTflag == 1){
     RITval = encoderValue2 ;
     tft.setTextColor(RED,BLACK);
     tft.setTextSize(2);
     tft.setCursor(280, PosiVFO2);
     tft.println("RIT");
     if (RITval == 0){
      tft.setCursor(320, PosiVFO2);
      tft.print("       ");
     }
     else{
     tft.setCursor(320, PosiVFO2);
     tft.print((RITval*10));
     }
    tft.setTextColor(WHITE2,BLACK);
    tft.setTextSize(10);
    if ( BandeF == 4) {
      tft.setCursor(0, PosiFR);
    }
    else{
      tft.setCursor(0, PosiFR);
      tft.print(" ");
      tft.setCursor(60, PosiFR);
    }
    tft.print((FRtx + (RITval*10)));
    if (Vfo == 1){
     VFOcamende ((FRtx + (RITval*10)));
    }
  }
  
  //-------- GETITION BOUTONS -----------------------
 
   flagBoutCodeur = digitalRead(encoder2bout);
  if (flagBoutCodeur == 0){
  // PAScomende (); 
  }
 
}

//****************************************************************************************************************************
// ----- CLACUL DE LA FREQUENCE V.F.O ET ENVOI
//*****************************************************************************************************************************

void VFOcamende(long FRV){
 
  tft.setTextSize(2);
  relayBande ( FRV );
   if (SSb == 0)
    {
      Fr = (FiLSB - FRV);
     tft.setTextColor(CYAN,BLACK);
     tft.setCursor(0, 50);
     tft.println("LSB");
    }
    else
    {
      Fr = (FiUSB + FRV);
      tft.setTextColor(CYAN,BLACK);
      tft.setCursor(0, 50);
      tft.println("USB");
    }
    
    // Serial.print("fr desirré : ");
   //  Serial.println(Fr);
 
     DDS.setfreq(Fr, 0); 
}
//******************************************************************************************************************
//------ PAS DES VFOs------------------------------------------------------------------------------------------
//***************************************************************************************************************

void PAScomende(){
   PAS =(PAS * 10) ;
    if (PAS > 1000) PAS = 10; // MAX 1000Hz
    tft.drawLine(220, (PosiFR + 80), 420, (PosiFR + 80),BLACK );// EFASEMENT
    if (PAS == 10)    tft.drawLine(362, (PosiFR + 80), 412, (PosiFR + 80),YELLOW );// 10Hz
    if (PAS == 100)   tft.drawLine(302, (PosiFR + 80), 350, (PosiFR + 80),YELLOW );// 100Hz
    if (PAS == 1000)  tft.drawLine(240, (PosiFR + 80), 290, (PosiFR + 80),YELLOW );// 1000Hz
    encoderValue = (FRtx / PAS); // MODIF VFO A
    encoderValue2 = (FRtx2 / PAS); // MODIF VFO B
   
}

//************************************************************************************************************************************
//-----------  S-METRE -------------------------------------------------------------------------------------------------------------
//***********************************************************************************************************************************

void sMetre (int siniale){
 siT ++ ;
// Serial.println(siT);
 if (siT > 250){ //TEMP MEMOIRE
  tft.drawLine(siM, 309, siM, 317,BLACK );
  tft.drawLine((siM + 1), 309, (siM + 1), 317,BLACK );
  siM = 0 ;
  siT = 0 ;
 }
 if (siniale > 479) siniale = 479;
 if (siniale > siM){
  siM = siniale;
  tft.drawLine(siM, 309, siM, 317,GREEN );
  tft.drawLine((siM + 1), 309, (siM + 1), 317,GREEN );
 }
  int COLOR = WHITE ;
 // siniale = (siniale / 0.9);
  
  if ( siniale > si){
    while (1){
      if (si > 271) COLOR = RED ;
      tft.drawLine(si, 309, si, 317, COLOR); 
      si ++;
      if (si == siniale ){
        
        return ;  
      }
  }
 } 
  if ( siniale < si){
   while (1){
      si --; 
      tft.drawLine(si, 309, si, 317, BLACK); 
      if (si == siniale )  return ;  
    }
    for (int i=0; i <= 7; i++){ 
    tft.drawLine((i*68), 299, (i*68), 319, RED);
    tft.drawLine(((i*68)+34), 306, ((i*68)+34), 319, RED);
  }
 }
}
void sMetreSlo (int siniale){
 if (siniale > 479) siniale = 479;
  
  int COLOR = WHITE ;
 // siniale = (siniale / 0.9);
  
  if ( siniale > si){
    if (si != siniale ){
      if (si > 271) COLOR = RED ;
      tft.drawLine(si, 309, si, 317, COLOR); 
      si ++;  
    }  
  }
  
  if ( siniale < si){
   if (si != siniale ){
      tft.drawLine(si, 309, si, 317, BLACK); 
      si --;  
    }
  }
}

//*********************************************************************************************************************
//----- TIME

void Time() {
  tft.setTextSize(2);
  tft.setTextColor(WHITE,BLACK);
  tft.setCursor(70, 50);
  tmElements_t tm;

  if (RTC.read(tm)) {

   // tft.println("S");
    print2digits(tm.Hour);
    tft.write(':');
    print2digits(tm.Minute);
    tft.write(':');
    print2digits(tm.Second);
    tft.print("  ");
    tft.print(tm.Day);
    tft.write('/');
    tft.print(tm.Month);
    tft.write('/');
    tft.print(tmYearToCalendar(tm.Year));
    tft.println();
  } else {
    if (RTC.chipPresent()) {
      tft.println("Le DS1307 est arrêté. Veuillez exécuter le SetTime");
     // tft.println("example to initialize the time and begin running.");
      //Serial.println();
    } else {
      tft.println("Erreur de lecture  BUS I2C : RTC");
      //Serial.println();
    }
   // delay(9000);
  }
 // delay(1000);
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    tft.write('0');
  }
  tft.print(number);
}

//*********************************************************************************************************************
// -------- COMMUTATION BANDE--------------------------------------------------------------------

void relayBande (long FRr){
  // Serial.println(FRr);
  if (FRr > 3500000 && FRr < 3800000){
  digitalWrite(r80, 1);
  digitalWrite(r60, 0);
  digitalWrite(r40, 0);
  digitalWrite(r20, 0);
  digitalWrite(rG, 0);
  OrBandeflag = 0;
  }
  else if (FRr > 5351500 && FRr < 5366500){
  digitalWrite(r80, 0);
  digitalWrite(r60, 1);
  digitalWrite(r40, 0);
  digitalWrite(r20, 0);
  digitalWrite(rG, 0);
  OrBandeflag = 0;
  }
  else if (FRr > 7000000 && FRr < 7200000){
  digitalWrite(r80, 0);
  digitalWrite(r60, 0);
  digitalWrite(r40, 1);
  digitalWrite(r20, 0);
  digitalWrite(rG, 0);
  OrBandeflag = 0;
  }
 else if (FRr > 14000000 && FRr < 14350000){
  digitalWrite(r80, 0);
  digitalWrite(r60, 0);
  digitalWrite(r40, 0);
  digitalWrite(r20, 1);
  digitalWrite(rG, 0);
  OrBandeflag = 0;
  }
  else{
  digitalWrite(r80, 0);
  digitalWrite(r60, 0);
  digitalWrite(r40, 0);
  digitalWrite(r20, 0);
  digitalWrite(rG, 1);
  OrBandeflag = 1;
  }
}
//********************************************************************************************************************
//---- afichage du VFO A-------------------------

void afVFOA (int colorVA){
   tft.setTextColor(colorVA,BLACK);
   tft.setTextSize(10);
   if ( FRtx > 9999999) {
    tft.setCursor(0, PosiFR);
   }
   else{
    tft.setCursor(0, PosiFR);
    tft.print(" ");
    tft.setCursor(60, PosiFR);
   }
   tft.print(FRtx);
}
void afVFOB (int colorVA){
   tft.setTextColor(colorVA,BLACK);
   tft.setTextSize(5);
   tft.setCursor(40, PosiVFO2);
   if ( FRtx2 < 10000000) {
    tft.print(" ");
   } 
   tft.print(FRtx2);
}
//***********************************************************************************************************************
//-------- clavier-----------------------------------------------------------------------------------------

void ClavierFR (int Vale){
  Vale = Vale - 48 ;
 
  
  if (Vfo == 1){
   if (cotClavier == 6){
   tft.setTextColor(RED,BLACK);
   tft.setTextSize(10);
    tft.setCursor(0, PosiFR);
    tft.print("        ");
    FRtx = 0;
    afVFOA (BLUE) ;
   }
   FRtx = (FRtx + Vale);
   afVFOA (BLUE) ;
   FRtx =(FRtx * 10) ;
   cotClavier -- ;
   if (cotClavier == 1){
    FRtx = (FRtx * 100);
    afVFOA (WHITE) ;
    encoderValue = (FRtx / PAS);
    cotClavier =6 ;
   }
  }
 
 if (Vfo == 2){
   if (cotClavier == 6){
    tft.setTextColor(RED,BLACK);
    tft.setTextSize(5);
    tft.setCursor(40, PosiVFO2);
    tft.print("        ");
    FRtx2 = 0;
    afVFOB (BLUE) ;
   }
   FRtx2 = (FRtx2 + Vale);
   afVFOB (BLUE) ;
   FRtx2 =(FRtx2 * 10) ;
   cotClavier -- ;
   if (cotClavier == 1){
    FRtx2 = (FRtx2 * 100);
    afVFOB (WHITE) ;
    encoderValue2 = (FRtx2 / PAS);
    cotClavier =6 ;
   }
  } 
 
}
