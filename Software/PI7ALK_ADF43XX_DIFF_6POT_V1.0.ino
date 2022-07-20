/* 
 ADF4350/51 PLL Frequency Init and beacon message.

  Beacon is modulating by changing resistor values of
  the OCXO control voltage. Differential control CW
  Potmeter P0 is always active.
  
 " NEVER TESTED  might have errors "

   
 CW beacon software for PLL ADF4351 and Aruino Nano
 can be compiled with arduino IDE.

  Inspired by:
 
 V 0.1 Ondra OK1CDJ 9/2018 ondra@ok1cdj.com
 
 Parts of this code is based on routines written by PA3FYM

 Morse Based on Simple Arduino Morse Beacon  
 Written by Mark VandeWettering K6HX

 PI4 sending from code from BO OZ2M, https://www.rudius.net/oz2m/ 
 PI4 Data created by https://rudius.net/oz2m/ngnb/pi4encoding.php
 
 PI4, 146 symbols take 24.333382s
 ~ symbol time = 166.664ms
 ~ 6Hz symbol_rate
 

 Connections for PCB Master OCXO 
----------------------------------------------------------------
 Please check the ADF4351 datasheet or use ADF4351 software
 to get PLL register values. ADF reference OCXO is 10 Mhz.

 
 Arduino PIN layout
 ----------------------------------------------------------------

 Directly adressed using CPU registers
 
 D12 PB4 ADF4531 clock (CLK)   
 D11 PB3 ADF4531 data (DATA)
 D10 PB2 ADF4531 latch enable (LE)

 IDE selected
 
 D8 PB0  GPST = 8 1-PPM & Testmode

 D2 PD2    CW = 2;  CPU to Switch 1 Control Pin
 D3 PD3 SPACE = 3;  CPU to Switch 2 Control Pin
  
 D4 PD4 PI4_0 = 4;  CPU to Switch 3 Control Pin
 D5 PD5 PI4_1 = 5;  CPU to Switch 4 Control Pin
 D6 PD6 PI4_2 = 6;  CPU to Switch 5 Control Pin
 D7 PD7 PI4_3 = 7;  CPU to Switch 6 Control Pin



  Potmeter Setup
----------------------------------------------------------------
 Needs hardware to test


 Frequency DATA for 3400.925 mhz PI7ALK
 
  PI4 Tone | Caculated Frequncy    | Potemeter   
----------------------------------------------------------------
 PI4 tone0 : 3.400.924.882,8125 Hz ;  P1 
 PI4 tone1 : 3.400.925.117,1875 Hz ;  P2 
 PI4 tone2 : 3.400.925.351,5625 Hz ;  P3
 PI4 tone3 : 3.400.925.585,9375 Hz ;  P4 
  
 CW-Space  : 3.400.924.600,0000 Hz ;  P5
 CW        : 3.400.926.000,0000 Hz ;  P0 

 Version
 ----------------------------------------------------------------
 V1.0 Initial release.
      Differentai 6 Potmeter Control for CW, SPACE and
      PI4_0 to PI4_3. the CW potmeter is always active.  

*/



struct t_mtab {
  char c, pat;
} ;

// Bits represent morse code from LSB  1 dah, 0 dit, last bit is always 1 so "P" = 22 = (1)0110 

struct t_mtab morsetab[] = {
  {'.', 106},
  {',', 115},
  {'?', 76},
  {'/', 41},
  {'A', 6},
  {'B', 17},
  {'C', 21},
  {'D', 9},
  {'E', 2},
  {'F', 20},
  {'G', 11},
  {'H', 16},
  {'I', 4},
  {'J', 30},
  {'K', 13},
  {'L', 18},
  {'M', 7},
  {'N', 5},
  {'O', 15},
  {'P', 22},
  {'Q', 27},
  {'R', 10},
  {'S', 8},
  {'T', 3},
  {'U', 12},
  {'V', 24},
  {'W', 14},
  {'X', 25},
  {'Y', 29},
  {'Z', 19},
  {'1', 62},
  {'2', 60},
  {'3', 56},
  {'4', 48},
  {'5', 32},
  {'6', 33},
  {'7', 35},
  {'8', 39},
  {'9', 47},
  {'0', 63}
} ;

#define N_MORSE  (sizeof(morsetab)/sizeof(morsetab[0]))

int dotlen;
int dashlen;

/* Variable for millis timers */
const uint32_t timeout_time = 30000;
const uint32_t car_time = 59500;

bool cw_only = false;


/* Arduino Nano Pin definition we need 6 pins to control 6 switches
   and 1 for GPS 1 PPM and Testmode selection */


const int8_t CW = 2;       // CW to CMOS Control Pin
const int8_t SPACE = 3;    // SPACE to CMOS Control Pin
  
const int8_t PI4_0 = 4;    // PI4 to CMOS Control Pin
const int8_t PI4_1 = 5;    // PI4 to CMOS Control Pin
const int8_t PI4_2 = 6;    // PI4 to CMOS Control Pin
const int8_t PI4_3 = 7;    // PI4 to CMOS Control Pin

const int8_t GPST = 8;     // GPS Input Pin


char *txstr;

/*
 Data created by https://rudius.net/oz2m/ngnb/pi4encoding.php
 146 symbols take 24.333382s , symbol time = 166.664ms ~ 6Hz symbol_rate
 PA3AXA/B, 432,200 Mhz
 
*/

// PI7ALK, 3400,925 Mhz
//
int8_t fsymbols[] = {2,0,3,2,0,1,3,3,1,2,3,2,3,2,1,2,0,1,2,2,0,1,2,2,2,3,3,2,2,1,3,1,3,0,0,1,
                     1,1,1,3,0,0,1,1,2,1,3,3,1,0,1,2,3,3,0,1,3,0,1,2,0,2,0,0,1,3,3,3,1,0,3,0,
                     1,2,0,2,0,2,3,3,3,3,3,2,1,0,0,3,0,2,3,2,3,2,0,0,2,3,0,0,1,3,0,2,2,0,2,3,
                     1,0,2,2,0,1,3,2,2,1,1,3,2,3,3,1,0,1,3,0,3,2,3,2,3,2,2,0,2,1,1,3,0,2,2,2,1,1};


/* Init PI4 symbol time in ms
   A delay of 166; this gives a measured latch time of 167,6 is ~ ms 1.3ms */
const int dsymbol = 165;
 
/*Int PWM values L0 is half range, expected range 27, 127, 227 Hz
 Values need to be adjusted to the hardware in the sendpi4().*/

/* ADF PLL registers */
long int r0, r1, r2, r3, r4, r5;


// Write data to ADF code developed by PA0FYM, PA3AXA Hardware
//------------------------------------------------------------

void write2PLL(uint32_t PLLword) {          // clocks 32 bits word  directly to the ADF4351
                                            // msb (b31) first, lsb (b0) last

  noInterrupts();                           // disable interrupts to keep accurate timing. 
  
  for (byte i=32; i>0; i--) {               // PLL word 32 bits
     
    (PLLword & 0x80000000? PORTB |= 0b00001000 : PORTB &= 0b11110111);   // data on PB3
                                                                               
    PORTB |= 0b00010000;                   // clock in bit on rising edge of CLK (PB4 = 1)
    PORTB &= 0b11101111;                   // CLK (PB4 = 0)      
    PLLword <<= 1;                         // rotate left for next bit
    }
    PORTB |= 0b00000100;                   // latch in PLL word on rising edge of LE (PB2 = 1)
    PORTB &= 0b11111011;                   // LE (PB2 = 0)      

  interrupts();                           // enable interrupts 

} 


// FSK CW Routines
//------------------------------------------------------------

void dit(){

  // FSK CW - 400 Hz
  
  digitalWrite(CW, HIGH);              // CW
  delay(dotlen);
  
  digitalWrite(SPACE, HIGH);          // SET CW SPACE -400
  delay(dotlen);
  digitalWrite(SPACE, LOW);           // CLEAR CW SPACE -400  
  }


void dash(){

  // FSK CW - 400 Hz
  
  digitalWrite(CW, HIGH);             // CW Default always on
  delay(dashlen);
  
  digitalWrite(SPACE, HIGH);          // SET SPACE -400
  delay(dotlen);
  digitalWrite(SPACE, LOW);           // CLEAR CW SPACE -400
  }


// Look up a character in the tokens array and send it
void send(char c)
{
  int i ;
  if (c == ' ') {

    delay(7 * dotlen) ;
    return ;
  }
  for (i = 0; i < N_MORSE; i++) {
    if (morsetab[i].c == c) {
      unsigned char p = morsetab[i].pat ;

      while (p != 1) {
        if (p & 1)
          dash() ;
        else
          dit() ;
        p = p / 2 ;
      }
      delay(2 * dotlen) ;
      return ;
    }
  }
 //if we drop off the end, then we send a space

}

void sendmsg(char *txstr)
{
  while (*txstr)
    send(*txstr++) ;
}




/* Send PI4 with 6 x GPI
------------------------------------------------------------
 We have to deal with symbols 0 to 3 and with 4 frequency's

  PI4 tone0               : 3.400.924.882,8125 Hz
  PI4 tone1               : 3.400.925.117,1875 Hz
  PI4 tone2               : 3.400.925.351,5625 Hz
  PI4 tone3               : 3.400.925.585,9375 Hz

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  
  
  const int8_t PI4_0 = D4;    // Define Pin PD4
  const int8_t PI4_1 = D5;    // Define Pin PD5
  const int8_t PI4_2 = D6;    // Define Pin PD6
  const int8_t PI4_3 = D7;    // Define Pin PD7

*/

void sendpi4(){
 
  for (int tx = 0 ; tx < 147 ; tx++){

    
      delay(dsymbol);
      
      /* We have to clear all tones */
      
      digitalWrite(PI4_0, LOW);
      digitalWrite(PI4_1, LOW);
      digitalWrite(PI4_2, LOW);
      digitalWrite(PI4_3, LOW);
      
      digitalWrite(SPACE, LOW);           
       
    switch (fsymbols[tx]){

      case 0:                     // 3.400.924.882,8125 Hz
 
      digitalWrite(PI4_0, HIGH);
      break;
      
      case 1:                     // 3.400.925.117,1875 Hz

      digitalWrite(PI4_1, HIGH);
      break;
     
      case 2:                     // 3.400.925.351,5625 Hz

      digitalWrite(PI4_2, HIGH);
      break;
      
      case 3:                     // 3.400.925.585,9375 Hz

      digitalWrite(PI4_3, HIGH);
      break;  
    }       

  }

  /* Make sure PI4 tones are cleared */
  
   digitalWrite(PI4_0, LOW);
   digitalWrite(PI4_1, LOW);
   digitalWrite(PI4_2, LOW);
   digitalWrite(PI4_3, LOW);

}


                                   
// Setup Hardware Defaults
//------------------------------------------------------------

void setup () {

delay(2000);                     // Wait for ADF5341 to powerup

/* Pre Init to program the ADF4350/51 */

  DDRB  = 0xff; // PB are all outputs 
  PORTB = 0x00; // make PB low

/*

Calculated Frequencies PI7ALK

CW carrier/CW mark frequency      : 3.400.925.000 Hz
CW FSK space frequency, -400 Hz   : 3.400.924.600 Hz
                      PI4 tone0   : 3.400.924.882,8125 Hz
                      PI4 tone1   : 3.400.925.117,1875 Hz
                      PI4 tone2   : 3.400.925.351,5625 Hz
                      PI4 tone3   : 3.400.925.585,9375 Hz

 F_out ADF43XX = 3400.925.000 MHz +2dBm  only RF port 10Mhz REF 


 /* Set to MAX Channel Spacing for correct CW frequency (25khz in this case) */
 
  r0 = 0x00AA0128;
  r1 = 0x08008C81;
  r2 = 0x60004E42; // Low spur mode
  r3 = 0x000004B3; 
  r4 = 0x00085034;  // +2 dbm RF OUTPUT
  r5 = 0x00580005;


/* write from r5 to r0 to init ADF4350/ADF4351 */
 
     write2PLL(r5); 
     write2PLL(r4); 
     write2PLL(r3);
     write2PLL(r2);
     write2PLL(r1);
     write2PLL(r0);

 /* Setup CMOS Switch PinMode */
 
  pinMode(CW, OUTPUT);
  pinMode(SPACE, OUTPUT);
  
  pinMode(PI4_0, OUTPUT);
  pinMode(PI4_1, OUTPUT);
  pinMode(PI4_2, OUTPUT);
  pinMode(PI4_3, OUTPUT);

/* Setup PIN D8  for GPI_IN 1PPM & Test mode */ 
  
  pinMode(GPST, INPUT_PULLUP );     // GPS to control PPM input


/* set speed and text for CW in WPM */
     
     int wpm = 12;

     dotlen = (1200 / wpm);
     dashlen = (3 * (1200 / wpm));
     txstr = "PI7ALK JO22IP";


  
  /* Here we select to start the Alignment en Testmode*/  
  if ( digitalRead(GPST) == LOW ) 
          {
          delay(1000); // simple debouce 
          testmode();
          }

/* Start Up Defaults */

digitalWrite(CW, HIGH);

digitalWrite(SPACE, LOW);
digitalWrite(PI4_0, LOW);
digitalWrite(PI4_1, LOW);
digitalWrite(PI4_2, LOW);
digitalWrite(PI4_3, LOW); 
         
} // End Setup



// Testmode
//-------------------------------------------------------------------------------------------------

void testmode(){


/* Helper tool to Align 6 frequency's first align CW than move to 
 * the second option and PI4-RX to align all 5 other frequency's 
 * 
 * Not tested yet feedback is always appreciated
 * 
 */
       
while ( digitalRead(GPST) == HIGH )
      {
      delay(dsymbol);
      digitalWrite(CW, HIGH);
      digitalWrite(SPACE, LOW);
      digitalWrite(PI4_0, LOW);
      digitalWrite(PI4_1, LOW);
      digitalWrite(PI4_2, LOW);
      digitalWrite(PI4_3, LOW);
      } 

while ( digitalRead(GPST) == HIGH )
      {
      delay(dsymbol);
      digitalWrite(CW, HIGH);
      digitalWrite(SPACE, LOW);
      digitalWrite(PI4_0, LOW);
      digitalWrite(PI4_1, LOW);
      digitalWrite(PI4_2, LOW);
      digitalWrite(PI4_3, LOW);  
      
      delay(dsymbol);
      digitalWrite(CW, HIGH);
      digitalWrite(SPACE, HIGH);
      digitalWrite(PI4_0, LOW);
      digitalWrite(PI4_1, LOW);
      digitalWrite(PI4_2, LOW);
      digitalWrite(PI4_3, LOW);
      
      delay(dsymbol);
      digitalWrite(CW, HIGH);
      digitalWrite(SPACE, LOW);
      digitalWrite(PI4_0, HIGH);
      digitalWrite(PI4_1, LOW);
      digitalWrite(PI4_2, LOW);
      digitalWrite(PI4_3, LOW); 

      delay(dsymbol);
      digitalWrite(CW, HIGH);
      digitalWrite(SPACE, LOW);
      digitalWrite(PI4_0, LOW);
      digitalWrite(PI4_1, HIGH);
      digitalWrite(PI4_2, LOW);
      digitalWrite(PI4_3, LOW); 

      delay(dsymbol);
      digitalWrite(CW, HIGH);
      digitalWrite(SPACE, LOW);
      digitalWrite(PI4_0, LOW);
      digitalWrite(PI4_1, LOW);
      digitalWrite(PI4_2, HIGH);
      digitalWrite(PI4_3, LOW); 
      
      delay(dsymbol);
      digitalWrite(CW, HIGH);
      digitalWrite(SPACE, LOW);
      digitalWrite(PI4_0, LOW);
      digitalWrite(PI4_1, LOW);
      digitalWrite(PI4_2, LOW);
      digitalWrite(PI4_3, HIGH);
      }
      /* Run it again */
      testmode();

} // To end testmode Reboot



/* main loop total sequence should last 60 sec when transmitting PI4 */
//------------------------------------------------------------------------------------------------- 

void loop() {



      // Start Time looptime
      uint32_t loop_time = millis();

      // T = 0, PI4 message
      if ( cw_only == false ){
        sendpi4();
        }

      digitalWrite(CW, HIGH);            // SET CW
      // T = 24.333
      digitalWrite(SPACE, HIGH);         // SET SPACE   -400 hz
      delay(667);
      digitalWrite(SPACE, LOW);          // CLEAR SPACE -400 hz

      // T = 25, CW Message
      sendmsg(txstr);

      // T ~ 40, SPACE for 500mS
      digitalWrite( SPACE, HIGH);         // SET SPACE   -400 hz
      delay(500);
      digitalWrite(SPACE, LOW);          // CLEAR SPACE -400 hz
                      
      // Util T = 59.5 CW  
      
      do {
        /* 
           For external PPM make this a little shorter so we have some
           waiting time for the puls to arrive.
           
           The car_time and loop_time variables needs to be
           uint32_t, the same as millis() then this
           will never fail.

           Early resync option when PPM pulse returns
      
         */    
            delay(10);      // Reduce loop speed
         
           } while ( (millis() - loop_time) <= car_time );
          

      // T=59,5
      digitalWrite(SPACE, HIGH);   // SET SPACE -400

      loop_time = millis();

      do {
          /*
         Time out Wait for External 1 PPM Pulse 
         after 30 sec no PPM we switch to CW beacon only
         until PPM returns
        */
           if ( (millis() - loop_time) >= timeout_time ){
            sendmsg(txstr);
            loop_time = millis();
           }
          
        delay(10);      // Reduce loop speed
        
        /* When LOW we have 1-PPM and restart */ 
        } while (digitalRead(GPST) == HIGH );

        cw_only = false;
        digitalWrite(SPACE, LOW);   // CLEAR SPACE -400
        
} // Mainloop end
      
