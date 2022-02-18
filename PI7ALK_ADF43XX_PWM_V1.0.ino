/* 
 ADF4350/51 PLL Frequency Init and beacon message.

  Beacon can be PWM by modulating the OCXO control
  voltage, or CW by switching PLL on the ADF4351

 Inspired by:
   
 CW beacon software for PLL ADF4351 and ATTiny45 
 can be compiled with arduino 
 v 0.1 Ondra OK1CDJ 9/2018 ondra@ok1cdj.com
 
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

 
 Attiny85 PIN layout
 ----------------------------------------------------------------

 PB4 ADF4531 clock (CLK)   
 PB3 ADF4531 data (DATA)
 PB2 ADF4531 latch enable (LE)

 PB0 PWM for PI4 beacon ( might need CW reversing code )
 PB1 ADF4531 Control can be used for RX/TX frequency control or
 external Morsecode input for CW of FSK TX.

  PWM range and Calculation
----------------------------------------------------------------
 PLL steps in 200 khz to match the PI4 frequency requirement
 we use the PWM from the Attiny 85 at a PWM rate of 31,25 Khz.
 Standard Aruino IDE is to slow and produces audible modulation.
 
 Attiny85 Clock is internal clock is set to 8 Mhz
 
 All measurements are made from a starting point with PWM set
 to 50% = 127. This should be the CW Carrier Frequency 

 There is a buildin test mode this initiated by holding the 1PPM
 input to ground during boot. The frequency will loop between
 3 frequency's by just changing the PWM values between 27, 127
 and 227, this should result in 3 frequency's that cover the whole
 PI4 frequency range e.g -400 hz to +585.

 A 2 second  ground off 1PPM input changefrom loop to PWM 127 this should match
 the desired CW Freqency of in this case 3400.925.000 hz. A 2 second
 ground 1PPM input will return to loop between 27.127.227 PWM. 

 The exit test mode reset the Arduino Nano.

 After this alignment we can modify the PLL output on 3400 MHZ with a
 resolution between ~ 4 to 6 Hz due to poor linearity with the test systems.
 
 PVM value for the calculated STEPS
 
  PI4 Tone | Caculated Frequncy    | PWM Offset   
----------------------------------------------------------------
 PI4 tone0 : 3.400.924.882,8125 Hz ;  157 
 PI4 tone1 : 3.400.925.117,1875 Hz ;  107 
 PI4 tone2 : 3.400.925.351,5625 Hz ;   68 
 PI4 tone3 : 3.400.925.585,9375 Hz ;   27 
  
 CW-Space  : 3.400.924.600,0000 Hz ;  227 

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

/* Attiny85 Pin definition */
const int8_t PWM_pin = 1;    // Define PWM Pin PB1
const int8_t GPI_pin = 0;    // Define GPI Input Pin PB0

char *txstr;

/*
 Data created by https://rudius.net/oz2m/ngnb/pi4encoding.php
 146 symbols take 24.333382s , symbol time = 166.664ms ~ 6Hz symbol_rate
 PA3AXA/B, 432,200 Mhz
 
*/

 // PA3AXA/B
 //
int8_t fsymbols[] = {2,0,3,2,2,1,1,1,1,0,3,2,3,0,3,2,0,1,0,0,0,3,2,0,0,3,1,0,2,3,3,3,3,2,0,3,
                     3,1,3,1,0,0,1,1,2,1,3,3,1,0,1,0,1,3,2,1,3,0,1,0,2,2,0,2,1,1,1,1,3,2,1,0,
                     1,0,2,0,0,2,3,3,3,1,1,2,1,0,0,3,2,0,1,2,3,2,2,2,2,1,2,2,1,1,2,0,0,2,2,3,
                     3,0,0,2,0,1,3,2,0,1,3,3,0,1,3,3,0,1,3,2,3,0,3,0,1,2,0,0,2,3,3,3,2,2,0,0,1,3};


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
  
  analogWrite(PWM_pin, 127);           // PWM CW
  delay(dotlen);
  
  analogWrite(PWM_pin, 227);           // PWM CW SPACE -400
  delay(dotlen);  
  }


void dash(){

  // FSK CW - 400 Hz
  
  analogWrite(PWM_pin, 127);           // PWM CW
  delay(dashlen);
  
  analogWrite(PWM_pin, 227);           // PWM CW SPACE -400
  delay(dotlen);
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




/* Send PI4
------------------------------------------------------------
 We have to deal with symbols 0 to 3 and with 4 frequency's

  PI4 tone0               : 3.400.924.882,8125 Hz
  PI4 tone1               : 3.400.925.117,1875 Hz
  PI4 tone2               : 3.400.925.351,5625 Hz
  PI4 tone3               : 3.400.925.585,9375 Hz

*/

void sendpi4(){
 
  for (int tx = 0 ; tx < 147 ; tx++){

    switch (fsymbols[tx]){
       
      case 0:                     // 3.400.924.882,8125 Hz

      delay(dsymbol);
      analogWrite(PWM_pin, 157);
      break;
      
      case 1:                     // 3.400.925.117,1875 Hz

      delay(dsymbol);
      analogWrite(PWM_pin, 107);
      break;
     
      case 2:                     // 3.400.925.351,5625 Hz
      
      delay(dsymbol);
      analogWrite(PWM_pin, 68);
      break;
      
      case 3:                     // 3.400.925.585,9375 Hz

      delay(dsymbol);
      analogWrite(PWM_pin, 27);
      break;  
    }       

  }

}


                                   
// Setup Hardware Defaults
//------------------------------------------------------------

void setup () {

delay(2000);                     // Wait for ADF5341 to powerup


 /*  Set TIMER1 to Fast PWM 8-bit TOP=255
  *  
  *  TCCR1  : Timer/Counter1 Control Register
  *  PWM1A  : Enable PWM Timer 1
  *  COM1A1 : CompareMode Select, Clear Compared Match
  *  CS10   : No prescaler 8bit counter clocked @ 8 Mhz
  *           equals a PWM rate of 31,372 Knz
  *

  * Arduino NANO Timers, IDE uses Timer 0
  *  
  *  Timer0 = PIN D5 &  D6
  *  Timer1 = PIN D9 & D10
  *  Timer2 = PIN D3 & D11
  *
  * Arunino Nano Timer 1 & Timer 2
 */ 
   
   /* Nano Timer1 PIN D9 & D10 */
   // TCCR1B = TCCR1B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
  
  /* Nano Timer 2 PIN D3 & D11 */
  // TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz
   
  /* Attiny85 Timer1 */
  TCCR1 = _BV(PWM1A)  | _BV(COM1A1) | _BV(CS10);

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


Programed Frequency's PI7ALK 9CM 3400,92500 Mhz

 r0 = 0xAA0AED0; Fout = 3400.924.600 MHz
 r0 = 0xAA0AED8; Fout = 3400.924.800 MHz
 r0 = 0xAA0AEE0; Fout = 3400.925.000 MHz
 r0 = 0xAA0AEE8; Fout = 3400.925.200 MHz
 r0 = 0xAA0AEF0; Fout = 3400.925.400 MHz
 r0 = 0xAA0AEF8; Fout = 3400.925.600 MHz

 Other registers

  r0 = 0xAA0AEE0;
  r1 = 0x80061A9; after init 0x180061A9;
  r2 = 0x60040E42; // Low spur mode
  r3 = 0x4B3; 
  r4 = 0x80502C;   // -1 dbm RF OUTPUT
  r5 = 0x580005;

 CW carrier/CW FSK mark frequency     : 3.400.925.000 Hz r0 = 0xAA0AEE0
 CW FSK space frequency, -400 Hz    : 3.400.924.600 Hz r0 = 0xAA0AED0

 After Base setting r2 to r5 don't change 
 Fout = 3400.925.000 MHz +5dBm  only RF port 10Mhz REF 
 Default Register Value */

  r0 = 0xAA0AEE0;
  r1 = 0x80061A9;
  r2 = 0x60040E42; // Low spur mode
  r3 = 0x4B3; 
  r4 = 0x80502C;  // -1 dbm RF OUTPUT
  r5 = 0x580005;

/* write from r5 to r0 to init ADF4350/ADF4351 */
 
     write2PLL(r5); 
     write2PLL(r4); 
     write2PLL(r3);
     write2PLL(r2);
     write2PLL(r1);
     write2PLL(r0);  

/* Setup PIN PB0 for PWM control */

  pinMode(PWM_pin,OUTPUT );       // Pin to control OCXO reference
 analogWrite(PWM_pin, 127);       // Set PWM to halfrange
 
 // digitalWrite(PWM_pin, LOW );  // Disable PWM for TEST


/* set speed of morse in WPM */
     
     int wpm = 12;

     dotlen = (1200 / wpm);
     dashlen = (3 * (1200 / wpm));
     txstr = "PA3AXA/B JO22PJ";


 /* Setup PIN D12  for GPI_IN 1PPM */ 
  
 pinMode(GPI_pin, INPUT_PULLUP );     // GPI_pin to control PPM input
 delay(100);                          // might not be needed

  
  /* Here we select to start the Alignment en Testmode*/  
  if ( digitalRead(GPI_pin) == LOW ) 
          { 
          testmode();
          }

         
} // End Setup



/* Testmode */
//-------------------------------------------------------------------------------------------------

void testmode(){


      // Set CW Carrier
      r0 = 0xAA0AEE0;                     // CW 
      write2PLL(r0);                      // ADF Write

while ( digitalRead(GPI_pin) == HIGH )
      {
      // Align Frequency Span
      analogWrite(PWM_pin, 127);    // PWM Correction 0 Hz
      delay(dsymbol);
      
      analogWrite(PWM_pin, 27);     // PWM Correction MAX+
      delay(dsymbol);

      analogWrite(PWM_pin, 227);    // PWM Correction MAX-
      delay(dsymbol);
      }
      
      delay(2000);
      
while ( digitalRead(GPI_pin) == HIGH )
      {
      // Align Carrier Frequentie 
      analogWrite(PWM_pin, 127);    // PWM Correction 0 Hz
      delay(dsymbol);
      }
      delay(2000);
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

      analogWrite(PWM_pin, 127);          // PWM CW
      // T = 24.333, CW_FSK
      analogWrite(PWM_pin, 227);          // PWM CW_FSK -400
      delay(667);

      // T = 25,  CW_FSK Message
      sendmsg(txstr);

      // T ~ 40,  CW_FSK
      analogWrite(PWM_pin, 227);          // PWM CW_FSK -400
      delay(500);
                      
      // T = YY, CW Carrier
      analogWrite(PWM_pin, 127);          // PWM CW
         
      do {
        /* Clock is to fast, with Attiny85 with no crystal so never right 
           Needs GPS PPM ( pulse / minute ) to sync TX
           Clock error compesation 59500 - 350.

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
      analogWrite(PWM_pin, 227);          // PWM CW_FSK -400

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

        } while (digitalRead(GPI_pin) == HIGH );

        cw_only = false;
        
} // Mainloop end
      
