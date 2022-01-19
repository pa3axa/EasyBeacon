/* 
 ADF4350/51 PLL Frequency Init and CW + PI4 beacon message.

  Beacon with PWM by modulating the OCXO control
  voltage, to accuratly generated the PI4 sequence
  ADF4351 has limitations to program the correct
  frequency's 

 Inspired by:
   
 CW beacon software for PLL ADF4351 and ATTiny45 
 can be compiled with arduino IDE
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

 
 Arduino NANO PIN layout
----------------------------------------------------------------

 PB0 ADF4531 latch enable (LE)
 PB1 ADF4531 data (DATA)
 PB2 ADF4531 clock (CLK)   


 D3 / NANO  PIN 3   - PWM for PI4 beacon
 D12 / NANO PIN 12  - 1PPM is indicate start of a minute
                      and testmode during boot. 
 
 PWM range and Calculation
----------------------------------------------------------------
 PLL steps in 200 khz to match the PI4 frequency requirement
 we use the PWM from the Attiny 85 at a PWM rate of 31,25 Khz.
 Standard Aruino IDE is to slow and produces audible modulation.
 
 Attiny85 Clock is internal clock is set to 8 Mhz
 
 All measurements are made from a starting point with PWM set
 to 50% = 127. This should be the CW Carrier Frequency 

 There is a buildin test mode this initiated by holding the 1PPM
 input to groud during boot. The frequency will loop between
 3 frequency's by just changing the PWM values between 27, 127
 and 227, this should result in 3 frequency's extact 100 Hz apart.

 A 2 second  ground off 1PPM input changefrom loop to PWM 127 this should match
 the desired CW Freqency of in this case 3400.925.000 hz. A 2 second
 ground 1PPM input will return to loop between 27.127.227 PWM. 

 The exit test mode reset the Arduino Nano.

 After this alignment we can modify the PLL output with 1 Hz / PWM Digit.
 
 PVM value for the calculated STEPS

  PI4 Tone | Caculated Frequncy   | PLL Frequnecy  | PWM Offset   
----------------------------------------------------------------
 PI4 tone0 : 3.400.924.882,8125 Hz; 3.400.924.800     +82 
 PI4 tone1 : 3.400.925.117,1875 Hz; 3.400.925.200     -83 
 PI4 tone2 : 3.400.925.351,5625 Hz; 3.400.924.400     -49 
 PI4 tone3 : 3.400.925.585,9375 Hz; 3.400.924.600     -15 


 PWM Frequency Offset to 8bit PWM Value
----------------------------------------------------------------
 PI4-0  +82 = 209
 Pi4-1  -83 = 44
 PI4-2  -49 = 78
 PI4-3  -15 = 112


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

/* Nano Pin definition */
const int8_t PWM_pin = 3;    // Define PWM Pin D3
const int8_t GPI_pin = 12;   // Define 1PPM Input Pin D13

char *txstr;

/*
 Data created by https://rudius.net/oz2m/ngnb/pi4encoding.php
 146 symbols take 24.333382s , symbol time = 166.664ms ~ 6Hz symbol_rate
 
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


/* Write data to ADF code developed by PA0FYM, PI7ALK Hardware */
//------------------------------------------------------------

void write2PLL(uint32_t PLLword) {          // clocks 32 bits word  directly to the ADF4351
                                            // msb (b31) first, lsb (b0) last

  noInterrupts();                           // disable interrupts to keep accurate timing. 
  
  for (byte i=32; i>0; i--) {               // PLL word 32 bits
     
    (PLLword & 0x80000000? PORTB |= 0b000000010 : PORTB &= 0b11111101);   // data on PB1
                                                                               
    PORTB |= 0b00000100;                   // clock in bit on rising edge of CLK (PB2 = 1)
    PORTB &= 0b11111011;                   // CLK (PB2 = 0)      
    PLLword <<= 1;                         // rotate left for next bit
    }
    PORTB |= 0b00000001;                   // latch in PLL word on rising edge of LE (PB0 = 1)
    PORTB &= 0b11111110;                   // LE (PB0 = 0)      

  interrupts();                           // enable interrupts 

} 


/* FSK CW Routines */
//------------------------------------------------------------

void dit(){

  // FSK CW - 400 Hz
  r0 = 0xAA0AEE0;                    // CW
  write2PLL(r0);                     // Write to ADF
  delay(dotlen);
  r0 = 0xAA0AED0;                    // CW SPACE -400
  write2PLL(r0);                     // Write to ADF
  delay(dotlen);  
  }


void dash(){

  // FSK CW - 400 Hz

  r0 = 0xAA0AEE0;                    // CW
  write2PLL(r0);                     // Write to ADF
  delay(dashlen);
  r0 = 0xAA0AED0;                    // CW SPACE -400
  write2PLL(r0);                     // Write to ADF
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

    PI4 tone0 :	3.400.924.882,8125 Hz
    PI4 tone1 :	3.400.925.117,1875 Hz
    PI4 tone2 :	3.400.925.351,5625 Hz
    PI4 tone3 :	3.400.925.585,9375 Hz
*/

void sendpi4(){
 
  for (int tx = 0 ; tx < 147 ; tx++){

    switch (fsymbols[tx]){
       
      case 0:                     // 3.400.924.882,8125 Hz

      r0 = 0xAA0AED8;             // 3.400.924.800,000 hz
      delay(dsymbol);
      analogWrite(PWM_pin, 209);  // PWM Correction +82 Hz
      write2PLL(r0);
      break;
      
      case 1:					            // 3.400.925.117,1875 Hz

      r0 = 0xAA0AEE8;             // 3.400.925.200,0000 hz
      delay(dsymbol);
      analogWrite(PWM_pin, 44);   // PWM Correction -83 Hz
      write2PLL(r0);
      break;
     
      case 2: 					          // 3.400.925.351,5625 Hz

      r0 = 0xAA0AEF0;             // 3.400.925.400,0000 Hz
      delay(dsymbol);
      analogWrite(PWM_pin, 78);   // PWM Correction -49 Hz
      write2PLL(r0);
      break;
      
      case 3: 					          // 3.400.925.585,9375 Hz

      r0 = 0xAA0AEF8;             // 3.400.925.600,0000 Hz
      delay(dsymbol);
      analogWrite(PWM_pin, 112);  // PWM Correction -15 Hz
      write2PLL(r0); 
      break;  
    }       

  }

}


                                   
/* Setup Hardware Defaults */
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
  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz


/* Pre Init to program the ADF4350/51 */

  DDRB  = 0xff; // PB are all outputs 
  PORTB = 0x00; // make PB low

/*

Calculated Frequencies PI7ALK

CW carrier/CW mark frequency 		  :	3.400.925.000 Hz
CW FSK space frequency, -400 Hz 	:	3.400.924.600 Hz
                      PI4 tone0 	:	3.400.924.882,8125 Hz
                      PI4 tone1 	:	3.400.925.117,1875 Hz
                      PI4 tone2 	:	3.400.925.351,5625 Hz
                      PI4 tone3 	:	3.400.925.585,9375 Hz


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

 CW carrier/CW FSK mark frequency 		:	3.400.925.000 Hz r0 = 0xAA0AEE0
 CW FSK space frequency, -400 Hz 	    :	3.400.924.600 Hz r0 = 0xAA0AED0

 After Base setting r2 to r5 don't change 
 Fout = 3400.925.000 MHz -1dBm  only RF port 10Mhz REF
 
 Default Register Values */

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

 pinMode(PWM_pin, OUTPUT );      // Pin to control OCXO reference
 analogWrite(PWM_pin, 127);       // Set PWM to halfrange
 
/* set speed of morse in WPM */
     
     int wpm = 12;

     dotlen = (1200 / wpm);
     dashlen = (3 * (1200 / wpm));
     txstr = "PI7ALK JO22IP";

 /* Setup PIN D12  for GPI_IN 1PPM */ 
  
 pinMode(GPI_pin, INPUT_PULLUP );     // GPI_pin to control PPM input

   
 delay(100);             // might not be needed
    
 r1 = 0x180061A9;        // disable VCO band switching after PLL frequentie init
 write2PLL(r1);

  
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
      analogWrite(PWM_pin, 127);    // PWM Correction 0 Hz
      delay(dsymbol);
      
      analogWrite(PWM_pin, 27);     // PWM Correction -100 Hz
      delay(dsymbol);

      analogWrite(PWM_pin, 227);    // PWM Correction +100 Hz
      delay(dsymbol);
      }
      
      delay(2000);
      
while ( digitalRead(GPI_pin) == HIGH )
      { 
      analogWrite(PWM_pin, 127);    // CW Carrier
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

      analogWrite(PWM_pin, 127);  // PWM Correction 0 Hz

      // T = 24.333, CW_FSK
      r0 = 0xAA0AED0;                     // CW_FSK -400
      write2PLL(r0);                      // ADF write
      delay(667);

      // T = 25,  CW_FSK Message
      sendmsg(txstr);

      // T ~ 40,  CW_FSK
      r0 = 0xAA0AED0;                     // CW_FSK -400
      write2PLL(r0);                      // ADF write
      delay(500);
                      
      // T = YY, CW Carrier
      r0 = 0xAA0AEE0;                     // CW 
      write2PLL(r0);                      // ADF Write

         
      do {
        /* If clock is to fast, with Attiny85 with no crystal so never right 
           Needs GPS PPM ( pulse / minute ) to sync TX
           ATTiny Clock error compesation 59500 - 350.

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
      r0 = 0xAA0AED0;                     // CW_FSK -400
      write2PLL(r0);                      // ADF write

      loop_time = millis();

      do {
          /*
         Time out Wait for External 1 PPM Pulse 
         after 30 sec no PPM we switch to CW beacon only
         until PPM returns.
         
         To indicate 1PPM lost we send now CW -400Hz
         after FSK Call transmission
        
        */
           if ( (millis() - loop_time) >= timeout_time ){
            sendmsg(txstr);
            loop_time = millis();
           }
          
        delay(10);      // Reduce loop speed

        } while (digitalRead(GPI_pin) == HIGH );

        cw_only = false;
        
} // Mainloop end
      
