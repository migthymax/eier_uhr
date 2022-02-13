// PIN DEFINITIONS
//
// Uno R3
//
// PB1 <- switch in
//
// PB4 -> led on
// PB3 -> audio on
//
//
////////////////////////////////////////////////////////////////////

// We are sligthy slower than advertised
#define F_CPU 1044000UL

#include <stdio.h>

#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>


#define PORT_SFX              (1 << DDB3)
#define PIN_SFX               (1 << PB3)

#define PORT_LED              (1 << DDB4)
#define PIN_LED               (1 << PB4)


#define PORT_TST              (1 << PCINT1)
#define PIN_TST               ( PINB & PORT_TST )

#define TIMER_EPOCH_MIN       1
#define PRESS_INTERVAL_SEC    5
#undef  ENABLE_EPOCH_EXT

#define adc_disable()         (ADCSRA &= ~(1<<ADEN))
#define ac_disable()          (ACSR = (1<<ACD))

#define soundOn( void )       PORTB |= PIN_SFX;
#define soundOff( void )      PORTB &= ~PIN_SFX;
#define soundToggle( void )   PORTB ^= PIN_SFX;

#define ledOn( void )         PORTB |= PIN_LED;
#define ledOff( void )        PORTB &= ~PIN_LED;
#define ledToggle( void )     PORTB ^= PIN_LED;

void ledFlash( void ) {
  ledToggle();  
  _delay_ms( 50 );
  ledToggle();  
}

void soundFlash( void ) {
  ledToggle();  
  _delay_ms( 50 );
  ledToggle();  
}

volatile unsigned int milliseconds;
volatile unsigned char seconds;
volatile unsigned char absSeconds;
volatile unsigned char minutes;

volatile unsigned char buttonPush;

signed char timerMinutes = -1;

// Gets called whenever the someone pressed the button
ISR( PCINT0_vect ) {
  if( PIN_TST ) {
    buttonPush = 1;
  }
}

ISR( TIMER0_COMPA_vect ) {
  // Count time
  if( ++milliseconds == 1000 )  {
    ++absSeconds;
    milliseconds = 0;
    if( ++seconds == 60 ) {
      seconds = 0;
      ++minutes;
    }
  }
}

void setupTimer0( void ) {
  // Configure Timer statics CTC with compare match
  TCCR0A = (1<<WGM01);
  // Set prescalar to 1/8,
  TCCR0B = (1<<CS01);
  // ( F_CPU/8 Prescalar/1000Hz = 132 cycles/ms where 1000Hz equals 1 ms
  OCR0A = 132 -1;
}

void startTimer0( void ) {
  cli(); 
  { // disabled interrupts
    // Init timer start values
    milliseconds  = 0;
    seconds       = 0;
    minutes       = 0;

    // Start Timer for each ~ms
    TIMSK0 |= (1<<OCIE0A);
  } // enable interrupts
  sei();
}

void stopTimer0( void ) {
  cli();
  { // disabled interrupts
    TIMSK0 |= ~(1<<OCIE0A);
  } // enable interrupts
  sei();
}

void setupPins( void ) {    
  // Configure Output&Input Ports 
  DDRB = ( PORT_LED | PORT_SFX ) & ~( PORT_TST );
  // Clear Output Ports 
  PORTB &= ~( PIN_LED | PIN_SFX );
}

void setupButton( void ) {    
  // Configure Pin change interupt on PORT_TST is PCINT1
  PCICR  |= (1 << PCIE0);       // set PCIE0 to enable PCMSK0 scan
  PCMSK0  |= PORT_TST;          // set PCINT1 to trigger an interrupt on state change 
}

void setupSleep( void ) {
  // disable unneeded stuff
  adc_disable();      // ADC Converter
  ac_disable();       // Analog comparator is default
  // Brown out detection is default
  // Internal Voltage Reference is default
  // Watch dog is default
}

void enterSleep( void ) {
  sleep_enable();
  sleep_cpu();
}

////////////////////////////////////////////////////////////////////
int main( void ) {
  cli(); 
  { // disable interrupts
    setupPins();
    setupButton();
    setupTimer0();
    setupSleep();
  } // enable interrupts
  sei();

  // the loop function runs over and over again forever
  while ( 1 ) {
    set_sleep_mode( SLEEP_MODE_PWR_DOWN );
    enterSleep();

    unsigned int localMintutes = 0;
    
    cli();
    { // disable interrupts
      ledOn();

      absSeconds    = 0;
      timerMinutes  = TIMER_EPOCH_MIN;
      buttonPush    = 0;
      
      startTimer0();
    } // enable interrupts
    sei();

    do {
      set_sleep_mode( SLEEP_MODE_IDLE );
      enterSleep();

      cli();
      { // disable interrupts
        if( buttonPush && ( absSeconds <= PRESS_INTERVAL_SEC ) ) {
          ledFlash();

#ifdef ENABLE_EPOCH_EXT
          timerMinutes += TIMER_EPOCH_MIN;
#endif

          absSeconds = 0;
        }

        buttonPush = 0;
        localMintutes = minutes;
      } // enable interrupts
      sei();
    } while( timerMinutes > localMintutes );

    cli();
    { // disable interrupts
      soundOn();
      _delay_ms( 350 );
      soundOff();
      ledOff();

      timerMinutes  = 0;
      buttonPush    = 0;

    } // enable interrupts
    sei();
  }

  return 0;
}