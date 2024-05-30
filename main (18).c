/*
 * Project1DigitalSystems3.c
 *
 * Created: 27/02/2024 
 * Author : Joel Kavanagh (22336168) - Done Individually 
 
 This program satisfies the 'Project 1 Specification' as per the EE4524 Digital Systems 3 Brightspace site.
 There are header comments over each function explaining its purpose and implementation.
 
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>

/* Use a struct to make the association between PORTs and bits connected to the LED array more explicit */
struct LED_BITS
{
	PORT_t *LED_PORT;
	uint8_t bit_mapping;
};

struct LED_BITS LED_Array[10] = {
	{&PORTC, PIN5_bm}, {&PORTC, PIN4_bm}, {&PORTA, PIN0_bm}, {&PORTF, PIN5_bm}, {&PORTC, PIN6_bm}, {&PORTB, PIN2_bm}, {&PORTF, PIN4_bm}, {&PORTA, PIN1_bm}, {&PORTA, PIN2_bm}, {&PORTA, PIN3_bm}
};


volatile uint16_t global_adc_value = 0; // Global variable to store ADC result
volatile uint8_t new_adc_result_available = 0; // Flag to indicate a new ADC result is available
volatile uint8_t inverted_display_mode = 0; // 0 for normal Cylon, 1 for inverted Cylon

#define NUM_LED_BITS 10 // The number of LEDs in the LED shield
#define THREE_0V 614    // ADC reading value that corresponds to 3V

// Function Prototypes
void move_leds(void);
void display_thermometer_format(void);
void ClearPorts();
void SetPorts();


/**
 * Function Name: InitialiseLED_PORT_bits
 * Description: Initializes the LED port bits as per the Project 1 Specification Appendix. Sets PORTE bit 1 as the input and enables its pull-up resistor.
 * @param none
 * @return None
 */
void InitialiseLED_PORT_bits()
{
	PORTC.DIR = PIN6_bm | PIN5_bm | PIN4_bm;  /*(1<<6) | (1<<5) | (1<<4); 0x70;*/		/* PC4-UNO D1 (TXD1), PC5-UNO D0 (RXD1), PC6 - UNO D4  */
	PORTA.DIR = PIN3_bm | PIN2_bm | PIN1_bm | PIN0_bm; /*(1<<1) | (1<<0);   0x0f; */      /* PA1-UNO D7, PA0 - UNO D2, PA2- LED8, PA3 - LED9  */
	PORTB.DIR = PIN2_bm; /*0x04;*/		/* PB2 - UNO D5 */
	PORTF.DIR = PIN5_bm | PIN4_bm; /*(1<<5) | (1<<4);   0x30; */		/* PF5 - UNO D3, PF4 UNO D6 */
	
	/* Set PORTE bit 1 as input */
	PORTE.DIRCLR = 0b00000010;

	/* Enable pull-up resistor on PORTE bit 1 */
	PORTE.PIN1CTRL |= PORT_PULLUPEN_bm; 
}

/**
 * Function Name: CLOCK_init
 * Description: Initializes the clock configuration by disabling the CLK_PER Prescaler.
 * @param none
 * @return None
 */
void CLOCK_init (void)
{
	/* Disable CLK_PER Prescaler */
	ccp_write_io( (void *) &CLKCTRL.MCLKCTRLB , (0 << CLKCTRL_PEN_bp));
	/* If set from the fuses during device programming, the CPU will now run at 20MHz (default is /6) */
}

/**
 * Function Name: TCA0_init_bits
 * Description: Initializes timer TCA0 as per the Project 1 Specification. Selects 2441 as top value (PER) to give the closest possible approximation of 0.125s
 * As the prescale value was required to be /1024. This will then be used in the TCA0 ISR to display the cylon pattern at this speed, and changed using a counter 
 * overflow for the 0.5s speed when the ADC result is greater than 3V (approximated to 0.4999).
 * @param none
 * @return None
 */
void TCA0_init_bits(void)
{

	TCA0.SINGLE.INTCTRL = 0b00000001;		/* Counter overflow interrupt option */
	TCA0.SINGLE.CTRLB = 0b00000000;			/* Normal Mode selected - TOP value in PER register */
	TCA0.SINGLE.EVCTRL = 0b00000000;		/* TCA0 can count events from the EVENT module - disable this option */
	
	/* Now set the PER register to 2411, our top value */
	/* This is because 20MHz/1024  gives TCA0clk period = 51.2us and 51.2us*2441 = 0.12498 - approximation of 0.125s (not visible to the eye) */
	// Approximation made as /1024 required in Project 1 Spec
	
	TCA0.SINGLE.PER = 2441;					/* The TCA0 counter will overflow when it reaches the PER value */
	TCA0.SINGLE.CTRLA = 0b00001111;			/* Prescale set to /1024 and enable TCA0 (start count) */

}

/**
 * Function Name: ADC_Init
 * Description: Initializes Analog to Digital Converter as per the Project 1 Specification. 
 * @param none
 * @return None
 */
void ADC_Init(void)
{
	/* ADC0 register bits are set/cleared by writing binary patterns to the registers  */
	
	ADC0.CTRLA = 0b00000010;		/* 10-bit resolution selected, Free Running Mode selected, ADC0 not enabled yet */
	ADC0.CTRLB = 0b00000000;		/* Simple No Accumulation operation selected, this line could be omitted */
	ADC0.CTRLC = 0b01010110;		/* SAMPCAP=1; REFSEL: VDD; PRESC set to DIV128 */
	ADC0.CTRLD = 0b00100000;		/* INITDLY set to 16 CLK_ADC cycles, default 0 would be OK so this line could be omitted */
	ADC0.MUXPOS = 0b00000011;		/* Select AIN3 (shared with PORTD3), decision based on the Shield and adapters we use */
	ADC0.INTCTRL = 0b00000001;		/* Enable an interrupt when conversion complete (RESRDY) */
	
	ADC0.CTRLA |= 0b00000001;		/* Enable ADC0 and leave the other CTRLA bits unchanged, note |= */
	ADC0.COMMAND = 0b00000001;		/* Start the first Conversion, Free Running means the later ones start automatically */

}


/**
 * Function Name: RTC_PIT_init
 * Description: Initializes Real time counter as per the Project 1 Specification. Selects 1.024 kHz internal oscillator  and sets the RTC clock cycles for a clock period of 1024 = 1 second.
 * This will later combine with a software counter in the RTC ISR to give the desired 16 second time period.
 * Also enables periodic interrupts.
 * @param none
 * @return None
 */
void RTC_PIT_init()
{
    /* Initialize RTC: */
	while (RTC.STATUS > 0)
	{ /* Wait for all registers to be synchronized */ }
	/* 1.024kHz Internal Oscillator  */
	RTC.CLKSEL = 0b00000001;

	while (RTC.PITSTATUS > 0)
	{ /* Wait for all registers to be synchronized */ }
	RTC.PITCTRLA = 0b01001001;/* RTC Clock Cycles 1024, PIT enabled */
	/* With 1024 kHz, a clock period of 1024 means 1 second */
	RTC.PITINTCTRL = 0b00000001; /* Periodic Interrupt: enabled */
}

/**
 * Function Name: main
 * Description: Calls relevant functions, enables global interrupts.
 * Tests PORTE bit 1 (push button 0) - if pressed, checks for new ADC result - if found, displays thermometer format and clears new ADC result flag
 * @param none
 * @return type int
 */
int main(void)
{
	 // Call relevant functions
    	CLOCK_init();
    	InitialiseLED_PORT_bits();
		ADC_Init();
		TCA0_init_bits();
		RTC_PIT_init();
		
		sei(); //enable global interrupts

    while (1) 
    {
				// Test PORTE bit 1
				if (!(PORTE.IN & (1 << 1))) { // if button is pressed (logic 0)
				
					if (new_adc_result_available == 1) { // Check for new ADC result available
						
						display_thermometer_format(); // Display ADC result in thermometer format
						
						new_adc_result_available = 0; // Clear new ADC result available flag
					}
				}

    }
}

/**
 * Function Name: ISR(TCA0_OVF_vect)
 * Description: Interrupt Service Routine for the Timer (TCA0).
 * Uses Local variable overflow_counter to implement a software counter giving the desired cylon speed (using the one PER top value) by comparing the defined THREE_0V
 * with the global_adc_value when the push button is not pressed.
 * For 0V <= ADC input 0 voltage <= 3V, the on/off time is 0.125 second
 * For 3V < ADC input 0 voltage <= 5V, the on/off time is 0.5 seconds
 * A software overflow works here as 0.125s*4 = 0.5s
 * Also clears the interrupt flag (2 possible implementations shown)
 * @param none
 * @return None
 */
ISR(TCA0_OVF_vect)
{
	static uint8_t overflow_counter = 0;
	overflow_counter ++;
	
	
			if ((PORTE.IN & (1 << 1))) { // if push button is not pressed
				if(global_adc_value <= THREE_0V) { // 0.125s
					move_leds();
					overflow_counter = 0;
				}
				
			    else if (global_adc_value > THREE_0V) {
				if(overflow_counter >= 4){ // software counter overflow to allow 0.5 seconds (as 0.125*4 = 0.5)
					move_leds();
					overflow_counter = 0;
				}
				}
			}
	
	
	#ifdef BINARY_SETTINGS
	TCA0.SINGLE.INTFLAGS = 0b00000001;   /* Writing 1 to the flag bit clears it */
	#else
	/* Same implementation using explicit bit naming: */
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
	#endif
	
}

/**
 * Function Name: ISR(ADC0_RESRDY_vect)
 * Description: Interrupt Service Routine for ADC. Reads the ADC result into a global variable global_adc_value. Clears the RESRDY flag. Sets flag to indicate new ADC result available 
 * (to be checked in main). 
 * @param none
 * @return None
 */
ISR(ADC0_RESRDY_vect) {
	
	global_adc_value = ADC0.RES; // Read the ADC result into the global variable
	
	ADC0.INTFLAGS = 0b00000001; // clear RESRDY flag by writing 1 to it
	

	new_adc_result_available = 1; // Set flag to indicate new ADC result is available
	
}

/**
 * Function Name: ISR(RTC_PIT_vect)
 * Description: Interrupt Service Routine for the Real-Time Counter.
 * Uses Local variable PITCount to implement a software counter giving the desired 16 second time period between cylon and inverted cylon 
 * display modes. 
 * @param none
 * @return None
 */
ISR(RTC_PIT_vect)
{
	static uint8_t PITCount;
	PITCount += 1;/* No of times ISR occurred since PINCount set
	to 0 */
	if (PITCount >= 16) { //  wait 16 seconds 
		inverted_display_mode = !inverted_display_mode; //toggle between cylon and inverted cylon
		PITCount = 0;/* Restart counting */
	}
	/* Clear flag by writing '1': */
	RTC.PITINTFLAGS = 0b00000001;  /* Clear the Interrupt Request Flag */

}

/**
 * Function Name: move_leds
 * Description: uses local variables current_led and direction to either display the normal cylon pattern or the inverted cylon based on the display mode
 * @param none
 * @return None
 */
void move_leds(void)
{
	
	 static uint8_t current_led = 0; // Keep track of the current LED
	 static int8_t direction = 1; // Direction of movement, 1 for forward, -1 for backward

	 if (inverted_display_mode == 0) {
		 // Normal Cylon Display
		 ClearPorts(); // Ensure all LEDs are off initially
		 LED_Array[current_led].LED_PORT->OUTSET = LED_Array[current_led].bit_mapping; // Turn on the current LED
		 } else {
		 // Inverted Cylon Display
		 SetPorts(); // Use the SetPorts function to turn on all LEDs
		 LED_Array[current_led].LED_PORT->OUTCLR = LED_Array[current_led].bit_mapping; // Then turn off the current LED for the 'walking zero' effect
	 }

	 // Calculate the next LED position
	 current_led += direction;

	 // Change direction if we hit the end
	 if (current_led >= NUM_LED_BITS - 1 || current_led <= 0) {
		 direction = -direction;
	 }
	
}

/**
 * Function Name: display_thermometer_format
 * Description: Displays 'thermometer' format based on the voltage applied to AIN3 (Potentiometer) when push button 0 (PORTE bit 1) is engaged
 * @param void
 * @return void
 */
void display_thermometer_format(void) {
	
	    ClearPorts(); // Clear all LEDs before setting new pattern
		
		 const uint16_t full_scale = 1023; // Assuming a 10-bit ADC
		 uint8_t leds_to_light = 0;

		 // Determine the number of LEDs to light based on the ADC value range
		 if (global_adc_value < (full_scale * 1 / 10)) leds_to_light = 1;
		 else if (global_adc_value < (full_scale * 2 / 10)) leds_to_light = 2;
		 else if (global_adc_value < (full_scale * 3 / 10)) leds_to_light = 3;
		 else if (global_adc_value < (full_scale * 4 / 10)) leds_to_light = 4;
		 else if (global_adc_value < (full_scale * 5 / 10)) leds_to_light = 5;
		 else if (global_adc_value < (full_scale * 6 / 10)) leds_to_light = 6;
		 else if (global_adc_value < (full_scale * 7 / 10)) leds_to_light = 7;
		 else if (global_adc_value < (full_scale * 8 / 10)) leds_to_light = 8;
		 else if (global_adc_value < (full_scale * 9 / 10)) leds_to_light = 9;
		 else leds_to_light = 10; // ADC reading is in the highest range

		 // Light up LEDs from 0 up to the determined number
		 for (uint8_t i = 0; i < leds_to_light; i++) {
			 LED_Array[i].LED_PORT->OUTSET = LED_Array[i].bit_mapping;
		 }

	}

/**
 * Function Name: ClearPorts
 * Description: Turns off the LED array - to be used in the cylon display, useful for switching in between cylon and thermometer modes
 * as it clears the thermometer output
 * @param none
 * @return void
 */
void ClearPorts() {
	for (uint8_t i = 0; i < NUM_LED_BITS; i++) {
		LED_Array[i].LED_PORT->OUTCLR = LED_Array[i].bit_mapping;
	}
}

/**
 * Function Name: SetPorts
 * Description: Lights up the LED array - to be used in the inverted cylon display
 * @param none
 * @return void
 */
void SetPorts() {
	
	uint8_t i;
	
	for (i = 0; i <= (NUM_LED_BITS - 1); i += 1) {
		LED_Array[i].LED_PORT->OUTSET = LED_Array[i].bit_mapping;
	}
}