//
//  Tastenblinky.c
//  Tastenblinky
//
//  Created by Sysadmin on 03.10.07.
//  Copyright Ruedi Heimlihcer 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <inttypes.h>
#include <avr/delay.h>
#include "lcd.c"
#include <avr/interrupt.h>

//#include "spi.c"

#include "def.h"

#include "spi_ram.c"
#include "spi_eeprom.c"

uint16_t loopCount0=0;
uint16_t loopCount1=0;
uint16_t loopCount2=0;


#define LOOPDELAY 0

volatile uint16_t RAM_Array[SPI_BUFSIZE];

volatile uint8_t mastersignal = 0;
volatile uint8_t portbhistory = 0xFF;     // default is high because the pull-up

void slaveinit(void)
{
	LOOPLED_DDR |= (1<<LOOPLED_PIN);
   LOOPLED_PORT |= (1<<LOOPLED_PIN);

	DDRB &= ~(1<<PORTB0);	//Bit 0 von PORT B als Eingang fuer Taster
	PORTB |= (1<<PORTB0);	//HI

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT C als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT C als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT C als Ausgang fuer LCD

   LCD_PORT |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT C als Ausgang fuer LCD
 	LCD_PORT |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT C als Ausgang fuer LCD
	LCD_PORT |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT C als Ausgang fuer LCD
   
   /**
	 * Pin Change Interrupt enable on PCINT1 (PB7)
	 */
   
   DDRB &= ~(1 << DDB7); // Clear the PB0, PB1, PB2 pin
   // PBt (PCINT0, PCINT1, PCINT2 pin) are now inputs
   
   PORTB |= (1 << PORTB7) ; // turn On the Pull-up
   // PBz are now inputs with pull-up enabled

	PCICR |= (1<<PCIE1);
	PCMSK0 |= (1<<PCINT7);


   //Pin 0 von   als Ausgang fuer OSZI
	OSZIPORTDDR |= (1<<OSZI_PULS_A);	//Pin 0 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_A);		// HI
	
   OSZIPORTDDR |= (1<<OSZI_PULS_B);		//Pin 1 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_B);		//Pin   von   als Ausgang fuer OSZI

	
}

char SPI_get_put_char(int cData)
{
   
   //Putchar -- Master
   /* Start transmission */
   SPDR = cData;
   /* Wait for transmission complete */
   while(!(SPSR & (1<<SPIF)))
      ;
   /* Return data register */
   return SPDR;
}



void SPI_RAM_init(void) // SS-Pin fuer EE aktivieren
{
   SPI_RAM_DDR |= (1<<SPI_RAM_CS_PIN); // EE-CS-PIN Ausgang
   SPI_RAM_PORT |= (1<<SPI_RAM_CS_PIN);// HI
}

void SPI_EE_init(void) // SS-Pin fuer EE aktivieren
{
   SPI_EE_DDR |= (1<<SPI_EE_CS_PIN); // EE-CS-PIN Ausgang
   SPI_EE_PORT |= (1<<SPI_EE_CS_PIN);// HI
}




void SPI_PORT_Init(void)
{
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO);
   SPI_PORT &= ~(1<<SPI_MISO); // HI
   
   SPI_DDR |= (1<<SPI_MOSI);

   
   SPI_DDR |= (1<<SPI_SCK);
   SPI_PORT &= ~(1<<SPI_SCK); // LO

   SPI_DDR |= (1<<SPI_SS);
   SPI_PORT |= (1<<SPI_SS); // HI
   
 }

//https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328

ISR (PCINT0_vect)
{
   uint8_t changedbits;
   
   
   changedbits = PINB ^ portbhistory;
   portbhistory = PINB;
   
   
   if(changedbits & (1 << PB0))
   {
      /* PCINT0 changed */
   }
   
   if(changedbits & (1 << PB1))
   {
      /* PCINT1 changed */
   }
   
   if(changedbits & (1 << PB2))
   {
      /* PCINT2 changed */
   }
   
}


ISR(PCINT1_vect)
{
	if (PINB & _BV(PB7))
   {
      mastersignal = 1;

   }
}


int main (void)
{
	
	slaveinit();
	
   SPI_PORT_Init();
   
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	lcd_puts("Guten Tag\0");
	_delay_ms(1000);
   
	lcd_cls();
   lcd_gotoxy(0,0);
	lcd_puts("MASTER 168\0");
   volatile uint8_t outcounter=0;
   volatile uint8_t indata=0;
   //sei();
   SPI_PORT &= ~(1<<SPI_SS); // SS LO, Start
   
   //spiram_init(0x00); // blockiert wenn kein SPI-Device
   
   SPI_PORT |= (1<<SPI_SS); // SS HI End
   
   //
#pragma mark while
   volatile   uint8_t testdata =0x00;
   volatile   uint8_t testaddress =0x00;
   volatile   uint8_t errcount =0x00;
	while (1) 
	{
		loopCount0 ++;
      uint8_t ramposition = outcounter / 0x08;
      //ramposition = 0xA0;
		if (loopCount0 >=0x0FFF)
		{
         loopCount1++;
         LOOPLED_PORT ^= (1<<LOOPLED_PIN);
         
  			if ((loopCount1 >0xFFFF) )//&& (!(Programmstatus & (1<<MANUELL))))
			{
            
            //LCD_PORT ^= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT C als Ausgang fuer LCD

/*
           
            lcd_gotoxy(0,1);
            lcd_putint(testdata);
            loopCount1 =0;
 */
         }
         {
            
            //spi_init();
            spiram_init();
            spieeprom_init();
            
            RAM_CS_LO;
            _delay_us(LOOPDELAY);
            spiram_write_status(0x00);
            _delay_us(LOOPDELAY);
            RAM_CS_HI; // SS HI End
            _delay_us(400);
            
            
            //if (outcounter%2 == 0)
            {
               //  spiram_wrbyte(0x12, 0xAA);
               // _delay_us(40);
               //indata = spiram_rdbyte(ramposition);
            }
            
            //else
            
            RAM_CS_LO;
            
            _delay_us(LOOPDELAY);
            OSZI_A_LO;
            spiram_wrbyte(testaddress, testdata);
            OSZI_A_HI;
            RAM_CS_HI;
            _delay_us(400);
            RAM_CS_LO;
            _delay_us(LOOPDELAY);
            OSZI_B_LO;
            _delay_us(LOOPDELAY);
            indata = spiram_rdbyte(testaddress);
            _delay_us(LOOPDELAY);
            OSZI_B_HI;
            
            RAM_CS_HI;
            
            if (!(testdata == indata))
            {
               errcount++;
            }
            
            if (outcounter%16 == 0)
            {
               testdata++;
               testaddress--;
            }
            
            _delay_us(LOOPDELAY);
            lcd_gotoxy(0,1);
            lcd_putint(testdata);
            lcd_putc(' ');
            lcd_putint(indata);
            lcd_putc(' ');
            lcd_putint(errcount);
            loopCount1 =0;
            //
            
            outcounter++;
            loopCount2++;
            loopCount1=0;
			}
			
			loopCount0 =0;
		}
 		
	}
	
	
	return 0;
}
