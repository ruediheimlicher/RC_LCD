//
//  Tastenblinky.c
//  Tastenblinky
//
//  Created by Sysadmin on 03.10.07.
//  Copyright Ruedi Heimlicher 2007. All rights reserved.
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
#include "spi_adc.c"
#include "spi_ram.c"
#include "spi_eeprom.c"

uint16_t loopCount0=0;
uint16_t loopCount1=0;
uint16_t loopCount2=0;


#define LOOPDELAY 0

volatile uint16_t RAM_Array[SPI_BUFSIZE];

volatile uint8_t masterstatus = 0;
volatile uint8_t portbhistory = 0xFF;     // default is high because the pull-up

volatile    uint8_t outcounter=0;
volatile   uint8_t testdata =0x00;
volatile   uint8_t testaddress =0x00;
volatile   uint8_t errcount =0x00;
volatile uint8_t ram_indata=0;

volatile uint8_t eeprom_indata=0;
volatile   uint8_t eeprom_testdata =0x00;
volatile   uint8_t eeprom_testaddress =0x00;


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
   PCIFR |= (1<<PCIF0);
   PCICR |= (1<<PCIE0);
	PCMSK0 |= (1<<PCINT7);

   MASTER_DDR &= ~(1 << MASTER_EN_PIN); // Clear the PB7 pin
   // PB7 (PCINT7 pin) are now inputs
   
   MASTER_PORT |= (1 << MASTER_EN_PIN) ; // turn On the Pull-up
   // PB7 are now inputs with pull-up enabled

   MASTER_DDR |= (1 << SUB_BUSY_PIN); // DIS-Pin als Ausgang
   // PB7 (PCINT7 pin) are now inputs
   
   MASTER_PORT |= (1 << SUB_BUSY_PIN) ; // turn On the Pull-up
   // PB7 are now inputs with pull-up enabled



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



void SPI_RAM_init(void) // SS-Pin fuer RAM aktivieren
{
   SPI_RAM_DDR |= (1<<SPI_RAM_CS_PIN); // RAM-CS-PIN Ausgang
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

void spi_start(void) // SPI-Pins aktivieren
{
   //http://www.atmel.com/dyn/resources/prod_documents/doc2467.pdf  page:165
   //Master init
   // Set MOSI and SCK output, all others input
   SPI_DDR &= ~(1<<SPI_MISO_PIN);
   SPI_PORT &= ~(1<<SPI_MISO_PIN); // LO
   
   SPI_DDR |= (1<<SPI_MOSI_PIN);
   
   SPI_DDR |= (1<<SPI_SCK_PIN);
   SPI_PORT &= ~(1<<SPI_SCK_PIN); // LO
   
   SPI_DDR |= (1<<SPI_SS_PIN);
   SPI_PORT |= (1<<SPI_SS_PIN); // HI
   
   // RAM-CS bereit
   SPI_RAM_DDR |= (1<<SPI_RAM_CS_PIN); // RAM-CS-PIN Ausgang
   SPI_RAM_PORT |= (1<<SPI_RAM_CS_PIN);// HI
   
   // EE-CS bereit
   SPI_EE_DDR |= (1<<SPI_EE_CS_PIN); // EE-CS-PIN Ausgang
   SPI_EE_PORT |= (1<<SPI_EE_CS_PIN);// HI
}

void spi_end(void) // SPI-Pins deaktivieren
{
   SPI_DDR &= ~(1<<SPI_MOSI_PIN); // MOSI off
   SPI_DDR &= ~(1<<SPI_SCK_PIN); // SCK off
   SPI_DDR &= ~(1<<SPI_SS_PIN); // SS off
   
   SPI_RAM_DDR &= ~(1<<SPI_RAM_CS_PIN); // RAM-CS-PIN off
   SPI_EE_DDR &= ~(1<<SPI_EE_CS_PIN); // EE-CS-PIN off
   
}


//https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328

ISR (PCINT0_vect)
{
   //MASTER_EN_PORT ^= (1 << SUB_BUSY_PIN) ;
   uint8_t changedbits;
   changedbits = PINB ^ portbhistory;
   portbhistory = PINB;
   
   
   if(changedbits & (1 << MASTER_EN_PIN))
   {
      //MASTER_EN_PORT ^= (1 << SUB_BUSY_PIN) ;
      /* PCINT7 changed Master aendert Enable*/
      
      if ((MASTER_PIN & (1 << MASTER_EN_PIN) )) // EN-Pin Hi, OFF
      {
         if (masterstatus & (1<< MASTER_EN_BIT)) // bit ist gesetzt
         {
            
         }
         else // Bit ist noch nicht gesetzt
         {
            
         }
          
      }
      else
      {
         if (masterstatus & (1<<MASTER_EN_BIT)) // bit ist gesetzt
         {
            //masterstatus |= (1<< MASTER_EN_BIT);
            //MASTER_EN_PORT &= ~(1 << SUB_BUSY_PIN) ;

         }
         else // Bit ist noch nicht gesetzt
         {
            masterstatus |= (1<<MASTER_EN_BIT);
           MASTER_PORT &= ~(1 << SUB_BUSY_PIN) ;
            //MASTER_PORT ^= (1 << SUB_BUSY_PIN) ;

         }
         //MASTER_EN_PORT ^= (1 << SUB_BUSY_PIN) ;

         
         
         //MASTER_EN_PORT |= (1 << SUB_BUSY_PIN) ;
      }
   }
   
    
}


ISR(PCINT1_vect)
{
	if (PINB & _BV(PB7))
   {
      //masterstatus = 1;

   }
}


int main (void)
{
	
	slaveinit();
	
   SPI_PORT_Init(); //Pins fuer SPI aktivieren, incl. SS
   
   SPI_RAM_init(); // SS-Pin fuer RAM aktivieren
   
   SPI_EE_init(); // SS-Pin fuer EE aktivieren
   
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	lcd_puts("Guten Tag\0");
	//_delay_ms(1000);
   
	lcd_cls();
   lcd_gotoxy(0,0);
	lcd_puts("SPI_LCD\0");
   volatile uint8_t outcounter=0;
   volatile uint8_t indata=0;
   sei();
    
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
		if (loopCount0 >=0x4FFF)
		{
         loopCount1++;
         LOOPLED_PORT ^= (1<<LOOPLED_PIN);
         
  			if ((loopCount1 >0xFFFF) )//&& (!(Programmstatus & (1<<MANUELL))))
			{
            
            //LCD_PORT ^= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT C als Ausgang fuer LCD

/*
           
            lcd_gotoxy(0,1);
            lcd_putint(testdata);
 
 */
            loopCount1 =0;
         }
         
			
			loopCount0 =0;
		}
      
      
      
      if (MASTER_PIN & (1 << MASTER_EN_PIN)) // Zeitfenster ist geschlossen, reset Slave
      {
         _delay_us(5);
         masterstatus |= (1<<SUB_END_BIT);
         masterstatus &= ~(1<<SUB_START_BIT);
         MASTER_PORT |= (1 << SUB_BUSY_PIN) ;
         
         if (outcounter%0xFFFF == 0)
         {
            testdata++;
            testaddress--;
            
            
            lcd_gotoxy(0,0);
            //lcd_putint(testdata);
            //lcd_putc('*');
            lcd_putint(ram_indata);
            lcd_putc('+');
            lcd_putint(errcount);
            //lcd_putc('+');
            
         }
         outcounter++;


      }
      else  // Zeitfenster vom master ist offen
      {
           
         if (!(masterstatus & (1<<SUB_START_BIT)))
         {
            _delay_us(1);
            
            MASTER_PORT &= ~(1 << SUB_BUSY_PIN) ; // busy-Meldung an Master
            
            masterstatus |= (1<<SUB_START_BIT);
            masterstatus &= ~(1<<SUB_END_BIT);
            
            // Task des Slave mit RAM und EEPROM erledigen
            
            
            
            // Datapaket des Masters lesen
            spiram_init();
            
            
            // statusregister schreiben
             RAM_CS_LO;
             _delay_us(LOOPDELAY);
             spiram_write_status(0x00);
             _delay_us(LOOPDELAY);
             RAM_CS_HI; // SS HI End
             _delay_us(2);
            
            // testdata in-out
            RAM_CS_LO;
            
            _delay_us(LOOPDELAY);
            //      OSZI_A_LO;
            spiram_wrbyte(testaddress, testdata);
            //     OSZI_A_HI;
            RAM_CS_HI;
            
            // Kontrolle
            _delay_us(2);
            RAM_CS_LO;
            _delay_us(LOOPDELAY);
            //     OSZI_B_LO;
            _delay_us(LOOPDELAY);
            ram_indata = spiram_rdbyte(testaddress);
            _delay_us(LOOPDELAY);
            //     OSZI_B_HI;
            RAM_CS_HI;
            // Fehler zaehlen
            if (!(testdata == ram_indata))
            {
               errcount++;
            }

            
            
            
            MASTER_PORT |= (1 << SUB_BUSY_PIN) ; // end- busy-Meldung an Master
            //masterstatus &= ~(1<<SUB_START_BIT);
         }
         // End Task 
      }
      
      
      
      
 		
	}
	
	
	return 0;
}
