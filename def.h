//Oszi
#define OSZIPORT           PORTD
#define OSZIPORTDDR        DDRD
#define OSZIPORTPIN        PIND
#define OSZI_PULS_A        0
#define OSZI_PULS_B        1


#define OSZI_A_LO OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZI_A_HI OSZIPORT |= (1<<OSZI_PULS_A)
#define OSZI_A_TOGG OSZIPORT ^= (1<<OSZI_PULS_A)


#define OSZI_B_LO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZI_B_HI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZI_B_TOGG OSZIPORT ^= (1<<OSZI_PULS_B)

#define SPI_PORT     PORTB
#define SPI_PIN      PINB
#define SPI_DDR      DDRB

#define SPI_SS       2

#define SPI_MOSI     3
#define SPI_MISO     4
#define SPI_SCK      5

#define LOOPLED_PORT	PORTD
#define LOOPLED_DDR	DDRD
#define LOOPLED_PIN	4



#define TASTE0				   2
#define TASTE1             3

// SPI

#define SPI_BUFSIZE 8

#define STARTDELAYBIT       0
#define HICOUNTBIT          1

#define WDTBIT              7

// CNC12
#define CMD_PORT            PORTD   //    PORTB
#define CMD_DDR             DDRD    //    DDRB
#define CMD_PIN             PIND    //    PINB


#define KANAL_PORT            PORTB   //    PORTB
#define KANAL_DDR             DDRB    //    DDRB


#define KANAL_PIN          4
#define KANAL_LO           KANAL_PORT &= ~(1<<KANAL_PIN)
#define KANAL_HI           KANAL_PORT |= (1<<KANAL_PIN)


// ADC
// CNC12
#define ADC_PORT            PORTF   //    PORTF
#define ADC_DDR             DDRF    //    DDRF
#define ADC_PIN             PINF    //    PINF


// Bit

#define ADC_START 0  // Start Messung Batteriespannung mit internem ADC

#define POT_START 0  //    Start Messung Potentiometer
#define POT_MITTE 7  //    Mittelwerte der Potentiometer speichern

#define ANZ_POT   6

#define POT_FAKTOR 1.20


#define EE_WREN   0
#define EE_WRITE  1
#define EE_READ   2
