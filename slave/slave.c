#define F_CPU 1200000UL
#define MYADDRESS 0x4

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define SDA PB1
#define SCL PB2

#define ALLOFF   PORTB |= (1 << PB3) | (1 << PB4)
#define GREENON  PORTB &= ~(1 << PB3)
#define REDON    PORTB &= ~(1 << PB4)
#define RELEON   PORTB |= 1 << PB0
#define RELEOFF  PORTB &= ~(1 << PB0)

/**
 * data for EEPROM
 */
volatile uint8_t MyAddr;
volatile uint16_t busByte;
volatile unsigned int position;
volatile unsigned int address;
volatile unsigned int parserState;

uint8_t up(volatile uint8_t *port, uint8_t pin) {
    if (*port & (1 << pin)) {
        return 1;
    }
    return 0;
}

/**
 * externall interrupts when change 1 to 0
 * detect start sequence
 */
ISR(INT0_vect) {

    // and clock is up
    if (up(&PINB, SCL) && parserState == 0) {
        cli();
        while(up(&PINB, SCL));

        parserState = 1;
        position = 0;
        busByte = 0xffff;
    }
}

/**
 * Disable all interrupts and start read the data
 * when parserState is set to 1
 * The method read 16 bit and next
 * set the address and return the data
 */
unsigned int readData(void) {
    cli();

    unsigned int data;

    while(position != 16) {
        // signal from SCL
        if (up(&PINB, SCL)) {

            // read data
            if (!up(&PINB, SDA)) {
                busByte &= ~(1 << position);
            }

            // wait till counter down
            while(up(&PINB, SCL));
            position++;
        }
    }

    address = busByte & 0xff;
    data    = busByte >> 8;
    parserState = 0;
    busByte = 0xffff;

    sei();

    return data;
}

/**
 * Setting output by incomming data
 */
void setSignal(data) {
    switch (data) {
    // red
    case 0x0:
        ALLOFF;
        RELEOFF;
        REDON;
        break;
    // green
    case 0x1:
        ALLOFF;
        GREENON;
        RELEON;
        break;
    // nothing
    case 0xd:
        ALLOFF;
        RELEOFF;
        break;
    }
}

/**
 * Method for store a value into EEPROM memory
 * @uiAddress - address byte
 * @ucData    - data value
 */
void EEPROM_write(unsigned int uiAddress, unsigned char ucData) {
    // Disable all interupts
    cli();

    // Wait for completion of previous write
    while(EECR & (1 << EEPE));

    // Set up address and Data Registers
    EEAR = uiAddress;
    EEDR = ucData;

    // Write logical one to EEMPE
    EECR |= (1 << EEMPE);

    // Start eeprom write by setting EEPE
    EECR |= (1 << EEPE);

    // Enable interrupts
    sei();
}

/**
 * Method for read a value from EEPROM memory
 * @uiAddress - address byte
 * returns data value
 */
unsigned char EEPROM_read(unsigned int uiAddress) {
    // Disable all interupts
    cli();

    // Wait for completion of previous write
    while(EECR & (1 << EEPE));

    // Set up address register
    EEAR = uiAddress;

    // Start eeprom read by writing EERE
    EECR |= (1 << EERE);

    // Enable interrupts
    sei();

    // Return data from Data Register
    return (unsigned char)EEDR;
}

/**
 * Method for set address of the slave
 * @data - address value
 */
void doProgramingMode(unsigned char data) {
    unsigned int i;
    unsigned int addr;

    REDON;
    GREENON;
    _delay_ms(1000);
     ALLOFF;
    _delay_ms(1000);

    for (i = 0; i < data; i++) {
        _delay_ms(250);
        GREENON;
        _delay_ms(250);
        ALLOFF;
    }

    REDON;
    _delay_ms(1000);

    EEPROM_write(0, data);
    addr = EEPROM_read(0);

    if (addr) {
        ALLOFF;
        _delay_ms(1000);
        for (i = 0; i < addr; i++) {
            ALLOFF;
            _delay_ms(250);
            GREENON;
            _delay_ms(250);
        }
        ALLOFF;
        MyAddr = addr;
    }
}

void cpuInit(void) {

    // output
    DDRB |= (1 << PB0) | (1 << PB3) | (1 << PB4);

    // input
    DDRB &= ~(1 << PB2);
    DDRB &= ~(1 << PB1);

    // interrupt when change 1 to 0
    MCUCR |= (1 << ISC01);

    GIMSK |= 0x40;
    SREG |= 0x80;

    ALLOFF;
    REDON;

    address = 0x0;
    parserState = 0;
    busByte = 0xffff;

    MyAddr = EEPROM_read(0);
    if (MyAddr == 0x0) {
        MyAddr = MYADDRESS;
    }
    MyAddr = MYADDRESS;
}

int main() {

    unsigned char data1 = 0x0;
    unsigned char data2 = 0x0;
    unsigned char dataCounter = 0;

    cpuInit();

    for(;;) {
        // first data packet
        if (parserState && dataCounter == 0) {
            data1 = readData();
            dataCounter++;
        }
        // second data packet
        if (parserState && dataCounter == 1) {
            data2 = readData();
            dataCounter = 0;
        }
        // if first and second are the same and address
        // is my address change value
        if (address == MyAddr && data1 == data2) {
            setSignal(data1);
        }
        // kanal 255 is a programmming mode
        if (address == 0xff && data1 == data2) {
            doProgramingMode(data2);
        }
    }
}
