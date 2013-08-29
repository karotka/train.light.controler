#define F_CPU 8000000UL
#define I2CSPEED 1000
#define LENGHT(_array) (sizeof(_array) / sizeof(_array[0]))

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define SDA0 PORTC &= ~(1 << PC4) // 0
#define SDA1 PORTC |= 1 << PC4    // 1

#define SCL0 PORTC &= ~(1 << PC5)
#define SCL1 PORTC |= 1 << PC5

volatile int kanal = 0x01;
volatile int bValue;
volatile int cValue;
volatile unsigned char dataset[10] = {0x0};

inline unsigned char bitIsSet(byte, bit)  {
    return byte & (1 << bit);
}

void I2C_delay() {
    _delay_us(I2CSPEED);
}

void I2C_delay2() {
    _delay_us(I2CSPEED/2);
}

void delay_ms(int d){
    unsigned int i;
    for (i = 0 ; i < d; ++i){
        _delay_ms(1);
    }
}

// i2c start bit sequence
void I2C_start(void) {
    SDA1;
    I2C_delay();
    SCL1;
    I2C_delay();
    SDA0;
    I2C_delay();
    SCL0;
    I2C_delay();
}

// i2c stop bit sequence
void I2C_stop(void) {
    SDA0;
    SCL1;
    I2C_delay();
    SDA1;
    I2C_delay();
    SCL0;
    I2C_delay();
    SDA0;
}

void I2C_tx(unsigned char data) {
    unsigned int i;
    for (i = 0; i < 8; i++) {
        if (bitIsSet(data, i)) {
            SDA1;
        } else {
            SDA0;
        }

        I2C_delay2();
        SCL1;
        I2C_delay2();
        SCL0;
        I2C_delay2();
        SDA0;
        I2C_delay();
    }
}

void I2C_send(unsigned int kanal, unsigned char data) {
    I2C_start();
    I2C_tx(kanal);
    I2C_tx(data);
    I2C_stop();
}

int debounce(volatile uint8_t *port, uint8_t pin1, uint8_t pin2) {
    if (!(*port & (1 << pin1)) && !(*port & (1 << pin2))) {
        _delay_ms(50);
        if ((*port & (1 << pin1)) && (*port & (1 << pin2))) {
            _delay_ms(50);
            return 0;
        }
    } else if (!(*port & (1 << pin2))) {
        _delay_ms(50);
        if (*port & (1 << pin2)) {
            _delay_ms(50);
            return -1;
        }
    } else if (!(*port & (1 << pin1))) {
        _delay_ms(50);
        if (*port & (1 << pin1)) {
            _delay_ms(50);
            return 1;
        }
    }
    return 2;
}

int debounceColor(volatile uint8_t *port,
                  uint8_t pin1, uint8_t pin2,
                  uint8_t pin3, uint8_t pin4) {

    // Red and blue together
    if (!(*port & (1 << pin1)) && !(*port & (1 << pin4))) {
        _delay_ms(200);
        return 0xff;
    } else

    // Red
    if (!(*port & (1 << pin1))) {
        _delay_ms(200);
        return 0x0;
    } else

    // green
    if (!(*port & (1 << pin2))) {
        _delay_ms(200);
        return 0x1;
    } else

    if (!(*port & (1 << pin3))) {
        _delay_ms(200);
        return 0x3;
    } else

    if (!(*port & (1 << pin4))) {
        _delay_ms(200);
        return 0xd;
    }

    return -1;
}

void LCD_init(void) {
    PORTB &= ~((1 << PB0) | (1 << PB1));
    PORTD &= ~((1 << PD5) | (1 << PD6) | (1 << PD7) | (1 << PD4) | (1 << PD3));
}

void LCD_set(unsigned char character) {
    LCD_init();
    switch (character) {
    case 0:
        PORTB |= (1 << PB0) | (1 << PB1);
        PORTD |= (1 << PD6) | (1 << PD7) | (1 << PD4) | (1 << PD3);
        break;
    case 1:
        PORTB |= (1 << PB0) | (1 << PB1);
        break;
    case 2:
        PORTB |= (1 << PB0);
        PORTD |= (1 << PD7) | (1 << PD5) | (1 << PD4) | (1 << PD3);
        break;
    case 3:
        PORTB |= (1 << PB0) | (1 << PB1) | (1 << PB4);
        PORTD |= (1 << PD3) | (1 << PD5) | (1 << PD7);
        break;
    case 4:
        PORTB |= (1 << PB1) | (1 << PB0);
        PORTD |= (1 << PD5) | (1 << PD6);
        break;
    case 5:
        PORTB |= (1 << PB1);
        PORTD |= (1 << PD3) | (1 << PD5) | (1 << PD6) | (1 << PD7);
        break;
    case 6:
        PORTB |= (1 << PB1);
        PORTD |= (1 << PD3) | (1 << PD5) | (1 << PD6) | (1 << PD7) | (1 << PD4);
        break;
    case 7:
        PORTB |= (1 << PB0) | (1 << PB1);
        PORTD |= 1 << PD7;
        break;
    case 8:
        PORTB |= (1 << PB0) | (1 << PB1);
        PORTD |= (1 << PD5) | (1 << PD6) | (1 << PD7) | (1 << PD4) | (1 << PD3);
        break;
    case 9:
        PORTB |= (1 << PB0) | (1 << PB1);
        PORTD |= (1 << PD5) | (1 << PD6) | (1 << PD7) | (1 << PD3);
        break;
    }
}

ISR(TIMER0_OVF_vect) {

    bValue = debounce(&PIND, PD1, PD0);
    if (bValue != 2) {
        kanal += bValue;
    }

    if (kanal > 0xa) {
        kanal = 0x1;
    } else if (kanal < 0x1) {
        kanal = 0xa;
    }

    LCD_set(kanal % 10);
    cValue = debounceColor(&PINC, PC3, PC2, PC1, PC0);

    if (cValue != -1) {
        if (cValue == 0xff) {
            I2C_send(0xff, kanal);
            I2C_send(0xff, kanal);
            I2C_send(0xff, kanal);
            I2C_send(0xff, kanal);
        } else {
            dataset[kanal] = cValue;
        }

        //I2C_send(kanal, dataset[kanal]);
        //I2C_send(kanal, dataset[kanal]);

        // toto: predelat
        if (kanal == 1 && cValue == 0x1) {
            dataset[2] = 0x0;
            dataset[3] = 0x0;
        }
        if (kanal == 2 && cValue == 0x1) {
            dataset[1] = 0x0;
            dataset[3] = 0x0;
        }
        if (kanal == 3 && cValue == 0x1) {
            dataset[1] = 0x0;
            dataset[2] = 0x0;
        }
        /*
        if (kanal == 3 && cValue == 0x1) {
            dataset[4] = 0x0;
        }
        if (kanal == 4 && cValue == 0x1) {
            dataset[3] = 0x0;
        }
        */
    }
}

void cpuInit(void) {
    // Set up ports
    DDRD = 0xfc;
    DDRB = 0xff;
    DDRC = 0x30;

    PORTD = 0x0;
    PORTB = 0x0;
    PORTC = 0x0;

    // Initialize 8b Timer1
    TCCR0A = 0x0;             // normal timer operation, no pwm
    TCCR0B = 0x1;             // no prescaling
    TCNT0  = 0;
    TIMSK0 = 0x1;             // enable timer/counter1 overflow interrupt

    sei();
}

int main(void) {

    cpuInit();

    unsigned int i, j;
    unsigned int items = LENGHT(dataset);

    for (;;) {
        for (i = 0; i < 3; i++) {
            delay_ms(1);
            for (j = 1; j < items; j++) {
                cli();
                I2C_send(j, dataset[j]);
                I2C_send(j, dataset[j]);
                sei();
            }
        }
    }

    return 1;
}
