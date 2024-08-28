/*
 * onewire.c
 *
 *  Created on: 03-Jul-2024
 *      Author: shoaib
 */

#include <stdio.h>
#include <stdint.h>
#include <msp430.h>
#include <onewire.h>


uint8_t onewire_reset(void)
{
    uint8_t retries = 125;
    uint8_t r;

    DS18B20_DIR &= ~DS18B20_PIN; // Set as input

    while (--retries && !(DS18B20_IN & DS18B20_PIN)) {__delay_cycles(2);}
    if (retries == 0) {return 0;} // Timeout, no device found

    DS18B20_DIR |= DS18B20_PIN; // Set as output
    DS18B20_OUT &= ~DS18B20_PIN; // Pull low
    __delay_cycles(500); // Hold low for 500us

    DS18B20_DIR &= ~DS18B20_PIN; // Set as input
    __delay_cycles(80); // Wait for 80us

    r = !(DS18B20_IN & DS18B20_PIN);
    __delay_cycles(420); // Wait for remainder of time slot

    return r;
}

void onewire_write_bit(uint8_t v)
{
    if (v & 1) {
        DS18B20_DIR |= DS18B20_PIN; // Set as output
        DS18B20_OUT &= ~DS18B20_PIN; // Pull low
        __delay_cycles(10);
        DS18B20_OUT |= DS18B20_PIN; // Release
        __delay_cycles(55);
    } else {
        DS18B20_DIR |= DS18B20_PIN; // Set as output
        DS18B20_OUT &= ~DS18B20_PIN; // Pull low
        __delay_cycles(65);
        DS18B20_OUT |= DS18B20_PIN; // Release
        __delay_cycles(5);
    }
}

uint8_t onewire_read_bit(void)
{
    uint8_t r;

    DS18B20_DIR |= DS18B20_PIN; // Set as output
    DS18B20_OUT &= ~DS18B20_PIN; // Pull low
    __delay_cycles(3);
    DS18B20_DIR &= ~DS18B20_PIN; // Set as input
    __delay_cycles(10);
    r = (DS18B20_IN & DS18B20_PIN) ? 1 : 0;
    __delay_cycles(53);

    return r;
}

void onewire_write(uint8_t v, uint8_t power /* = 0 */)
{
    uint8_t bitMask;
    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
        onewire_write_bit((bitMask & v) ? 1 : 0);
    }
    if (!power) {
        DS18B20_DIR &= ~DS18B20_PIN; // Set as input
        DS18B20_OUT &= ~DS18B20_PIN; // Pull low
    }
}

uint8_t onewire_read(void)
{
    uint8_t r = 0;
    uint8_t bitMask;

    for (bitMask = 0x01; bitMask; bitMask <<= 1)
    {
        if (onewire_read_bit()) r |= bitMask;
    }
    return r;
}

void onewire_select(uint8_t rom[8])
{
    onewire_write(0x55, 0); // Match ROM command
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        onewire_write(rom[i], 0);
    }
}

void ds_set_resolution(uint8_t rom[8], uint8_t resolution)
{
    // Write Scratchpad command (0x4E)
    onewire_reset();
    onewire_select(rom);
    onewire_write(0x4E, 0); // Write Scratchpad command

    // Write configuration register value
    // Default configuration for 9-bit resolution is 0x1F
    onewire_write(resolution, 0); // Configuration register
    // Writing TH and TL bytes with default values
    onewire_write(0x00, 0); // TH register (temperature high byte)
    onewire_write(0x00, 0); // TL register (temperature low byte)
}

float ds_getTemp(uint8_t rom[8])
{
    uint8_t data[9];
    int16_t raw;
    float celsius;

    while (!onewire_reset())
    {
        ser_output("Error: no sensor\r\n");
        __delay_cycles(1000000);
    }

    onewire_reset();
    onewire_select(rom); // Select sensor
//    onewire_write(0xCC, 0); // Read scratchpad
    onewire_write(0x44, 1); // Start temperature conversion with parasite power on

    __delay_cycles(93750); // Wait for conversion 750ms or 100ms

    onewire_reset();
    onewire_select(rom); // Select sensor
//    onewire_write(0xCC, 0); // Read scratchpad
    onewire_write(0xBE, 0); // Read scratchpad

    uint8_t i;
    for (i = 0; i < 9; i++)
    {
        data[i] = onewire_read();
    }

    raw = (data[1] << 8) | data[0];
    celsius = (float) raw / 16.0;
    return celsius;
}
