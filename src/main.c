#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#include "bmp280.h"

#define MISO 16
#define CS   17
#define SCLK 18
#define MOSI 19

#define SPI_PORT spi0

int32_t t_fine;
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;


int32_t compTemp(int32_t adc_T){
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t) dig_T1)) * ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t) dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
uint32_t compPress(int32_t adc_P)
{
    int32_t var1, var2;
    uint32_t p;
    var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
    var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
    var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
    var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
    if (p < 0x80000000)
    {
        p = (p << 1) / ((uint32_t)var1);
    }
    else
    {
        p = (p / (uint32_t)var1) * 2;
    }

    var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
    var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
    return p;
}

void read_temp_comp(){
    uint8_t buffer[6], reg;

    reg = 0x88 | 0x80;
    gpio_put(CS, 0);
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, buffer, 6);
    gpio_put(CS, 1);

    dig_T1 = buffer[0] | (buffer[1] << 8);
    dig_T2 = buffer[2] | (buffer[3] << 8);
    dig_T3 = buffer[4] | (buffer[5] << 8);
}

void read_press_comp(){
    uint8_t buffer[18], reg;

    reg = 0x8E | 0x80;
    gpio_put(CS, 0);
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, buffer, 18);
    gpio_put(CS, 1);

    dig_P1 = buffer[0] | (buffer[1] << 8);
    dig_P2 = buffer[2] | (buffer[3] << 8);
    dig_P3 = buffer[4] | (buffer[5] << 8);
    dig_P4 = buffer[6] | (buffer[7] << 8);
    dig_P5 = buffer[8] | (buffer[9] << 8);
    dig_P6 = buffer[10] | (buffer[11] << 8);
    dig_P7 = buffer[12] | (buffer[13] << 8);
    dig_P8 = buffer[14] | (buffer[15] << 8);
    dig_P9 = buffer[16] | (buffer[17] << 8);
}

int main(){
    stdio_init_all(); // Initialise I/O for USB Serial

    spi_init(SPI_PORT, 500000); // Initialise spi0 at 500kHz
    
    //Initialise GPIO pins for SPI communication
    gpio_set_function(MISO, GPIO_FUNC_SPI);
    gpio_set_function(SCLK, GPIO_FUNC_SPI);
    gpio_set_function(MOSI, GPIO_FUNC_SPI);

    // Configure Chip Select
    gpio_init(CS); // Initialise CS Pin
    gpio_set_dir(CS, GPIO_OUT); // Set CS as output
    gpio_put(CS, 1); // Set CS High to indicate no currect SPI communication

    read_temp_comp(); // Read factory calibration/compensation values
    read_press_comp();

    // Write Operation Example! Set oversampling and power on chip
    uint8_t data[2]; // Array to store data to be sent
    data[0] = 0xF4 & 0x7F; // Remove first bit to indicate write operation
    data[1] = 0x27; // Data to be sent
    gpio_put(CS, 0); // Indicate beginning of communication
    spi_write_blocking(SPI_PORT, data, 2); // Send data[]
    gpio_put(CS, 1); // Signal end of communication

    int32_t temperature, rawtemp, rawpress;
    uint32_t pressure;
    uint8_t reg, buffer[6];

    while(1){
        reg = 0xF7 | 0X80;
        gpio_put(CS, 0);
        spi_write_blocking(SPI_PORT, &reg, 1);
        spi_read_blocking(SPI_PORT, 0, buffer, 6);
        gpio_put(CS, 1);

        rawtemp = ((uint32_t) buffer[3] << 12) | ((uint32_t) buffer[4] << 4) | ((uint32_t) buffer[5] >> 4);

        temperature = compTemp(rawtemp);

        rawpress = ((uint32_t) buffer[0] << 12) | ((uint32_t) buffer[1] << 4) | ((uint32_t) buffer[2] >> 4);

        pressure = compPress(rawpress);

        printf("Temp = %.2f C \n", temperature / 100.00);
        printf("Pressure = %.2f mbar\n", pressure / 100.00);

        sleep_ms(1000);
    }
}
