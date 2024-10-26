#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>  // For usleep
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define OLED_CS_PIN 10        // Chip Select pin for OLED
#define OLED_DC_PIN 17        // Data/Command pin for OLED
#define OLED_RST_PIN 4       // Reset pin for OLED
#define SPI_CHANNEL 0         // SPI channel (0 or 1 on Raspberry Pi)
#define SPI_SPEED   1000000   // SPI speed (1 MHz)

#define WIDTH 128
#define HEIGHT 64
#define BUFFER_SIZE (WIDTH * HEIGHT / 2)  // Each pixel is 4 bits

// Buffer for the display data
uint8_t display_buffer[BUFFER_SIZE];

// GPIO helper functions
void gpio_set_pin_high(int pin) {
    digitalWrite(pin, HIGH);
}

void gpio_set_pin_low(int pin) {
    digitalWrite(pin, LOW);
}

// SPI transfer function
void spi_transfer(uint8_t *data, uint16_t length) {
    wiringPiSPIDataRW(SPI_CHANNEL, data, length);
}

// Reset the display
void ssd1325_reset(void) {
    gpio_set_pin_low(OLED_RST_PIN);      // Pull RST low
    usleep(10 * 1000);                   // Delay for 10 ms
    gpio_set_pin_high(OLED_RST_PIN);     // Pull RST high
    usleep(10 * 1000);                   // Delay for 10 ms
}

// Send a command byte to the OLED
void ssd1325_send_command(uint8_t cmd) {
    gpio_set_pin_low(OLED_CS_PIN);       // Select the OLED
    gpio_set_pin_low(OLED_DC_PIN);       // Set DC low for command mode
    spi_transfer(&cmd, 1);               // Send the command byte
    gpio_set_pin_high(OLED_CS_PIN);      // Deselect the OLED
}

// Send data bytes to the OLED
void ssd1325_send_data(uint8_t *data, uint16_t size) {
    gpio_set_pin_low(OLED_CS_PIN);       // Select the OLED
    gpio_set_pin_high(OLED_DC_PIN);      // Set DC high for data mode
    spi_transfer(data, size);            // Send the data bytes
    gpio_set_pin_high(OLED_CS_PIN);      // Deselect the OLED
}

// Initialize the OLED display
void ssd1325_init(void) {
    ssd1325_send_command(0xAE);  // Display OFF
    ssd1325_send_command(0xA0);  // Set Re-map / Dual COM Line Mode
    ssd1325_send_command(0x10);  // Set Re-map to 128x64
    ssd1325_send_command(0xA1);  // Set Display Start Line
    ssd1325_send_command(0x00);
    ssd1325_send_command(0xA2);  // Set Display Offset
    ssd1325_send_command(0x60);
    ssd1325_send_command(0xA4);  // Set Display Mode - Normal Display
    ssd1325_send_command(0xAB);  // Function Selection
    ssd1325_send_command(0x01);  // Enable internal VDD regulator
    ssd1325_send_command(0x81);  // Set Contrast Current
    ssd1325_send_command(0x7F);  // Contrast value
    ssd1325_send_command(0xB1);  // Set Phase Length
    ssd1325_send_command(0xF2);
    ssd1325_send_command(0xB3);  // Set Display Clock Divide Ratio / Oscillator Frequency
    ssd1325_send_command(0xD0);  // Display Clock divide ratio
    ssd1325_send_command(0xBB);  // Set Pre-charge Voltage
    ssd1325_send_command(0x3A);  // Pre-charge voltage value
    ssd1325_send_command(0xBE);  // Set VCOMH Voltage
    ssd1325_send_command(0x3E);
    ssd1325_send_command(0xA6);  // Set Normal Display (not inverted)
    ssd1325_send_command(0xAF);  // Display ON
}

// Update the OLED display with the buffer data
void ssd1325_update(uint8_t *buffer) {
    // Set column and row addressing for entire screen
    ssd1325_send_command(0x15);  // Set Column Address
    ssd1325_send_command(0x00);  // Column start address
    ssd1325_send_command(0x3F);  // Column end address (0x3F = 63 in 4-bit mode)

    ssd1325_send_command(0x75);  // Set Row Address
    ssd1325_send_command(0x00);  // Row start address
    ssd1325_send  // Write buffer to OLED display RAM
    ssd1325_send_data(buffer, BUFFER_SIZE);
}

int main(void) {
    // Initialize wiringPi and set GPIO modes
    wiringPiSetup();
    pinMode(OLED_CS_PIN, OUTPUT);
    pinMode(OLED_DC_PIN, OUTPUT);
    pinMode(OLED_RST_PIN, OUTPUT);

    // Initialize SPI
    wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED);

    // Reset and initialize the display
    ssd1325_reset();
    ssd1325_init();

    // Clear the display buffer and update
    memset(display_buffer, 0x00, BUFFER_SIZE);  // Clear the buffer (all pixels off)
    ssd1325_update(display_buffer);             // Update display with blank buffer

    // Example: Draw something in the buffer here and call ssd1325_update(display_buffer)
    for (int row=0; row<HEIGHT; row++ )
        for (int col=0; col<WIDTH/2; col++ )
        {
            if((row/2)==0)
            {
                if((col/2==0))
                    *(display_buffer+row*(WIDTH/2)+col) = 0xFF;
                else
                    *(display_buffer+row*(WIDTH/2)+col) = 0x00;
            }
            else
            {
                if((col/2==0))
                    *(display_buffer+row*(WIDTH/2)+col) = 0x00;
                else
                    *(display_buffer+row*(WIDTH/2)+col) = 0xFF;
            }
        }


    return 0;
}
