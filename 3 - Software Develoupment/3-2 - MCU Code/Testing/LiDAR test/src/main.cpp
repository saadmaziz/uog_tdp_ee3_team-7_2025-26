#include <mbed.h>

// Pin Mappings based on L432KC Nucleo
#define SDA_PIN D4
#define SCL_PIN D5
#define EN_PIN  D6
#define BUZ_PIN A6

// TMF8806 default 7-bit I2C address is 0x41. 
// Mbed requires the 8-bit shifted I2C address.
#define TMF8806_ADDR (0x41 << 1)

// Peripheral initialization
I2C i2c(SDA_PIN, SCL_PIN);
DigitalOut en(EN_PIN);
PwmOut buzzer(BUZ_PIN);

// Helper function to write a single byte to a register
void writeReg(uint8_t reg, uint8_t val) {
    char data[2] = {(char)reg, (char)val};
    i2c.write(TMF8806_ADDR, data, 2);
}

// Helper function to read a single byte from a register
uint8_t readReg(uint8_t reg) {
    char val = 0;
    i2c.write(TMF8806_ADDR, (char*)&reg, 1, true); // Write register address (no stop)
    i2c.read(TMF8806_ADDR, &val, 1);               // Read back the value
    return val;
}

// Bootloader startup sequence (Datasheet Fig 16)
void initTMF8806() {
    en = 0; ThisThread::sleep_for(10ms);
    en = 1; ThisThread::sleep_for(2ms);
    
    while (readReg(0xE0) != 0x00) ThisThread::sleep_for(1ms); // Wait for sleep
    writeReg(0xE0, 0x01);                                     // Wake bootloader
    while (readReg(0xE0) != 0x41) ThisThread::sleep_for(1ms); // Wait for ready
    writeReg(0x02, 0xC0);                                     // Start App0
    while (readReg(0x00) != 0xC0) ThisThread::sleep_for(1ms); // Wait for App0
    
    writeReg(0xE1, 0x01);                                     // Clear INT flag
}

int main() {
    printf("Starting TMF8806 ToF Sensor...\n");
    i2c.frequency(400000); // 400 kHz Fast Mode
    
    initTMF8806();
    printf("TMF8806 Initialized & Running!\n");

    Timer loopTimer;
    loopTimer.start();

    while (true) {
        loopTimer.reset();

        // Config: StartReg(0x06), SpreadSpec(x2), NoCalib, 2.5m, GPIOs Off, Thresh=6, Single, 900k iters
        char config[] = {0x06, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x06, 0x00, 0x84, 0x03};
        i2c.write(TMF8806_ADDR, config, sizeof(config));
        
        writeReg(0x10, 0x02); // Dispatch the measurement command

        // Wait for measurement to complete by polling INT status (bit 0 of 0xE1)
        int timeout = 100;
        while ((readReg(0xE1) & 0x01) == 0 && timeout--) {
            ThisThread::sleep_for(1ms);
        }

        if (timeout > 0) {
            // Read object detection results
            char res[4];
            char reg = 0x20;
            i2c.write(TMF8806_ADDR, &reg, 1, true);
            i2c.read(TMF8806_ADDR, res, 4); // [RESULT_NUMBER, RESULT_INFO, PEAK_0_LSB, PEAK_1_MSB]
            
            uint16_t distance = res[2] | (res[3] << 8);
            uint8_t reliability = res[1] & 0x3F;

            printf("Distance: %d mm \t Reliability: %d\n", distance, reliability);

            // Update Buzzer Pitch
            if (reliability > 0 && distance > 0 && distance <= 2500) {
                float freq = 2000.0f - (distance * 0.7f);
                if (freq < 100.0f) freq = 100.0f; // Minimum 100Hz
                
                buzzer.period(1.0f / freq);
                buzzer.write(0.5f); // 50% duty cycle (beep on)
            } else {
                buzzer.write(0.0f); // Silence
            }
        } else {
            printf("Measurement timeout!\n");
            buzzer.write(0.0f); // Silence
        }

        writeReg(0xE1, 0x01); // Clear INT flag
    }
}