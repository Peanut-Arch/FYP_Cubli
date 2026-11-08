#ifndef MAGNETIC_SENSOR_PIO_I2C_H
#define MAGNETIC_SENSOR_PIO_I2C_H

#include <SimpleFOC.h>
#include "hardware/gpio.h"

#define AS5600_I2C_ADDR    0x36
#define AS5600_REG_RAW_H   0x0C

class MagneticSensorPIOI2C : public Sensor {
public:
    volatile uint32_t i2c_fail_count = 0;

    MagneticSensorPIOI2C(PIO /*pio*/, uint /*sm*/, uint sda_pin, uint scl_pin)
        : sda_pin_(sda_pin), scl_pin_(scl_pin) {}

    void init() override {
        gpio_init(sda_pin_);
        gpio_init(scl_pin_);

        gpio_set_dir(sda_pin_, GPIO_IN);
        gpio_set_dir(scl_pin_, GPIO_IN);

        gpio_put(sda_pin_, 0);
        gpio_put(scl_pin_, 0);

        gpio_pull_up(sda_pin_);
        gpio_pull_up(scl_pin_);

        writeRegisterPair(0x07, 0x03, 0x00);

        this->velocity = 0;
        this->angle_prev = getSensorAngle();
        this->angle_prev_ts = _micros();
    }

    float getSensorAngle() override {
        uint16_t raw = readRegister16(AS5600_REG_RAW_H);
        return (float)raw * (2.0f * M_PI / 4096.0f);
    }

private:
    uint sda_pin_;
    uint scl_pin_;
    uint16_t last_raw_ = 0;

    // ---------- Open-drain helpers ----------
    inline void sda_low()  { gpio_set_dir(sda_pin_, GPIO_OUT); }
    inline void sda_high() { gpio_set_dir(sda_pin_, GPIO_IN);  }
    inline void scl_low()  { gpio_set_dir(scl_pin_, GPIO_OUT); }
    inline void scl_high() { gpio_set_dir(scl_pin_, GPIO_IN);  }
    inline bool sda_read() { return gpio_get(sda_pin_); }

    inline void delay_half() {
        for (int i = 0; i < 12; i++) {
            __asm volatile ("nop");
        }
    }

    void i2c_start() {
        sda_high(); scl_high(); delay_half();
        sda_low();  delay_half();
        scl_low();  delay_half();
    }

    void i2c_stop() {
        sda_low();  delay_half();
        scl_high(); delay_half();
        sda_high(); delay_half();
    }

    bool i2c_write_byte(uint8_t byte) {
        for (int i = 7; i >= 0; i--) {
            if (byte & (1 << i)) sda_high();
            else                 sda_low();
            delay_half();
            scl_high(); delay_half();
            scl_low();  delay_half();
        }
        sda_high();
        delay_half();
        scl_high(); delay_half();
        bool ack = !sda_read();
        scl_low();  delay_half();
        return ack;
    }

    uint8_t i2c_read_byte(bool ack) {
        uint8_t byte = 0;
        sda_high();
        for (int i = 7; i >= 0; i--) {
            scl_high(); delay_half();
            if (sda_read()) byte |= (1 << i);
            scl_low();  delay_half();
        }
        if (ack) sda_low(); else sda_high();
        delay_half();
        scl_high(); delay_half();
        scl_low();  delay_half();
        sda_high();
        return byte;
    }

    void writeRegisterPair(uint8_t reg, uint8_t val1, uint8_t val2) {
        i2c_start();
        i2c_write_byte(AS5600_I2C_ADDR << 1);
        i2c_write_byte(reg);
        i2c_write_byte(val1);
        i2c_write_byte(val2);
        i2c_stop();
    }

    uint16_t readRegister16(uint8_t reg) {
        i2c_start();
        if (!i2c_write_byte(AS5600_I2C_ADDR << 1)) {
            i2c_stop();
            i2c_fail_count++;
            return last_raw_;
        }
        if (!i2c_write_byte(reg)) {
            i2c_stop();
            i2c_fail_count++;
            return last_raw_;
        }
        i2c_start();
        if (!i2c_write_byte((AS5600_I2C_ADDR << 1) | 1)) {
            i2c_stop();
            i2c_fail_count++;
            return last_raw_;
        }
        uint8_t msb = i2c_read_byte(true);
        uint8_t lsb = i2c_read_byte(false);
        i2c_stop();

        last_raw_ = (((uint16_t)msb << 8) | lsb) & 0x0FFF;
        return last_raw_;
    }
};

#endif