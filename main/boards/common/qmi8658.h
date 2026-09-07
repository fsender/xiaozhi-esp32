#ifndef __QMI8658_H__
#define __QMI8658_H__

#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdint>

class Qmi8658 {
public:
    Qmi8658(i2c_master_bus_handle_t i2c_bus, uint8_t addr = 0x6A)
        : i2c_bus_(i2c_bus), addr_(addr), initialized_(false), i2c_device_(nullptr) {}

    bool Init() {
        if (!AddDevice()) {
            return false;
        }

        // Probe WHO_AM_I
        uint8_t who_am_i = ReadReg(kRegWhoAmI);
        if (who_am_i != kWhoAmIExpected) {
            ESP_LOGE(TAG, "WHO_AM_I mismatch: expected 0x%02X, got 0x%02X", kWhoAmIExpected,
                     who_am_i);
            return false;
        }
        ESP_LOGI(TAG, "QMI8658 detected (WHO_AM_I=0x%02X)", who_am_i);

        // Soft reset: set bit 6 of CTRL1
        WriteReg(kRegCtrl1, 0x40);
        vTaskDelay(pdMS_TO_TICKS(50));

        // Configure CTRL1: address increment, I2C/SPI auto
        WriteReg(kRegCtrl1, 0x60);

        // Configure CTRL2: accel ±8g (0x08), 250Hz ODR (0x32)
        WriteReg(kRegCtrl2, 0x08 | 0x32);

        // Configure CTRL3: gyro ±512dps (0x14), 250Hz ODR (0x32)
        WriteReg(kRegCtrl3, 0x14 | 0x32);

        // Configure CTRL5: default LPF
        WriteReg(kRegCtrl5, 0x00);

        // Enable accel (bit0) and gyro (bit1)
        WriteReg(kRegCtrl7, 0x03);

        // Verify enable took effect
        uint8_t ctrl7 = ReadReg(kRegCtrl7);
        if ((ctrl7 & 0x03) != 0x03) {
            ESP_LOGE(TAG, "Failed to enable sensors, CTRL7=0x%02X", ctrl7);
            return false;
        }

        initialized_ = true;
        ESP_LOGI(TAG, "Initialized: accel ±8g/250Hz, gyro ±512dps/250Hz");
        return true;
    }

    bool ReadAccel(float& x, float& y, float& z) {
        if (!initialized_) {
            return false;
        }

        uint8_t buf[6];
        if (!ReadRegs(kRegAccelXL, buf, 6)) {
            return false;
        }

        int16_t raw_x = static_cast<int16_t>(buf[0] | (buf[1] << 8));
        int16_t raw_y = static_cast<int16_t>(buf[2] | (buf[3] << 8));
        int16_t raw_z = static_cast<int16_t>(buf[4] | (buf[5] << 8));

        // ±8g range: sensitivity = 4096 LSB/g
        constexpr float kAccelSensitivity = 4096.0f;
        x = raw_x / kAccelSensitivity;
        y = raw_y / kAccelSensitivity;
        z = raw_z / kAccelSensitivity;
        return true;
    }

    bool ReadGyro(float& x, float& y, float& z) {
        if (!initialized_) {
            return false;
        }

        uint8_t buf[6];
        if (!ReadRegs(kRegGyroXL, buf, 6)) {
            return false;
        }

        int16_t raw_x = static_cast<int16_t>(buf[0] | (buf[1] << 8));
        int16_t raw_y = static_cast<int16_t>(buf[2] | (buf[3] << 8));
        int16_t raw_z = static_cast<int16_t>(buf[4] | (buf[5] << 8));

        // ±512dps range: sensitivity = 64 LSB/(dps)
        constexpr float kGyroSensitivity = 64.0f;
        x = raw_x / kGyroSensitivity;
        y = raw_y / kGyroSensitivity;
        z = raw_z / kGyroSensitivity;
        return true;
    }

    bool IsInitialized() const { return initialized_; }

private:
    static constexpr const char* TAG = "QMI8658";

    // Register addresses
    static constexpr uint8_t kRegWhoAmI = 0x00;
    static constexpr uint8_t kRegCtrl1 = 0x02;
    static constexpr uint8_t kRegCtrl2 = 0x03;
    static constexpr uint8_t kRegCtrl3 = 0x04;
    static constexpr uint8_t kRegCtrl5 = 0x06;
    static constexpr uint8_t kRegCtrl7 = 0x0E;
    static constexpr uint8_t kRegAccelXL = 0x35;
    static constexpr uint8_t kRegGyroXL = 0x3B;

    static constexpr uint8_t kWhoAmIExpected = 0x05;

    i2c_master_bus_handle_t i2c_bus_;
    uint8_t addr_;
    bool initialized_;
    i2c_master_dev_handle_t i2c_device_;

    bool AddDevice() {
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr_,
            .scl_speed_hz = 400 * 1000,
            .scl_wait_us = 0,
            .flags =
                {
                    .disable_ack_check = 0,
                },
        };
        esp_err_t err = i2c_master_bus_add_device(i2c_bus_, &cfg, &i2c_device_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
            return false;
        }
        return true;
    }

    bool WriteReg(uint8_t reg, uint8_t value) {
        uint8_t buf[2] = {reg, value};
        esp_err_t err = i2c_master_transmit(i2c_device_, buf, 2, 100);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "WriteReg(0x%02X) failed: %s", reg, esp_err_to_name(err));
            return false;
        }
        return true;
    }

    uint8_t ReadReg(uint8_t reg) {
        uint8_t val = 0;
        esp_err_t err = i2c_master_transmit_receive(i2c_device_, &reg, 1, &val, 1, 100);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ReadReg(0x%02X) failed: %s", reg, esp_err_to_name(err));
        }
        return val;
    }

    bool ReadRegs(uint8_t reg, uint8_t* buf, size_t len) {
        esp_err_t err = i2c_master_transmit_receive(i2c_device_, &reg, 1, buf, len, 100);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ReadRegs(0x%02X, %u) failed: %s", reg, (unsigned)len,
                     esp_err_to_name(err));
            return false;
        }
        return true;
    }
};

#endif  // __QMI8658_H__
