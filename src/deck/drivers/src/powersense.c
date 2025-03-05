/**
 * Crazyflie I2C Communication with INA260 Current/Voltage Monitor
 *
 * This file implements a custom deck driver for the Crazyflie that interfaces
 * with the INA260 current/voltage/power monitor over I2C.
 */

 #include <stdint.h>
 #include <string.h>
 #include <stdbool.h>
 
 #include "deck.h"
 #include "deck_core.h"
 #include "i2cdev.h"
 #include "param.h"
 #include "FreeRTOS.h"
 #include "task.h"
 #include "debug.h"
 #include "log.h"
 
 // INA260 I2C Address (typically 0x40, but can be changed via address pins)
 #define INA260_I2C_ADDRESS      0x40
 #define I2C_BUS                 I2C1_DEV  // Use I2C1 bus on Crazyflie
 
 // INA260 Register Addresses
 #define INA260_REG_CONFIG       0x00  // Configuration register
 #define INA260_REG_CURRENT      0x01  // Current measurement register
 #define INA260_REG_VOLTAGE      0x02  // Bus voltage measurement register
 #define INA260_REG_POWER        0x03  // Power measurement register
 #define INA260_REG_MASK_ENABLE  0x06  // Mask/Enable register
 #define INA260_REG_ALERT_LIMIT  0x07  // Alert limit register
 #define INA260_REG_MFG_ID       0xFE  // Manufacturer ID register (should be 0x5449)
 #define INA260_REG_DIE_ID       0xFF  // Die ID register (should be 0x2270)
 
 // INA260 Configuration Settings
 #define INA260_CONFIG_RESET              0x8000  // Reset bit
 #define INA260_CONFIG_AVG_1              0x0000  // 1 sample average
 #define INA260_CONFIG_AVG_4              0x0200  // 4 sample average
 #define INA260_CONFIG_AVG_16             0x0400  // 16 sample average
 #define INA260_CONFIG_AVG_64             0x0600  // 64 sample average
 #define INA260_CONFIG_AVG_128            0x0800  // 128 sample average
 #define INA260_CONFIG_AVG_256            0x0A00  // 256 sample average
 #define INA260_CONFIG_AVG_512            0x0C00  // 512 sample average
 #define INA260_CONFIG_AVG_1024           0x0E00  // 1024 sample average
 #define INA260_CONFIG_VBUSCT_140         0x0000  // Voltage conversion time 140us
 #define INA260_CONFIG_VBUSCT_204         0x0040  // Voltage conversion time 204us
 #define INA260_CONFIG_VBUSCT_332         0x0080  // Voltage conversion time 332us
 #define INA260_CONFIG_VBUSCT_588         0x00C0  // Voltage conversion time 588us
 #define INA260_CONFIG_VBUSCT_1100        0x0100  // Voltage conversion time 1.1ms
 #define INA260_CONFIG_VBUSCT_2116        0x0140  // Voltage conversion time 2.116ms
 #define INA260_CONFIG_VBUSCT_4156        0x0180  // Voltage conversion time 4.156ms
 #define INA260_CONFIG_VBUSCT_8244        0x01C0  // Voltage conversion time 8.244ms
 #define INA260_CONFIG_ISHCT_140          0x0000  // Current conversion time 140us
 #define INA260_CONFIG_ISHCT_204          0x0008  // Current conversion time 204us
 #define INA260_CONFIG_ISHCT_332          0x0010  // Current conversion time 332us
 #define INA260_CONFIG_ISHCT_588          0x0018  // Current conversion time 588us
 #define INA260_CONFIG_ISHCT_1100         0x0020  // Current conversion time 1.1ms
 #define INA260_CONFIG_ISHCT_2116         0x0028  // Current conversion time 2.116ms
 #define INA260_CONFIG_ISHCT_4156         0x0030  // Current conversion time 4.156ms
 #define INA260_CONFIG_ISHCT_8244         0x0038  // Current conversion time 8.244ms
 #define INA260_CONFIG_MODE_SHUTDOWN      0x0000  // Shutdown mode
 #define INA260_CONFIG_MODE_TRIG_SHUNT    0x0001  // Triggered shunt voltage measurement
 #define INA260_CONFIG_MODE_TRIG_BUS      0x0002  // Triggered bus voltage measurement
 #define INA260_CONFIG_MODE_TRIG_BOTH     0x0003  // Triggered shunt and bus voltage measurement
 #define INA260_CONFIG_MODE_CONT_SHUNT    0x0005  // Continuous shunt current measurement
 #define INA260_CONFIG_MODE_CONT_BUS      0x0006  // Continuous bus voltage measurement
 #define INA260_CONFIG_MODE_CONT_BOTH     0x0007  // Continuous shunt and bus voltage measurement
 
 // Read interval in milliseconds
 #define INA260_READ_INTERVAL_MS 10
 
 // Global variables
 static bool isInit = false;
 static TaskHandle_t ina260TaskHandle;
 
 // Measured values
 static float currentInAmps = 0.0f;
 static float voltageInVolts = 0.0f;
 static float powerInWatts = 0.0f;
 static bool enableMeasurements = true;
 
 // Parameter and logging definitions
 PARAM_GROUP_START(ina260)
 PARAM_ADD(PARAM_FLOAT, current, &currentInAmps)
 PARAM_ADD(PARAM_FLOAT, voltage, &voltageInVolts)
 PARAM_ADD(PARAM_FLOAT, power, &powerInWatts)
 PARAM_ADD(PARAM_UINT8, enable, &enableMeasurements)
 PARAM_GROUP_STOP(ina260)
 
 LOG_GROUP_START(ina260)
 LOG_ADD(LOG_FLOAT, current, &currentInAmps)
 LOG_ADD(LOG_FLOAT, voltage, &voltageInVolts)
 LOG_ADD(LOG_FLOAT, power, &powerInWatts)
 LOG_GROUP_STOP(ina260)
 
 /**
  * Write 16-bit value to INA260 register
  *
  * @param reg    Register address
  * @param value  Value to write
  * @return       true if successful
  */
 static bool ina260WriteReg(uint8_t reg, uint16_t value) {
     uint8_t data[2];
     data[0] = (value >> 8) & 0xFF;  // MSB first (big-endian)
     data[1] = value & 0xFF;         // LSB
 
     return i2cdevWriteReg8(I2C_BUS, INA260_I2C_ADDRESS, reg, 2, data);
 }
 
 /**
  * Read 16-bit value from INA260 register
  *
  * @param reg    Register address
  * @param value  Pointer to store read value
  * @return       true if successful
  */
 static bool ina260ReadReg(uint8_t reg, uint16_t *value) {
     uint8_t data[2];
 
     if (i2cdevReadReg8(I2C_BUS, INA260_I2C_ADDRESS, reg, 2, data)) {
         *value = ((uint16_t)data[0] << 8) | data[1];  // MSB first (big-endian)
         return true;
     }
 
     return false;
 }
 
 /**
  * Configure the INA260 sensor
  *
  * @return true if successful
  */
 static bool ina260Configure(void) {
     // Reset the device
     if (!ina260WriteReg(INA260_REG_CONFIG, INA260_CONFIG_RESET)) {
         return false;
     }
 
     // Wait for reset to complete
     vTaskDelay(M2T(10));
 
     // Configure for continuous bus voltage and current measurements
     // with 16 sample averaging and moderate conversion times
     uint16_t config = INA260_CONFIG_AVG_16 |       // 16 sample average
                       INA260_CONFIG_VBUSCT_588 |   // 588µs bus voltage conversion time
                       INA260_CONFIG_ISHCT_588 |    // 588µs shunt current conversion time
                       INA260_CONFIG_MODE_CONT_BOTH; // Continuous mode for both measurements
 
     return ina260WriteReg(INA260_REG_CONFIG, config);
 }
 
 /**
  * Check if INA260 is present by reading manufacturer ID
  *
  * @return true if INA260 is detected
  */
 static bool ina260CheckId(void) {
     uint16_t mfgId, dieId;
 
     if (!ina260ReadReg(INA260_REG_MFG_ID, &mfgId) ||
         !ina260ReadReg(INA260_REG_DIE_ID, &dieId)) {
         return false;
     }
 
     // Texas Instruments manufacturer ID is 0x5449
     // INA260 Die ID is 0x2270
     return (mfgId == 0x5449 && dieId == 0x2270);
 }
 
 /**
  * Read current measurement from INA260
  *
  * @return current in Amperes (positive for current flowing from supply to load)
  */
 static float ina260ReadCurrent(void) {
     uint16_t rawCurrent;
 
     if (!ina260ReadReg(INA260_REG_CURRENT, &rawCurrent)) {
         return 0.0f;
     }
 
     // Convert from 2's complement format
     int16_t signedCurrent = (int16_t)rawCurrent;
 
     // LSB is 1.25mA per bit
     return signedCurrent * 0.00125f;
 }
 
 /**
  * Read bus voltage measurement from INA260
  *
  * @return voltage in Volts
  */
 static float ina260ReadVoltage(void) {
     uint16_t rawVoltage;
 
     if (!ina260ReadReg(INA260_REG_VOLTAGE, &rawVoltage)) {
         return 0.0f;
     }
 
     // LSB is 1.25mV per bit
     return rawVoltage * 0.00125f;
 }
 
 /**
  * Read power measurement from INA260
  *
  * @return power in Watts
  */
 static float ina260ReadPower(void) {
     uint16_t rawPower;
 
     if (!ina260ReadReg(INA260_REG_POWER, &rawPower)) {
         return 0.0f;
     }
 
     // LSB is 10mW per bit
     return rawPower * 0.01f;
 }
 
 /**
  * Task that periodically reads from INA260
  */
 static void ina260Task(void *param) {
     TickType_t lastWakeTime = xTaskGetTickCount();
 
     while (1) {
         if (enableMeasurements) {
             // Read all measurements
             currentInAmps = ina260ReadCurrent();
             voltageInVolts = ina260ReadVoltage();
             powerInWatts = ina260ReadPower();
 
            //  DEBUG_PRINT("INA260: Current=%.3fA, Voltage=%.3fV, Power=%.3fW\n",
            //              currentInAmps, voltageInVolts, powerInWatts);
         }
 
         // Run at regular intervals
         vTaskDelayUntil(&lastWakeTime, M2T(INA260_READ_INTERVAL_MS));
     }
 }
 
 /**
  * Initialize the INA260 deck
  *
  * @return true if initialization succeeded
  */
 static bool ina260DeckInit(void) {
     if (isInit) {
         return true;
     }
 
     // Initialize I2C bus
     if (!i2cdevInit(I2C_BUS)) {
        //  DEBUG_PRINT("INA260: I2C init failed\n");
         return false;
     }
 
     // Check if INA260 is present
     if (!ina260CheckId()) {
        //  DEBUG_PRINT("INA260: Device not found\n");
         return false;
     }
 
     // Configure the device
     if (!ina260Configure()) {
        //  DEBUG_PRINT("INA260: Configuration failed\n");
         return false;
     }
 
     // Create task for periodic readings
     xTaskCreate(ina260Task, "ina260", configMINIMAL_STACK_SIZE, NULL, 3, &ina260TaskHandle);
 
    //  DEBUG_PRINT("INA260: Initialized successfully\n");
     isInit = true;
     return true;
 }
 
 /**
  * Test the INA260 deck
  *
  * @return true if test passed
  */
 static bool ina260DeckTest(void) {
     if (!isInit) {
         return false;
     }
 
     // Test by reading voltage
     float voltage = ina260ReadVoltage();
     bool testResult = (voltage > 0.0f);  // Assume voltage should be positive if connected
 
    //  DEBUG_PRINT("INA260 test %s. Voltage: %.3fV\n",
    //              testResult ? "passed" : "failed", voltage);
 
     return testResult;
 }
 
 /**
  * Power down the INA260 deck
  */
//  static void ina260DeckPowerOff(void) {
//      // Disable measurements
//      enableMeasurements = false;
 
//      // Put INA260 into shutdown mode
//      ina260WriteReg(INA260_REG_CONFIG, INA260_CONFIG_MODE_SHUTDOWN);
 
//      // Wait a bit to make sure any ongoing I2C transaction completes
//      vTaskDelay(M2T(10));
 
//      // Delete the task
//      if (ina260TaskHandle != NULL) {
//          vTaskDelete(ina260TaskHandle);
//          ina260TaskHandle = NULL;
//      }
 
//      isInit = false;
//      DEBUG_PRINT("INA260: Powered off\n");
//  }
 
 /**
  * Public function to read current
  *
  * @return current in Amperes
  */
 float ina260GetCurrent(void) {
     if (!isInit) {
         return 0.0f;
     }
 
     return ina260ReadCurrent();
 }
 
 /**
  * Public function to read voltage
  *
  * @return voltage in Volts
  */
 float ina260GetVoltage(void) {
     if (!isInit) {
         return 0.0f;
     }
 
     return ina260ReadVoltage();
 }
 
 /**
  * Public function to read power
  *
  * @return power in Watts
  */
 float ina260GetPower(void) {
     if (!isInit) {
         return 0.0f;
     }
 
     return ina260ReadPower();
 }
 
 // Deck driver definition
 static const DeckDriver ina260_deck = {
     .name = "ina260-deck",
     .init = ina260DeckInit,
     .test = ina260DeckTest
 };
 
 // Register the deck driver with the system
 DECK_DRIVER(ina260_deck);
 
//  // Expose public API functions
//  extern "C" {
//      float ina260GetCurrent(void);
//      float ina260GetVoltage(void);
//      float ina260GetPower(void);
//  }
 