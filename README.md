# MS5837
Arduino sketch for the MS5837-30BA  Barometer

Methods for configuring the MS5837-30BA barometer, checking the CRC, reading the calibration data, reading temperature and pressure raw data and constructing scaled temperature and pressure. Intended to work with the CMWX1ZZABZ (STM32L082 and SX1276) LoRaWAN-enabled MCU but the sketch is easily modified to work with ant I2C-enabled MCU.

I had to add a special WriteCommand method to the I2CDev API for the specific way the data registers have to be read with Measurement Specialties I2C barometers.
