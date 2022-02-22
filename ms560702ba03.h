#ifndef DRIVER_SUBMODULE_TEMPERATURE_SENSOR_MS560702BA03_MS560702BA03_H_
#define DRIVER_SUBMODULE_TEMPERATURE_SENSOR_MS560702BA03_MS560702BA03_H_

#define CC1310
#define MS567072BA03

#include <stdint.h>

#ifdef CC1310

#include <ti/drivers/I2C.h>

#endif

#ifdef CC1312R1

#include <ti/drivers/I2C.h>
#include "ti_drivers_config.h"

#endif

#ifdef NRF52

#include "nrf_drv_twi.h"

#endif

#if defined(CC1310) || defined(CC1312R1)

void set_ms560702ba03_i2c_instance(I2C_Handle i2cInstance);

#endif

#ifdef MS567072BA03

#ifdef CC1312R1

#include <ti/drivers/I2C.h>
#include "ti_drivers_config.h"

#endif

#ifdef NRF52

#include "nrf_drv_twi.h"

#endif

#ifdef NRF52

extern volatile uint8_t app_timer_100_ms_timer;
extern volatile uint8_t twi_read_done;
extern volatile uint8_t twi_write_done;
extern volatile uint8_t twi_address_nack;

void set_ms560702ba03_i2c_instance(nrf_drv_twi_t twi_t);

#endif

uint8_t init_ms560702ba03(void);

void set_ms560702ba03_OSR(uint16_t osr);

uint8_t set_ms560702ba03_conversion_pressure(void);

uint8_t set_ms560702ba03_measure_pressure(void);

uint8_t get_ms560702ba03_pressure(uint32_t *pressure_output);

uint8_t set_ms560702ba03_conversion_temperature(void);

uint8_t set_ms560702ba03_measure_temperature(void);

uint8_t get_ms560702ba03_temperature(uint32_t *temperature_output);

void set_ms560702ba03_convert_temperature_pressure(uint32_t temperature,
                                                   uint32_t pressure);

int32_t get_ms560702ba03_convert_pressure(void);

int16_t get_ms560702ba03_convert_temperature(void);

#endif

#endif /* DRIVER_SUBMODULE_TEMPERATURE_SENSOR_MS560702BA03_MS560702BA03_H_ */
