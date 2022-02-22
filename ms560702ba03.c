#include "ms560702ba03.h"
#include "string.h"
#include "math.h"

#ifdef MS567072BA03

#if defined(CC1310) || defined(CC1312R1)

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

#endif

#ifdef NRF52

#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#endif

#define MS567072BA03_ADDRESS 0x77

#define MS567072BA03_PROM_READ 0xA0

static uint32_t C1 = 0x00;
static uint32_t C2 = 0x00;
static uint32_t C3 = 0x00;
static uint32_t C4 = 0x00;
static uint32_t C5 = 0x00;
static uint32_t C6 = 0x00;

static uint8_t CONV_D1 = 0x40;
static uint8_t CONV_D2 = 0x50;
static uint8_t CONV_DELAY = 1;

static double TEMP = 0;
static double P = 0;

#if defined(CC1310) || defined(CC1312R1)

static I2C_Handle i2c;
static I2C_Transaction i2cTransaction;

#endif

#ifdef NRF52

static nrf_drv_twi_t twi;

#endif

#ifdef NRF52

static void idle_state_handle(void) {

    if (NRF_LOG_PROCESS() == false) {

        nrf_pwr_mgmt_run();

    }

}

#endif

static void wait_100_ms()
{

#if defined(CC1310) || defined(CC1312R1)

    Task_sleep(100 * 1000 / Clock_tickPeriod);

#endif

#ifdef NRF52

    app_timer_100_ms_timer = 0x00;

    while (!app_timer_100_ms_timer) {

        idle_state_handle();

    }

#endif

}

#if defined(CC1310) || defined(CC1312R1)

void set_ms560702ba03_i2c_instance(I2C_Handle i2cInstance)
{

    i2c = i2cInstance;

}

#endif

#ifdef NRF52

void set_ms560702ba03_i2c_instance(nrf_drv_twi_t twi_t) {

    twi = twi_t;

}

#endif

static uint8_t i2c_write8(uint8_t address, uint8_t reg)
{

    uint8_t i2c_state = 0x00;
    uint8_t txBuffer = reg;

#if defined(CC1310) || defined(CC1312R1)

    if (i2c != NULL)
    {

        i2cTransaction.writeBuf = &txBuffer;
        i2cTransaction.readBuf = NULL;
        i2cTransaction.slaveAddress = address;

        i2cTransaction.writeCount = 1;
        i2cTransaction.readCount = 0;
        i2c_state = I2C_transfer(i2c, &i2cTransaction);

    }

#endif

#ifdef NRF52

    twi_write_done = 0x00;
    twi_address_nack = 0x00;

    nrf_drv_twi_tx(&twi, address, &txBuffer, 1, false);

    while (!twi_write_done) {

        idle_state_handle();

    }

    if (twi_address_nack) {

        i2c_state = 0x00;

    } else {

        i2c_state = 0x01;

    }

#endif

    return i2c_state;

}

static uint8_t i2c_read_buffer(uint8_t address, uint8_t *read_value,
                               const uint8_t buffer_size)
{

    uint8_t i2c_state = 0x00;
    uint8_t rxBuffer[buffer_size];

#if defined(CC1310) || defined(CC1312R1)

    if (i2c != NULL)
    {

        i2cTransaction.writeBuf = NULL;
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.slaveAddress = address;

        i2cTransaction.writeCount = 0;
        i2cTransaction.readCount = buffer_size;
        i2c_state = I2C_transfer(i2c, &i2cTransaction);

        memcpy(read_value, rxBuffer, buffer_size);

    }

#endif

    return i2c_state;

}

uint8_t init_ms560702ba03(void)
{

    uint8_t i2c_state = 0x01;
    uint8_t calibration_buffer[2] = { 0x00 };

    //Reset MS567072
    i2c_write8(MS567072BA03_ADDRESS, 0x1E);

    wait_100_ms(10);

    //Read Calibration Value C1
    i2c_state = i2c_state
            & i2c_write8(MS567072BA03_ADDRESS, MS567072BA03_PROM_READ + 2);
    i2c_state = i2c_state
            & i2c_read_buffer(MS567072BA03_ADDRESS, calibration_buffer,
                              sizeof(calibration_buffer));
    C1 = calibration_buffer[0] << 8 | calibration_buffer[1];

    //Read Calibration Value C2
    memset(calibration_buffer, 0x00, sizeof(calibration_buffer));
    i2c_state = i2c_state
            & i2c_write8(MS567072BA03_ADDRESS, MS567072BA03_PROM_READ + 4);
    i2c_state = i2c_state
            & i2c_read_buffer(MS567072BA03_ADDRESS, calibration_buffer,
                              sizeof(calibration_buffer));
    C2 = calibration_buffer[0] << 8 | calibration_buffer[1];

    //Read Calibration Value C3
    memset(calibration_buffer, 0x00, sizeof(calibration_buffer));
    i2c_state = i2c_state
            & i2c_write8(MS567072BA03_ADDRESS, MS567072BA03_PROM_READ + 6);
    i2c_state = i2c_state
            & i2c_read_buffer(MS567072BA03_ADDRESS, calibration_buffer,
                              sizeof(calibration_buffer));
    C3 = calibration_buffer[0] << 8 | calibration_buffer[1];

    //Read Calibration Value C4
    memset(calibration_buffer, 0x00, sizeof(calibration_buffer));
    i2c_state = i2c_state
            & i2c_write8(MS567072BA03_ADDRESS, MS567072BA03_PROM_READ + 8);
    i2c_state = i2c_state
            & i2c_read_buffer(MS567072BA03_ADDRESS, calibration_buffer,
                              sizeof(calibration_buffer));
    C4 = calibration_buffer[0] << 8 | calibration_buffer[1];

    //Read Calibration Value C2
    memset(calibration_buffer, 0x00, sizeof(calibration_buffer));
    i2c_state = i2c_state
            & i2c_write8(MS567072BA03_ADDRESS, MS567072BA03_PROM_READ + 10);
    i2c_state = i2c_state
            & i2c_read_buffer(MS567072BA03_ADDRESS, calibration_buffer,
                              sizeof(calibration_buffer));
    C5 = calibration_buffer[0] << 8 | calibration_buffer[1];

    //Read Calibration Value C2
    memset(calibration_buffer, 0x00, sizeof(calibration_buffer));
    i2c_state = i2c_state
            & i2c_write8(MS567072BA03_ADDRESS, MS567072BA03_PROM_READ + 12);
    i2c_state = i2c_state
            & i2c_read_buffer(MS567072BA03_ADDRESS, calibration_buffer,
                              sizeof(calibration_buffer));
    C6 = calibration_buffer[0] << 8 | calibration_buffer[1];

    return i2c_state;

}

void set_ms560702ba03_OSR(uint16_t osr)
{

    switch (osr)
    {
    case 256:
        CONV_D1 = 0x40;
        CONV_D2 = 0x50;
        CONV_DELAY = 1;
        break;
    case 512:
        CONV_D1 = 0x42;
        CONV_D2 = 0x52;
        CONV_DELAY = 2;
        break;
    case 1024:
        CONV_D1 = 0x44;
        CONV_D2 = 0x54;
        CONV_DELAY = 3;
        break;
    case 2048:
        CONV_D1 = 0x46;
        CONV_D2 = 0x56;
        CONV_DELAY = 5;
        break;
    case 4096:
        CONV_D1 = 0x48;
        CONV_D2 = 0x58;
        CONV_DELAY = 10;
        break;
    default:
        CONV_D1 = 0x40;
        CONV_D2 = 0x50;
        CONV_DELAY = 1;
        break;
    }

}

uint8_t set_ms560702ba03_conversion_pressure(void)
{

    return i2c_write8(MS567072BA03_ADDRESS, CONV_D1);

}

uint8_t set_ms560702ba03_measure_pressure(void)
{

    return i2c_write8(MS567072BA03_ADDRESS, 0x00);

}

uint8_t get_ms560702ba03_pressure(uint32_t *pressure_output)
{

    uint8_t i2c_state = 0x01;

    uint32_t output = 0x00;
    uint8_t value[3] = { 0x00 };

    i2c_state = i2c_read_buffer(MS567072BA03_ADDRESS, value, sizeof(value));

    output = value[0] << 16;
    output |= value[1] << 8;
    output |= value[2];

    *pressure_output = output;

    return i2c_state;

}

uint8_t set_ms560702ba03_conversion_temperature(void)
{

    return i2c_write8(MS567072BA03_ADDRESS, CONV_D2);

}

uint8_t set_ms560702ba03_measure_temperature(void)
{

    return i2c_write8(MS567072BA03_ADDRESS, 0x00);

}

uint8_t get_ms560702ba03_temperature(uint32_t *temperature_output)
{

    uint8_t i2c_state = 0x01;

    uint32_t output = 0x00;
    uint8_t value[3] = { 0x00 };

    i2c_state = i2c_read_buffer(MS567072BA03_ADDRESS, value, sizeof(value));

    output = value[0] << 16;
    output |= value[1] << 8;
    output |= value[2];

    *temperature_output = output;

    return i2c_state;

}

void set_ms560702ba03_convert_temperature_pressure(uint32_t temperature,
                                                   uint32_t pressure)
{

    double dT = 0;
    double OFF = 0;
    double SENS = 0;

    dT = temperature - C5 * pow(2, 8);
    OFF = C2 * pow(2, 16) + dT * C4 / pow(2, 7);
    SENS = C1 * pow(2, 15) + dT * C3 / pow(2, 8);
    TEMP = (2000 + (dT * C6) / pow(2, 23));
    P = (((pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15));

    // perform higher order corrections
    double T2 = 0.0;
    double OFF2 = 0.0;
    double SENS2 = 0.0;

    if (TEMP < 2000)
    {
        T2 = dT * dT / pow(2, 31);
        OFF2 = 5 * (TEMP - 2000) * (TEMP - 2000) / pow(2, 1);
        SENS2 = 5 * (TEMP - 2000) * (TEMP - 2000) / pow(2, 2);
        if (TEMP < -1500)
        {
            OFF2 += 7 * (TEMP + 1500) * (TEMP + 1500);
            SENS2 += 11 * (TEMP + 1500) * (TEMP + 1500) / pow(2, 1);
        }
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
    P = (((pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15));

}

int32_t get_ms560702ba03_convert_pressure(void)
{

    return P;

}

int16_t get_ms560702ba03_convert_temperature(void)
{

    return TEMP / 10;

}

#endif

