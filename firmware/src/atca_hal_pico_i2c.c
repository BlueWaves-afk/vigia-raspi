/**
 * cryptoauthlib I2C HAL for Raspberry Pi Pico 2 (RP2350).
 * Bus 1 = GP2/GP3, matching atecc608a_driver.c.
 */

#include "cryptoauthlib.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <string.h>

typedef struct {
    int ref_ct;
    i2c_inst_t *i2c;
} atca_pico_i2c_hal_t;

static atca_pico_i2c_hal_t g_i2c_hal[2];

static i2c_inst_t *bus_to_i2c(uint8_t bus)
{
    return (bus == 0) ? i2c0 : i2c1;
}

static uint8_t cfg_device_address(ATCAIfaceCfg *cfg)
{
#ifdef ATCA_ENABLE_DEPRECATED
    return cfg->atcai2c.slave_address;
#else
    return cfg->atcai2c.address;
#endif
}

void hal_delay_ms(uint32_t delay)
{
    sleep_ms(delay);
}

void hal_delay_us(uint32_t delay)
{
    sleep_us(delay);
}

ATCA_STATUS hal_i2c_init(ATCAIface iface, ATCAIfaceCfg *cfg)
{
    if (!iface || !cfg)
        return ATCA_BAD_PARAM;

    const uint8_t bus = cfg->atcai2c.bus;
    if (bus >= 2)
        return ATCA_BAD_PARAM;

    atca_pico_i2c_hal_t *hal = &g_i2c_hal[bus];
    if (hal->ref_ct == 0) {
        hal->ref_ct = 1;
        hal->i2c = bus_to_i2c(bus);
    } else {
        hal->ref_ct++;
    }

    iface->hal_data = hal;
    return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_post_init(ATCAIface iface)
{
    (void)iface;
    return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_send(ATCAIface iface, uint8_t word_address,
                         uint8_t *txdata, int txlength)
{
    if (!iface || !iface->mIfaceCFG)
        return ATCA_BAD_PARAM;

    atca_pico_i2c_hal_t *hal = (atca_pico_i2c_hal_t *)iface->hal_data;
    if (!hal || !hal->i2c)
        return ATCA_NOT_INITIALIZED;

    const uint8_t dev_addr_7 = cfg_device_address(iface->mIfaceCFG) >> 1;
    uint8_t buf[256];
    int total = 1;

    buf[0] = word_address;
    if (txdata && txlength > 0) {
        if (txlength > (int)sizeof(buf) - 1)
            return ATCA_BAD_PARAM;
        memcpy(&buf[1], txdata, (size_t)txlength);
        total = txlength + 1;
    }

    const int rc = i2c_write_blocking(hal->i2c, dev_addr_7, buf, (size_t)total, false);
    return (rc == total) ? ATCA_SUCCESS : ATCA_COMM_FAIL;
}

ATCA_STATUS hal_i2c_receive(ATCAIface iface, uint8_t word_address,
                            uint8_t *rxdata, uint16_t *rxlength)
{
    if (!iface || !iface->mIfaceCFG || !rxdata || !rxlength)
        return ATCA_BAD_PARAM;

    atca_pico_i2c_hal_t *hal = (atca_pico_i2c_hal_t *)iface->hal_data;
    if (!hal || !hal->i2c)
        return ATCA_NOT_INITIALIZED;

    const uint8_t dev_addr_7 = (word_address != 0xFFu)
        ? (word_address >> 1)
        : (cfg_device_address(iface->mIfaceCFG) >> 1);

    const int rc = i2c_read_blocking(hal->i2c, dev_addr_7, rxdata, *rxlength, false);
    if (rc != (int)*rxlength)
        return ATCA_COMM_FAIL;

    return ATCA_SUCCESS;
}

ATCA_STATUS hal_i2c_control(ATCAIface iface, uint8_t option, void *param, size_t paramlen)
{
    (void)iface;
    (void)option;
    (void)param;
    (void)paramlen;
    return ATCA_UNIMPLEMENTED;
}

ATCA_STATUS hal_i2c_release(void *hal_data)
{
    atca_pico_i2c_hal_t *hal = (atca_pico_i2c_hal_t *)hal_data;
    if (!hal)
        return ATCA_BAD_PARAM;

    if (hal->ref_ct > 0)
        hal->ref_ct--;

    return ATCA_SUCCESS;
}
