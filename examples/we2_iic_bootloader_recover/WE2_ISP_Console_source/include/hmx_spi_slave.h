#ifndef HMXSPISLAVE_H
#define HMXSPISLAVE_H

#include "hmx_lib.h"

bool DRV_spi_slave_init(void *handle, void *gpio_handle);
bool DRV_spi_slave_receive_data(void *handle, CB_FUNC_SPI_T rx_cb, unsigned int *data_size, DataType_t *data_type);
bool DRV_spi_slave_halt(bool force);
bool DRV_spi_slave_read_packet(unsigned char *data);

#endif