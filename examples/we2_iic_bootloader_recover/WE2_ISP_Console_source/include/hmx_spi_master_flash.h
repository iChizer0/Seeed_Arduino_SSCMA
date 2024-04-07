#ifndef HMXSPIMASTERFLASH_H
#define HMXSPIMASTERFLASH_H
#include "hmx_lib.h"

typedef enum VENDOR_TYPE_S
{
    UNKNOWN_VENDOR = -1,
	SUPPORT_SINGLE_VENDOR = 0,
    VENDOR_MXIC = 1,
    VENDOR_WINBOND = 2,
} VENDOR_TYPE_T;

typedef struct {
	unsigned char vendor_name[256];
	unsigned char vendor_info;
	int			  vendor_type;
} SPI_FLASH_VENDOR_INFO_t;

#define DW_SPI_MXIC_FLASH_QUAD_ENABLE_MASK 0x40
#define DW_SPI_MXIC_FLASH_QUAD_ENABLE_POS 6
#define DW_SPI_WINBOND_FLASH_QUAD_ENABLE_MASK 0x02
#define DW_SPI_WINBOND_FLASH_QUAD_ENABLE_POS 1
#define DW_SPI_FLASH_CMD_WRITE_STATUS 0x01
#define DW_SPI_FLASH_CMD_WRITE_STATUS2 0x31

#define FLASH_SECTER_SIZE 0x1000


/*typedef enum{
	DW_SPI_FLASH_VENDOR_MXIC=0xc2,
	DW_SPI_FLASH_VENDOR_GIGADEV=0xc8,
	DW_SPI_FLASH_VENDOR_WINBOND=0xef,
}SPI_FLASH_VENDOR_INFO;*/


ErrType_t DRV_spi_flash_init(void *handle, SPIMasterClock_t clock);
ErrType_t DRV_spi_flash_read_info(void *handle, unsigned char *info);
bool DRV_spi_flash_set_quad_mode(void *handle, bool enable);
bool DRV_spi_flash_busy_check(void *handle);
ErrType_t DRV_spi_flash_waitWIP(void *handle);
ErrType_t DRV_spi_flash_write_enable(void *handle);
ErrType_t DRV_spi_flash_erase_data(void *handle, unsigned int start_addr, unsigned int data_size);
ErrType_t DRV_spi_flash_erase_all(void *handle);
ErrType_t DRV_spi_flash_read_data(void *handle, unsigned int data_addr, unsigned int data_size, unsigned char *data);
ErrType_t DRV_spi_flash_clear_protect(void *handle);
#endif