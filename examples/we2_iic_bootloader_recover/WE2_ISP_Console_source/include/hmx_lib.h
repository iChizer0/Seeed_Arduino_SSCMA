#ifndef HMXLIB_H
#define HMXLIB_H
#include "LibFT4222.h"

#ifdef __cplusplus
 #define __EXTERN_C  extern "C"
#else
 #define __EXTERN_C
#endif

 #define HMXLIB_API __declspec(dllexport)
 #define HMXLIB_C_API __EXTERN_C __declspec(dllexport)

//typedef void *HANDLE;
#ifndef __cplusplus
typedef unsigned int bool;
#endif

//typedef void *HANDLE;

#ifndef UINT8
#define UINT8 unsigned char 
#endif
#ifndef UINT32
#define UINT32 unsigned int 
#endif
typedef void (*CB_FUNC_SPI_T) (unsigned int data);
/** Status Type **/
typedef enum 
{
	ret_PASS = 0,                 /**< Pass status */ 
	ret_FAIL = -1,                /**< Fail status */ 
	ret_NoDev = -2,               /**< Status - unable to find device */ 
	ret_NotFinishRecv = -3,  		  /**< Status - SPI receive progress still running */ 
	ret_NotFinishSend = -4,
	ret_ErrParameter = -5,
	ret_NoSupportFlash = -6,
}ErrType_t;


typedef enum 
{
    DATA_TYPE_JPG = 0x01,
    DATA_TYPE_BMP = 0x02,
    DATA_TYPE_VO_CMD = 0x03,
    DATA_TYPE_HUNN_PRESENT = 0x04,
    DATA_TYPE_WKUP_METHOD = 0x05,
    DATA_TYPE_HUMN_LV_TIMEOUT = 0x06,
    DATA_TYPE_LANGUAGE = 0x07,
    #ifdef TV01
    DATA_TYPE_HUNN_LOCK_COUNT = 0x08,
    DATA_TYPE_EYE_PROT_TIMEOUT = 0x09,
    #endif
    DATA_TYPE_VOICE_TIMEOUT = 0x0A,
    DATA_TYPE_DBG_MSG = 0x10,
    DATA_TYPE_WARN_MSG = 0x11,
    DATA_TYPE_CTRL_MSG = 0x12,
    DATA_TYPE_META_DATA = 0x13,
    DATA_TYPE_JPG_L = 0x14,
    DATA_TYPE_JPG_R = 0x15,
    DATA_TYPE_RAW_IMG = 0x16,
    DATA_TYPE_BIN_DATA = 0x20,
    DATA_TYPE_INCORRECT_DATA = 0x3F,
    DATA_TYPE_TV_STANDBY = 0x40,

    DATA_TYPE_AREA_SUMMARY = 0x5A,
    DATA_TYPE_AREA_DETAIL = 0x5C,

    DATA_TYPE_QRCODE_RESULT = 0x60,
    DATA_TYPE_WR_CONFIG = 0x61,
    DATA_TYPE_SENSOR_REG = 0x62,

    DATA_TYPE_PDM = 0x90,
    DATA_TYPE_ALANGO = 0x91,

    DATA_TYPE_RAW_HEADER_IMG_MONO   = 0x70,
    DATA_TYPE_RAW_HEADER_IMG_BGR    = 0x71,
    DATA_TYPE_RAW_HEADER_IMG_YUV422_8U3C = 0x72,
    DATA_TYPE_RAW_HEADER_IMG_YUV420_8U3C = 0x73,
    DATA_TYPE_RAW_HEADER_IMG_BAYER = 0x74,
    DATA_TYPE_RAW_HEADER_IMG_YUYV = 0x75,
    DATA_TYPE_RAW_HEADER_IMG_RGB565 = 0x76,
    DATA_TYPE_RAW_HEADER_IMG_RGB555 = 0x77,
    DATA_TYPE_RAW_HEADER_IMG_RGB444 = 0x78,
    DATA_TYPE_RAW_HEADER_IMG_GBR    = 0x79,
    DATA_TYPE_RAW_HEADER_IMG_YVYU = 0x7A,
    DATA_TYPE_RAW_HEADER_IMG_UYVY = 0x7B,
    DATA_TYPE_RAW_HEADER_IMG_VYUY = 0x7C,
    DATA_TYPE_RAW_HEADER_IMG_BGR565 = 0x7D,
    DATA_TYPE_RAW_HEADER_IMG_BGR555 = 0x7E,
    DATA_TYPE_RAW_HEADER_IMG_BGR444 = 0x7F,
    DATA_TYPE_RAW_HEADER_IMG_RGB_1PIX  = 0x80,
    DATA_TYPE_RAW_HEADER_IMG_BGR_1PIX = 0x81,
}DataType_t;
typedef enum {
	LOAD_PARA_SPI_TYPE_JPG               = 0x01,
	LOAD_PARA_SPI_TYPE_RAW               = 0x02,
	LOAD_PARA_SPI_TYPE_META_DATA         = 0x03,
}DataType_load_para_t;

typedef enum 
{
	Mode_Single		= 0x01,        /**< transfer mode - single */ 
	Mode_Dual		= 0x02,        /**< transfer mode - dual */ 
	Mode_Quad		= 0x03,        /**< transfer mode - quad */ 
}FlashMode_t;

typedef enum 
{
	MasterClock_60M	=0x0,        
	MasterClock_30M	,        
	MasterClock_15M		,        
	MasterClock_7500K		,        
	MasterClock_3750K		,        

	MasterClock_48M	=0x10,        
	MasterClock_24M	,        
	MasterClock_12M	,       
	MasterClock_6M		,        
	MasterClock_3M		,        
	MasterClock_1500K	,        
	MasterClock_750K		,        
	MasterClock_375K		,        
	
	MasterClock_80M		=0x30,        
	MasterClock_40M		,        
	MasterClock_20M		,        
	MasterClock_10M		,        
	MasterClock_5M		,        
	MasterClock_2500K		,        
	MasterClock_1250K		,        
	MasterClock_625K	 ,       
}SPIMasterClock_t;


/****************************************************
 * Functions Declaration                            *
 ***************************************************/

/**
  * @brief  Function used to initialize the library. 
  *
  * @param  None
  *
  * @return Return the error status of initial, return ret_PASS when success. return ret_NoDev if unable to find device
  */
HMXLIB_C_API ErrType_t init_lib();

/**
  * @brief  Function used to de-initialize the library. 
  *
  * @param  None
  *
  * @return Return the error status of initial, return ret_PASS when success. 
  */
HMXLIB_C_API ErrType_t de_init_lib();

/**
  * @brief  Function used to get the library version. 
  *
  * @param  major	major ID pointer, the value will return 0 ~ 0xFF
  *
  * @param  minor	minor ID pointer, the value will return 0 ~ 0xFF
  *
  * @return Return the error status of initial, return ret_PASS when success. 
  */
HMXLIB_C_API ErrType_t get_lib_version(unsigned char *major, unsigned char *minor);


/**
  * @brief  Function used to initialize SPI in slave mode. 
  *
  * @param  None
  *
  * @return Return the error status of initial, return ret_PASS when success. return ret_FAIL if unable to initial SPI
  */
HMXLIB_C_API ErrType_t spi_slave_init();

/**
  * @brief  Function used to initialize SPI in master mode. 
  *
  * @param  freq frequency to set
  *
  * @return Return the error status of initial, return ret_PASS when success. return ret_FAIL if unable to initial SPI
  */
HMXLIB_C_API ErrType_t spi_master_init(SPIMasterClock_t freq);


HMXLIB_C_API ErrType_t spi_flash_init(SPIMasterClock_t clcok);
/**
  * @brief  Function used to initialize I2C in master mode.
  *
  * @param  speed		i2c master speed, value in kbps. default set to 400.
  *
  * @return Return the error status of initial, return ret_PASS when success. return ret_FAIL if unable to initial I2C
  */
HMXLIB_C_API ErrType_t i2c_master_init(int speed);


/**
  * @brief  Function used to read I2C data 
  *
  * @param  id		slave ID, the value is between 0 ~ 0x7F
  *
  * @param  addr	address value, this is a 4 bytes address
  *
  * @return Return the error status of initial, return ret_PASS when success. return ret_FAIL if unable to initial I2C
  */
HMXLIB_C_API unsigned int i2c_single_read(unsigned char id, unsigned char *addr);

/**
  * @brief  Function used to write I2C data 
  *
  * @param  id		slave ID, the value is between 0 ~ 0x7F
  *
  * @param  addr	address value, this is a 4 bytes address
  *
  * @param  data	data to write, this is a 4 bytes data
  *
  * @return Return the error status of initial, return ret_PASS when success. return ret_FAIL if unable to initial I2C
  */
HMXLIB_C_API void i2c_single_write(unsigned char id, unsigned char *addr, unsigned char *data); 



HMXLIB_C_API void I2C_burst_read(UINT8 id, UINT8 *addr, UINT32 addr_size, UINT8 *data, UINT32 data_size);

HMXLIB_C_API void I2C_burst_write(UINT8 id, UINT8 *addr, UINT32 addr_size, UINT8 *data, UINT32 data_size);

HMXLIB_C_API void I2C_burst_read_no_addr(UINT8 id, UINT8 *data, UINT32 data_size);

HMXLIB_C_API void I2C_burst_write_no_addr(UINT8 id, UINT8 *data, UINT32 data_size);

HMXLIB_C_API void I2C_cmd_write(UINT8 id, UINT8 *data, UINT32 data_size);
HMXLIB_C_API void I2C_cmd_write_ex(UINT8 id, UINT8 *data, UINT32 data_size, UINT8 flag);
HMXLIB_C_API void I2C_cmd_write_hs(UINT8 id, UINT8 *data, UINT32 data_size);
HMXLIB_C_API void I2C_cmd_read(UINT8 id, UINT8 *data, UINT32 data_size);
HMXLIB_C_API void I2C_cmd_read_ex(UINT8 id, UINT8 *data, UINT32 data_size, UINT8 flag);
HMXLIB_C_API BOOL GPIO_Read(UINT8 pin_no);
HMXLIB_C_API void GPIO_Write(UINT8 pin_no, BOOL bValue);

/**
  * @brief  Function used to read SPI slave data. this is non-blocking function. callback function use to indicates receive is finish 
  *
  * @param  rx_cb		callback function when receive done
  *
  * @param  data_size	data size pointer, indicates how many data receive	
  *
  * @param  data_type	data type pointer, indicates what kind of data is it
  *
  * @return Return the error status of current progress, return ret_PASS when success. else return ret_FAIL 
  */
HMXLIB_C_API ErrType_t spi_slave_receive_data(CB_FUNC_SPI_T rx_cb, unsigned int *data_size, DataType_t *data_type);

/**
  * @brief  Function used to read SPI slave data to target memory location when receive operation is done. 
  *
  * @param  data		data array pointer, upper application should allocates memory with size get from function "spi_slave_receive_data"
  *
  * @return Return the error status of current progress, return ret_PASS when success. else return ret_FAIL 
  */
HMXLIB_C_API ErrType_t spi_slave_read_packet(unsigned char *data);

/**
  * @brief  Function used to stop SPI slave operation
  *
  * @param  force		1 will stop any un-saved data. 0 will wait operation is finish
  *
  * @return Return the error status of current progress, return ret_PASS when success. else return ret_FAIL 
  */
HMXLIB_C_API ErrType_t spi_slave_halt(bool force);     

/**
  * @brief  Function used to send data by SPI slave
  *
  * @param  cb		callback function when send done
  *
  * @param  data_size	data size, indicates how many data to send
  *
  * @param  data_type	data type, indicates what kind of data is it
  *
  * @param  data		data array pointer, indicates data location 
  *
  * @return Return the error status of current progress, return ret_PASS when success. else return ret_FAIL 
  */
HMXLIB_C_API ErrType_t spi_slave_send_data(CB_FUNC_SPI_T cb, unsigned int data_size, DataType_t data_type,  unsigned char *data);


HMXLIB_C_API ErrType_t spi_master_halt(bool force);
                                                       
HMXLIB_C_API ErrType_t spi_master_send_data(CB_FUNC_SPI_T cb, unsigned int data_size, unsigned char *data);

HMXLIB_C_API ErrType_t spi_master_send_simple_data(CB_FUNC_SPI_T cb, unsigned int data_size, unsigned char *data);

HMXLIB_C_API ErrType_t spi_master_recv_data(CB_FUNC_SPI_T cb, unsigned int *data_size);

HMXLIB_C_API ErrType_t spi_master_recv_simple_data(CB_FUNC_SPI_T cb, unsigned int *data_size, DataType_t *data_type);

HMXLIB_C_API ErrType_t spi_master_stop_recv_simple_data();

HMXLIB_C_API ErrType_t spi_master_read_packet(unsigned char *data);

HMXLIB_C_API ErrType_t spi_master_recv_ack(CB_FUNC_SPI_T cb, unsigned int *data_size);

HMXLIB_C_API ErrType_t spi_master_isp_enable_burstmode_inc4();

HMXLIB_C_API ErrType_t spi_master_isp_enable_burstmode_inc0();

HMXLIB_C_API ErrType_t spi_master_isp_reset_burstmode();

HMXLIB_C_API ErrType_t spi_master_isp_burst_write(unsigned int reg_addr, unsigned int data_size, unsigned char *data);

HMXLIB_C_API ErrType_t spi_master_isp_burst_read(unsigned int reg_addr, unsigned int data_size, unsigned char *data);

HMXLIB_C_API ErrType_t spi_master_isp_single_write(unsigned int reg_addr, unsigned int *data);

HMXLIB_C_API ErrType_t spi_master_isp_single_read(unsigned int reg_addr, unsigned int *data);

HMXLIB_C_API ErrType_t spi_master_isp_single_rw_init_exp();

HMXLIB_C_API ErrType_t spi_master_isp_single_write_exp(unsigned int reg_addr, unsigned int *data);

HMXLIB_C_API ErrType_t spi_master_isp_single_read_exp(unsigned int reg_addr, unsigned int *data);

HMXLIB_C_API ErrType_t spi_master_isp_1byte_write(unsigned char reg_addr, unsigned char data);

HMXLIB_C_API ErrType_t spi_master_isp_1byte_read(unsigned char reg_addr, unsigned char *data);

HMXLIB_C_API ErrType_t spi_flash_read_info(unsigned char *info);

HMXLIB_C_API ErrType_t spi_flash_change_mode(FlashMode_t mode);

HMXLIB_C_API ErrType_t spi_flash_read_data(unsigned int data_addr, unsigned int data_size, unsigned char *data);

HMXLIB_C_API ErrType_t spi_flash_write_data(unsigned int data_addr, unsigned int data_size, unsigned char *data);

HMXLIB_C_API unsigned short spi_flash_write_percent_check();

HMXLIB_C_API ErrType_t spi_flash_erase_data(unsigned int start_addr, unsigned int flash_size);

HMXLIB_C_API ErrType_t spi_flash_erase_all();

HMXLIB_C_API bool spi_flash_busy_check();

HMXLIB_C_API ErrType_t spi_flash_clear_protect(void);

HMXLIB_C_API  ErrType_t FT4222_SPIRESET(void);

HMXLIB_C_API  ErrType_t FT4222_CHIPRESET(void);

HMXLIB_C_API  ErrType_t FT4222_SPIRESETBUFFER(void);
#endif 
