#ifndef HMXCOMMON_H
#define HMXCOMMON_H

//#include <QDebug>

// SPI Transfer
#if 0
//sync(1 byte)
#define SPI_SYNC_CMD       0x5A
#endif
//sync(2 byte)
#define SPI_SYNC_CMD1      0xC0
#define SPI_SYNC_CMD2      0x5A

#define SPI_SYNC_DATA      0xA3

//#define SPI_CMD_WRITE       0x00
//#define SPI_CMD_READ        0x01
//#define SPI_CMD_REQUEST     0x02
#if 0
//sync(1 byte)+command(1 byte)+size(2 bytes)
#define SPI_SYNC_POS	0x00
#define SPI_SYNC_SIZE	0x01

#define SPI_CMD_POS		0x01
#define SPI_CMD_SIZE	0x01

#define SPI_SIZE_POS	0x02
#define SPI_SIZE_SIZE	0x02

#define DATA_TYPE_POS	0x00
#define DATA_TYPE_SIZE	0x01
#define DATA_RAW_POS	DATA_TYPE_POS+DATA_TYPE_SIZE

#define DATA_CRC_SIZE	0x01

#define SPI_HEADER		SPI_SYNC_SIZE+SPI_CMD_SIZE+SPI_SIZE_SIZE

#define SPI_STA_POS		0x02
#define SPI_STA_SIZE	0x01
#endif
//sync(2 byte)+command(1 byte)+size(2 bytes)
#define SPI_SYNC_POS	0x00
#define SPI_SYNC_SIZE	0x02

#define SPI_CMD_POS		SPI_SYNC_POS+SPI_SYNC_SIZE
#define SPI_CMD_SIZE	0x01

#define SPI_SIZE_POS	SPI_CMD_POS+SPI_CMD_SIZE
#define SPI_SIZE_SIZE	0x02

#define DATA_TYPE_POS	SPI_SIZE_POS+SPI_SIZE_SIZE
#define DATA_TYPE_SIZE	0x01

#define DATA_RAW_POS	DATA_TYPE_POS+DATA_TYPE_SIZE

#define DATA_CRC_SIZE	0x01

#define SPI_HEADER		SPI_SYNC_SIZE+SPI_CMD_SIZE+SPI_SIZE_SIZE

#define SPI_STA_POS		0x02
#define SPI_STA_SIZE	0x01

///////
#define SPI_SYNC_POS_V2     0x0
#define SPI_SYNC_SIZE_V2	0x02

#define SPI_DATA_TYPE_POS_V2 SPI_SYNC_POS_V2+SPI_SYNC_SIZE_V2
#define SPI_DATA_TYPE_SIZE_V2 0x1

#define SPI_LENGTH_POS_V2	SPI_DATA_TYPE_POS_V2+SPI_DATA_TYPE_SIZE_V2
#define SPI_LENGTH_SIZE_V2	0x04

#define SPI_HEADER_V2		SPI_LENGTH_POS_V2+SPI_LENGTH_SIZE_V2


//
#define SPI_SYNC_DATA_SIZE	0x01

/** SPI transfer Type **/
/*typedef enum 
{
    SPI_CMD_WRITE   = 0x00, 
	SPI_CMD_READ    = 0x01,  
	SPI_CMD_REQUEST = 0x02,     
}SpiTransferType_t;*/


/** SPI data Type **/
/*typedef enum 
{
    SPI_DATA_JPG   = 0x00, 
	SPI_DATA_BMP   = 0x01, 
	SPI_DATA_DBG   = 0x10, 
	SPI_DATA_WARN  = 0x11, 
	SPI_DATA_CTRL  = 0x12, 
	SPI_DATA_BIN   = 0x20, 
}SpiDataType_t;*/

typedef enum 
{
	RECV_INIT_STATE				= 1,        
	RECV_DATA_PACKET_STATE		= 2,
	RECV_RESULT_STATE			= 3,
	//ACK_STATE				= 4,
}ReceiveSTATE_t;

typedef enum 
{
	PREPARE_STATE				= 0,
	SEND_GPIO_STATE 			= 1,        
	SEND_READSIZE_STATE			= 2,
	SEND_INIT_STATE				= 3,
	SEND_DATA_PACKET_STATE		= 4,
	SEND_RESULT_STATE			= 5,
	SEND_RE_DATA_PACKET_STATE	= 6,
}SendSTATE_t;

typedef enum 
{
	W_CMD = 0x00,              /**< indicate a write command, WE-I send data to PC */ 
	R_CMD = 0x01,              /**< indicate a read command, WE-I request for data from PC */ 
	Size_CMD = 0x02,           /**< indicate a request command, to query actual data size */ 
	ACK_CMD = 0x03,
	STA_CMD = 0x04,
}CommandType_t;

typedef enum 
{
	RESULT_PASS			= 0x00,  
	RESULT_INCORRECT	= 0x01,
}StatusCmdResult_t;


#if 0
#define dbg_printf(f_, ...) printf(f_, ##__VA_ARGS__)
#else
#ifdef FT4222_DBG_MSG
#define dbg_printf(f_, ...) qDebug(f_, ##__VA_ARGS__)
#else
#define dbg_printf(f_, ...)
#endif
#endif

#define RESULT_BUFFER_SIZE			3


#define START_FRAME_BIT 0x80
#define END_FRAME_BIT	0x40

#define RAW_DATA_MAX 65000
#define SALVE_RAW_DATA_MAX 10

#define NONBLOCKING_SPIS 1

#endif 
