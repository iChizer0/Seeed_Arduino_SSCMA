// HMX_WEI_LIB.cpp :
//

#include "hmx_lib.h"
#include "hmx_common.h"
#include "hmx_spi_master_flash.h"
#include "hmx_spi_slave.h"
#include "common.h"

#include <vector>
#include <string>

#include "ftd2xx.h"
#include "LibFT4222.h"

#include <process.h>


//#define dbg_printf(f_, ...) printf((f_), __VA_ARGS__)

using namespace std;

#define VERSION_MAJOR 1
#define VERSION_MINOR 6

#define SAVELOG 1




//#define DW_SPI_FLASH_VENDOR_MXIC 0xc2
//#define DW_SPI_FLASH_VENDOR_WINBOND 0xef


#define MAX_SPI_CLOCK 80000000
#define MIN_SPI_CLOCK 46875
const int sys_clk_rate[4] = {60000000, 24000000, 48000000, 80000000};



union sizeUnion {
    uint32 sizeInt;
    struct
    {
        uint8 b1;
        uint8 b2;
        uint8 b3;
        uint8 b4; // or number them in reverse order
    } myBytes;
};

union sizeShortUnion {
    uint16 sizeShort;
    struct
    {
        uint8 b1;
        uint8 b2;
    } myBytes;
};

typedef struct spi_info_struct
{
    FT_HANDLE ftHandle;
    FT_HANDLE ftGPIOHandle;
    vector<unsigned char> recvBuf;
    vector<unsigned char> dataBuf; // data need to be parsed
    vector<unsigned char> rawData;
    vector<unsigned char> sizeBuf; // 4 bytes of data includes data size info
    CB_FUNC_SPI_T rx_cb;
    unsigned int *datasize;
    DataType_t *datatype;
    //unsigned char *data;
} spi_info_t, *spi_info_pt;

typedef struct spi_flash_info_struct
{
    FT_HANDLE ftHandle;
    vector<unsigned char> rawData;
    unsigned int addr;
    unsigned int data_size;
} spi_flash_info_t, *spi_flash_info_pt;

vector<FT_DEVICE_LIST_INFO_NODE> g_FTAllDevList;
vector<FT_DEVICE_LIST_INFO_NODE> g_FT4222DevList;

static FT_HANDLE ftdevHandle = NULL;
FT_HANDLE ftdevGPIOHandle = NULL;
//FT_STATUS ftdevStatus;

//SPI master, Flash use parameter
unsigned char g_flash_read_mode = Mode_Single;
unsigned short flash_write_percent;
spi_flash_info_t f_info;
// -1 : not flash found/0 : MXIC found/1 : winbond found
int g_cur_flash_type = UNKNOWN_VENDOR;

GPIO_Dir gpio_group[4] = {GPIO_OUTPUT, GPIO_OUTPUT, GPIO_INPUT, GPIO_OUTPUT};
UINT8 write_packet[16], read_packet[16], read_ack_packet[2], write_ack_packet[2];

HANDLE hth_spi_receive;
HANDLE hth_spi_send;
HANDLE hth_spi_flash;

unsigned int thID_spi_receive;
unsigned int thID_spi_receive_simple;
//vector<unsigned char> spi_recvBuf;
//vector<unsigned char> spi_dataBuf;
//vector<unsigned char> spi_completePacket;

unsigned int thID_spi_send;
unsigned int thID_spi_flash;

bool spi_start_receive_flag = false;
bool spi_start_send_flag = false;
//bool spi_finish_receive_flag = false;

spi_info_t spi_receive_info, spi_send_info;

DataType_t send_type;
unsigned int send_size;


#ifdef SAVELOG
FILE *pFile;
#endif

inline std::string DeviceFlagToString(DWORD flags)
{
    std::string msg;
    msg += (flags & 0x1)? "DEVICE_OPEN" : "DEVICE_CLOSED";
    msg += ", ";
    msg += (flags & 0x2)? "High-speed USB" : "Full-speed USB";
    return msg;
}

void ListFtUsbDevices()
{
    FT_STATUS ftStatus = 0;

    DWORD numOfDevices = 0;
    ftStatus = FT_CreateDeviceInfoList(&numOfDevices);

    for (DWORD iDev = 0; iDev < numOfDevices; ++iDev)
    {
        FT_DEVICE_LIST_INFO_NODE devInfo;
        memset(&devInfo, 0, sizeof(devInfo));

        ftStatus = FT_GetDeviceInfoDetail(iDev, &devInfo.Flags, &devInfo.Type, &devInfo.ID, &devInfo.LocId,
                                          devInfo.SerialNumber,
                                          devInfo.Description,
                                          &devInfo.ftHandle);

        if (FT_OK == ftStatus)
        {
            dbg_printf("Dev %d:", (int)iDev);
            dbg_printf(" Flags= 0x%x, (%s)", (unsigned int)devInfo.Flags,
                       DeviceFlagToString(devInfo.Flags).c_str());
            dbg_printf(" Type= 0x%x", (unsigned int)devInfo.Type);
            dbg_printf(" ID= 0x%x", (unsigned int)devInfo.ID);
            dbg_printf(" LocId= 0x%x", (unsigned int)devInfo.LocId);
            dbg_printf(" SerialNumber= %s", devInfo.SerialNumber);
            dbg_printf(" Description= %s", devInfo.Description);
            dbg_printf(" ftHandle= 0x%x", (unsigned int)devInfo.ftHandle);


            const std::string desc = devInfo.Description;
            g_FTAllDevList.push_back(devInfo);

            if (desc == "FT4222" || desc == "FT4222 A")
            {
                g_FT4222DevList.push_back(devInfo);
            }
        }
    }
}

ErrType_t get_lib_version(unsigned char *major, unsigned char *minor)
{
    if (major == nullptr)
        return ret_FAIL;

    if (minor == nullptr)
        return ret_FAIL;

    *major = VERSION_MAJOR;
    *minor = VERSION_MINOR;

    return ret_PASS;
}

ErrType_t init_lib()
{
    FT_STATUS ftStatus;

	g_FTAllDevList.clear();
	g_FT4222DevList.clear();

    ListFtUsbDevices();

    if (g_FT4222DevList.empty())
    {
        dbg_printf("No FT4222 device is found!\n");
        return ret_NoDev;
    }

    //for I2C or SPI control.
    //remember I2C,  SPI cannot control at the same time
    //need to re-initial the peripheral if switch between them

    //ftStatus = FT_Open(0, &ftdevHandle);
    ftStatus = FT_OpenEx((PVOID)"FT4222 A", FT_OPEN_BY_DESCRIPTION, &ftdevHandle);

    if (FT_OK != ftStatus)
    {
        dbg_printf("Open a FT4222 device failed(ftdevHandle)!\n");
        return ret_FAIL;
    }

    //for GPIO control
    ftStatus = FT_OpenEx((PVOID)"FT4222 B", FT_OPEN_BY_DESCRIPTION, &ftdevGPIOHandle);

    if (FT_OK != ftStatus)
    {
        dbg_printf("Open a FT4222 device failed(ftdevGPIOHandle)!\n");
        return ret_FAIL;
    }

    //initial data here
    write_packet[0] = 0x00;
    write_packet[2] = 0x01;
    write_packet[4] = 0x02;
    write_packet[6] = 0x03;

    write_packet[8] = 0x04;
    write_packet[10] = 0x05;
    write_packet[12] = 0x06;
    write_packet[14] = 0x07;
    //read_packet[16]
    read_packet[0] = 0x00;
    read_packet[2] = 0x01;
    read_packet[4] = 0x02;
    read_packet[6] = 0x03;

    read_packet[8] = 0x08;
    read_packet[10] = 0x09;
    read_packet[12] = 0x0A;
    read_packet[14] = 0x0B;

    write_ack_packet[0] = 0x0C;
    write_ack_packet[1] = 0x01;

    read_ack_packet[0] = 0x0C;
    read_ack_packet[1] = 0x00;

    spi_start_receive_flag = false;
    //spi_finish_receive_flag = false;

    return ret_PASS;
}

ErrType_t de_init_lib()
{
	FT_STATUS ftStatus;

    ftStatus = FT4222_UnInitialize(ftdevHandle);

	if (FT_OK != ftStatus)
    {
        dbg_printf("FT4222_UnInitialize failed!\n");
        return ret_FAIL;
    }

	ftStatus = FT_Close(ftdevHandle);

	if (FT_OK != ftStatus)
    {
        dbg_printf("FT_Close failed!\n");
        return ret_FAIL;
    }


    ftStatus = FT4222_UnInitialize(ftdevGPIOHandle);

	/*if (FT_OK != ftStatus)
    {
        dbg_printf("FT4222_UnInitialize failed!\n");
        return ret_FAIL;
    }*/

	ftStatus = FT_Close(ftdevGPIOHandle); 

	if (FT_OK != ftStatus)
    {
        dbg_printf("FT_Close failed!\n");
        return ret_FAIL;
    }

    return ret_PASS;
}

ErrType_t spi_master_init(SPIMasterClock_t freq)
{
    FT_STATUS ftStatus;

	FT4222_ClockRate clk = (FT4222_ClockRate)((freq & 0xF0) >> 4);
    FT4222_SPIClock div = (FT4222_SPIClock)(freq & 0xF);

    dbg_printf("Init FT4222 SPI MASTER CLK: 0x%08X!!!\n", freq);

    if (ftdevHandle != NULL)
    {

        ftStatus = FT4222_SetClock(ftdevHandle, clk);
        if (FT_OK != ftStatus)
        {
            dbg_printf("Set FT4222 clock failed!\n");
            return ret_FAIL;
        }

        //set SPI master to 1.5Mhz
        ftStatus = FT4222_SPIMaster_Init(ftdevHandle, SPI_IO_SINGLE, div, CLK_IDLE_LOW, CLK_LEADING, 0x01);
        if (FT4222_OK != ftStatus)
        {
            dbg_printf("Init FT4222 as SPI master device failed!\n");
            return ret_FAIL;
        }

        ftStatus = FT4222_SPI_SetDrivingStrength(ftdevHandle, DS_8MA, DS_8MA, DS_8MA);
        if (FT_OK != ftStatus)
        {
            dbg_printf("Set SPI Slave driving strength failed!\n");
            return ret_FAIL;
        }

        ftStatus = FT_SetUSBParameters(ftdevHandle, 4 * 1024, 0);
        if (FT_OK != ftStatus)
        {
            dbg_printf("FT_SetUSBParameters failed!\n");
            return ret_FAIL;
        }

        //need also to init GPIO-3 to output
        ftStatus = FT4222_GPIO_Init(ftdevGPIOHandle, gpio_group);
        if (FT_OK != ftStatus)
        {
            dbg_printf("set GPIO failed!\n");
            return ret_FAIL;
        }
        //disable suspend out , enable gpio 2
        ftStatus = FT4222_SetSuspendOut(ftdevGPIOHandle, 0);
        if (FT_OK != ftStatus)
        {
            dbg_printf("set GPIO failed!\n");
            return ret_FAIL;
        }
        //disable interrupt , enable gpio 3
        ftStatus = FT4222_SetWakeUpInterrupt(ftdevGPIOHandle, 0);
        if (FT_OK != ftStatus)
        {
            dbg_printf("set GPIO failed!\n");
            return ret_FAIL;
        }

        //set GPIO3 to low
        ftStatus = FT4222_GPIO_Write(ftdevGPIOHandle, GPIO_PORT3, 0);
        if (FT_OK != ftStatus)
        {
            dbg_printf("set GPIO failed!\n");
            return ret_FAIL;
        }

		ftStatus = FT4222_GPIO_Write(ftdevGPIOHandle, GPIO_PORT0, 1);
        if (FT_OK != ftStatus)
        {
            dbg_printf("set GPIO failed!\n");
            return ret_FAIL;
        }

		
		ftStatus = FT4222_GPIO_Write(ftdevGPIOHandle, GPIO_PORT1, 1);
        if (FT_OK != ftStatus)
        {
            dbg_printf("set GPIO failed!\n");
            return ret_FAIL;
        }

        return ret_PASS;
    }
    return ret_FAIL;
}

ErrType_t spi_slave_init()
{
    if(DRV_spi_slave_init(ftdevHandle,ftdevGPIOHandle)!= true)
	{
		return ret_FAIL;
	}

    return ret_PASS;
}


#if 0
// 2 sync bytes scenario
unsigned __stdcall threadSpiReceive_Write_CMD(void *arglist)
{
    uint16 sizeTransferred = 0;
    uint16 rxSize, raw_data_sz = 0;

    //sizeShortUnion write_size;
    sizeUnion write_size_v2;
    bool header_flag = false;

    //Arg *p = (Arg *)arglist;
    spi_info_t *p = (spi_info_t *)arglist;

    //FT4222_SPISlave_Write(p->ftHandle, test_buffer, RESULT_BUFFER_SIZE, &sizeTransferred);

    while (spi_start_receive_flag)
    {
        //
        while (spi_start_receive_flag && (FT4222_SPISlave_GetRxStatus(p->ftHandle /*ftdevHandle*/, &rxSize) == FT_OK))
        {
			if (rxSize > 0)
            {
                p->recvBuf.resize(rxSize);

                if (FT4222_SPISlave_Read(p->ftHandle, &(p->recvBuf[0]), rxSize, &sizeTransferred) == FT_OK)
                {
                    if (sizeTransferred != rxSize)
                    {
                        //printf("read data size is not equal\n");
                    }
                    else
                    {
                        // append data to dataBuf
                        p->dataBuf.insert(p->dataBuf.end(), p->recvBuf.begin(), p->recvBuf.end());
                    }
                }
                else
                {
                    //printf("FT4222_SPISlave_Read error\n");
                }
            }

		}
            
		while(spi_start_receive_flag && p->dataBuf.size()>=SPI_HEADER_V2 && header_flag == false)
		{
			//check if any sync byte inside
			while(p->dataBuf.size())
			{
				if(p->dataBuf[0] != SPI_SYNC_CMD1)
					p->dataBuf.erase(p->dataBuf.begin());
				else if(p->dataBuf.size() >=2 && p->dataBuf[1] != SPI_SYNC_CMD2)
					p->dataBuf.erase(p->dataBuf.begin());
			}
		}
            

    }

    //spi_finish_receive_flag = true;

    return 1;
}
#endif
#if 0
// 2 sync bytes scenario
unsigned __stdcall threadSpiReceive_Write_CMD(void *arglist)
{
    uint16 sizeTransferred = 0;
    uint16 rxSize, raw_data_sz = 0;

    ReceiveSTATE_t cur_state = RECV_INIT_STATE;
    //sizeShortUnion write_size;
    sizeUnion write_size_v2;
    bool receive_checksum_flag = false;
    bool start_of_frame = false;
    bool end_of_frame = false;

    //Arg *p = (Arg *)arglist;
    spi_info_t *p = (spi_info_t *)arglist;

    //FT4222_SPISlave_Write(p->ftHandle, test_buffer, RESULT_BUFFER_SIZE, &sizeTransferred);

    while (spi_start_receive_flag)
    {
        //
        if (FT4222_SPISlave_GetRxStatus(p->ftHandle /*ftdevHandle*/, &rxSize) == FT_OK)
        {
            if (rxSize > 0)
            {
                p->recvBuf.resize(rxSize);

                if (FT4222_SPISlave_Read(p->ftHandle, &(p->recvBuf[0]), rxSize, &sizeTransferred) == FT_OK)
                {
                    if (sizeTransferred != rxSize)
                    {
                        //printf("read data size is not equal\n");
                    }
                    else
                    {
                        // append data to dataBuf
                        p->dataBuf.insert(p->dataBuf.end(), p->recvBuf.begin(), p->recvBuf.end());
                    }
                }
                else
                {
                    //printf("FT4222_SPISlave_Read error\n");
                }
            }

            if (p->dataBuf.size() > 0)
            {
                if (cur_state == RECV_INIT_STATE)
                {
                    // skip all non "sync word " at the first byte.
                    while (p->dataBuf.size() > 0)
                    {
                        if (p->dataBuf.size() == 1)
                        {
                            if (p->dataBuf[SPI_SYNC_POS] == SPI_SYNC_CMD1)
                            {
                                //receive_time_flag = false;
                                break;
                            }
                        }
                        else // size > 1
                        {
                            if (p->dataBuf[SPI_SYNC_POS] == SPI_SYNC_CMD1)
                            {
                                if (p->dataBuf[SPI_SYNC_POS + 1] == SPI_SYNC_CMD2)
                                    break;
                            }
                        }
                        p->dataBuf.erase(p->dataBuf.begin());
                    }

                    if (p->dataBuf.size() >= SPI_HEADER_V2)
                    {
                        write_size_v2.myBytes.b1 = p->dataBuf[SPI_LENGTH_POS_V2];
                        write_size_v2.myBytes.b2 = p->dataBuf[SPI_LENGTH_POS_V2 + 1];
                        write_size_v2.myBytes.b3 = p->dataBuf[SPI_LENGTH_POS_V2 + 2];
                        write_size_v2.myBytes.b4 = p->dataBuf[SPI_LENGTH_POS_V2 + 3];

                        cur_state = RECV_DATA_PACKET_STATE;
                        p->dataBuf.erase(p->dataBuf.begin(), p->dataBuf.begin() + SPI_HEADER_V2);
                    }
                }

                if (cur_state == RECV_DATA_PACKET_STATE)
                {
                    if (p->dataBuf.size() >= write_size_v2.sizeInt)
                    {

                        p->rawData.resize(write_size_v2.sizeInt);
                        //copy data from start of complete packet
                        memcpy(&(p->rawData[0]), &(p->dataBuf[0]), (write_size_v2.sizeInt));

                        *(p->datasize) = p->rawData.size();

                        p->rx_cb(0);

                        cur_state = RECV_INIT_STATE;
                        p->dataBuf.erase(p->dataBuf.begin(), p->dataBuf.begin() + write_size_v2.sizeInt);
                    }
                }

                ////////////////
            }
        }
    }

    //spi_finish_receive_flag = true;

    return 1;
}
#endif

void make_size_buf(uint16 size, vector<unsigned char> *buf)
{
#if 1
    sizeShortUnion temp_size;

    buf->resize(4);
    buf->at(0) = 0x5A;

    temp_size.sizeShort = size;

    buf->at(1) = temp_size.myBytes.b1;
    buf->at(2) = temp_size.myBytes.b2;

    //checksum value
    buf->at(3) = 0xFF;
#else
    sizeUnion temp_size;

    buf->resize(6);
    buf->at(0) = 0xC0;
    buf->at(1) = 0x5A;

    temp_size.sizeInt = size;

    buf->at(2) = temp_size.myBytes.b1;
    buf->at(3) = temp_size.myBytes.b2;
    buf->at(4) = temp_size.myBytes.b3;
    buf->at(5) = temp_size.myBytes.b4;
#endif
}

void make_data_buf_checksum(uint16 size, vector<unsigned char> *buf, bool sof, bool eof, DataType_t type)
{
    unsigned char checksum = 0xFF;
    unsigned char type_char = 0;
    //count checksum with size data

    //
    if (sof)
        type_char |= START_FRAME_BIT;

    if (eof)
        type_char |= END_FRAME_BIT;

    type_char |= (unsigned char)type;

    //appen buf at end of size
    //        || data ..........()....................................||
    //  insert checksum at size
    buf->insert(buf->begin() + size, checksum);
    buf->insert(buf->begin(), type_char);
    buf->insert(buf->begin(), SPI_SYNC_DATA);
}

//sample slave send file thread
unsigned __stdcall threadSpiReceive_Request_CMD(void *arglist)
{
    uint16 sizeTransferred = 0;
    uint16 txSize = 0;
    unsigned int send_packet_size = 0, cur_pos = 0;
    SendSTATE_t cur_state = SEND_GPIO_STATE;
    //SendSTATE_t cur_state = PREPARE_STATE;
//    sizeShortUnion read_size;
    sizeUnion temp_size;
//    bool end_of_frame = false;

    spi_info_t *p = (spi_info_t *)arglist;

//    start_of_frame = true;
    //check if data size over max transfer size
    //while(spi_start_send_flag)
    {
        /*if(*p->datasize >RAW_DATA_MAX)
			txSize = RAW_DATA_MAX;
		else
		{
			txSize = *p->datasize;	
			end_of_frame = true;
		}*/

        send_packet_size = *p->datasize;

        /*if(cur_state == PREPARE_STATE)
		{
			p->sizeBuf.resize(6);
			p->sizeBuf[0] = 0xC0;
			p->sizeBuf[1] = 0x5A;

			temp_size.sizeInt = send_packet_size;

			p->sizeBuf[2] = temp_size.myBytes.b1;
			p->sizeBuf[3] = temp_size.myBytes.b2;
			p->sizeBuf[4] = temp_size.myBytes.b3;
			p->sizeBuf[5] = temp_size.myBytes.b4;

			cur_state = SEND_GPIO_STATE;
		}*/

        if (cur_state == SEND_GPIO_STATE)
        {
            ////set GPIO3 to low
            FT4222_GPIO_Write(p->ftGPIOHandle, GPIO_PORT3, 0);

            //set GPIO3 to high
            FT4222_GPIO_Write(p->ftGPIOHandle, GPIO_PORT3, 1);

            cur_state = SEND_READSIZE_STATE;
        }

        if (cur_state == SEND_READSIZE_STATE)
        {
            //make_size_buf(send_packet_size,&(p->sizeBuf));

#if 1
            p->sizeBuf.resize(7);
            p->sizeBuf[0] = 0xC0;
            p->sizeBuf[1] = 0x5A;

            p->sizeBuf[2] = (uint8_t)*p->datatype;

            temp_size.sizeInt = send_packet_size;

            p->sizeBuf[3] = temp_size.myBytes.b1;
            p->sizeBuf[4] = temp_size.myBytes.b2;
            p->sizeBuf[5] = temp_size.myBytes.b3;
            p->sizeBuf[6] = temp_size.myBytes.b4;
#endif
            //send 4 bytes of data size info
            dbg_printf("SLAVE HEADER FT4222_SPISlave_Write start\n");
            FT4222_SPISlave_Write(p->ftHandle, &(p->sizeBuf[0]), p->sizeBuf.size(), &sizeTransferred);

            dbg_printf("SLAVE HEADER FT4222_SPISlave_Write finish\n");

            //workaround check MASTER CLK IN
            uint16 rxSize;
            while(spi_start_send_flag)
            {
                rxSize = 0;
                if(FT4222_SPISlave_GetRxStatus(p->ftHandle, &rxSize) == FT4222_OK)
                {
                    if(rxSize>0)
                    {
                        break;
                    }
                }
            }

            if(!spi_start_send_flag)
            {
                ////set GPIO3 to low: STOP SEND FILE
                FT4222_GPIO_Write(p->ftGPIOHandle, GPIO_PORT3, 0);
//                FT4222_SPI_ResetTransaction(p->ftHandle, 0);
//                FT4222_SPI_Reset(p->ftHandle);
                dbg_printf("SLAVE SEND threadSpiReceive_Request_CMD STOP\n");
                return 1;
            }

            ////set GPIO3 to low
            FT4222_GPIO_Write(p->ftGPIOHandle, GPIO_PORT3, 0);

            cur_state = SEND_DATA_PACKET_STATE;
        }

        if (cur_state == SEND_DATA_PACKET_STATE)
        {
            cur_pos = 0;
            while (send_packet_size > 0 && spi_start_send_flag)
            {
                if (send_packet_size > RAW_DATA_MAX)
                    txSize = RAW_DATA_MAX;
                else
                {
                    txSize = send_packet_size;
                }

//                dbg_printf("DATA FT4222_SPISlave_Write start\n");
                FT4222_SPISlave_Write(p->ftHandle, &(p->rawData[cur_pos]), txSize, &sizeTransferred);
//                dbg_printf("DATA FT4222_SPISlave_Write finish\n");

                cur_pos += txSize;
                send_packet_size -= txSize;
            }

            //clear data in rawData
            //p->rawData.erase(p->rawData.begin(), p->rawData.begin()+send_packet_size);
            //clear op move to SEND_RESULT_STATE, when get pass result

            cur_state = SEND_RESULT_STATE;
        }
    }

    p->rx_cb(0);
    spi_start_send_flag = false;

    dbg_printf("SLAVE threadSpiReceive_Request_CMD finish\n");

    return 1;
}

#if 0
unsigned __stdcall threadSpiReceive_Request_CMD(void *arglist)
{
	uint16 sizeTransferred = 0;
	uint16 rxSize = 0, txSize = 0, send_packet_size = 0;
	SendSTATE_t cur_state = SEND_GPIO_STATE;
	sizeShortUnion read_size;
	bool start_of_frame = false;
	bool end_of_frame = false;

	spi_info_t *p = (spi_info_t*)arglist;


	start_of_frame = true;
	//check if data size over max transfer size
	while(spi_start_send_flag)
	{
		if(*p->datasize >RAW_DATA_MAX)
			txSize = RAW_DATA_MAX;
		else
		{
			txSize = *p->datasize;	
			end_of_frame = true;
		}

		send_packet_size = txSize +3; //1 byte for DATA_SYNC, 1 byte for checksum, 1 byte for data type


		if(cur_state == SEND_GPIO_STATE)
		{
			////set GPIO3 to low
			FT4222_GPIO_Write(p->ftGPIOHandle, GPIO_PORT3, 0) ;
			
			//set GPIO3 to high 
			FT4222_GPIO_Write(p->ftGPIOHandle, GPIO_PORT3, 1) ;

			cur_state = SEND_READSIZE_STATE;
		}
		
		if(cur_state == SEND_READSIZE_STATE)
		{
			make_size_buf(send_packet_size,&(p->sizeBuf));
			//send 4 bytes of data size info
			FT4222_SPISlave_Write(p->ftHandle, &(p->sizeBuf[0]), p->sizeBuf.size() , &sizeTransferred);

			cur_state = SEND_INIT_STATE;
		}
		
		if(cur_state == SEND_INIT_STATE)
		{
			while(FT4222_SPISlave_GetRxStatus(p->ftHandle, &rxSize) == FT_OK)
			{
				if(rxSize>0)
				{
					p->recvBuf.resize(rxSize);

					if(FT4222_SPISlave_Read(p->ftHandle,&(p->recvBuf[0]), rxSize, &sizeTransferred)== FT_OK)
					{
						if(sizeTransferred!= rxSize)
						{
							//printf("read data size is not equal\n");
						}
						else
						{
							// append data to dataBuf
							p->dataBuf.insert(p->dataBuf.end(), p->recvBuf.begin(), p->recvBuf.end());
						}
					}
					else
					{
						//printf("FT4222_SPISlave_Read error\n");
					}
				}


				if(p->dataBuf.size()>0)
				{
					while(p->dataBuf.size()>0)
					{
						if(p->dataBuf[SPI_SYNC_POS] == SPI_SYNC_CMD2)
						{
							//receive_time_flag = false;
							break;
						}
						p->dataBuf.erase(p->dataBuf.begin()); 
					}

					//todo- if dataBuf iszero and no SYNC cmd find
					

					

					//
					if(p->dataBuf.size() >= SPI_HEADER)
					{
						switch(p->dataBuf[SPI_CMD_POS])
						{
							case R_CMD: 
															
								if(SPI_SIZE_SIZE == 2)
								{
									read_size.myBytes.b1 =  p->dataBuf[SPI_SIZE_POS];
									read_size.myBytes.b2 =  p->dataBuf[SPI_SIZE_POS+1];
								}
								else
								{
									//currently only support 2 bytes data size
									read_size.sizeShort = 0;
									p->dataBuf.erase(p->dataBuf.begin(), p->dataBuf.begin()+SPI_HEADER);
									break;
								}

								//master got correct data size, we can pull GPIO to low 
								////set GPIO3 to low
								FT4222_GPIO_Write(p->ftGPIOHandle, GPIO_PORT3, 0) ;

								cur_state = SEND_DATA_PACKET_STATE;
								p->dataBuf.erase(p->dataBuf.begin(), p->dataBuf.begin()+SPI_HEADER);
								break;
							case Size_CMD:
								cur_state = SEND_READSIZE_STATE;
								p->dataBuf.erase(p->dataBuf.begin(), p->dataBuf.begin()+SPI_HEADER);
								break;
							case STA_CMD:
								if(p->dataBuf[SPI_STA_POS] == RESULT_INCORRECT)
								{
									//fail, master call to stop
									cur_state = SEND_GPIO_STATE ;
									spi_start_send_flag = false;
									p->rx_cb(1);

								}
								break;
							default:
								//incorrect command, erase 4 bytes of command data
								p->dataBuf.erase(p->dataBuf.begin(), p->dataBuf.begin()+SPI_HEADER);
								break;

						}
					}
				}

				if(cur_state == SEND_DATA_PACKET_STATE || cur_state == SEND_READSIZE_STATE)
					break;


			}
		}

		if(cur_state == SEND_DATA_PACKET_STATE || cur_state == SEND_RE_DATA_PACKET_STATE)
		{
			if(cur_state == SEND_DATA_PACKET_STATE)
				make_data_buf_checksum(txSize,&(p->rawData), start_of_frame , end_of_frame , *p->datatype);

			FT4222_SPISlave_Write(p->ftHandle, &(p->rawData[0]), send_packet_size , &sizeTransferred); // send_packet_size is txSize + 2.....+1 more for checksum, +1 more for data_sync byte

			//clear data in rawData
			//p->rawData.erase(p->rawData.begin(), p->rawData.begin()+send_packet_size);
			//clear op move to SEND_RESULT_STATE, when get pass result

			cur_state = SEND_RESULT_STATE;

		}

		if(cur_state == SEND_RESULT_STATE)
		{
			while(FT4222_SPISlave_GetRxStatus(p->ftHandle, &rxSize) == FT_OK)
			{
				if(rxSize>0)
				{
					p->recvBuf.resize(rxSize);

					if(FT4222_SPISlave_Read(p->ftHandle,&(p->recvBuf[0]), rxSize, &sizeTransferred)== FT_OK)
					{
						if(sizeTransferred!= rxSize)
						{
							//printf("read data size is not equal\n");
						}
						else
						{
							// append data to dataBuf
							p->dataBuf.insert(p->dataBuf.end(), p->recvBuf.begin(), p->recvBuf.end());
						}
					}
					else
					{
						//printf("FT4222_SPISlave_Read error\n");
					}
				}


				if(p->dataBuf.size()>0)
				{
					while(p->dataBuf.size()>0)
					{
						if(p->dataBuf[SPI_SYNC_POS] == SPI_SYNC_CMD2)
						{
							//receive_time_flag = false;
							break;
						}
						p->dataBuf.erase(p->dataBuf.begin()); 
					}

					//
					if(p->dataBuf.size() >= SPI_HEADER)
					{
						switch(p->dataBuf[SPI_CMD_POS])
						{
							case R_CMD:
															
								if(SPI_SIZE_SIZE == 2)
								{
									read_size.myBytes.b1 =  p->dataBuf[SPI_SIZE_POS];
									read_size.myBytes.b2 =  p->dataBuf[SPI_SIZE_POS+1];
								}
								else
								{
									//currently only support 2 bytes data size
									read_size.sizeShort = 0;
									p->dataBuf.erase(p->dataBuf.begin(), p->dataBuf.begin()+SPI_HEADER);
									break;
								}

								cur_state = SEND_RE_DATA_PACKET_STATE;
								p->dataBuf.erase(p->dataBuf.begin(), p->dataBuf.begin()+SPI_HEADER);
								break;
							case STA_CMD:
								//check 
								if(p->dataBuf[SPI_STA_POS] == RESULT_PASS)
								{
									//erase data already send
									p->rawData.erase(p->rawData.begin(), p->rawData.begin()+send_packet_size);

									*p->datasize = *p->datasize - txSize;
									//result is PASS
									if(*p->datasize <= 0)
										spi_start_send_flag = false;
									else
										start_of_frame = false;
									

									p->rx_cb(0);
								}
								else
								{
									//fail, master call to stop
									spi_start_send_flag = false;
									p->rx_cb(1);
								}

								cur_state = SEND_GPIO_STATE;
								p->dataBuf.erase(p->dataBuf.begin(), p->dataBuf.begin()+SPI_HEADER);
								break;
							default:
								//incorrect command, erase 4 bytes of command data
								p->dataBuf.erase(p->dataBuf.begin(), p->dataBuf.begin()+SPI_HEADER);
								break;

						}
					}
				}

				if(cur_state == SEND_GPIO_STATE || cur_state == SEND_RE_DATA_PACKET_STATE)
					break;


			}
		}

		

	}

    return 1;
}

#endif
//#define PACKET_SIZE 65500
#ifdef NONBLOCKING_SPIS
#define PACKET_SIZE 0x2000
#else
#define PACKET_SIZE 1000
#endif
unsigned __stdcall threadSpiMasterSend_CMD(void *arglist)
{
    spi_info_t *p = (spi_info_t *)arglist;

    //
    sizeUnion write_size;
    std::vector<unsigned char> sendBuf;
//    sizeShortUnion image_size;
    uint16 sizeTransferred = 0;
    uint32 cur_pos = 0;
    FT_STATUS ft4222_status;

    write_size.sizeInt = *(p->datasize);

    sendBuf.resize(6);
    sendBuf[0] = SPI_SYNC_CMD2;
    sendBuf[1] = W_CMD;
    sendBuf[2] = write_size.myBytes.b1;
    sendBuf[3] = write_size.myBytes.b2;
    sendBuf[4] = write_size.myBytes.b3;
    sendBuf[5] = write_size.myBytes.b4;

    ft4222_status = FT4222_SPIMaster_SingleWrite(p->ftHandle, &sendBuf[0], 6, &sizeTransferred, true);
    /*sendBuf.resize(8);
	sendBuf[0] = SPI_SYNC_CMD;
	sendBuf[1] = W_CMD;
	sendBuf[2] = write_size.myBytes.b1;
	sendBuf[3] = write_size.myBytes.b2;
	sendBuf[4] = write_size.myBytes.b3;
	sendBuf[5] = write_size.myBytes.b4;

	ft4222_status = FT4222_SPIMaster_SingleWrite(p->ftHandle,&sendBuf[0], 8, &sizeTransferred, true);*/

#ifdef SAVELOG
    pFile = fopen("savelog.txt", "a");
    if (pFile != NULL)
    {
        fprintf(pFile, "send file length %d\n", write_size.sizeInt);

        fclose(pFile);
    }

#endif

    if (p != NULL)
    {
        cur_pos = 0;
        do
        {
            Sleep(5);

            if (write_size.sizeInt > PACKET_SIZE)
                sendBuf.resize(PACKET_SIZE);
            else
                sendBuf.resize(write_size.sizeInt);

            if (write_size.sizeInt > PACKET_SIZE)
                memcpy(&sendBuf[0], &(p->rawData[cur_pos]), PACKET_SIZE);
            else
                memcpy(&sendBuf[0], &(p->rawData[cur_pos]), write_size.sizeInt);

            ft4222_status = FT4222_SPIMaster_SingleWrite(p->ftHandle, &sendBuf[0], sendBuf.size(), &sizeTransferred, true);

            if (write_size.sizeInt > PACKET_SIZE)
                write_size.sizeInt -= PACKET_SIZE;
            else
                write_size.sizeInt = 0;

            cur_pos += PACKET_SIZE;

        } while (write_size.sizeInt > 0 && spi_start_send_flag == true);

        p->rx_cb(0);
    }
    spi_start_send_flag = false;
    //

    dbg_printf("threadSpiMasterSend_CMD: 0x%08X\n", (uint)ft4222_status);

    return 1;
}

#define SYNC_length 2
#define ACK_PACKET_LENGTH 10
#define ACK_DATA 6

unsigned __stdcall threadSpiMasterRecv_ACK(void *arglist)
{
    spi_info_t *p = (spi_info_t *)arglist;

    std::vector<unsigned char> recvBuf;
    std::vector<unsigned char> appendBuf;
    FT_STATUS ft4222_status;
//    sizeUnion write_size;
    uint16 sizeTransferred = 0;
//    uint32 cur_pos = 0;
    int index_sync_pos = -1;
    int count = 0;

    recvBuf.resize(ACK_PACKET_LENGTH);

#if 1
    index_sync_pos = -1;
    while (index_sync_pos == -1 && spi_start_receive_flag == true)
    {
        recvBuf[0] = 0x00;
        recvBuf[1] = 0x00;
        recvBuf[2] = 0x00;
        recvBuf[3] = 0x00;
        recvBuf[4] = 0x00;
        recvBuf[5] = 0x00;
        //Sleep(100);

        if (p->ftHandle != NULL)
        {
            ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[0], ACK_PACKET_LENGTH, &sizeTransferred, true);

#ifdef SAVELOG
            pFile = fopen("savelog.txt", "a");
            if (pFile != NULL)
            {
                //if(recvBuf[2] == 0x39 && recvBuf[3] == 0xff && recvBuf[4] == 0x13 && recvBuf[5] == 0xA5)
                //fprintf (pFile, "recv ACK[%d] =>",count);
                //else if(recvBuf[2] == 0xBB && recvBuf[3] == 0x14 && recvBuf[4] == 0x69 && recvBuf[5] == 0x34)
                //fprintf (pFile, "recv NACK[%d] =>",count);
                //else
                //fprintf (pFile, "recv unknown[%d] =>",count);
                fprintf(pFile, "value :");
                for (count = 0; count < ACK_PACKET_LENGTH; count++)
                    fprintf(pFile, "%x:", recvBuf[count]);

                fprintf(pFile, "\n");

                fclose(pFile);
            }
#endif

            for (int i = 0; i < (ACK_PACKET_LENGTH - 1); i++)
            {
                if (recvBuf[i] == 0x5A && recvBuf[i + 1] == 0xA3)
                {
                    index_sync_pos = i;

                    if (index_sync_pos != 0)
                        if ((ACK_PACKET_LENGTH - index_sync_pos) > ACK_DATA)
                        {
                            //copy data to first position
                            for (int j = 0; j < ACK_DATA; j++)
                            {
                                recvBuf[0 + j] = recvBuf[index_sync_pos + j];
                            }
                        }
                    //
                    break;
                }
            }
        }

        Sleep(500);
    }

    /*if(index_sync_pos != 0)
	{
		appendBuf.resize(index_sync_pos);
		ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &appendBuf[0], (index_sync_pos),  &sizeTransferred, true); 

		recvBuf.insert(recvBuf.end(), appendBuf.begin(), appendBuf.end());

		for(int i=0;i<ACK_PACKET_LENGTH;i++)
		{
			recvBuf[0+i] = recvBuf[index_sync_pos+i];
		}
	}*/

#else

    recvBuf[0] = 0x00;
    recvBuf[1] = 0x00;
    recvBuf[2] = 0x00;
    recvBuf[3] = 0x00;
    recvBuf[4] = 0x00;
    recvBuf[5] = 0x00;

    count = 0;
    while (recvBuf[0] != 0x5A && spi_start_receive_flag == true)
    {
        Sleep(100);
        ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[0], 1, &sizeTransferred, false);

        /*count ++;

		if(count > 65000)
		{
			//return 0;
			count = 0;
		}*/
    }

    while (recvBuf[1] != 0xA3 && spi_start_receive_flag == true)
    {
        ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[1], 1, &sizeTransferred, false);
        count++;

        /*if(count > 65000)
		{
			//return 0;
			count = 0;
		}*/
        //Sleep(100);
    }

    ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[2], 4, &sizeTransferred, true);
#endif
    if (p != NULL)
    {
        p->rawData.resize(6);
        memcpy(&(p->rawData[0]), &recvBuf[0], 6);

        //*(p->datasize) = p->rawData.size();

        if (recvBuf[2] == 0x39 && recvBuf[3] == 0xff && recvBuf[4] == 0x13 && recvBuf[5] == 0xA5) // ACK
            p->rx_cb(0);
        else if (recvBuf[2] == 0xBB && recvBuf[3] == 0x14 && recvBuf[4] == 0x69 && recvBuf[5] == 0x34) //NACK
            p->rx_cb(1);
        else //other value
            p->rx_cb(2);
    }
    spi_start_receive_flag = false;

    dbg_printf("threadSpiMasterRecv_ACK: 0x%08X\n", (uint)ft4222_status);

    return 0;
}
unsigned __stdcall threadSpiMasterSendSimple_CMD(void *arglist)
{
	spi_info_t *p = (spi_info_t *)arglist;
	BOOL io_val = true;
	bool header_flag = false;
//	int count = 0;
	std::vector<unsigned char> sendBuf;
	uint16 sizeTransferred = 0;
	FT_STATUS  ft4222_status;
	sizeUnion write_size;
	uint32 cur_pos = 0;

	write_size.sizeInt = *(p->datasize);

	if(spi_start_send_flag)
	{
		//GPIO state
		FT4222_GPIO_Read(p->ftGPIOHandle, GPIO_PORT3, &io_val);

		//spi master send state 
		header_flag = false;
		sendBuf.resize(7);
		sendBuf[0] = SPI_SYNC_CMD1;
		sendBuf[1] = SPI_SYNC_CMD2;
		sendBuf[2] = 0x0;
		sendBuf[3] = write_size.myBytes.b1;	
		sendBuf[4] = write_size.myBytes.b2;
		sendBuf[5] = write_size.myBytes.b3;
		sendBuf[6] = write_size.myBytes.b4;

        dbg_printf("HEADER FT4222_SPIMaster_SingleWrite start\n");

		ft4222_status = FT4222_SPIMaster_SingleWrite(p->ftHandle, &sendBuf[0], 7, &sizeTransferred, true);

        dbg_printf("HEADER FT4222_SPIMaster_SingleWrite finish\n");

		if(ft4222_status != FT_OK)
		{
			spi_start_send_flag = false;
			io_val = false; 
			//FT4222_GPIO_Write(p->ftGPIOHandle, GPIO_PORT3, io_val);
			return 1;
		}
		else
		{

            header_flag = true;
            Sleep(10);
		}

		if((spi_start_send_flag) && (header_flag))
		{
			header_flag = false;

			cur_pos = 0;

			while ((cur_pos < write_size.sizeInt) && (spi_start_send_flag))
			{
				//Sleep(1);
				if ((write_size.sizeInt - cur_pos) > PACKET_SIZE)
				{
					ft4222_status = FT4222_SPIMaster_SingleWrite(p->ftHandle, &(p->rawData[cur_pos]), PACKET_SIZE, &sizeTransferred, true);
					cur_pos += PACKET_SIZE;
				}
				else
				{
					ft4222_status = FT4222_SPIMaster_SingleWrite(p->ftHandle, &(p->rawData[cur_pos]), (write_size.sizeInt - cur_pos), &sizeTransferred, true);
					cur_pos = write_size.sizeInt;
				}

                 dbg_printf("DATA FT4222_SPIMaster_SingleWrite finish\n");
			}

			p->rx_cb(0);
		}
	}

	spi_start_send_flag = false;

	io_val = false; 
	//FT4222_GPIO_Write(p->ftGPIOHandle, GPIO_PORT3, io_val);

    dbg_printf("threadSpiMasterSendSimple_CMD finish\n");

	return 0;
}

unsigned __stdcall threadSpiMasterRecvSimple_CMD(void *arglist)
{
	spi_info_t *p = (spi_info_t *)arglist;
	BOOL io_val = false;
	bool header_flag = false;
	int count = 0;
	std::vector<unsigned char> recvBuf;
	uint16 sizeTransferred = 0;
	FT_STATUS  ft4222_status;
	sizeUnion write_size;
	uint32 cur_pos = 0;

    while (spi_start_receive_flag)
    {
		//GPIO state 
		count = 0;
		while (spi_start_receive_flag)
		{
			//wait GPIO to high
			FT4222_GPIO_Read(p->ftGPIOHandle, GPIO_PORT2, &io_val);

			if (io_val)
			{
				break;
			}

			count++;

			Sleep(1);

			if (count > 200)
			{
				//self->thd_flag = false;
                dbg_printf("wait GPIO to high timeout\n");
				break;
			}
		}

		//spi master recv state 
		count = 0;
		header_flag = false;
		while((spi_start_receive_flag) && (io_val))
		{
			FT4222_GPIO_Read(p->ftGPIOHandle, GPIO_PORT2, &io_val);

			recvBuf.resize(7);

			for (int i = 0; i < 7; i++)
				recvBuf[i] = 0x00;

			ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[0], 7, &sizeTransferred, true);

			if(ft4222_status == FT_OK)
			{
				if(recvBuf[0] == 0xC0 && recvBuf[1] == 0x5A)
				{
					header_flag = true;
					break;
				}
                dbg_printf("FT4222_SPIMaster_SingleRead PACK fail 0x%08x 0x%08x\n", recvBuf[0], recvBuf[1]);
			}
		}

		//
		while((spi_start_receive_flag) && (header_flag))
		{
			header_flag = false;
			*(p->datatype) = (DataType_t)recvBuf[2];
			write_size.myBytes.b1 = recvBuf[3];
			write_size.myBytes.b2 = recvBuf[4];
			write_size.myBytes.b3 = recvBuf[5];
			write_size.myBytes.b4 = recvBuf[6];


			recvBuf.resize(write_size.sizeInt);

			for (unsigned int i = 0; i < write_size.sizeInt; i++)
			{
				recvBuf[i] = 0;
			}

			cur_pos = 0;

			while ((cur_pos < write_size.sizeInt) && (spi_start_receive_flag))
			{
#ifndef NONBLOCKING_SPIS
				Sleep(3);
#else
				//Sleep(3);
#endif
				if (write_size.sizeInt - cur_pos > PACKET_SIZE)
				{
					ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[cur_pos], PACKET_SIZE, &sizeTransferred, true);
					cur_pos += PACKET_SIZE;
				}
				else
				{
					ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[cur_pos], (write_size.sizeInt - cur_pos), &sizeTransferred, true);
					cur_pos = write_size.sizeInt;
				}
			}

			p->rawData.resize(write_size.sizeInt);
			//copy data from start of complete packet
			memcpy(&(p->rawData[0]), &recvBuf[0], (write_size.sizeInt));

			*(p->datasize) = p->rawData.size();
			p->rx_cb(0);
			//p->rx_cb(write_size.sizeInt);
		}



		//spi_start_receive_flag = false;
	}

	return 0;
}
#if 0
unsigned __stdcall threadSpiMasterRecvSimple_CMD(void *arglist)
{

    spi_info_t *p = (spi_info_t *)arglist;
    std::vector<unsigned char> recvBuf;
    uint16 sizeTransferred = 0;
    sizeUnion write_size;
    uint32 cur_pos = 0;
    FT_STATUS ft4222_status;
    BOOL io_val = false;
    int count = 0;
	int header_flag = 0;

    while (spi_start_receive_flag)
    {
        //wait GPIO to high

        FT4222_GPIO_Read(p->ftGPIOHandle, GPIO_PORT2, &io_val);

        if (io_val)
        {
            break;
        }

        count++;

        Sleep(10);

        if (count > 200)
        {
            p->rx_cb(1);
            spi_start_receive_flag = false;
            //return 0;
        }
    };

	if(!spi_start_receive_flag)
	{
		p->rx_cb(1);
		return 0;
	}

    recvBuf.resize(6);
#if 0
	/*for (int i = 0; i < 6; i++)
		recvBuf[i] = 0x00;

	do{
		recvBuf[0] = 0;
		ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[0], 6, &sizeTransferred, true);

	}while (recvBuf[0] != 0xC0 && recvBuf[1] != 0xC0 && recvBuf[2] != 0xC0 && recvBuf[3] != 0xC0 && recvBuf[4] != 0xC0 && recvBuf[5] != 0xC0);*/

	for (int i = 0; i < 6; i++)
		recvBuf[i] = 0x00;

	ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[0], 6, &sizeTransferred, true);

	header_flag = 0;
	for (int i = 0; i < 5; i++)
	{
		if(recvBuf[i] == 0xC0 && recvBuf[i+1] == 0x5A)
		{
			header_flag = 1;
			break;
		}
	}

	if(header_flag == 0)
	{
		p->rx_cb(1);
		spi_start_receive_flag = false;
		return 0;
	}
	
#endif


	count = 0;
    do
    {
		if(!spi_start_receive_flag)
		{
			p->rx_cb(1);
			return 0;
		}

       for (int i = 0; i < 6; i++)
            recvBuf[i] = 0x00;

        ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[0], 6, &sizeTransferred, true);

        count++;

        Sleep(10);

        if (count > 200)
        {
            p->rx_cb(1);
            spi_start_receive_flag = false;
            return 0;
        }

		//check if any value equal sync bytes
		for (int i = 0; i < 5; i++)
		{
			if(recvBuf[i] == 0xC0 && recvBuf[i+1] == 0x5A)
				break;
		}

    } while (recvBuf[0] != 0xC0 || recvBuf[1] != 0x5A); // for WE-I spi slave send header sync command
    //

    write_size.myBytes.b1 = recvBuf[2];
    write_size.myBytes.b2 = recvBuf[3];
    write_size.myBytes.b3 = recvBuf[4];
    write_size.myBytes.b4 = recvBuf[5];

    recvBuf.resize(write_size.sizeInt);

    for (int i = 0; i < write_size.sizeInt; i++)
    {
        recvBuf[i] = 0;
    }

    cur_pos = 0;

    while ((cur_pos < write_size.sizeInt) && spi_start_receive_flag == true)
    {
        Sleep(3);
        if (write_size.sizeInt - cur_pos > PACKET_SIZE)
        {
            ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[cur_pos], PACKET_SIZE, &sizeTransferred, true);
            cur_pos += PACKET_SIZE;
        }
        else
        {
            ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[cur_pos], (write_size.sizeInt - cur_pos), &sizeTransferred, true);
            cur_pos = write_size.sizeInt;
        }
    }

	if(!spi_start_receive_flag)
		return 0;

    p->rawData.resize(write_size.sizeInt);
    //copy data from start of complete packet
    memcpy(&(p->rawData[0]), &recvBuf[0], (write_size.sizeInt));

    *(p->datasize) = p->rawData.size();
    p->rx_cb(0);

    spi_start_receive_flag = false;

    return 0;
}

#endif
unsigned __stdcall threadSpiMasterRecv_CMD(void *arglist)
{
    spi_info_t *p = (spi_info_t *)arglist;

    std::vector<unsigned char> recvBuf;
    std::vector<unsigned char> sendBuf;
    sizeUnion write_size;
    uint16 sizeTransferred = 0;
    uint32 cur_pos = 0;
    int count = 0;
    FT_STATUS ft4222_status;

    recvBuf.resize(5);
    recvBuf[0] = 0x00;

#if 0
	while(recvBuf[0] != 0x5A || recvBuf[1] != 0xA3)
	{
		ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[0], 2,  &sizeTransferred, false); 
		count ++;

		if(count > 65000)
		{
			//printf("no cmd\n");
			return 0;
		}
	}

	ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[1], 4,  &sizeTransferred, true);
#else

    //
    recvBuf.resize(6);

    do
    {
        for (int i = 0; i < 6; i++)
            recvBuf[i] = 0x00;

        ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[0], 6, &sizeTransferred, true);

        count++;

        Sleep(10);

        if (count > 2000)
        {
            p->rx_cb(1);
            spi_start_receive_flag = false;
            return 0;
        }
    } while (recvBuf[0] != 0x5A || recvBuf[1] != 0xA3); // for WE-I spi slave send data sync command
                                                        //}while(recvBuf[0] != 0xC0 || recvBuf[1] != 0x5A);

    //
#endif
    write_size.myBytes.b1 = recvBuf[2];
    write_size.myBytes.b2 = recvBuf[3];
    write_size.myBytes.b3 = recvBuf[4];
    write_size.myBytes.b4 = recvBuf[5];

    //printf("data packet length is %d[%x][%x][%x][%x][%x]\n", write_size.sizeInt,recvBuf[0],recvBuf[1],recvBuf[2],recvBuf[3],recvBuf[4]);

    recvBuf.resize(write_size.sizeInt);

    for (uint32_t i = 0; i < write_size.sizeInt; i++)
    {
        recvBuf[i] = 0;
    }

    cur_pos = 0;

    while (cur_pos < write_size.sizeInt)
    {
        Sleep(5);
        if (write_size.sizeInt - cur_pos > PACKET_SIZE)
        {
            ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[cur_pos], PACKET_SIZE, &sizeTransferred, true);
            cur_pos += PACKET_SIZE;
        }
        else
        {
            ft4222_status = FT4222_SPIMaster_SingleRead(p->ftHandle, &recvBuf[cur_pos], (write_size.sizeInt - cur_pos), &sizeTransferred, true);
            cur_pos = write_size.sizeInt;
        }
    }

    p->rawData.resize(write_size.sizeInt);
    //copy data from start of complete packet
    memcpy(&(p->rawData[0]), &recvBuf[0], (write_size.sizeInt));

    *(p->datasize) = p->rawData.size();
    p->rx_cb(0);

    spi_start_receive_flag = false;

    dbg_printf("threadSpiMasterRecv_CMD: 0x%08X\n", (uint)ft4222_status);

    return 0;
}

unsigned __stdcall threadSpiMasterWriteFlaash(void *arglist)
{
    spi_flash_info_t *p = (spi_flash_info_t *)arglist;

    std::vector<unsigned char> sendDataBuf;
    std::vector<unsigned char> sendCmdBuf;
    std::vector<unsigned char> sendBuf;
    std::vector<unsigned char> recvBuf;
    uint16 sizeTransferred = 0;
    uint32 longsizeTransferred = 0;
    FT_STATUS ft4222_status;

    uint16 out_of_page_data_size = 0;

    unsigned int cur_pos = p->addr;
    unsigned int data_size = p->data_size;
    unsigned int total_size = 0;
//    unsigned char wip_status;

    /*if(data_size == 0)
		return ret_ErrParameter;

	if(data == NULL)
		return ret_ErrParameter;*/

    sendCmdBuf.resize(4);

    sendBuf.resize(1);
    sendBuf[0] = 0x06;

    recvBuf.resize(1);

    total_size = 0;
    flash_write_percent = 0;
    //check data address, if A0-A7 is not zero. data over page size should be seperate to another packet
    do
    {
		DRV_spi_flash_waitWIP(p->ftHandle);
        //
        out_of_page_data_size = 0x100 - (cur_pos & 0xFF);
        if ((data_size - total_size) < out_of_page_data_size)
            out_of_page_data_size = (data_size - total_size);

        //spi_flash_write_enable();
		DRV_spi_flash_write_enable(p->ftHandle);

        //spi_flash_waitWIP ();

        //sendDataBuf.resize(out_of_page_data_size);

        //if data is 256 bytes, addr A0-A7 should be 0
        //check current mode
        switch (g_flash_read_mode)
        {
        case Mode_Single:
            ft4222_status = FT4222_SPIMaster_SetLines(p->ftHandle, SPI_IO_SINGLE);
            if (FT4222_OK != ft4222_status)
            {
                dbg_printf("Init FT4222 as SPI master device failed!\n");
                return ret_FAIL;
            }
            //memcpy(&sendDataBuf[0], &(p->rawData[total_size]), sizeof(unsigned char)*out_of_page_data_size);

            sendCmdBuf[0] = 0x02;
            sendCmdBuf[1] = (cur_pos >> 16) & 0xff;
            sendCmdBuf[2] = (cur_pos >> 8) & 0xff;
            sendCmdBuf[3] = cur_pos & 0xff;

            ft4222_status = FT4222_SPIMaster_SingleWrite(p->ftHandle, &sendCmdBuf[0], sendCmdBuf.size(), &sizeTransferred, false);

            //ft4222_status = FT4222_SPIMaster_SingleWrite(p->ftHandle, &sendDataBuf[0], sendDataBuf.size(),  &sizeTransferred, true);
            ft4222_status = FT4222_SPIMaster_SingleWrite(p->ftHandle, &(p->rawData[total_size]), out_of_page_data_size, &sizeTransferred, true);

            break;
        case Mode_Quad:

            //ft4222_status = FT4222_SPIMaster_Init(p->ftHandle, SPI_IO_QUAD, CLK_DIV_16, CLK_IDLE_LOW, CLK_LEADING, 0x01);

            ft4222_status = FT4222_SPIMaster_SetLines(p->ftHandle, SPI_IO_QUAD);
            if (FT4222_OK != ft4222_status)
            {
                dbg_printf("Init FT4222 as SPI master device failed!\n");
                return ret_FAIL;
            }

            sendDataBuf.resize(out_of_page_data_size + 4);
            memcpy(&sendDataBuf[4], &(p->rawData[total_size]), sizeof(unsigned char) * out_of_page_data_size);

            if (g_cur_flash_type == VENDOR_MXIC)
            {
                sendDataBuf[0] = 0x38;
                sendDataBuf[1] = (cur_pos >> 16) & 0xff;
                sendDataBuf[2] = (cur_pos >> 8) & 0xff;
                sendDataBuf[3] = cur_pos & 0xff;

                ft4222_status = FT4222_SPIMaster_MultiReadWrite(ftdevHandle, NULL, &sendDataBuf[0], 1, sendDataBuf.size() - 1, 0, &longsizeTransferred);
            }
            else if (g_cur_flash_type == VENDOR_WINBOND)
            {
                sendDataBuf[0] = 0x32;
                sendDataBuf[1] = (cur_pos >> 16) & 0xff;
                sendDataBuf[2] = (cur_pos >> 8) & 0xff;
                sendDataBuf[3] = cur_pos & 0xff;

                ft4222_status = FT4222_SPIMaster_MultiReadWrite(ftdevHandle, NULL, &sendDataBuf[0], 4, sendDataBuf.size() - 4, 0, &longsizeTransferred);
            }

            //ft4222_status = FT4222_SPIMaster_SingleWrite(ftdevHandle,&sendCmdBuf[0], sendCmdBuf.size(), &sizeTransferred, false);

            //ft4222_status = FT4222_SPIMaster_SingleWrite(ftdevHandle, &sendDataBuf[0], sendDataBuf.size(),  &sizeTransferred, true);
            //ft4222_status = FT4222_SPIMaster_MultiReadWrite(ftdevHandle, NULL, &sendDataBuf[0], 1,sendDataBuf.size()-1,0, &longsizeTransferred) ;

            break;
        default:
            break;
        }

        total_size += out_of_page_data_size;
        cur_pos += out_of_page_data_size;

        if (total_size != 0)
            flash_write_percent = (total_size * 100) / data_size;

    } while (total_size < data_size);

    flash_write_percent = 100;

    return 0;
}

ErrType_t spi_slave_read_packet(unsigned char *data)
{
#if 0
    if (spi_receive_info.rawData.size() != 0)
        memcpy(data, &(spi_receive_info.rawData[0]), spi_receive_info.rawData.size());
    else
        return ret_FAIL;

    return ret_PASS;
#endif
	if(DRV_spi_slave_read_packet(data) != true)
		return ret_FAIL;

	return ret_PASS;
}

ErrType_t spi_slave_receive_data(CB_FUNC_SPI_T rx_cb, unsigned int *data_size, DataType_t *data_type)
{
#if 0
    if (spi_start_receive_flag == true)
        return ret_NotFinishRecv;

    if (spi_start_send_flag == true)
        return ret_NotFinishSend;

    //if (FT4222_OK != FT4222_SPI_ResetTransaction(ftdevHandle, 0))
    //{
    // purge usb tx/rx and SPI FIFO cache
    //    return ret_FAIL;
    //}

    spi_start_receive_flag = true;

    *data_size = 0;
    *data_type = Data_JPG;

    spi_receive_info.ftHandle = ftdevHandle;

    spi_receive_info.datasize = data_size;
    spi_receive_info.datatype = data_type;
    spi_receive_info.rx_cb = rx_cb;

    //wait to receive
    hth_spi_receive = (HANDLE)_beginthreadex(NULL, 0, threadSpiReceive_Write_CMD, &spi_receive_info, 0, &thID_spi_receive);

    return ret_PASS;
#endif
	if(DRV_spi_slave_receive_data(ftdevHandle, rx_cb, data_size, data_type) != true)
		return ret_NotFinishSend;

	return ret_PASS;
}

ErrType_t spi_slave_send_data(CB_FUNC_SPI_T cb, unsigned int data_size, DataType_t data_type, unsigned char *data)
{
    if (spi_start_send_flag == true)
        return ret_NotFinishSend;

    if (spi_start_receive_flag == true)
        return ret_NotFinishRecv;

//    if (FT4222_OK != FT4222_SPI_ResetTransaction(ftdevHandle, 0))
//    {
//        // purge usb tx/rx and SPI FIFO cache
//        dbg_printf("!!!!!!!>FT4222_SPI_ResetTransaction fail\n");
//        return ret_FAIL;
//    }

    spi_start_send_flag = true;

    send_type = data_type;
    send_size = data_size;

    spi_send_info.ftHandle = ftdevHandle;
    spi_send_info.ftGPIOHandle = ftdevGPIOHandle;

    spi_send_info.datasize = &send_size;
    spi_send_info.datatype = &send_type;
    spi_send_info.rx_cb = cb;

    //append with sizeShort
    spi_send_info.rawData.resize(data_size);
    //copy data
    memcpy(&(spi_send_info.rawData[0]), data, data_size);

    hth_spi_send = (HANDLE)_beginthreadex(NULL, 0, threadSpiReceive_Request_CMD, &spi_send_info, 0, &thID_spi_send);
    //end if finish send data

    return ret_PASS;
}

ErrType_t spi_slave_halt(bool force)
{
#if 0
    spi_start_receive_flag = false;
    spi_start_send_flag = false;
    //todo - need to do force=TRUE part
#endif
	DRV_spi_slave_halt(true);

//    if (FT4222_OK != FT4222_SPI_Reset(ftdevHandle))
//    {
//        //purge usb tx/rx and SPI FIFO cache
//        dbg_printf("!!!!!!!>FT4222_SPI_Reset fail\n");
//        return ret_FAIL;
//    }
//    Sleep(100);

    dbg_printf("spi_slave_halt %d \n", force);

    return ret_PASS;
}

ErrType_t spi_master_send_data(CB_FUNC_SPI_T cb, unsigned int data_size, unsigned char *data)
{
    if (spi_start_send_flag == true)
        return ret_NotFinishSend;

    if (spi_start_receive_flag == true)
        return ret_NotFinishRecv;

    if (FT4222_OK != FT4222_SPI_ResetTransaction(ftdevHandle, 0))
    {
        // purge usb tx/rx and SPI FIFO cache
        return ret_FAIL;
    }

    spi_start_send_flag = true;

    send_size = data_size;

    spi_send_info.ftHandle = ftdevHandle;
    spi_send_info.ftGPIOHandle = ftdevGPIOHandle;

    spi_send_info.datasize = &send_size;
    spi_send_info.datatype = NULL;
    spi_send_info.rx_cb = cb;

    //append with sizeShort
    spi_send_info.rawData.resize(data_size);
    //copy data
    memcpy(&(spi_send_info.rawData[0]), data, data_size);

    hth_spi_send = (HANDLE)_beginthreadex(NULL, 0, threadSpiMasterSend_CMD, &spi_send_info, 0, &thID_spi_send);

    return ret_PASS;
}

ErrType_t spi_master_recv_data(CB_FUNC_SPI_T cb, unsigned int *data_size)
{
    if (spi_start_receive_flag == true)
        return ret_NotFinishRecv;

    if (spi_start_send_flag == true)
        return ret_NotFinishSend;

    //if (FT4222_OK != FT4222_SPI_ResetTransaction(ftdevHandle, 0))
    //{
    // purge usb tx/rx and SPI FIFO cache
    //    return ret_FAIL;
    //}

    spi_start_receive_flag = true;

    *data_size = 0;

    spi_receive_info.ftHandle = ftdevHandle;

    spi_receive_info.datasize = data_size;
    spi_receive_info.datatype = NULL;
    spi_receive_info.rx_cb = cb;

    //wait to receive
    hth_spi_receive = (HANDLE)_beginthreadex(NULL, 0, threadSpiMasterRecv_CMD, &spi_receive_info, 0, &thID_spi_receive);

    return ret_PASS;
}

ErrType_t spi_master_recv_simple_data(CB_FUNC_SPI_T cb, unsigned int *data_size, DataType_t *data_type)
{
    if (spi_start_receive_flag == true)
        return ret_NotFinishRecv;

    if (spi_start_send_flag == true)
        return ret_NotFinishSend;

    if (FT4222_OK != FT4222_SPI_ResetTransaction(ftdevHandle, 0))
    {
        //purge usb tx/rx and SPI FIFO cache
        return ret_FAIL;
    }

    spi_start_receive_flag = true;

    *data_size = 0;

    spi_receive_info.ftHandle = ftdevHandle;
    spi_receive_info.ftGPIOHandle = ftdevGPIOHandle;
    spi_receive_info.datasize = data_size;
    spi_receive_info.datatype = data_type;
    spi_receive_info.rx_cb = cb;

    //wait to receive
    hth_spi_receive = (HANDLE)_beginthreadex(NULL, 0, threadSpiMasterRecvSimple_CMD, &spi_receive_info, 0, &thID_spi_receive_simple);

    return ret_PASS;
}

ErrType_t spi_master_send_simple_data(CB_FUNC_SPI_T cb, unsigned int data_size, unsigned char *data)
{
    if (spi_start_receive_flag == true)
        return ret_NotFinishRecv;

    if (spi_start_send_flag == true)
        return ret_NotFinishSend;

//    if (FT4222_OK != FT4222_SPI_ResetTransaction(ftdevHandle, 0))
//    {
//        //purge usb tx/rx and SPI FIFO cache
//        dbg_printf("!!!!!!!>FT4222_SPI_ResetTransaction fail\n");
//        return ret_FAIL;
//    }

    spi_start_send_flag  = true;

	send_size = data_size;

	spi_send_info.ftHandle = ftdevHandle;
    spi_send_info.ftGPIOHandle = ftdevGPIOHandle;

    spi_send_info.datasize = &send_size;
    spi_send_info.datatype = NULL;
    spi_send_info.rx_cb = cb;

    //append with sizeShort
    spi_send_info.rawData.resize(data_size);
    //copy data
    memcpy(&(spi_send_info.rawData[0]), data, data_size);

    //wait to receive
    //hth_spi_receive = (HANDLE)_beginthreadex(NULL, 0, threadSpiMasterRecvSimple_CMD, &spi_receive_info, 0, &thID_spi_receive_simple);
	hth_spi_send = (HANDLE)_beginthreadex(NULL, 0, threadSpiMasterSendSimple_CMD, &spi_send_info, 0, &thID_spi_send);

    return ret_PASS;
}

ErrType_t spi_master_stop_recv_simple_data()
{
	//stop recv thd
	spi_start_receive_flag = false;

	return ret_PASS;
}

ErrType_t spi_master_recv_ack(CB_FUNC_SPI_T cb, unsigned int *data_size)
{
    if (spi_start_receive_flag == true)
        return ret_NotFinishRecv;

    if (spi_start_send_flag == true)
        return ret_NotFinishSend;

    //if (FT4222_OK != FT4222_SPI_ResetTransaction(ftdevHandle, 0))
    //{
    // purge usb tx/rx and SPI FIFO cache
    //    return ret_FAIL;
    //}

    spi_start_receive_flag = true;

    *data_size = 0;

    spi_receive_info.ftHandle = ftdevHandle;

    spi_receive_info.datasize = data_size;
    spi_receive_info.datatype = NULL;
    spi_receive_info.rx_cb = cb;

    //wait to receive
    hth_spi_receive = (HANDLE)_beginthreadex(NULL, 0, threadSpiMasterRecv_ACK, &spi_receive_info, 0, &thID_spi_receive);

    return ret_PASS;
}

ErrType_t spi_master_read_packet(unsigned char *data)
{
    if (spi_receive_info.rawData.size() != 0)
        memcpy(data, &(spi_receive_info.rawData[0]), spi_receive_info.rawData.size());
    else
        return ret_FAIL;

    return ret_PASS;
}

ErrType_t spi_master_halt(bool force)
{
    dbg_printf("spi_master_halt %d \n", force);

//    if(spi_start_receive_flag)
//        CloseHandle(hth_spi_receive);

//    if(spi_start_send_flag)
//        CloseHandle(hth_spi_send);


    spi_start_receive_flag = false;
    spi_start_send_flag = false;
    //todo - need to do force=TRUE part

//    if (FT4222_OK != FT4222_SPI_Reset(ftdevHandle))
//    {
//        //purge usb tx/rx and SPI FIFO cache
//        dbg_printf("!!!!!!!>FT4222_SPI_Reset fail\n");
//        return ret_FAIL;
//    }
//    Sleep(100);

    return ret_PASS;
}

ErrType_t spi_master_isp_enable_burstmode_inc4()
{
    uint8_t sendBuf_1[3] = {0xF2, 0x13, 0x31};
//    uint8_t sendBuf_1[3] = {0xF2, 0x13, 0x01}; //for test
    uint8_t sendBuf_2[3] = {0xF2, 0x0D, 0x11};
//    uint8_t sendBuf_2[3] = {0xF2, 0x0D, 0x10};
    uint16 sizeTransferred;
    FT_STATUS  ft4222_status1, ft4222_status2;

    dbg_printf("spi_master_isp_disable_burstmode_inc0 \n");

    sizeTransferred = 0;
    ft4222_status1 = FT4222_SPIMaster_SingleWrite(ftdevHandle, &sendBuf_1[0], 3, &sizeTransferred, true);
    ft4222_status2 = FT4222_SPIMaster_SingleWrite(ftdevHandle, &sendBuf_2[0], 3, &sizeTransferred, true);

    if(ft4222_status1 != FT4222_OK || ft4222_status2 != FT4222_OK)
    {
        return ret_FAIL;
    }

    return ret_PASS;
}

ErrType_t spi_master_isp_enable_burstmode_inc0()
{
    uint8_t sendBuf_1[3] = {0xF2, 0x13, 0x31};
//    uint8_t sendBuf_1[3] = {0xF2, 0x13, 0x21}; //for test
    uint8_t sendBuf_2[3] = {0xF2, 0x0D, 0x10};
    uint16 sizeTransferred;
    FT_STATUS  ft4222_status1, ft4222_status2;

    dbg_printf("spi_master_isp_disable_burstmode_inc4 \n");

    sizeTransferred = 0;
    ft4222_status1 = FT4222_SPIMaster_SingleWrite(ftdevHandle, &sendBuf_1[0], 3, &sizeTransferred, true);
    ft4222_status2 = FT4222_SPIMaster_SingleWrite(ftdevHandle, &sendBuf_2[0], 3, &sizeTransferred, true);

    if(ft4222_status1 != FT4222_OK || ft4222_status2 != FT4222_OK)
    {
        return ret_FAIL;
    }

    return ret_PASS;
}

ErrType_t spi_master_isp_reset_burstmode()
{
    uint8_t sendBuf_1[3] = {0xF2, 0x13, 0x00};
    uint8_t sendBuf_2[3] = {0xF2, 0x0D, 0x00};
    uint16 sizeTransferred;
    FT_STATUS  ft4222_status1, ft4222_status2;

    dbg_printf("spi_master_isp_reset_burstmode \n");

    sizeTransferred = 0;
    ft4222_status1 = FT4222_SPIMaster_SingleWrite(ftdevHandle, &sendBuf_1[0], 3, &sizeTransferred, true);
    ft4222_status2 = FT4222_SPIMaster_SingleWrite(ftdevHandle, &sendBuf_2[0], 3, &sizeTransferred, true);

    if(ft4222_status1 != FT4222_OK || ft4222_status2 != FT4222_OK)
    {
        return ret_FAIL;
    }

    return ret_PASS;
}

ErrType_t spi_master_isp_burst_write(unsigned int reg_addr, unsigned int data_size, unsigned char *data)
{
    if((data_size % 4) != 0)
    {
        dbg_printf("SPI ISP data must be 4 alignment\n");
        return ret_FAIL;
    }

    uint8_t *buf = new uint8_t[6+data_size];

    buf[0] = 0xF2;
    buf[1] = 0x00;
    buf[2] = (reg_addr & 0xFF);
    buf[3] = ((reg_addr>>8) & 0xFF);
    buf[4] = ((reg_addr>>16) & 0xFF);
    buf[5] = ((reg_addr>>24) & 0xFF);

    memcpy(&buf[6], data, data_size);

    uint16 sizeTransferred;
    FT_STATUS  ft4222_status;

    sizeTransferred = 0;
    ft4222_status = FT4222_SPIMaster_SingleWrite(ftdevHandle, &buf[0], 6+data_size, &sizeTransferred, true);

    delete [] buf;

    if(ft4222_status != FT4222_OK)
    {
        return ret_FAIL;
    }

    return ret_PASS;
}

ErrType_t spi_master_isp_burst_read(unsigned int reg_addr, unsigned int data_size, unsigned char *data)
{
    if((data_size % 4) != 0)
    {
        dbg_printf("SPI ISP data must be 4 alignment\n");
        return ret_FAIL;
    }

    uint8_t cmd_1[6];

    cmd_1[0] = 0xF2;
    cmd_1[1] = 0x00;
    cmd_1[2] = (reg_addr & 0xFF);
    cmd_1[3] = ((reg_addr>>8) & 0xFF);
    cmd_1[4] = ((reg_addr>>16) & 0xFF);
    cmd_1[5] = ((reg_addr>>24) & 0xFF);

    uint8_t cmd_2[3];

    cmd_2[0] = 0xF2;
    cmd_2[1] = 0x0C;
    cmd_2[2] = 0x00;

    uint32_t buf_length = 3+data_size;
    uint8_t *write_buf = new uint8_t[buf_length];
    uint8_t *read_buf = new uint8_t[buf_length];

    memset(write_buf, 0x00, 3+data_size);
    memset(read_buf, 0xFF, 3+data_size);

    write_buf[0] = 0xF3;
    write_buf[1] = 0x08;

    uint16 sizeTransferred = 0;
    FT_STATUS  ft4222_status1, ft4222_status2, ft4222_status3;

    sizeTransferred = 0;
    ft4222_status1 = FT4222_SPIMaster_SingleWrite(ftdevHandle, &cmd_1[0], 6, &sizeTransferred, true);
    ft4222_status2 = FT4222_SPIMaster_SingleWrite(ftdevHandle, &cmd_2[0], 3, &sizeTransferred, true);
    ft4222_status3 = FT4222_SPIMaster_SingleReadWrite(ftdevHandle, &read_buf[0], &write_buf[0],
            buf_length, &sizeTransferred, true);

//    ft4222_status3 = FT4222_SPIMaster_SingleWrite(ftdevHandle, &read_buf[0], buf_length, &sizeTransferred, true);

    memcpy(data, &read_buf[3], data_size);

    dbg_printf("0x%02X 0x%02X 0x%02X 0x%02X", read_buf[3], read_buf[4], read_buf[5], read_buf[6]);

    delete [] write_buf;
    delete [] read_buf;

    if(ft4222_status1 != FT4222_OK || ft4222_status2 != FT4222_OK || ft4222_status3 != FT4222_OK)
    {
        return ret_FAIL;
    }

    return ret_PASS;
}

ErrType_t spi_master_isp_single_write(unsigned int reg_addr, unsigned int *data)
{
    uint8_t cmd[12][3] = {
        {0xF2, 0x0D, 0x00},
        {0xF2, 0x00, 0x00},
        {0xF2, 0x01, 0x00},
        {0xF2, 0x02, 0x00},
        {0xF2, 0x03, 0x00},
        {0xF2, 0x04, 0x00},
        {0xF2, 0x05, 0x00},
        {0xF2, 0x06, 0x00},
        {0xF2, 0x07, 0x00},
        {0xF2, 0x0C, 0x01},
        {0xF2, 0x13, 0x31},
        {0xF2, 0x0D, 0x11},
    };

    cmd[1][2] = (reg_addr & 0xFF);
    cmd[2][2] = ((reg_addr >> 8) & 0xFF);
    cmd[3][2] = ((reg_addr >> 16) & 0xFF);
    cmd[4][2] = ((reg_addr >> 24) & 0xFF);
    cmd[5][2] = (*data & 0xFF);
    cmd[6][2] = ((*data >> 8) & 0xFF);
    cmd[7][2] = ((*data >> 16) & 0xFF);
    cmd[8][2] = ((*data >> 24) & 0xFF);


    uint16 sizeTransferred;
    FT_STATUS  ft4222_status;

    for(uint32_t i = 0; i < 12; i++)
    {
        sizeTransferred = 0;
        ft4222_status = FT4222_SPIMaster_SingleWrite(ftdevHandle, &cmd[i][0], 3, &sizeTransferred, true);

        if(ft4222_status != FT4222_OK)
        {
            dbg_printf("spi_master_isp_single_write: 0x%02X 0x%02X 0x%02X fail\n",
                       cmd[i][0], cmd[i][1], cmd[i][2]);
            return ret_FAIL;
        }
    }

    return ret_PASS;
}

ErrType_t spi_master_isp_single_read(unsigned int reg_addr, unsigned int *data)
{
    uint8_t cmd[13][4] = {
        {0xF2, 0x13, 0x00, 0x00},
        {0xF2, 0x0D, 0x00, 0x00},
        {0xF2, 0x00, 0x00, 0x00},
        {0xF2, 0x01, 0x00, 0x00},
        {0xF2, 0x02, 0x00, 0x00},
        {0xF2, 0x03, 0x00, 0x00},
        {0xF2, 0x0C, 0x00, 0x00},
        {0xF3, 0x08, 0x00, 0x00},
        {0xF3, 0x09, 0x00, 0x00},
        {0xF3, 0x0A, 0x00, 0x00},
        {0xF3, 0x0B, 0x00, 0x00},
        {0xF2, 0x13, 0x31, 0x00},
        {0xF2, 0x0D, 0x11, 0x00},
    };

    cmd[2][2] = (reg_addr & 0xFF);
    cmd[3][2] = ((reg_addr >> 8) & 0xFF);
    cmd[4][2] = ((reg_addr >> 16) & 0xFF);
    cmd[5][2] = ((reg_addr >> 24) & 0xFF);

    uint8_t cmd_length[13] = {3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 3, 3};
    uint8_t r_buf[4];
    uint8_t val[4];


    memset((uint8_t *)data, 0xFF, 4);

    uint16 sizeTransferred = 0;
    FT_STATUS  ft4222_status;
    uint8_t pos_cnt = 0;

    for(uint32_t i = 0; i<13; i++)
    {
        memset(&r_buf[0], 0xFF, 4);
        sizeTransferred = 0;
        ft4222_status = FT4222_SPIMaster_SingleReadWrite(ftdevHandle, &r_buf[0], &cmd[i][0],
                cmd_length[i], &sizeTransferred, true);

        if(ft4222_status != FT4222_OK)
        {
            dbg_printf("spi_master_isp_single_read: 0x%02X 0x%02X 0x%02X fail\n",
                       cmd[i][0], cmd[i][1], cmd[i][2]);
            return ret_FAIL;
        }

        if(i >= 7 && i <= 10)
        {
            val[pos_cnt] = r_buf[3];
            pos_cnt++;
        }
    }


    dbg_printf("0x%02X 0x%02X 0x%02X 0x%02X", val[0], val[1], val[2], val[3]);

    memcpy((uint8_t *)data, &val[0], 4);


    return ret_PASS;
}

ErrType_t spi_master_isp_single_rw_init_exp()
{
    uint8_t cmd[2][3] = {
//        {0xF2, 0x13, 0x00},
//        {0xF2, 0x0D, 0x00},
//        {0xF2, 0x13, 0x11},
//        {0xF2, 0x0D, 0x10},
//        {0xF2, 0x13, 0x31},
//        {0xF2, 0x0D, 0x10},
//        {0xF2, 0x13, 0x01},
//        {0xF2, 0x0D, 0x11},
//        {0xF2, 0x13, 0x11},
//        {0xF2, 0x0D, 0x11},
        {0xF2, 0x13, 0x21},
        {0xF2, 0x0D, 0x11},
    };

    uint16 sizeTransferred;
    FT_STATUS  ft4222_status;

    for(uint32_t i = 0; i < 2; i++)
    {
        sizeTransferred = 0;
        ft4222_status = FT4222_SPIMaster_SingleWrite(ftdevHandle, &cmd[i][0], 3, &sizeTransferred, true);

        if(ft4222_status != FT4222_OK)
        {
            dbg_printf("spi_master_isp_single_write: 0x%02X 0x%02X 0x%02X fail\n",
                       cmd[i][0], cmd[i][1], cmd[i][2]);
            return ret_FAIL;
        }
    }

    return ret_PASS;
}

ErrType_t spi_master_isp_single_write_exp(unsigned int reg_addr, unsigned int *data)
{
    uint8_t cmd[9][3] = {
        {0xF2, 0x00, 0x00},
        {0xF2, 0x01, 0x00},
        {0xF2, 0x02, 0x00},
        {0xF2, 0x03, 0x00},
        {0xF2, 0x04, 0x00},
        {0xF2, 0x05, 0x00},
        {0xF2, 0x06, 0x00},
        {0xF2, 0x07, 0x00},
        {0xF2, 0x0C, 0x01},
    };

    cmd[0][2] = (reg_addr & 0xFF);
    cmd[1][2] = ((reg_addr >> 8) & 0xFF);
    cmd[2][2] = ((reg_addr >> 16) & 0xFF);
    cmd[3][2] = ((reg_addr >> 24) & 0xFF);
    cmd[4][2] = (*data & 0xFF);
    cmd[5][2] = ((*data >> 8) & 0xFF);
    cmd[6][2] = ((*data >> 16) & 0xFF);
    cmd[7][2] = ((*data >> 24) & 0xFF);


    uint16 sizeTransferred;
    FT_STATUS  ft4222_status;

    for(uint32_t i = 0; i < 9; i++)
    {
        sizeTransferred = 0;
        ft4222_status = FT4222_SPIMaster_SingleWrite(ftdevHandle, &cmd[i][0], 3, &sizeTransferred, true);

        if(ft4222_status != FT4222_OK)
        {
            dbg_printf("spi_master_isp_single_write: 0x%02X 0x%02X 0x%02X fail\n",
                       cmd[i][0], cmd[i][1], cmd[i][2]);
            return ret_FAIL;
        }
    }

    return ret_PASS;
}

ErrType_t spi_master_isp_single_read_exp(unsigned int reg_addr, unsigned int *data)
{
    uint8_t cmd[9][4] = {
        {0xF2, 0x00, 0x00, 0x00},
        {0xF2, 0x01, 0x00, 0x00},
        {0xF2, 0x02, 0x00, 0x00},
        {0xF2, 0x03, 0x00, 0x00},
        {0xF2, 0x0C, 0x00, 0x00},
        {0xF3, 0x08, 0x00, 0x00},
        {0xF3, 0x09, 0x00, 0x00},
        {0xF3, 0x0A, 0x00, 0x00},
        {0xF3, 0x0B, 0x00, 0x00},
    };

    cmd[0][2] = (reg_addr & 0xFF);
    cmd[1][2] = ((reg_addr >> 8) & 0xFF);
    cmd[2][2] = ((reg_addr >> 16) & 0xFF);
    cmd[3][2] = ((reg_addr >> 24) & 0xFF);

    uint8_t cmd_length[9] = {3, 3, 3, 3, 3, 4, 4, 4, 4};
    uint8_t r_buf[4];
    uint8_t val[4];


    memset((uint8_t *)data, 0xFF, 4);

    uint16 sizeTransferred = 0;
    FT_STATUS  ft4222_status;
    uint8_t pos_cnt = 0;

    for(uint32_t i = 0; i<9; i++)
    {
        memset(&r_buf[0], 0xFF, 4);
        sizeTransferred = 0;
        ft4222_status = FT4222_SPIMaster_SingleReadWrite(ftdevHandle, &r_buf[0], &cmd[i][0],
                cmd_length[i], &sizeTransferred, true);

        if(ft4222_status != FT4222_OK)
        {
            dbg_printf("spi_master_isp_single_read: 0x%02X 0x%02X 0x%02X fail\n",
                       cmd[i][0], cmd[i][1], cmd[i][2]);
            return ret_FAIL;
        }

        if(i >= 5 && i <= 8)
        {
            val[pos_cnt] = r_buf[3];
            pos_cnt++;
        }
    }


    dbg_printf("0x%02X 0x%02X 0x%02X 0x%02X", val[0], val[1], val[2], val[3]);

    memcpy((uint8_t *)data, &val[0], 4);


    return ret_PASS;
}

ErrType_t spi_master_isp_1byte_write(unsigned char reg_addr, unsigned char data)
{
    uint16 sizeTransferred;
    FT_STATUS  ft4222_status;

    uint32_t buf_length = 3;
    uint8_t *write_buf = new uint8_t[buf_length];

    uint8_t *read_buf = new uint8_t[buf_length];
    memset(read_buf, 0xFF, buf_length);

    write_buf[0] = 0xF2;
    write_buf[1] = reg_addr;
    write_buf[2] = data;

    sizeTransferred = 0;
    ft4222_status = FT4222_SPIMaster_SingleWrite(ftdevHandle, &write_buf[0], buf_length, &sizeTransferred, true);
//    ft4222_status = FT4222_SPIMaster_SingleReadWrite(ftdevHandle, &read_buf[0], &write_buf[0],
//            buf_length, &sizeTransferred, true);

//    dbg_printf("0x%02X 0x%02X 0x%02X\n", read_buf[0], read_buf[1], read_buf[2]);

    if(ft4222_status != FT4222_OK)
    {
        return ret_FAIL;
    }

    return ret_PASS;
}

ErrType_t spi_master_isp_1byte_read(unsigned char reg_addr, unsigned char *data)
{
    uint16 sizeTransferred;
    FT_STATUS  ft4222_status;

    uint32_t buf_length = 4;
    uint8_t *write_buf = new uint8_t[buf_length];

    uint8_t *read_buf = new uint8_t[buf_length];
    memset(read_buf, 0xFF, buf_length);

    write_buf[0] = 0xF3;
    write_buf[1] = reg_addr;
    write_buf[2] = 0x00;
    write_buf[3] = 0x00;

    sizeTransferred = 0;
    ft4222_status = FT4222_SPIMaster_SingleReadWrite(ftdevHandle, &read_buf[0], &write_buf[0],
            buf_length, &sizeTransferred, true);

    dbg_printf("0x%02X 0x%02X 0x%02X 0x%02X\n", read_buf[0], read_buf[1], read_buf[2], read_buf[3]);

    *data = read_buf[3];

    if(ft4222_status != FT4222_OK)
    {
        return ret_FAIL;
    }

    return ret_PASS;
}

// 46875(24M/512) ~80000000(80M/1)
void calculate_clock(unsigned int clock, FT4222_SPIClock div, FT4222_ClockRate clk)
{
    if (clock > MAX_SPI_CLOCK)
    {
        div = CLK_NONE;
        clk = SYS_CLK_80;
    }
    else if (clock <= MIN_SPI_CLOCK)
    {
        div = CLK_DIV_512;
        clk = SYS_CLK_24;
    }
    else
    {
    }

    dbg_printf("calculate_clock: %d, %d \n", div, clk);
}

////////////////SPI flash//////////////////////////////////////////
ErrType_t spi_flash_init(SPIMasterClock_t clock)
{
    return DRV_spi_flash_init(ftdevHandle, clock);
}


ErrType_t spi_flash_change_mode(FlashMode_t mode)
{
//    FT_STATUS ftStatus;
//    unsigned char wip_status = 0;
    std::vector<unsigned char> sendBuf;
    std::vector<unsigned char> recvBuf;
//    uint16 sizeTransferred = 0;
//    uint32 longsizeTransferred = 0;
    //check if initial

    //
    switch (mode)
    {
    case Mode_Single:

        //spi_flash_set_quad_mode(0);
		DRV_spi_flash_set_quad_mode(ftdevHandle,0);

        g_flash_read_mode = Mode_Single;
        break;
    case Mode_Dual:

        //spi_flash_set_quad_mode(0);
		DRV_spi_flash_set_quad_mode(ftdevHandle,0);

        g_flash_read_mode = Mode_Dual;
        break;
    case Mode_Quad:

        //spi_flash_set_quad_mode(1);
		DRV_spi_flash_set_quad_mode(ftdevHandle,1);

        g_flash_read_mode = Mode_Quad;

        break;

    default:
        break;
    }

    return ret_PASS;
}

ErrType_t spi_flash_read_info(unsigned char *info)
{
	DRV_spi_flash_read_info(ftdevHandle, info);
    return ret_PASS;
}



ErrType_t spi_flash_clear_protect(void)
{
	DRV_spi_flash_clear_protect(ftdevHandle);
    return ret_PASS;
}



ErrType_t spi_flash_read_data(unsigned int data_addr, unsigned int data_size, unsigned char *data)
{
	return DRV_spi_flash_read_data(ftdevHandle, data_addr, data_size, data);
}

ErrType_t spi_flash_write_data(unsigned int data_addr, unsigned int data_size, unsigned char *data)
{
    if (data_size == 0)
        return ret_ErrParameter;

    if (data == NULL)
        return ret_ErrParameter;

    f_info.ftHandle = ftdevHandle;

    f_info.data_size = data_size;
    f_info.addr = data_addr;
    //append with sizeShort
    f_info.rawData.resize(data_size);
    //copy data
    memcpy(&(f_info.rawData[0]), data, data_size);

    hth_spi_flash = (HANDLE)_beginthreadex(NULL, 0, threadSpiMasterWriteFlaash, &f_info, 0, &thID_spi_flash);

    return ret_PASS;
}

unsigned short spi_flash_write_percent_check()
{
    return flash_write_percent;
}


ErrType_t spi_flash_erase_data(unsigned int start_addr, unsigned int data_size)
{
	

    return DRV_spi_flash_erase_data(ftdevHandle, start_addr, data_size);
}

ErrType_t spi_flash_erase_all()
{
    return DRV_spi_flash_erase_all(ftdevHandle);
}

bool spi_flash_busy_check()
{
    return DRV_spi_flash_busy_check(ftdevHandle);
}

//////////////////////////////////// I2C ///////////////////////////////////

ErrType_t i2c_master_init(int speed)
{
    FT_STATUS ftStatus;

    dbg_printf("Init FT4222 I2C MASTER CLK:%dK!!!\n", speed);

    if (speed == 0)
        speed = 400;

    ftStatus = FT4222_I2CMaster_Init(ftdevHandle, speed);
    if (FT_OK != ftStatus)
    {
        dbg_printf("Init FT4222 as I2C master device failed!");
        return ret_FAIL;
    }

    return ret_PASS;
}

unsigned int i2c_single_read(unsigned char id, unsigned char *addr)
{
    UINT16 sizeTransferred = 0;
    UINT8 ret_packet[2] = {0x0f, 0xff};
    unsigned int test_time;
    UINT8 no_timeout = 1;
    FT_STATUS ftStatus;

    read_packet[9] = 0xFF;
    read_packet[11] = 0xFF;
    read_packet[13] = 0xFF;
    read_packet[15] = 0xFF;


    // address size must be 4
    read_packet[1] = addr[0];
    read_packet[3] = addr[1];
    read_packet[5] = addr[2];
    read_packet[7] = addr[3];

#ifdef CHECK_AHBBUS_FIRST
    test_time = 0;
    while (1)
    {
        test_time++;
        ftStatus = FT4222_I2CMaster_WriteEx(ftdevHandle, id, START, ret_packet, 1, &sizeTransferred);

        ftStatus = FT4222_I2CMaster_ReadEx(ftdevHandle, id, Repeated_START | STOP, &ret_packet[1], 1, &sizeTransferred);

        if (ret_packet[1] == 0x0)
        {
            no_timeout = 1;
            break;
        }
        if (test_time >= 1000)
        {
            no_timeout = 0;
            break;
        }
    }

    if(no_timeout == 0)
    {
        return ((read_packet[15] << 24) | (read_packet[13] << 16) | (read_packet[11] << 8) | (read_packet[9] << 0));
    }
#endif

    //address
    ftStatus = FT4222_I2CMaster_Write(ftdevHandle, id, &read_packet[0], 2, &sizeTransferred);
    ftStatus = FT4222_I2CMaster_Write(ftdevHandle, id, &read_packet[2], 2, &sizeTransferred);
    ftStatus = FT4222_I2CMaster_Write(ftdevHandle, id, &read_packet[4], 2, &sizeTransferred);
    ftStatus = FT4222_I2CMaster_Write(ftdevHandle, id, &read_packet[6], 2, &sizeTransferred);


    read_ack_packet[0] = 0x0C;
    read_ack_packet[1] = 0x00;
    //
    ftStatus = FT4222_I2CMaster_Write(ftdevHandle, id, read_ack_packet, 2, &sizeTransferred);

#ifndef CHECK_AHBBUS_FIRST
    test_time = 0;
    while (1)
    {
        test_time++;
        ftStatus = FT4222_I2CMaster_WriteEx(ftdevHandle, id, START, ret_packet, 1, &sizeTransferred);

        ftStatus = FT4222_I2CMaster_ReadEx(ftdevHandle, id, Repeated_START | STOP, &ret_packet[1], 1, &sizeTransferred);

        if (ret_packet[1] == 0x0)
        {
            no_timeout = 1;
            break;
        }
        if (test_time >= 1000)
        {
            no_timeout = 0;
            break;
        }
    }
#endif

    if(no_timeout == 1)
    {
        //FT4222_I2CMaster_Read(ftdevHandle, id, &read_packet[8], 2, &sizeTransferred);
        FT4222_I2CMaster_WriteEx(ftdevHandle, id, START, &read_packet[8], 1, &sizeTransferred);
        FT4222_I2CMaster_ReadEx(ftdevHandle, id, Repeated_START | STOP, &read_packet[9], 1, &sizeTransferred);

        //FT4222_I2CMaster_Read(ftdevHandle, id, &read_packet[10], 2, &sizeTransferred);
        FT4222_I2CMaster_WriteEx(ftdevHandle, id, START, &read_packet[10], 1, &sizeTransferred);
        FT4222_I2CMaster_ReadEx(ftdevHandle, id, Repeated_START | STOP, &read_packet[11], 1, &sizeTransferred);

        //FT4222_I2CMaster_Read(ftdevHandle, id, &read_packet[12], 2, &sizeTransferred);
        FT4222_I2CMaster_WriteEx(ftdevHandle, id, START, &read_packet[12], 1, &sizeTransferred);
        FT4222_I2CMaster_ReadEx(ftdevHandle, id, Repeated_START | STOP, &read_packet[13], 1, &sizeTransferred);

        //FT4222_I2CMaster_Read(ftdevHandle, id, &read_packet[14], 2, &sizeTransferred);
        FT4222_I2CMaster_WriteEx(ftdevHandle, id, START, &read_packet[14], 1, &sizeTransferred);
        FT4222_I2CMaster_ReadEx(ftdevHandle, id, Repeated_START | STOP, &read_packet[15], 1, &sizeTransferred);
    }
    else
    {
        read_packet[9] = 0xFF;
        read_packet[11] = 0xFF;
        read_packet[13] = 0xFF;
        read_packet[15] = 0xFF;
    }

    return ((read_packet[15] << 24) | (read_packet[13] << 16) | (read_packet[11] << 8) | (read_packet[9] << 0));

    dbg_printf("i2c_single_read 0x%08X\n", (uint)ftStatus);
}

void i2c_single_write(unsigned char id, unsigned char *addr, unsigned char *data)
{
    UINT16 sizeTransferred = 0;
    UINT8 ret_packet[2] = {0x0f, 0xff};
    unsigned int test_time;

    // address size must be 4
    write_packet[1] = addr[0];
    write_packet[3] = addr[1];
    write_packet[5] = addr[2];
    write_packet[7] = addr[3];

    //data size must be 4
    write_packet[9] = data[0];
    write_packet[11] = data[1];
    write_packet[13] = data[2];
    write_packet[15] = data[3];

    //address
    FT4222_I2CMaster_Write(ftdevHandle, id, write_packet, 2, &sizeTransferred);
    FT4222_I2CMaster_Write(ftdevHandle, id, write_packet + 2, 2, &sizeTransferred);
    FT4222_I2CMaster_Write(ftdevHandle, id, write_packet + 4, 2, &sizeTransferred);
    FT4222_I2CMaster_Write(ftdevHandle, id, write_packet + 6, 2, &sizeTransferred);

    //data
    FT4222_I2CMaster_Write(ftdevHandle, id, write_packet + 8, 2, &sizeTransferred);
    FT4222_I2CMaster_Write(ftdevHandle, id, write_packet + 10, 2, &sizeTransferred);
    FT4222_I2CMaster_Write(ftdevHandle, id, write_packet + 12, 2, &sizeTransferred);
    FT4222_I2CMaster_Write(ftdevHandle, id, write_packet + 14, 2, &sizeTransferred);

    //
    FT4222_I2CMaster_Write(ftdevHandle, id, write_ack_packet, 2, &sizeTransferred);

    test_time = 0;
    while (1)
    {
        test_time++;
        FT4222_I2CMaster_WriteEx(ftdevHandle, id, START, ret_packet, 1, &sizeTransferred);

        FT4222_I2CMaster_ReadEx(ftdevHandle, id, Repeated_START | STOP, ret_packet + 1, 1, &sizeTransferred);

        if (ret_packet[1] == 0x0)
            break;
        if (test_time >= 1000)
            break;
    }

}

void I2C_burst_read(UINT8 id, UINT8 *addr, UINT32 addr_size, UINT8 *data, UINT32 data_size)
{
    UINT8 packet[8], count = 0;
    UINT16 sizeTransferred = 0;
    FT_STATUS ftStatus;

    for (UINT32 i = 0; i < addr_size; i++)
    {
        packet[i] = addr[i];
        count += 1;
    }

    //address
    ftStatus = FT4222_I2CMaster_WriteEx(ftdevHandle, id, START, packet, count, &sizeTransferred);

    if (FT4222_OK == ftStatus)
    {
        // write data success
        dbg_printf("I2C_burst_read: write done!!\n");
    }
    else
    {
        // write data failed
        dbg_printf("I2C_burst_read: write fail!!\n");
    }

    ftStatus = FT4222_I2CMaster_ReadEx(ftdevHandle, id, Repeated_START | STOP, data, data_size, &sizeTransferred);

    if (FT4222_OK == ftStatus)
    {
        // read data success
        dbg_printf("I2C_burst_read: read done!!\n");
    }
    else
    {
        // read data failed
        dbg_printf("I2C_burst_read: read fail!!\n");
    }

    return;
}

void I2C_burst_write(UINT8 id, UINT8 *addr, UINT32 addr_size, UINT8 *data, UINT32 data_size)
{
    //UINT8 packet[8], count=0;
    UINT8 *packet;
    UINT32 count = 0;
    UINT16 sizeTransferred = 0;
    FT_STATUS ftStatus;

    packet = new UINT8[addr_size + data_size];

    for (UINT32 i = 0; i < addr_size; i++)
    {

        packet[i] = addr[i];
        count += 1;
    }

    for (UINT32 j = 0; j < data_size; j++)
    {
        packet[j + count] = data[j];
        count += 1;
    }

    ftStatus = FT4222_I2CMaster_Write(ftdevHandle, id, packet, count, &sizeTransferred);

    if (FT4222_OK == ftStatus)
    {
        // write data success
        dbg_printf("I2C_burst_write: write done!!\n");
    }
    else
    {
        // write data failed
        dbg_printf("I2C_burst_write: write fail!!\n");
    }

    delete packet;

    return;
}

void I2C_burst_read_no_addr(UINT8 id, UINT8 *data, UINT32 data_size)
{
    UINT16 sizeTransferred = 0;
    FT_STATUS ftStatus;

    ftStatus = FT4222_I2CMaster_ReadEx(ftdevHandle, id, START | STOP, data, data_size, &sizeTransferred);

    if (FT4222_OK == ftStatus)
    {
        // read data success
        dbg_printf("I2C_burst_read_no_addr: read done!!\n");
    }
    else
    {
        // read data failed
        dbg_printf("I2C_burst_read_no_addr: read fail!!\n");
    }

    return;
}

void I2C_burst_write_no_addr(UINT8 id, UINT8 *data, UINT32 data_size)
{
    //UINT8 packet[8], count=0;
    UINT8 *packet;
    UINT32 count = 0;
    UINT16 sizeTransferred = 0;
    FT_STATUS ftStatus;

    packet = new UINT8[data_size];

    for (uint32_t j = 0; j < data_size; j++)
    {
        packet[j + count] = data[j];
        count += 1;
    }

    ftStatus = FT4222_I2CMaster_Write(ftdevHandle, id, packet, count, &sizeTransferred);
    //ftStatus = FT4222_I2CMaster_WriteEx(ftdevHandle, id, START | STOP, data, data_size, &sizeTransferred);

    if (FT4222_OK == ftStatus)
    {
        // write data success
        dbg_printf("I2C_burst_write_no_addr: write done!!\n");
    }
    else
    {
        // write data failed
        dbg_printf("I2C_burst_write_no_addr: write fail!!\n");
    }

    delete packet;

    return;
}

void I2C_cmd_read(UINT8 id, UINT8 *data, UINT32 data_size)
{
    UINT16 sizeTransferred = 0;
    FT_STATUS ftStatus;

    ftStatus = FT4222_I2CMaster_ReadEx(ftdevHandle, id, START | STOP, data, data_size, &sizeTransferred);

    if (FT4222_OK == ftStatus)
    {
        // read data success
        dbg_printf("I2C_cmd_read: read done!!\n");
    }
    else
    {
        // read data failed
        dbg_printf("I2C_cmd_read: read fail!!\n");
    }

    return;
}

void I2C_cmd_read_ex(UINT8 id, UINT8 *data, UINT32 data_size, UINT8 flag)
{
    UINT16 sizeTransferred = 0;
    FT_STATUS ftStatus;

    ftStatus = FT4222_I2CMaster_ReadEx(ftdevHandle, id, flag, data, data_size, &sizeTransferred);

    if (FT4222_OK == ftStatus)
    {
        // read data success
        dbg_printf("I2C_cmd_read_ex: read done!!\n");
    }
    else
    {
        // read data failed
        dbg_printf("I2C_cmd_read_ex: read fail!!\n");
    }

    return;
}

void I2C_cmd_write(UINT8 id, UINT8 *data, UINT32 data_size)
{
//    UINT32 count = 0;
    UINT16 sizeTransferred = 0;
    FT_STATUS ftStatus = 0x78;

    ftStatus = FT4222_I2CMaster_Write(ftdevHandle, id, data, data_size, &sizeTransferred);
    //	ftStatus = FT4222_I2CMaster_WriteEx(ftdevHandle, id, START | STOP, data, data_size, &sizeTransferred);

    if (FT4222_OK == ftStatus)
    {
        // write data success
        dbg_printf("I2C_cmd_write: write done!!\n");
    }
    else
    {
        // write data failed
        dbg_printf("I2C_cmd_write: write fail!!\n");
    }
}

void I2C_cmd_write_ex(UINT8 id, UINT8 *data, UINT32 data_size, UINT8 flag)
{
//    UINT32 count = 0;
    UINT16 sizeTransferred = 0;
    FT_STATUS ftStatus = 0x78;

#if 0
    dbg_printf("I2C_cmd_write packet = \n");
    for (int i =0; i <data_size; i++)
    {
        dbg_printf("	%x ", data[i]);
    }
    dbg_printf("\n");
#endif

    //ftStatus = FT4222_I2CMaster_Write(ftdevHandle, id, data, data_size, &sizeTransferred);
    ftStatus = FT4222_I2CMaster_WriteEx(ftdevHandle, id, flag, data, data_size, &sizeTransferred);

    if (FT4222_OK == ftStatus)
    {
        // write data success
        dbg_printf("I2C_cmd_write_ex: write done!!\n");
    }
    else
    {
        // write data failed
        dbg_printf("I2C_cmd_write_ex: write fail!!\n");
    }
}

BOOL GPIO_Read(UINT8 pin_no)
{
    BOOL gpio_val = 0;
    GPIO_Dir gpioDir[4];
    gpioDir[0] = GPIO_OUTPUT;
    gpioDir[1] = GPIO_OUTPUT;
    gpioDir[2] = GPIO_INPUT;
    gpioDir[3] = GPIO_INPUT;

    FT_STATUS ftStatus = 0x78;
    ftStatus = FT4222_GPIO_Init(ftdevGPIOHandle, gpioDir);
    //dbg_printf("FT4222_GPIO_Init (%d)\n", ftStatus);

    //disable suspend out , enable gpio 2
    ftStatus = 0x78;
    ftStatus = FT4222_SetSuspendOut(ftdevGPIOHandle, false);
    //dbg_printf("FT4222_SetSuspendOut (%d)\n", ftStatus);

    //disable interrupt , enable gpio 3
    ftStatus = FT4222_SetWakeUpInterrupt(ftdevGPIOHandle, 0);

    ftStatus = 0x78;
    ftStatus = FT4222_GPIO_Read(ftdevGPIOHandle, (GPIO_Port) pin_no, &gpio_val);

    dbg_printf("GPIO_Read 0x%08X\n", (uint)ftStatus);

    return  gpio_val; 
}

void GPIO_Write(UINT8 pin_no, BOOL bValue)
{
//    BOOL gpio_val = 0;
    GPIO_Dir gpioDir[4];
    gpioDir[0] = GPIO_OUTPUT;
    gpioDir[1] = GPIO_OUTPUT;
    gpioDir[2] = GPIO_OUTPUT;
    gpioDir[3] = GPIO_OUTPUT;

    FT_STATUS ftStatus = 0x78;
    ftStatus = 0x78;
    ftStatus = FT4222_GPIO_Init(ftdevGPIOHandle, gpioDir);
    //dbg_printf("FT4222_GPIO_Init (%d)\n", ftStatus);

    //disable suspend out , enable gpio 2
    ftStatus = 0x78;
    ftStatus = FT4222_SetSuspendOut(ftdevGPIOHandle, false);
    //dbg_printf("FT4222_SetSuspendOut (%d)\n", ftStatus);

    //disable interrupt , enable gpio 3
    ftStatus = FT4222_SetWakeUpInterrupt(ftdevGPIOHandle, 0);

    ftStatus = 0x78;
    ftStatus = FT4222_GPIO_Write(ftdevGPIOHandle, (GPIO_Port) pin_no, bValue);

    dbg_printf("GPIO_Write (0x%08X)\n", (uint)ftStatus);
}

void I2C_cmd_write_hs(UINT8 id, UINT8 *data, UINT32 data_size)
{
    UINT32 count = 0;
    UINT16 sizeTransferred = 0;
    FT_STATUS ftStatus = 0x78;

    BOOL gpio_val = 0;
    GPIO_Dir gpioDir[4];
    gpioDir[0] = GPIO_OUTPUT;
    gpioDir[1] = GPIO_OUTPUT;
    gpioDir[2] = GPIO_INPUT;
    gpioDir[3] = GPIO_INPUT;

    ftStatus = 0x78;
    ftStatus = FT4222_GPIO_Init(ftdevGPIOHandle, gpioDir);
    //dbg_printf("FT4222_GPIO_Init (%d)\n", ftStatus);

    //disable suspend out , enable gpio 2
    ftStatus = 0x78;
    ftStatus = FT4222_SetSuspendOut(ftdevGPIOHandle, false);
    //dbg_printf("FT4222_SetSuspendOut (%d)\n", ftStatus);

    dbg_printf("\n");
    dbg_printf("Wait FT4222_GPIO[2] --> High level...(10 sec)\n");

    while (1)
    {
        ftStatus = 0x78;
        ftStatus = FT4222_GPIO_Read(ftdevGPIOHandle, GPIO_PORT2, &gpio_val);
        //dbg_printf("FT4222_GPIO[2]:%d (status:%d)\n", gpio_val, ftStatus);
        
        if(gpio_val == 1)
        {
            dbg_printf("\n");
            ftStatus = FT4222_I2CMaster_Write(ftdevHandle, id, data, data_size, &sizeTransferred);
            break;
        }

        count++;
        Sleep(1); /*1ms*/

        if (count > 10000)
        {
            dbg_printf("Wait timeout...\n");
            break;
        }
    }

    dbg_printf("I2C_cmd_write_hs (0x%08X)\n", (uint)ftStatus);
}

ErrType_t FT4222_SPIRESET(void)
{
    FT4222_STATUS ft4222Status;

    dbg_printf("=======> FT4222_SPIRESET start\n ");

    ft4222Status = FT4222_SPI_ResetTransaction(ftdevHandle, 0);
    if (FT4222_OK == ft4222Status)
    {
        // chip has been reset
        dbg_printf("=======> FT4222_SPI_ResetTransactio done\n ");
        return ret_PASS;
    }
    else
    {
        // chip reset failed
        dbg_printf("!!!!!===> FT4222_SPI_ResetTransactio fail\n ");
        return ret_FAIL;
    }
}

ErrType_t FT4222_CHIPRESET(void)
{
    FT4222_STATUS ft4222Status/*, ft4222Status2*/;

    dbg_printf("=======> FT4222_CHIPRESET start\n ");

    ft4222Status = FT4222_ChipReset(ftdevHandle);
//    ft4222Status2 = FT4222_ChipReset(ftdevGPIOHandle);
    if (FT4222_OK == ft4222Status)
    {
        // chip has been reset
        dbg_printf("=======> FT4222_CHIPRESET(ftdevHandle) done\n ");
        FT_Close(ftdevHandle);
        return ret_PASS;
    }
    else
    {
        // chip reset failed
        dbg_printf("!!!!!===> FT4222_CHIPRESET(ftdevHandle) fail\n ");
        return ret_FAIL;
    }

//    if (FT4222_OK == ft4222Status2)
//    {
//        // chip has been reset
//        dbg_printf("=======> FT4222_CHIPRESET (ftdevGPIOHandle) done\n ");
//        FT_Close(ftdevGPIOHandle);
//        return ret_PASS;
//    }
//    else
//    {
//        // chip reset failed
//        dbg_printf("!!!!!===> FT4222_CHIPRESET (ftdevGPIOHandle) fail\n ");
//        return ret_FAIL;
//    }
}

ErrType_t FT4222_SPIRESETBUFFER(void)
{
    FT_STATUS ftStatus;

    dbg_printf("=======> FT4222_SPIRESETBUFFER start\n ");

    ftStatus = FT_Purge(ftdevHandle, FT_PURGE_RX | FT_PURGE_TX); // Purge both Rx and Tx buffers
    if (ftStatus == FT_OK)
    {
        dbg_printf("=======> FT4222_SPIRESETBUFFER done\n ");
        return ret_PASS;
    }
    else
    {
        dbg_printf("!!!!!===> FT4222_SPIRESETBUFFER fail\n ");
        return ret_FAIL;
    }
}
