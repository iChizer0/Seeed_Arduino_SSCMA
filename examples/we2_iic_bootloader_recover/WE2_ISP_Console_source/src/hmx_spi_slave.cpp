#include "hmx_spi_slave.h"
#include "hmx_common.h"

#include "ftd2xx.h"
#include "LibFT4222.h"

#include <vector>
#include <process.h>
#include <stdint.h>



using namespace std;

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


GPIO_Dir spis_gpio_group[4] = {GPIO_OUTPUT, GPIO_OUTPUT, GPIO_INPUT, GPIO_OUTPUT};
bool spi_slave_start_receive_flag = false;
bool spi_slave_start_send_flag = false;

spi_info_t spi_slave_receive_info, spi_slave_send_info;

unsigned int thID_spi_slave_receive;
HANDLE hth_spi_slave_receive;
extern HANDLE hth_spi_send;

static uint32_t frm_cnt = 0;
static uint32_t readpacket_cnt = 0;


// 2 sync bytes scenario
unsigned __stdcall spi_slave_receive_thd(void *arglist)
{
    uint16 sizeTransferred = 0;
    uint16 rxSize;

    //sizeShortUnion write_size;
    sizeUnion data_size_v2;
    bool header_flag = false;

    //Arg *p = (Arg *)arglist;
    spi_info_t *p = (spi_info_t *)arglist;

    //FT4222_SPISlave_Write(p->ftHandle, test_buffer, RESULT_BUFFER_SIZE, &sizeTransferred);
	p->recvBuf.clear();
	p->dataBuf.clear();
	p->rawData.clear();

    while (spi_slave_start_receive_flag)
    {
        //MAX QUEUE SIZE: 64K(65536)
        rxSize = 0;
        if (spi_slave_start_receive_flag && (FT4222_SPISlave_GetRxStatus(p->ftHandle /*ftdevHandle*/, &rxSize) == FT4222_OK))
        {
			if (rxSize > 0)
            {
                readpacket_cnt++;
                dbg_printf("|||||||||>SPI SLAVE RECV PACKET OK: %d\n", readpacket_cnt);

                p->recvBuf.resize(rxSize);

                if (FT4222_SPISlave_Read(p->ftHandle, &(p->recvBuf[0]), rxSize, &sizeTransferred) == FT4222_OK)
                {
                    if (sizeTransferred != rxSize)
                    {
                        dbg_printf("!!!!!!!!!>read data size is not equal %d != %d\n", rxSize, sizeTransferred);
                    }
                    else
                    {
                        // append data to dataBuf
                        p->dataBuf.insert(p->dataBuf.end(), p->recvBuf.begin(), p->recvBuf.end());
                    }
                }
                else
                {
                    dbg_printf("!!!!!!!!!>FT4222_SPISlave_Read error\n");
                }
            }
			//comment here, sleep here cause release version image crop
//			else
//				Sleep(1);

            if(p->dataBuf.size()>=SPI_HEADER_V2/*7*/ && header_flag == false)
			{
				//check if any sync byte inside
				while(spi_slave_start_receive_flag && p->dataBuf.size() >= 1)
				{
					if(p->dataBuf[SPI_SYNC_POS_V2] != SPI_SYNC_CMD1)
                    {
//						p->dataBuf.erase(p->dataBuf.begin());
                        p->dataBuf.clear();
                        dbg_printf("!!!!!!!!!>SPI_SYNC_CMD1 fail size: %d\n", p->dataBuf.size());
                        break;
                    }
                    else if(p->dataBuf.size() ==1)
                        break;
                    else if(p->dataBuf.size() >=2 && p->dataBuf[SPI_SYNC_POS_V2+1] != SPI_SYNC_CMD2)
                    {
                        //p->dataBuf.erase(p->dataBuf.begin(),p->dataBuf.begin()+2);
                        p->dataBuf.clear();
                        dbg_printf("!!!!!!!!!>SPI_SYNC_CMD2 fail size: %d\n", p->dataBuf.size());
                        break;
                    }
					else 
					{
						//p->dataBuf[0] == SPI_SYNC_CMD1
						//p->dataBuf[1] == SPI_SYNC_CMD2
						if(p->dataBuf.size() >= SPI_HEADER_V2)
						{
							*(p->datatype) = (DataType_t)p->dataBuf[SPI_DATA_TYPE_POS_V2];
							data_size_v2.myBytes.b1 = p->dataBuf[SPI_LENGTH_POS_V2];
							data_size_v2.myBytes.b2 = p->dataBuf[SPI_LENGTH_POS_V2+1];
							data_size_v2.myBytes.b3 = p->dataBuf[SPI_LENGTH_POS_V2+2];
							data_size_v2.myBytes.b4 = p->dataBuf[SPI_LENGTH_POS_V2+3];
							header_flag = true;
							p->dataBuf.erase(p->dataBuf.begin(), p->dataBuf.begin() + SPI_HEADER_V2);
						}

						break;
					}
				}

			}

			if( header_flag == true && (p->dataBuf.size() >= data_size_v2.sizeInt))
			{
                frm_cnt++;
                dbg_printf("|||||||||>SPI SLAVE RECV FRAME COUNT OK: %d\n", frm_cnt);

				header_flag = false;
				p->rawData.resize(data_size_v2.sizeInt);
				//copy data from start of complete packet
				memcpy(&(p->rawData[0]), &(p->dataBuf[0]), (data_size_v2.sizeInt));

				*(p->datasize) = p->rawData.size();

				p->rx_cb(0);

				p->dataBuf.erase(p->dataBuf.begin(), p->dataBuf.begin() + data_size_v2.sizeInt);
			}

		}
        else if(spi_slave_start_receive_flag)
        {
            dbg_printf("!!!!!!!!!>SPI SLAVE READ FAIL\n");
        }

    }

    //spi_finish_receive_flag = true;

    return 1;
}



bool DRV_spi_slave_init(void *handle, void *gpio_handle)
{
    FT_STATUS ftStatus;

    dbg_printf("DRV_spi_slave_init!\n");

    if ((FT_HANDLE)handle != NULL && gpio_handle != NULL)
    {
        dbg_printf("DRV_spi_slave_init!\n");

        ftStatus = FT4222_SetClock(handle, SYS_CLK_80);
        if (FT_OK != ftStatus)
        {
            dbg_printf("Set FT4222 clock to 80MHz failed!\n");
            return false;
        }

        ftStatus = FT4222_SPISlave_InitEx(handle, SPI_SLAVE_NO_PROTOCOL);
		//ftStatus = FT4222_SPISlave_Init(handle);
        if (FT_OK != ftStatus)
        {
            dbg_printf("Init FT4222 as SPI slave device failed!\n");
            return false;
        }

        ftStatus = FT4222_SPI_SetDrivingStrength(handle, DS_4MA, DS_4MA, DS_4MA);
        if (FT_OK != ftStatus)
        {
            dbg_printf("Set SPI Slave driving strength failed!\n");
            return false;
        }

        ftStatus = FT_SetUSBParameters(handle, /*4*/8 * 1024, 0);
        if (FT_OK != ftStatus)
        {
            dbg_printf("FT_SetUSBParameters failed!\n");
            return false;
        }

#if 1
        //need also to init GPIO-3 to output
        ftStatus = FT4222_GPIO_Init(gpio_handle, spis_gpio_group);
        if (FT_OK != ftStatus)
        {
            dbg_printf("set GPIO failed!\n");
            return false;
        }
        //disable suspend out , enable gpio 2
        ftStatus = FT4222_SetSuspendOut(gpio_handle, 0);
        if (FT_OK != ftStatus)
        {
            dbg_printf("set GPIO failed!\n");
            return false;
        }
        //disable interrupt , enable gpio 3
        ftStatus = FT4222_SetWakeUpInterrupt(gpio_handle, 0);
        if (FT_OK != ftStatus)
        {
            dbg_printf("set GPIO failed!\n");
            return false;
        }

        //set GPIO3 to low
        ftStatus = FT4222_GPIO_Write(gpio_handle, GPIO_PORT3, 0);
        if (FT_OK != ftStatus)
        {
            dbg_printf("set GPIO failed!\n");
            return false;
        }
#endif
    }
    else
        return false;

    dbg_printf("DRV_spi_slave_init done!\n");

    return true;
}


bool DRV_spi_slave_receive_data(void *handle, CB_FUNC_SPI_T rx_cb, unsigned int *data_size, DataType_t *data_type)
{
    if (spi_slave_start_receive_flag == true)
        return false;

    if (spi_slave_start_send_flag == true)
        return false;

//    if (FT4222_OK != FT4222_SPI_ResetTransaction(handle, 0))
//    {
//        //purge usb tx/rx and SPI FIFO cache
//        dbg_printf("!!!!!!!!!!>FT4222_SPI_ResetTransaction fail\n");
//        return false;
//    }

    spi_slave_start_receive_flag = true;

    *data_size = 0;
    *data_type = DATA_TYPE_INCORRECT_DATA;

    spi_slave_receive_info.ftHandle = handle;

    spi_slave_receive_info.datasize = data_size;
    spi_slave_receive_info.datatype = data_type;
    spi_slave_receive_info.rx_cb = rx_cb;

    //wait to receive
    hth_spi_slave_receive = (HANDLE)_beginthreadex(NULL, 0, spi_slave_receive_thd, &spi_slave_receive_info, 0, &thID_spi_slave_receive);

    return true;
}

bool DRV_spi_slave_halt(bool force)
{
//    if(spi_slave_start_receive_flag)
//        CloseHandle(hth_spi_slave_receive);

//    if(spi_slave_start_send_flag)
//        CloseHandle(hth_spi_send);

    spi_slave_start_receive_flag = false;
    spi_slave_start_send_flag = false;
    //todo - need to do force=TRUE part

//    spi_slave_receive_info.recvBuf.clear();
//    spi_slave_receive_info.dataBuf.clear();
//    spi_slave_receive_info.rawData.clear();

    //clear queue buffer


    frm_cnt = 0;
    readpacket_cnt = 0;

    dbg_printf("DRV_spi_slave_halt %d\n", force);

    return true;
}

bool DRV_spi_slave_read_packet(unsigned char *data)
{
    if (spi_slave_receive_info.rawData.size() != 0)
        memcpy(data, &(spi_slave_receive_info.rawData[0]), spi_slave_receive_info.rawData.size());
    else
        return false;

    return true;
}
