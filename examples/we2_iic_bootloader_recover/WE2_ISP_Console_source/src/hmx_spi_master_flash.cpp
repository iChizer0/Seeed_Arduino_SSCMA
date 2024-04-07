#include "hmx_lib.h"

#include "hmx_spi_master_flash.h"
#include "hmx_common.h"
#include "ftd2xx.h"
#include "LibFT4222.h"
#include <vector>
#include <process.h>

using namespace std;

#define SPI_VENDOR_SIZE_N(p,type) sizeof(p) / sizeof(type) 

extern int g_cur_flash_type;
extern unsigned char g_flash_read_mode;

SPI_FLASH_VENDOR_INFO_t flash_info[] = {
	{"DW_SPI_FLASH_VENDOR_MXIC",0xc2,VENDOR_MXIC},
	{"DW_SPI_FLASH_VENDOR_GIGADEV",0xc8,SUPPORT_SINGLE_VENDOR},
	{"DW_SPI_FLASH_VENDOR_WINBOND",0xef,VENDOR_WINBOND}
};

bool check_flash_quad_mode(unsigned char data);
bool spi_flash_set_quad_mode(bool enable);



ErrType_t DRV_spi_flash_read_info(void *handle, unsigned char *info)
{
    std::vector<unsigned char> recvBuf;
    std::vector<unsigned char> sendBuf;
    uint16 sizeTransferred = 0;
//    unsigned char wip_status = 0;
    FT_STATUS ft4222_status;
	int loop_count = 0;

    //
    //if(flash_read_mode != Mode_Single)
    {
        ft4222_status = FT4222_SPIMaster_SetLines((FT_HANDLE)handle, SPI_IO_SINGLE);
        if (FT4222_OK != ft4222_status)
        {
            //dbg_printf("set FT4222 as SPI master device failed!\n");
            return ret_FAIL;
        }
    }

    sendBuf.resize(1);
    recvBuf.resize(3);

    sendBuf[0] = 0x9F;

    ft4222_status = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle, &sendBuf[0], sendBuf.size(), &sizeTransferred, false);

    ft4222_status = FT4222_SPIMaster_SingleRead((FT_HANDLE)handle, &recvBuf[0], recvBuf.size(), &sizeTransferred, true);

	loop_count = SPI_VENDOR_SIZE_N(flash_info,SPI_FLASH_VENDOR_INFO_t);
	
	while(loop_count)
	{
		loop_count -= 1;
		if(recvBuf[0] == flash_info[loop_count].vendor_info)
			g_cur_flash_type = flash_info[loop_count].vendor_type;
	}


    /*if (recvBuf[0] == DW_SPI_FLASH_VENDOR_MXIC)
        cur_flash_type = VENDOR_MXIC;
    else if (recvBuf[0] == DW_SPI_FLASH_VENDOR_WINBOND)
        cur_flash_type = VENDOR_WINBOND;
    else
        cur_flash_type = UNKNOWN_VENDOR;*/

    if (info != NULL)
    {
        info[0] = recvBuf[0];
        info[1] = recvBuf[1];
        info[2] = recvBuf[2];

        //update flash info
    }

    return ret_PASS;
}

bool check_flash_quad_mode(unsigned char data)
{
    if (g_cur_flash_type == VENDOR_MXIC)
    {
        //MXIC
        if (data & 0x40)
            return true;
        else
            return false;
    }
    else if (g_cur_flash_type == VENDOR_WINBOND)
    {
        //Winbond
        if (data & 0x02)
            return true;
        else
            return false;
    }
	return false;
}

bool DRV_spi_flash_set_quad_mode(void *handle, bool enable)
{
    std::vector<unsigned char> sendBuf;
    std::vector<unsigned char> recvBuf;
    uint16 sizeTransferred = 0;
    unsigned char status_reg = 0;
    //read quad status
    FT_STATUS ftStatus;

    sendBuf.resize(1);
    recvBuf.resize(1);

    ftStatus = FT4222_SPIMaster_SetLines((FT_HANDLE)handle, SPI_IO_SINGLE);

    if (g_cur_flash_type == VENDOR_MXIC)
    {
        sendBuf[0] = 0x05;
        ftStatus = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle, &sendBuf[0], sendBuf.size(), &sizeTransferred, false);
        ftStatus = FT4222_SPIMaster_SingleRead((FT_HANDLE)handle, &recvBuf[0], recvBuf.size(), &sizeTransferred, true);
        status_reg = recvBuf[0];

        status_reg &= ~(DW_SPI_MXIC_FLASH_QUAD_ENABLE_MASK);
        status_reg = status_reg | (enable << DW_SPI_MXIC_FLASH_QUAD_ENABLE_POS);
    }
    else if (g_cur_flash_type == VENDOR_WINBOND)
    {
        sendBuf[0] = 0x35;
        ftStatus = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle, &sendBuf[0], sendBuf.size(), &sizeTransferred, false);
        ftStatus = FT4222_SPIMaster_SingleRead((FT_HANDLE)handle, &recvBuf[0], recvBuf.size(), &sizeTransferred, true);
        status_reg = recvBuf[0];

        status_reg &= ~(DW_SPI_WINBOND_FLASH_QUAD_ENABLE_MASK);
        status_reg = status_reg | (enable << DW_SPI_WINBOND_FLASH_QUAD_ENABLE_POS);
    }

    if (check_flash_quad_mode(recvBuf[0]) != enable)
    {
        sendBuf.resize(2);

        DRV_spi_flash_write_enable(handle);
        DRV_spi_flash_waitWIP(handle);

        if (g_cur_flash_type == VENDOR_MXIC)
        {
            sendBuf[0] = DW_SPI_FLASH_CMD_WRITE_STATUS;
            sendBuf[1] = status_reg;
        }
        else if (g_cur_flash_type == VENDOR_WINBOND)
        {
            sendBuf[0] = DW_SPI_FLASH_CMD_WRITE_STATUS2;
            sendBuf[1] = status_reg;
        }

        ftStatus = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle, &sendBuf[0], sendBuf.size(), &sizeTransferred, true);

        DRV_spi_flash_waitWIP(handle);
    }

    dbg_printf("DRV_spi_flash_set_quad_mode: 0x%08X\n", (uint)ftStatus);

    return TRUE;
}

ErrType_t DRV_spi_flash_waitWIP(void *handle)
{
    std::vector<unsigned char> sendBuf;
    std::vector<unsigned char> recvBuf;
    uint16 sizeTransferred = 0;
    unsigned char wip_status;
    FT_STATUS ft4222_status;

    sendBuf.resize(1);
    recvBuf.resize(1);

    ft4222_status = FT4222_SPIMaster_SetLines((FT_HANDLE)handle, SPI_IO_SINGLE);
    if (FT4222_OK != ft4222_status)
    {
        return ret_FAIL;
    }

    //wait WIP status
    do
    {
        //read status
        sendBuf[0] = 0x05;
        ft4222_status = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle, &sendBuf[0], sendBuf.size(), &sizeTransferred, false);
        ft4222_status = FT4222_SPIMaster_SingleRead((FT_HANDLE)handle, &recvBuf[0], recvBuf.size(), &sizeTransferred, true);
        wip_status = recvBuf[0] & 0x01;

        //if(total_size != 0)
        //	flash_write_percent = (total_size*100)/data_size;

    } while (wip_status);

    return ret_PASS;
}

ErrType_t DRV_spi_flash_write_enable(void *handle)
{
    std::vector<unsigned char> sendBuf;
    uint16 sizeTransferred = 0;
    FT_STATUS ft4222_status;

    sendBuf.resize(1);

    ft4222_status = FT4222_SPIMaster_SetLines((FT_HANDLE)handle, SPI_IO_SINGLE);
    if (FT4222_OK != ft4222_status)
    {
        return ret_FAIL;
    }

    //set write enable
    sendBuf[0] = 0x06;
    ft4222_status = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle, &sendBuf[0], sendBuf.size(), &sizeTransferred, true);

	return ret_PASS;
}

ErrType_t DRV_spi_flash_clear_protect(void *handle)
{
    std::vector<unsigned char> sendBuf;
	std::vector<unsigned char> recvBuf;
	uint16 sizeTransferred = 0;
    FT_STATUS ft4222_status;
	unsigned char wip_status;

    recvBuf.resize(1);

    sendBuf.resize(1);

    ft4222_status = FT4222_SPIMaster_SetLines((FT_HANDLE)handle, SPI_IO_SINGLE);
    if (FT4222_OK != ft4222_status)
    {
        return ret_FAIL;
    }

    //set write enable
    sendBuf[0] = 0x06;
    ft4222_status = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle, &sendBuf[0], sendBuf.size(), &sizeTransferred, true);

	//wait WIP status
    do
    {
        //read status
        sendBuf[0] = 0x05;
        ft4222_status = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle, &sendBuf[0], sendBuf.size(), &sizeTransferred, false);
        ft4222_status = FT4222_SPIMaster_SingleRead((FT_HANDLE)handle, &recvBuf[0], recvBuf.size(), &sizeTransferred, true);
        wip_status = recvBuf[0] & 0x01;

        //if(total_size != 0)
        //	flash_write_percent = (total_size*100)/data_size;

    } while (wip_status);

	sendBuf.resize(3);
	sendBuf[0] = 0x01;
	sendBuf[1] = 0x00;
	sendBuf[2] = 0x00;

	ft4222_status = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle, &sendBuf[0], sendBuf.size(), &sizeTransferred, true);


	return ret_PASS;
}

ErrType_t DRV_spi_flash_erase_all(void *handle)
{
	
    std::vector<unsigned char> sendBuf;
    std::vector<unsigned char> recvBuf;
    uint16 sizeTransferred = 0;
    FT_STATUS ft4222_status;
//    unsigned char wip_status;

    sendBuf.resize(1);
    recvBuf.resize(1);

    //if(flash_read_mode != Mode_Single)
    {
        ft4222_status = FT4222_SPIMaster_SetLines((FT_HANDLE)handle, SPI_IO_SINGLE);
        if (FT4222_OK != ft4222_status)
        {
            return ret_FAIL;
        }
    }

    DRV_spi_flash_waitWIP((FT_HANDLE)handle);

    DRV_spi_flash_write_enable((FT_HANDLE)handle);

    //erase all
    sendBuf[0] = 0x60;
    ft4222_status = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle, &sendBuf[0], sendBuf.size(), &sizeTransferred, true);

    return ret_PASS;
}

ErrType_t DRV_spi_flash_erase_data(void *handle, unsigned int start_addr, unsigned int data_size)
{
    std::vector<unsigned char> recvBuf;
    std::vector<unsigned char> sendBuf;
    uint16 sizeTransferred = 0;
//    uint32 longsizeTransferred = 0;
    uint16 packet_size = 0;
    FT_STATUS ft4222_status;
    unsigned int total_size = 0;

    if (data_size == 0)
        return ret_ErrParameter;

    total_size = 0;
    do
    {
        if ((data_size - total_size) >= FLASH_SECTER_SIZE)
            packet_size = FLASH_SECTER_SIZE;
        else
            packet_size = (data_size - total_size);
        //check current mode
        DRV_spi_flash_waitWIP((FT_HANDLE)handle);

        DRV_spi_flash_write_enable((FT_HANDLE)handle);

        ft4222_status = FT4222_SPIMaster_SetLines((FT_HANDLE)handle, SPI_IO_SINGLE);
        if (FT4222_OK != ft4222_status)
        {
            return ret_FAIL;
        }

        sendBuf.resize(4);
        recvBuf.resize(packet_size);
        sendBuf[0] = 0x20;
        sendBuf[1] = ((start_addr + total_size) >> 16) & 0xff;
        sendBuf[2] = ((start_addr + total_size) >> 8) & 0xff;
        sendBuf[3] = (start_addr + total_size) & 0xff;

        ft4222_status = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle, &sendBuf[0], sendBuf.size(), &sizeTransferred, true);

        total_size += packet_size;
    } while (total_size < data_size);

    return ret_PASS;
}

ErrType_t DRV_spi_flash_read_data(void *handle, unsigned int data_addr, unsigned int data_size, unsigned char *data)
{
    std::vector<unsigned char> recvBuf;
    std::vector<unsigned char> sendBuf;
    uint16 sizeTransferred = 0;
    uint32 longsizeTransferred = 0;
    uint16 packet_size = 0;
    FT_STATUS ft4222_status;
    unsigned int total_size = 0;

    if (data_size == 0)
        return ret_ErrParameter;

    if (data == NULL)
        return ret_ErrParameter;

    if (g_flash_read_mode == Mode_Single)
    {
        ft4222_status = FT4222_SPIMaster_SetLines((FT_HANDLE)handle, SPI_IO_SINGLE);
        if (FT4222_OK != ft4222_status)
        {
            return ret_FAIL;
        }
    }

    if (g_flash_read_mode == Mode_Dual)
    {
        ft4222_status = FT4222_SPIMaster_SetLines((FT_HANDLE)handle, SPI_IO_DUAL);
        if (FT4222_OK != ft4222_status)
        {
            return ret_FAIL;
        }
    }

    if (g_flash_read_mode == Mode_Quad)
    {
        ft4222_status = FT4222_SPIMaster_SetLines((FT_HANDLE)handle, SPI_IO_QUAD);
        if (FT4222_OK != ft4222_status)
        {
            return ret_FAIL;
        }
    }

    total_size = 0;
    do
    {
        if ((data_size - total_size) > 65500)
            packet_size = 65500;
        else
            packet_size = (data_size - total_size);
        //check current mode
        switch (g_flash_read_mode)
        {
        case Mode_Single:
            sendBuf.resize(4);
            recvBuf.resize(packet_size);
            sendBuf[0] = 0x03;
            sendBuf[1] = ((data_addr + total_size) >> 16) & 0xff;
            sendBuf[2] = ((data_addr + total_size) >> 8) & 0xff;
            sendBuf[3] = (data_addr + total_size) & 0xff;

            ft4222_status = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle, &sendBuf[0], sendBuf.size(), &sizeTransferred, false);

            ft4222_status = FT4222_SPIMaster_SingleRead((FT_HANDLE)handle, &recvBuf[0], recvBuf.size(), &sizeTransferred, true);

            memcpy(data + total_size, &recvBuf[0], sizeof(unsigned char) * packet_size);

            break;
        case Mode_Dual:
            sendBuf.resize(5);
            recvBuf.resize(packet_size);
            sendBuf[0] = 0xBB;
            sendBuf[1] = ((data_addr + total_size) >> 16) & 0xff;
            sendBuf[2] = ((data_addr + total_size) >> 8) & 0xff;
            sendBuf[3] = (data_addr + total_size) & 0xff;
            sendBuf[4] = 0xff;

            //ft4222_status = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle,&sendBuf[0], sendBuf.size(), &sizeTransferred, false);

            //ft4222_status = FT4222_SPIMaster_SingleRead((FT_HANDLE)handle, &recvBuf[0], recvBuf.size(),  &sizeTransferred, true);
            ft4222_status = FT4222_SPIMaster_MultiReadWrite((FT_HANDLE)handle, &recvBuf[0], &sendBuf[0], 1, sendBuf.size() - 1, recvBuf.size(), &longsizeTransferred);

            memcpy(data + total_size, &recvBuf[0], sizeof(unsigned char) * packet_size);
            break;
        case Mode_Quad:
#if 1
            //4READ (4 x I/O read) 4x addr + 4x data
            sendBuf.resize(7);
            recvBuf.resize(packet_size);
            sendBuf[0] = 0xEB;
            sendBuf[1] = ((data_addr + total_size) >> 16) & 0xff;
            sendBuf[2] = ((data_addr + total_size) >> 8) & 0xff;
            sendBuf[3] = (data_addr + total_size) & 0xff;
            sendBuf[4] = 0x00; //enhance mode flag
            sendBuf[5] = 0xff; //dummy
            sendBuf[6] = 0xff; //dummy

            //ft4222_status = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle,&sendBuf[0], sendBuf.size(), &sizeTransferred, false);

            //ft4222_status = FT4222_SPIMaster_SingleRead((FT_HANDLE)handle, &recvBuf[0], recvBuf.size(),  &sizeTransferred, true);
            ft4222_status = FT4222_SPIMaster_MultiReadWrite((FT_HANDLE)handle, &recvBuf[0], &sendBuf[0], 1, sendBuf.size() - 1, recvBuf.size(), &longsizeTransferred);
#endif
#if 0
			//QREAD (1I/4O read) 1x addr + 4x data 
			sendBuf.resize(5);
			recvBuf.resize(packet_size);
			sendBuf[0] = 0x6B;
			sendBuf[1] = ((data_addr+total_size)>>16) & 0xff;
			sendBuf[2] = ((data_addr+total_size)>>8) & 0xff;
			sendBuf[3] = (data_addr+total_size) & 0xff;
			sendBuf[4] = 0xff;

			//ft4222_status = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle,&sendBuf[0], sendBuf.size(), &sizeTransferred, false);

			//ft4222_status = FT4222_SPIMaster_SingleRead((FT_HANDLE)handle, &recvBuf[0], recvBuf.size(),  &sizeTransferred, true);
			ft4222_status = FT4222_SPIMaster_MultiReadWrite((FT_HANDLE)handle, &recvBuf[0], &sendBuf[0], 5,0,recvBuf.size(), &longsizeTransferred) ;
#endif
            memcpy(data + total_size, &recvBuf[0], sizeof(unsigned char) * packet_size);
            break;
        default:
            break;
        }

        total_size += packet_size;
    } while (total_size < data_size);

    return ret_PASS;
}

ErrType_t DRV_spi_flash_init(void *handle, SPIMasterClock_t clock)
{
    FT_STATUS ftStatus;

    FT4222_ClockRate clk = (FT4222_ClockRate)((clock & 0xF0) >> 4);
    FT4222_SPIClock div = (FT4222_SPIClock)(clock & 0xF);
    if ((FT_HANDLE)handle != NULL)
    {
        //		ftStatus = FT4222_SetClock((FT_HANDLE)handle, SYS_CLK_48);
        //ftStatus = FT4222_SetClock((FT_HANDLE)handle, (FT4222_ClockRate)((clock&0xF0)>>4));
        ftStatus = FT4222_SetClock((FT_HANDLE)handle, clk);
        if (FT_OK != ftStatus)
        {
            return ret_FAIL;
        }

        //set SPI master to 1.5Mhz
        //		ftStatus = FT4222_SPIMaster_Init((FT_HANDLE)handle, SPI_IO_SINGLE, CLK_DIV_2, CLK_IDLE_LOW, CLK_LEADING, 0x01);
        //		ftStatus = FT4222_SPIMaster_Init((FT_HANDLE)handle, SPI_IO_SINGLE, (FT4222_SPIClock) (clock&0xF), CLK_IDLE_LOW, CLK_LEADING, 0x01);
        ftStatus = FT4222_SPIMaster_Init((FT_HANDLE)handle, SPI_IO_SINGLE, div, CLK_IDLE_LOW, CLK_LEADING, 0x01);
        if (FT4222_OK != ftStatus)
        {
            return ret_FAIL;
        }

        ftStatus = FT4222_SPI_SetDrivingStrength((FT_HANDLE)handle, DS_4MA, DS_4MA, DS_4MA);
        if (FT_OK != ftStatus)
        {
            return ret_FAIL;
        }

        ftStatus = FT_SetUSBParameters((FT_HANDLE)handle, 4 * 1024, 0);
        if (FT_OK != ftStatus)
        {
            return ret_FAIL;
        }

        g_flash_read_mode = Mode_Single;

        //try to get flash info
        //spi_flash_read_info(NULL);
		DRV_spi_flash_read_info((FT_HANDLE)handle, NULL);

        if (g_cur_flash_type == UNKNOWN_VENDOR)
        {
            return ret_NoSupportFlash;
        }

        return ret_PASS;
    }

    return ret_FAIL;
}

bool DRV_spi_flash_busy_check(void *handle)
{
    std::vector<unsigned char> sendBuf;
    std::vector<unsigned char> recvBuf;
    uint16 sizeTransferred = 0;
    FT_STATUS ft4222_status;

    sendBuf.resize(1);
    recvBuf.resize(1);

    //read status
    sendBuf[0] = 0x05;
    ft4222_status = FT4222_SPIMaster_SingleWrite((FT_HANDLE)handle, &sendBuf[0], sendBuf.size(), &sizeTransferred, false);
    ft4222_status = FT4222_SPIMaster_SingleRead((FT_HANDLE)handle, &recvBuf[0], recvBuf.size(), &sizeTransferred, true);

    dbg_printf("DRV_spi_flash_set_quad_mode: 0x%08X\n", (uint)ft4222_status);

    if (recvBuf[0] & 0x01)
        return true;
    else
        return false;
}
