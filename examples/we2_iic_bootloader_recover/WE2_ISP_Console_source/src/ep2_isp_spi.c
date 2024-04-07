/*
 * ep2_isp_spi.c
 *
 *  Created on: 2022¦~10¤ë4¤é
 *      Author: 901912
 */
#include "ep2_isp_spi.h"
#include "common.h"
#include "hmx_lib.h"


int ep2_isp_spi_proc(uint8_t *file_buf, uint32_t file_size)
{
    //Enable SPI WRITE
    spi_master_isp_1byte_write(ISP_ENABLE_REG_ADDR, REG_D8_ISP_EN+REG_D8_TEST_MODE+REG_D8_SPI_DO_EN);

    if(spi_master_isp_enable_burstmode_inc4() != ret_PASS)
    {
        return -1;
    }

    uint32_t val[1];
    val[0] = ISP_REG_DEFAULT_VAL;
    if(spi_master_isp_burst_write(ISP_CONTROL_ADDR, 4, (uint8_t *)&val[0]) != ret_PASS)
    {
        printf("ERROR!! SPI WRITE SETTING FAIL\n");

        return -1;
    }

    uint32_t addr = ISP_DATA_PORT_ADDR;
//    uint8_t buf[256+5];
    uint32_t size = file_size;
    uint32_t upt_val = 0;
    printProgress((double)upt_val/file_size);

#if (EPII_VERC_PPDONE == 0x01)
    uint32_t pp_counter_val = 0;
#endif

    do{
        if(size > 256)
        {
            upt_val += 256;
            size -= 256;
        }
        else
        {
            upt_val += size;
            size = 0;
        }


        if(spi_master_isp_burst_write(addr, 256, &file_buf[addr-ISP_DATA_PORT_ADDR]) != ret_PASS)
        {
            printf("ERROR!!!WRITE  0x%08X => 256 BYTES   (FAIL)\n", addr);
            break;
        }
        addr += 256;

#if (EPII_VERC_PPDONE == 0x01)
        pp_counter_val += 1;
#endif

        //check PP
        uint32_t pp_stat = 0;

#if (EPII_VERC_PPDONE == 0x01)
        uint32_t tmp_addr = ISP_CONTROL_ADDR + ISP_PPDONE_COUNTER_OFFSET;
#else
        uint32_t tmp_addr = ISP_CONTROL_ADDR + ISP_PPDONE_OFFSET;
#endif

        do
        {
            pp_stat = 0x0;

            spi_master_isp_burst_read(tmp_addr, 4, (uint8_t *)&pp_stat);

#if (EPII_VERC_PPDONE == 0x01)
        }while( !((pp_stat>>28) == 1 || (pp_stat&0xFFFFF) == pp_counter_val) );
#else
#if (EPII_VERB_PPDONE_WORKAROUND == 0x01)
        }while( !((pp_stat>>28) == 1 || (pp_stat>>28) == 2
                || (pp_stat>>28) == 3 || (pp_stat&0xFFFFFF) == (addr&0xFFFFFF) - 4) );
#else
        }while( !((pp_stat>>28) == 1 || (pp_stat>>28) == 2 || (pp_stat>>28) == 3) );
#endif
#endif


        //update progress dialog
        if(upt_val % PROGRESS_STEP == 0)
            printProgress((double)upt_val/file_size);
        //update progress dialog


        //CRC Error
        if((pp_stat>>28) == 1 || (pp_stat>>28) == 3)
        {
            printf("ERROR: PP_STAT %d\n", pp_stat>>28);
            uint32_t confict_addr = pp_stat & 0x00FFFFFF;

            uint32_t crc_rout, crc_wout;
            tmp_addr = ISP_CONTROL_ADDR + 0x04;
            spi_master_isp_burst_read(tmp_addr, 4, (uint8_t *)&crc_wout);

            tmp_addr = ISP_CONTROL_ADDR + 0x08;
            spi_master_isp_burst_read(tmp_addr, 4, (uint8_t *)&crc_rout);

            //clear CRC info
            tmp_addr = ISP_CONTROL_ADDR + ISP_STATUS_CLR_OFFEST;
            uint32_t clear_stat;
            spi_master_isp_burst_read(tmp_addr, 4, (uint8_t *)&clear_stat);

            if(crc_wout != crc_rout)
            {
                printf("CONFLICT ADDR: 0x%08X \nCRC ERROR: 0x%08X(RCRC) != 0x%08X(WCRC)\n", confict_addr,
                        crc_rout,
                        crc_wout);
                break;
            }
        }

    }while(size > 0);

    printProgress((double)upt_val/file_size);

    //Clear Programming Status
    {
        uint32_t tmp_addr;

//        uint32_t crc_rout, crc_wout;
//        tmp_addr = ISP_CONTROL_ADDR + 0x04;
//        spi_master_isp_burst_read(tmp_addr, 4, (uint8_t *)&crc_wout);
//
//        tmp_addr = ISP_CONTROL_ADDR + 0x08;
//        spi_master_isp_burst_read(tmp_addr, 4, (uint8_t *)&crc_rout);

        //clear CRC info
        tmp_addr = ISP_CONTROL_ADDR + ISP_STATUS_CLR_OFFEST;
        uint32_t clear_stat;
        spi_master_isp_burst_read(tmp_addr, 4, (uint8_t *)&clear_stat);
    }

    //DISABLE SPI WRITE
    spi_master_isp_1byte_write(ISP_ENABLE_REG_ADDR, REG_OFF);

    return 0;
}
