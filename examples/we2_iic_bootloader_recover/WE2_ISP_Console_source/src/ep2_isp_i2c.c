/*
 * ep2_isp_i2c.c
 *
 *  Created on: 2022¦~10¤ë4¤é
 *      Author: 901912
 */
#include "ep2_isp_i2c.h"
#include "common.h"
#include "hmx_lib.h"


int ep2_isp_i2c_proc(uint8_t *file_buf, uint32_t file_size)
{
    uint32_t reg_addr[] = {ISP_CONTROL_ADDR};
    uint32_t reg_addr_scu = ISP_ENABLE_REG_ADDR;

    uint32_t val_scu = REG_D8_TEST_MODE + REG_D8_ISP_EN;

    uint32_t val[1];
    val[0] = ISP_REG_DEFAULT_VAL;

    I2C_burst_write(CTRL_SLVID, (uint8_t *)&reg_addr_scu, 1, (uint8_t *)&val_scu, 1);

    for(uint32_t i=0;i<sizeof(reg_addr)/sizeof(uint32_t);i++)
    {
        i2c_single_write(CTRL_SLVID, (uint8_t *)&reg_addr[i], (uint8_t *)&val[i]);
    }

    uint32_t addr = ISP_DATA_PORT_ADDR;
    uint8_t buf[256+5];
    uint32_t size = file_size;
    uint32_t upt_val = 0;
    printProgress((double)upt_val/file_size);

#if (EPII_VERC_PPDONE == 0x01)
    uint32_t pp_counter_val = 0;
#endif

    do{
        memset(buf, 0xFF, 261);
        buf[0] = 0xFA;
        buf[1] = addr & 0xFF;
        buf[2] = (addr >> 8) & 0xFF;
        buf[3] = (addr >> 16) & 0xFF;
        buf[4] = (addr >> 24) & 0xFF;

        if(size > 256)
        {
            memcpy(&buf[5], &file_buf[addr-ISP_DATA_PORT_ADDR], 256);
            upt_val += 256;
            size -= 256;
        }
        else
        {
            memcpy(&buf[5], &file_buf[addr-ISP_DATA_PORT_ADDR], size);
            upt_val += size;
            size = 0;
        }

        I2C_cmd_write_ex(DATA_SLVID, buf, 261, START_AND_STOP);

        addr += 256;

#if (EPII_VERC_PPDONE == 0x01)
        pp_counter_val += 1;
#endif

#if (EPII_VERC_PPDONE == 0x01)
        //check PP
        uint32_t pp_stat = 0;
        uint32_t tmp_addr = ISP_CONTROL_ADDR + ISP_PPDONE_COUNTER_OFFSET;

        do
        {
            pp_stat = 0x0;
            pp_stat = i2c_single_read(CTRL_SLVID, (uint8_t *)&tmp_addr);

        }while( !((pp_stat>>28) == 1 || (pp_stat & 0xFFFFF) == pp_counter_val) );

#else
        //check PP
        uint32_t pp_stat = 0;
        uint32_t tmp_addr = ISP_CONTROL_ADDR + ISP_PPDONE_OFFSET;
        do
        {
            pp_stat = 0x0;
            pp_stat = i2c_single_read(CTRL_SLVID, (uint8_t *)&tmp_addr);
#if (EPII_VERB_PPDONE_WORKAROUND == 0x01)
        }while( !((pp_stat>>28) == 1 || (pp_stat>>28) == 2
                || (pp_stat>>28) == 3 || (pp_stat&0xFFFFFF) == (addr&0xFFFFFF) - 4) );
#else
        }while( !((pp_stat>>28) == 1 || (pp_stat>>28) == 2 || (pp_stat>>28) == 3) );
#endif
#endif
        //update progress status
        if(upt_val % PROGRESS_STEP == 0)
            printProgress((double)upt_val/file_size);
        //update progress status

        //CRC Error
        if((pp_stat>>28) == 1 || (pp_stat>>28) == 3)
        {
            printf("ERROR: PP_STAT %d\n", pp_stat>>28);
            uint32_t confict_addr = pp_stat & 0x00FFFFFF;

            uint32_t crc_rout, crc_wout;
            tmp_addr = ISP_CONTROL_ADDR + 0x04;
            crc_wout = i2c_single_read(CTRL_SLVID, (uint8_t *)&tmp_addr);

            tmp_addr = ISP_CONTROL_ADDR + 0x08;
            crc_rout = i2c_single_read(CTRL_SLVID, (uint8_t *)&tmp_addr);

            //clear CRC info
            tmp_addr = ISP_CONTROL_ADDR + ISP_STATUS_CLR_OFFEST;
            i2c_single_read(CTRL_SLVID, (uint8_t *)&tmp_addr);

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

    {
        uint32_t tmp_addr;

//        uint32_t crc_rout, crc_wout;
//        tmp_addr = ISP_CONTROL_ADDR + 0x04;
//        crc_wout = i2c_single_read(CTRL_SLVID, (uint8_t *)&tmp_addr);
//
//        tmp_addr = ISP_CONTROL_ADDR + 0x08;
//        crc_rout = i2c_single_read(CTRL_SLVID, (uint8_t *)&tmp_addr);

        //clear CRC info
        tmp_addr = ISP_CONTROL_ADDR + ISP_STATUS_CLR_OFFEST;
        i2c_single_read(CTRL_SLVID, (uint8_t *)&tmp_addr);
    }

    val_scu = REG_OFF;
    I2C_burst_write(CTRL_SLVID, (UINT8 *)&reg_addr_scu, 1, (UINT8 *)&val_scu, 1);

    return 0;
}



