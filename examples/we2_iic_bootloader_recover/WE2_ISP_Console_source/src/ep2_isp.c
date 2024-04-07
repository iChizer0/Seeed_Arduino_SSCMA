#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include "ep2_isp_i2c.h"
#include "ep2_isp_spi.h"
#include "common.h"
#include "hmx_lib.h"


int main (int argc, char *argv[])
{
    int ret=0;

    if(argc != 3)
    {
        printf("USAGE: ISP.exe [FILE] [0: I2C, 1: SPI]\n");
        return 0;
    }

    uint8_t isp_mode = strtoul(argv[2], NULL, 10);

    printf("file_path:%s\n", argv[1]);

    FILE *fp;
    fflush(stdout);
    fp=fopen(argv[1], "rb");
    if(fp == NULL)
    {
        printf("\nERROR!! Can't fine file, please check file path\n");
        return -1;
    }
    fseek( fp, 0, SEEK_END);

    uint32_t filelen = ftell( fp );
    rewind(fp);
    uint8_t *image_buf = (uint8_t *)malloc(filelen);
    fread(image_buf, filelen, 1, fp);
    fclose(fp);

    printf("file length: %d bytes\n", filelen);

    if(init_lib() != ret_PASS)
    {
        printf("\nERROR!! Init FT4222 LIB FAIL\n");
        return -1;
    }


    if(isp_mode == 0)
    {
        if(i2c_master_init(1000) != ret_PASS)
        {
            ret = -1;
            printf("\nERROR!! Init FT4222 I2C 1M FAIL\n");
        }
        else
        {
            printf("I2C MODE START \n");
            ret = ep2_isp_i2c_proc(image_buf, filelen);
        }
    }
    else
    {

        if(spi_master_init(MasterClock_7500K) != ret_PASS)
        {
            printf("\nERROR!! Init FT4222 SPI 7.5M FAIL\n");
            ret = -1;
        }
        else
        {
            printf("SPI MODE START \n");
            ret = ep2_isp_spi_proc(image_buf, filelen);
        }
    }

    return ret;

}

