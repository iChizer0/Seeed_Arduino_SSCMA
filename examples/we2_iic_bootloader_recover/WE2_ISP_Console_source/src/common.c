#include <common.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


/*
 * for debug enable
 */
//#define __linux__

//bool file_exists(const char * filename)
//{
//    FILE *file;
//
//    if ((file = fopen(filename, "r")) != NULL)
//    {
//        fclose(file);
//        return true;
//    }
//    return false;
//}

void clear_screen()
{
#if defined(__linux__) || defined(__unix__) || defined(__APPLE__)
    system("clear");
#endif

#if defined(_WIN32) || defined(_WIN64)
    system("cls");
#endif
}

#define PBSTR "============================================================"
#define PBWIDTH 60

void printProgress(double percentage)
{
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% |%.*s%*s| ", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

uint32_t get_dec_selection()
{
    uint32_t return_int;
    uint32_t return_value = 0;
    char buffer[1024];

    while (!return_value) {
        scanf("%1024s", buffer);
        return_value = sscanf(buffer, "%d", &return_int);
        if(!return_value)
        {
            printf("input format error\n");
        }
    }

    return return_int;
}

uint32_t get_hex_selection()
{
    uint32_t return_int;
    uint32_t return_value = 0;
    char buffer[1024];

    while (!return_value) {
        scanf("%1024s", buffer);
        return_value = sscanf(buffer, "%x", &return_int);
        if(return_int < 0 || return_int >= UINT32_MAX)
        {
            printf("input value > 32bit\n");
            return_value = 0;
        }
        else if(!return_value)
        {
            printf("input format error\n");
        }
    }

    return return_int;
}


//#define POLY 0x8408 /* 1021H bit reversed */
//uint16_t crc16_ccitt(char *data_p, int length)
//{
//    unsigned char i;
//    unsigned int data;
//    unsigned int crc = 0xffff;
//    if (length == 0)
//        return (unsigned short) (crc);
//    if (data_p == NULL)
//        return (unsigned short) (crc);
//    do {
//        for (i = 0, data = (unsigned int) 0xff & *data_p++; i < 8; i++, data >>=
//        1) {
//            if ((crc & 0x0001) ^ (data & 0x0001))
//                crc = (crc >> 1) ^ POLY;
//            else
//                crc >>= 1;
//        }
//    } while (--length);
//    // Uncomment to change from little to big Endian
//    //crc = ((crc & 0xff) << 8) | ((crc & 0xff00) >> 8);
//    return (uint16_t) (crc);
//}


