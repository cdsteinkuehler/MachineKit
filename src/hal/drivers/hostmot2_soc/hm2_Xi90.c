//
//    Copyright (C) 2007-2008 Sebastian Kuzminsky
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//
//======================================================================================
//
//======================================================================================
// CODE NOTES:
//------------
// This module origianlly was for using the EPP parallel port. It has been modified to 
// use the BBB gpmc port.
// 16 bit wide, asynchronous, multiplexed address and data

#include "config_module.h"
 
#if defined(BUILD_SYS_KBUILD)
#include <asm/io.h>
#endif
 
#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_math.h"
#include "rtapi_string.h"

#include "hal.h"
 
#include "hal/drivers/Xylotex-HM2/bitfile.h"
#include "hal/drivers/Xylotex-HM2/hostmot2-lowlevel.h"
// we probably don't need the xi90.h file
// but we will need something for gpmc
#include "hal/drivers/Xylotex-HM2/hm2_Xi90.h"

static int comp_id;

#ifdef MODULE_INFO
MODULE_INFO(linuxcnc, "component:hm2_Xi90:LinuxCNC HAL driver for the Xylotex BBB Xi90 IO board with HostMot2 firmware.");
MODULE_INFO(linuxcnc, "license:GPL");
#endif // MODULE_INFO

MODULE_LICENSE("GPL");


int debug_epp = 0;
RTAPI_MP_INT(debug_epp, "Developer/debug use only!  Enable debug logging of most EPP\ntransfers.");

static char *config[HM2_XI90_MAX_BOARDS];
RTAPI_MP_ARRAY_STRING(config,HM2_XI90_MAX_BOARDS, "config string(s) for the Xi90 board(s) (see hostmot2(9) manpage)");



//
// this data structure keeps track of all the Xi90 boards found
//

static hm2_Xi90_t board[HM2_XI90_MAX_BOARDS];
static int num_boards;

//--------------------------------------------------------------------------------------
// GPMC code
//--------------------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
 

#define GPMC_BASE 0x50000000

#define GPMC_CHIPSELECTCONFIGDISPLACEMENT (0x30 / 4)

#define GPMC_SYSCONFIG (0x10 / 4)

#define GPMC_CONFIG0 (0x50 / 4)

#define GPMC_CONFIG1 (0x60 / 4)
#define GPMC_CONFIG2 (0x64 / 4)
#define GPMC_CONFIG3 (0x68 / 4)
#define GPMC_CONFIG4 (0x6c / 4)
#define GPMC_CONFIG5 (0x70 / 4)
#define GPMC_CONFIG6 (0x74 / 4)
#define GPMC_CONFIG7 (0x78 / 4)

#define CONFIG0 0x00

//#define CONFIG1 ((1<<25) |(1<<12) | (1<<9) | 3) 
#define CONFIG1 0x02001203
#define CONFIG2 0x000C0C00 
#define CONFIG3 0x00040400 
#define CONFIG4 0x0A040B04 
#define CONFIG5 0x040D1F1F 
#define CONFIG6 0x040404C4 
#define CONFIG7 0x00000f41 

#define DEBUG
static int devmemfd = -1;
//-------------------------------------------------------------------------------------
// This FIRST section of code is to allow access to the GPMC bus
//--------------------------------------------------------------------------------------
// 
//--------------------------------------------------------------------------------------
//map memory of length "len" at offset "offset"
static void* util_mapmemoryblock(off_t offset, size_t len)
{
	devmemfd = open("/dev/mem", O_RDWR | O_SYNC);
	void* registers = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, devmemfd, offset);
	if (registers == MAP_FAILED) {
		printf("Map failed\n");
	}
	return registers;
}
//--------------------------------------------------------------------------------------
// 
//--------------------------------------------------------------------------------------
static void util_unmapmemoryblock(void* block, size_t len)
{
	munmap((void*) block, len);
	if (devmemfd != -1) {
		close(devmemfd);
	}
}

#define REGLEN 0x10000000

static volatile uint32_t* registers = NULL;

//--------------------------------------------------------------------------------------
// 
//--------------------------------------------------------------------------------------
void gpmc_setup(void)
{

	//gpmc_mapregisters();
        registers = (uint32_t*) util_mapmemoryblock(GPMC_BASE, REGLEN);
	if (registers != MAP_FAILED) {
		int chipselect = 0;
		int displacement = GPMC_CHIPSELECTCONFIGDISPLACEMENT * chipselect;

		//Reset the module, TODO: is this needed?
		*(registers + GPMC_SYSCONFIG) = 0x12;
		*(registers + GPMC_SYSCONFIG) = 0x10;

		// disable before playing with the registers..
		*(registers + displacement + GPMC_CONFIG7) = 0x00000000;

		*(registers + displacement + GPMC_CONFIG0) = CONFIG0;
		*(registers + displacement + GPMC_CONFIG1) = CONFIG1;
		*(registers + displacement + GPMC_CONFIG2) = CONFIG2;
		*(registers + displacement + GPMC_CONFIG3) = CONFIG3;
		*(registers + displacement + GPMC_CONFIG4) = CONFIG4; 
		*(registers + displacement + GPMC_CONFIG5) = CONFIG5;
		*(registers + displacement + GPMC_CONFIG6) = CONFIG6;
		*(registers + displacement + GPMC_CONFIG7) = CONFIG7;
		//gpmc_unmapregisters();
                util_unmapmemoryblock((void*) registers, REGLEN);
	}
     else
        {fprintf(stderr,"map registers FAILED\n");
        }
}

volatile uint16_t* extbus;
//--------------------------------------------------------------------------------------
// 
//--------------------------------------------------------------------------------------
void bus_init() {
	gpmc_setup();
	extbus = (uint16_t*) util_mapmemoryblock(0x01000000, 0x1FFFF);
}
//--------------------------------------------------------------------------------------
// 
//--------------------------------------------------------------------------------------
void bus_shutdown() {
	util_unmapmemoryblock((void*) extbus, 0xFFFF);
}
//-------------------------------------------------------------------------------------
// This SECOND section of code is for communicating between the FPGA and MachineKit
// using the memory mapped GPMC
//-------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------
static inline u32 hm2_Xi90_gpmc_read32(hm2_Xi90_t *board, u16 addr) 
{
    __u32 data;
    __u16 a, b;
    a=*(uint16_t  *)(extbus + (addr));
    b=*(uint16_t  *)(extbus + (addr+2));
    data = a + (b<<16);
    return data;
}
//--------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------
static inline void hm2_Xi90_gpmc_write32(__u32 word, hm2_Xi90_t *board, u16 addr)
{ 
static __u32 old_word;
 

*(uint16_t *)(extbus + (addr)) =(uint16_t)word;           // first the lo word
*(uint16_t *)(extbus + (addr+2)) = (uint16_t)(word >> 16);// hi word triggers write
}

//
// misc generic helper functions
//
//--------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------
// FIXME: this is bogus
static void hm2_Xi90_nanosleep(unsigned long int nanoseconds) {
    long int max_ns_delay;

    max_ns_delay = rtapi_delay_max();

    while (nanoseconds > max_ns_delay) {
        rtapi_delay(max_ns_delay);
        nanoseconds -= max_ns_delay;
    }

    rtapi_delay(nanoseconds);
}

//--------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------
// 
// these are the low-level i/o functions exported to the hostmot2 driver
//

int hm2_Xi90_read(hm2_lowlevel_io_t *this, u32 addr, void *buffer, int size) 
{
int bytes_remaining = size;
int count;

    hm2_Xi90_t *board = this->private;


// this expects autoincrement, so increment it here instead;
// for this version, the variable size should always be 4 
// if MachineKit wants to read a byte from the FPGA is should read a long
// and extract the byte
    for (count=0; bytes_remaining > 3; bytes_remaining -= 4,count+=4) {
        *((u32*)buffer) = hm2_Xi90_gpmc_read32(board,addr+count);
        buffer += 4;
    }
    return 1;  // success
}


//--------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------

int hm2_Xi90_write(hm2_lowlevel_io_t *this, u32 addr, void *buffer, int size) {
    int bytes_remaining = size;
    hm2_Xi90_t *board = this->private;
int count;

// this expects autoincrement, so increment it here instead;
// for this version, the variable size should always be 4 
    for (count=0; bytes_remaining > 3; bytes_remaining -= 4,count+=4)
       {
        hm2_Xi90_gpmc_write32(*((u32*)buffer), board,addr+count);
        buffer += 4;
       }
    return 1;
}
//--------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------
// return 0 if the board has been reset, -errno if not
int hm2_Xi90_reset(hm2_lowlevel_io_t *this) {
    hm2_Xi90_t *board = this->private;
    __u8 byte;

    //
    // this resets the FPGA *only* if it's currently configured with the
    // HostMot2 or GPIO firmware
    //
// Need to modfy for BBB version
fprintf(stderr,"SKIPPING RESET FPGA in hm2_Xi90_reset\n");
    return 0;
}



//--------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------
//
// setup and cleanup code
//
static void hm2_Xi90_cleanup(void) {
    int i;

    // NOTE: hal_malloc() doesnt have a matching free
    // gpmc shutdown
    bus_shutdown();
    for (i = 0; i < num_boards; i ++) {
        hm2_lowlevel_io_t *this = &board[i].llio;
        THIS_PRINT("releasing board\n");
        hm2_unregister(this);
    }
}

//--------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------
static int hm2_Xi90_setup(void) {
    int i;

    // zero the board structs
    memset(board, 0, HM2_XI90_MAX_BOARDS * sizeof(hm2_Xi90_t));
    num_boards = 0;
    bus_init();

    for (i = 0; config[i] != NULL; i ++) {
        hm2_lowlevel_io_t *this;
        int r;


        rtapi_snprintf(board[i].llio.name, sizeof(board[i].llio.name), "%s.%d", HM2_LLIO_NAME, i);
        board[i].llio.comp_id = comp_id;

        board[i].llio.read = hm2_Xi90_read;
        board[i].llio.write = hm2_Xi90_write;
//        board[i].llio.program_fpga = hm2_Xi90_program_fpga;
        board[i].llio.reset = hm2_Xi90_reset;

        board[i].llio.num_ioport_connectors = 3;
        board[i].llio.pins_per_connector = 24;
        board[i].llio.ioport_connector_name[0] = "P1";
        board[i].llio.ioport_connector_name[1] = "P2";
        board[i].llio.ioport_connector_name[2] = "P3";
        board[i].llio.num_leds = 8;
        board[i].llio.private = &board[i];

        this = &board[i].llio;


            board[i].llio.fpga_part_number = "6slx9tqg144";

        THIS_DBG("detected FPGA '%s'\n", board[i].llio.fpga_part_number);

        r = hm2_register(&board[i].llio, config[i]);
        if (r != 0) {
            return r;
        }
        num_boards ++;
    }

    return 0;
}
//--------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------
int rtapi_app_main(void) {
    int r = 0;
    comp_id = hal_init(HM2_LLIO_NAME);
    if (comp_id < 0) return comp_id;

    r = hm2_Xi90_setup();
    if (r) {
        hm2_Xi90_cleanup();
        hal_exit(comp_id);
    } else {
        hal_ready(comp_id);
    }

    return r;
}
//--------------------------------------------------------------------------------------
// 
//--------------------------------------------------------------------------------------
void rtapi_app_exit(void) {
    hm2_Xi90_cleanup();
    hal_exit(comp_id);
    fprintf(stderr,"driver unloaded\n");
}
//--------------------------------------------------------------------------------------
// EOF
//--------------------------------------------------------------------------------------
