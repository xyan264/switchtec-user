/*
 * Microsemi Switchtec(tm) PCIe Management Library
 * Copyright (c) 2017, Microsemi Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifdef __linux__

#include "../switchtec_priv.h"
#include "switchtec/switchtec.h"
#include "gasops.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stddef.h>
#include <assert.h>
#include <string.h>

#include <termios.h>

struct switchtec_uart{
	struct switchtec_dev dev;
	int fd;
};

#define to_switchtec_uart(d) \
	((struct switchtec_uart *) \
	 ((char *)d - offsetof(struct switchtec_uart, dev)))

#ifdef __CHECKER__
#define __force __attribute__((force))
#else
#define __force
#endif

static void uart_close(struct switchtec_dev *dev)
{
	struct switchtec_uart *udev =  to_switchtec_uart(dev);
	
	if (dev->gas_map)
		munmap((void __force *)dev->gas_map, dev->gas_map_size);

	close(udev->fd);
	free(udev);
}

static int map_gas(struct switchtec_dev *dev)
{
	void *addr;
	dev->gas_map_size = 4 << 20;

	addr = mmap(NULL, dev->gas_map_size, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (addr == MAP_FAILED)
		return -1;
	
	dev->gas_map = (gasptr_t __force)addr;	
	
	return 0;
}

#undef __force

static gasptr_t uart_gas_map(struct switchtec_dev *dev, int writeable, size_t *map_size)
{
	if (map_size)
		*map_size = dev->gas_map_size;

	return dev->gas_map;	
}

//gaswr <address> <dword 1> [dword 2] [ … ], max 14 dwords
#define GASWR_MAX_DWORDS			(16)
#define GASWR_MAX_PAYLOAD			(14)
#define GASWR_HDR_DWORDS			(2)
//gas_reg_write() success
//...
//0x00000000:0000>
#define GASWR_RTN_CHARS				(50*2)
#define GASWR_RTN_DWORDS			((GASWR_RTN_CHARS)/(CHARS_PER_DWORD))

//gasrd <address> [dword count]
#define GASRD_MAX_DWORDS			(3)
//gas_reg_read <0x0> [1]
//...
//0x00000000:0000>
#define GASRD_RTN_CHARS				(50*2)
#define GASRD_RTN_DWORDS			((GASRD_RTN_CHARS)/(CHARS_PER_DWORD))

#define CHARS_PER_DWORD				(11)
#define BITS_PER_CHAR				(10)
#define MICROSECONDS				(1000000)
#define UART_BAUD_RATE				(230400)
//#define	UART_DWORD_DELAY_US			(((MICROSECONDS)/(UART_BAUD_RATE))*(BITS_PER_CHAR)*(CHARS_PER_DWORD))
#define	UART_DWORD_DELAY_US			(500)
#define REDUNDANT_DELAY				(2)

static void uart_memcpy_from_gas(struct switchtec_dev *dev, void *dest, const void __gas *src, size_t n)
{
	int ret;
	int rty_cnt = 3;
	int rd_dw_len;
	char *b;
	int raddr, rnum, idx;
	char *tok;
	const char *delims = "\n";
	char gas_rd[1024];
	char gas_rd_rtn[102400];
	
	struct switchtec_uart *udev = to_switchtec_uart(dev);
	
	uint32_t addr = (uint32_t)(src - (void __gas *)dev->gas_map);

	rd_dw_len = (n/4 + (n%4?1:0));
	b = gas_rd;
	b += snprintf(b, sizeof(gas_rd), "gasrd 0x%x", addr);	
	b += snprintf(b, sizeof(gas_rd), " %d", rd_dw_len);
 	b += snprintf(b, sizeof(gas_rd), "\r");
	
	ret = tcflush(udev->fd,TCIOFLUSH);
	if (ret < 0){
		perror("tcflush error\n");
		exit(1);
	}

	ret = write(udev->fd, gas_rd, strlen(gas_rd));
	if (ret != strlen(gas_rd)){
		perror("write error\n");
		exit(1);
	}

	ret = tcdrain(udev->fd);
	if (ret < 0){
		perror("tcdrain error\n");
		exit(1);
	}
	
	usleep(GASRD_MAX_DWORDS*UART_DWORD_DELAY_US);
	usleep((GASRD_RTN_DWORDS+rd_dw_len)*UART_DWORD_DELAY_US);

	memset(gas_rd_rtn, 0, sizeof(gas_rd_rtn));
rd_rtn_retry:
	ret = read(udev->fd, gas_rd_rtn + strlen(gas_rd_rtn), sizeof(gas_rd_rtn));
	if (ret < 0){
		perror("read error\n");
		exit(1);
	}

	if(sscanf(gas_rd_rtn, "%*[^<]<0x%x> [%d]", &raddr, &rnum) != 2){
		usleep(500);
		goto rd_rtn_retry;
	}
	if(sscanf(gas_rd_rtn, "%*[^:]:%x>", &idx) != 1){
		usleep(500);
		if (rty_cnt > 0){
			rty_cnt--;
			goto rd_rtn_retry;
		}
	}

	assert(raddr == addr);
	assert(rnum == rd_dw_len);
	tok = strtok(gas_rd_rtn, delims);
	tok = strtok(NULL, delims);
	for (int i = 0; i<rnum; i++){
		tok = strtok(NULL, delims);
		*(uint32_t *) dest = strtol(tok, &tok, 0);
		dest += 4;
	}
}

#define create_gas_read(type, suffix) \
	static type uart_gas_read ## suffix(struct switchtec_dev *dev, \
		type __gas *addr) \
	{ \
		type ret; \
		uart_memcpy_from_gas(dev, &ret, addr, sizeof(ret)); \
		return ret; \
	}
create_gas_read(uint32_t, 32);
create_gas_read(uint64_t, 64);

static uint8_t uart_gas_read8(struct switchtec_dev *dev, uint8_t __gas *addr)
{
	uint32_t val;
	uint32_t tmpaddr = (uint32_t)((void __gas *)addr - (void __gas *)0);
	uart_memcpy_from_gas(dev, &val, (void __gas *)((tmpaddr & (~0x3)) + (void __gas *)0), sizeof(val));
	switch (tmpaddr & 0x3){
		case 0:
		val = val & 0xff;
		break;
		case 1:
		val = (val >> 8) & 0xff;
		break;
		case 2:
		val = (val >> 16) & 0xff;
		break;
		case 3:
		val = (val >> 24) & 0xff;
		break;
	}
	return (uint8_t)val;
}

static uint16_t uart_gas_read16(struct switchtec_dev *dev, uint16_t __gas *addr)
{
	uint32_t val;
	uint32_t tmpaddr = (uint32_t)((void __gas *)addr - (void __gas *)0);
	uart_memcpy_from_gas(dev, &val, (void __gas *)((tmpaddr & (~0x3)) + (void __gas *)0), sizeof(val));
	switch (tmpaddr & 0x3){
		case 0:
		val = val & 0xffff;
		break;
		case 2:
		val = (val >> 16) & 0xffff;
		break;

	}
	return (uint16_t)val;
}

static void cli_dbginfo_off(struct switchtec_dev *dev)
{
	int ret;
	int idx;
	int rty_cnt = 3;
	char *dbginfo = "pscdbg 0 all\r";
	char rtn[102400];
	struct switchtec_uart *udev = to_switchtec_uart(dev);
	ret = tcflush(udev->fd,TCIOFLUSH);
	if (ret < 0){
		perror("tcflush error\n");
		exit(1);
	}
	ret = write(udev->fd, dbginfo, strlen(dbginfo));
	if (ret != strlen(dbginfo)){
		perror("write error\n");
		exit(1);
	}
	ret =  tcdrain(udev->fd);
	if (ret < 0){
		perror("tcdrain error\n");
		exit(1);
	}
	usleep(5000);
	memset(rtn, 0, sizeof(rtn));
cli_rtn_retry:
	ret = read(udev->fd, rtn, sizeof(rtn));
	if (ret < 0){
		perror("read error\n");
		exit(1);
	}

	if (sscanf(rtn, "%*[^:]:%x>", &idx) != 1){
		usleep(500);
		if (rty_cnt > 0){
			rty_cnt--;
			goto cli_rtn_retry;
		}
	}

}

static void uart_gas_write(struct switchtec_dev *dev, void __gas *dest, const void *src, size_t n)
{
	int ret = 0;
	int rty_cnt = 3;
	char gas_wr[1024];
	char gas_wr_rtn[102400];
	int idx;
	memset(gas_wr, 0, sizeof(gas_wr));
	struct switchtec_uart *udev =  to_switchtec_uart(dev);
	char *b = gas_wr;
	int wr_dw_len = n/4;

	uint32_t addr = (uint32_t)(dest - (void __gas *)dev->gas_map);
	b+= snprintf(b, sizeof(gas_wr), "gaswr 0x%x", addr);
	for (int i = 0; i<n; i+=4 ){
		b+= snprintf(b, sizeof(gas_wr), " 0x%x", *(uint32_t *)src);
		src += 4;
	}
	b+= snprintf(b, sizeof(gas_wr), "\r");
	
	ret = write(udev->fd, gas_wr, strlen(gas_wr));
	if (ret != strlen(gas_wr)){
		perror("write error\n");
		exit(1);
	}

	ret = tcdrain(udev->fd);
	if (ret < 0){
		perror("tcdrain error\n");
		exit(1);
	}
	
	usleep((GASWR_HDR_DWORDS+wr_dw_len)*UART_DWORD_DELAY_US);
	usleep((wr_dw_len*GASWR_RTN_DWORDS)*UART_DWORD_DELAY_US);

	memset(gas_wr_rtn, 0, sizeof(gas_wr_rtn));
wr_rtn_retry:
	ret = read(udev->fd, gas_wr_rtn + strlen(gas_wr_rtn), sizeof(gas_wr_rtn));
	if (ret < 0){
		perror("read error\n");
		exit(1);
	}

	if(sscanf(gas_wr_rtn, "%*[^:]:%x>", &idx) != 1){
		usleep(500);
		if (rty_cnt > 0){
			rty_cnt--;
			goto wr_rtn_retry;
		}
	}	

}

static void uart_memcpy_to_gas(struct switchtec_dev *dev, void __gas *dest, const void *src, size_t n)
{
	size_t cnt;
	size_t res;
	size_t rem;	

	while(n){
		res = n/4;
		rem = n%4; 
		cnt = (res + (rem ? 1 : 0)) > GASWR_MAX_PAYLOAD ? GASWR_MAX_PAYLOAD : (res + (rem ? 1: 0));
		uart_gas_write(dev, dest, src, cnt*4);
		dest += (cnt*4);
		src += (cnt*4);
		n -= (cnt*4) > n ? n : cnt*4;   	
	}
}

static void uart_gas_write8(struct switchtec_dev *dev, uint8_t val, uint8_t __gas *addr)
{
	fprintf(stderr, "gas_write8 is not supported for uart accesses\n");
	exit(1);
}

static void uart_gas_write16(struct switchtec_dev *dev, uint16_t val, uint16_t __gas *addr)
{
	fprintf(stderr, "gas_write16 is not supported for uart accesses\n");
	exit(1);
}
static void uart_gas_write32(struct switchtec_dev *dev, uint32_t val, uint32_t __gas *addr)
{
	uart_gas_write(dev, addr, &val, sizeof(uint32_t));
}
static void uart_gas_write64(struct switchtec_dev *dev, uint64_t val, uint64_t __gas *addr)
{
	uart_gas_write(dev, addr, &val, sizeof(uint64_t));
}

static const struct switchtec_ops uart_ops = {
	.close = uart_close,
	.gas_map = uart_gas_map,

	.cmd = gasop_cmd,
	.get_fw_version = gasop_get_fw_version,
	.pff_to_port = gasop_pff_to_port,
	.port_to_pff = gasop_port_to_pff,
	.flash_part = gasop_flash_part,
	.event_summary = gasop_event_summary,
	.event_ctl = gasop_event_ctl,
	.event_wait_for = gasop_event_wait_for,

	.gas_read8 = uart_gas_read8,
	.gas_read16 = uart_gas_read16,	
	.gas_read32 = uart_gas_read32,
	.gas_read64 = uart_gas_read64,
	.gas_write8 = uart_gas_write8,
	.gas_write16 = uart_gas_write16,
	.gas_write32 = uart_gas_write32,
	.gas_write64 = uart_gas_write64,
	
	.memcpy_to_gas = uart_memcpy_to_gas,
	.memcpy_from_gas = uart_memcpy_from_gas,
	.write_from_gas = NULL,
};

static int set_uart_attribs(int fd, int speed, int parity)
{
	struct termios uart_attribs;
	memset(&uart_attribs, 0, sizeof(uart_attribs));
	
	if (tcgetattr(fd, &uart_attribs) != 0)
	{
		return -1;
	}

	cfsetospeed(&uart_attribs, speed);
	cfsetispeed(&uart_attribs, speed);

	uart_attribs.c_iflag &= ~IGNBRK;
	uart_attribs.c_iflag &= ~(IXON | IXOFF | IXANY);
	uart_attribs.c_lflag = 0;
	uart_attribs.c_oflag = 0;
	uart_attribs.c_cflag = (uart_attribs.c_cflag & ~CSIZE) | CS8;
	uart_attribs.c_cflag |= (CLOCAL | CREAD);
	uart_attribs.c_cflag &= ~(PARENB | PARODD);
	uart_attribs.c_cflag |= parity;
	uart_attribs.c_cflag &= ~CSTOPB;
	uart_attribs.c_cflag &= ~CRTSCTS;
	uart_attribs.c_cc[VMIN] = 0;
	uart_attribs.c_cc[VTIME] = 5;

	if (tcsetattr(fd, TCSANOW, &uart_attribs))
		return -1;

	return 0;
}	

struct switchtec_dev *switchtec_open_uart(const char *path)
{
	struct switchtec_uart *udev;
	
	udev = malloc(sizeof(*udev));
	if (!udev)
		return NULL;

	udev->fd = open(path, O_RDWR | O_NOCTTY);
	if (udev->fd < 0)
		goto err_free;
	
	if (set_uart_attribs(udev->fd, B230400, 0) != 0)
		goto err_close_free;
			
	if (map_gas(&udev->dev))
		goto err_close_free;
	
	udev->dev.ops = &uart_ops;
	cli_dbginfo_off(&udev->dev);
	gasop_set_partition_info(&udev->dev); return &udev->dev; 
	
err_close_free:
	close(udev->fd);

err_free:
	free(udev);
	return NULL;
}

#endif

