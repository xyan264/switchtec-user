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
#include <stdbool.h>
#include "../crc8.h"

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
#define GASWR_MAX_CRC_PAYLOAD			(12)
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
	uint8_t crc_cal;
	int crc_rtn;
	uint8_t crc_buf[2*n];
	uint8_t *crc_ptr = crc_buf;
	uint32_t crc_cnt;
	uint32_t crc_addr;
	
	struct switchtec_uart *udev = to_switchtec_uart(dev);
	
	uint32_t addr = (uint32_t)(src - (void __gas *)dev->gas_map);

	rd_dw_len = (n/4 + (n%4?1:0));
	b = gas_rd;
	b += snprintf(b, sizeof(gas_rd), "gasrd -c 0x%x", addr);
	b += snprintf(b, sizeof(gas_rd), " %d", rd_dw_len);
 	b += snprintf(b, sizeof(gas_rd), "\r");

rd_crc_err:	
	memset(gas_rd_rtn, 0, sizeof(gas_rd_rtn));
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

rd_rtn_retry:
	ret = read(udev->fd, gas_rd_rtn + strlen(gas_rd_rtn), sizeof(gas_rd_rtn));
	if (ret < 0){
		perror("read error\n");
		exit(1);
	}

	if(sscanf(gas_rd_rtn, "%*[^<]<0x%x> [%d]%*[^:]: 0x%x%*[^:]:%x>", &raddr, &rnum, &crc_rtn, &idx) != 4){
		usleep(2000);
		if (rty_cnt > 0){
			rty_cnt--;
			goto rd_rtn_retry;
		}
	}

	assert(raddr == addr);
	assert(rnum == rd_dw_len);
	
	crc_addr = htobe32(addr);
	memcpy(crc_ptr, &crc_addr, sizeof(crc_addr));
	crc_ptr += sizeof(addr);

	tok = strtok(gas_rd_rtn, delims);
	tok = strtok(NULL, delims);
	for (int i = 0; i<rnum; i++){
		tok = strtok(NULL, delims);
		*(uint32_t *)crc_ptr = htobe32(strtol(tok, &tok, 0));
		crc_ptr += 4;
	}

	crc_cnt = sizeof(addr) + rd_dw_len*4;
	crc_cal = crc8(crc_buf, crc_cnt, 0, true);
	if (crc_cal != crc_rtn){
		usleep(2000);
		if (rty_cnt > 0){
			rty_cnt--;
			goto rd_crc_err;
		}
	}

	crc_ptr = crc_buf + sizeof(addr);
	for (crc_cnt = 0; crc_cnt < rd_dw_len; crc_cnt+=4){
		*(uint32_t *)crc_ptr = be32toh(*(uint32_t *)crc_ptr);
		crc_ptr +=4;
	}
	memcpy(dest, crc_buf+sizeof(addr), rd_dw_len*4);

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
		usleep(2000);
		if (rty_cnt > 0){
			rty_cnt--;
			goto cli_rtn_retry;
		}
	}

}

static void uart_gas_write(struct switchtec_dev *dev, void __gas *dest, const void *src, 
		size_t n, uint32_t mask)
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
	uint8_t crc_buf[2*n];
	uint8_t *crc_ptr = crc_buf;
	uint32_t crc;
	uint32_t crc_cal, crc_exp;
	uint32_t crc_cnt;

	uint32_t addr = (uint32_t)(dest - (void __gas *)dev->gas_map);
	if (mask){
		memcpy(crc_buf, &addr, sizeof(addr));
		memcpy(crc_buf+sizeof(addr), (uint8_t *)src, n);
		memcpy(crc_buf+sizeof(addr)+n, &mask, sizeof(mask));
		for(crc_cnt = 0; crc_cnt < (sizeof(addr)+n+sizeof(mask)); crc_cnt+=4){
			*(uint32_t *)crc_ptr = htobe32(*(uint32_t *)crc_ptr);
			crc_ptr += 4;
		}
		crc = crc8(crc_buf, sizeof(addr)+n+sizeof(mask), 0, true);
	}else{
		memcpy(crc_buf, &addr, sizeof(addr));
		memcpy(crc_buf+sizeof(addr), (uint8_t *)src, n);
		for(crc_cnt = 0; crc_cnt < (sizeof(addr)+n); crc_cnt+=4){
			*(uint32_t *)crc_ptr = htobe32(*(uint32_t *)crc_ptr);
			crc_ptr += 4;
		}
		crc = crc8(crc_buf, sizeof(addr)+n, 0, true);
	}

	if(mask)
		b+= snprintf(b, sizeof(gas_wr), "gaswr -c -m 0x%x", addr);
	else
		b+= snprintf(b, sizeof(gas_wr), "gaswr -c 0x%x", addr);

	for (int i = 0; i<n; i+=4 ){
		b+= snprintf(b, sizeof(gas_wr), " 0x%x", *(uint32_t *)src);
		src += 4;
	}
	if(mask)
		b+= snprintf(b, sizeof(gas_wr), " 0x%x 0x%x\r", mask, crc);
	else
		b+= snprintf(b, sizeof(gas_wr), " 0x%x\r", crc);

wr_crc_err:	
	memset(gas_wr_rtn, 0, sizeof(gas_wr_rtn));
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

wr_rtn_retry:
	ret = read(udev->fd, gas_wr_rtn + strlen(gas_wr_rtn), sizeof(gas_wr_rtn));
	if (ret < 0){
		perror("read error\n");
		exit(1);
	}

	if (sscanf(gas_wr_rtn, "%*[^:]: [0x%x/0x%x]%*[^:]:%x>", &crc_cal, &crc_exp, &idx) != 3){
		usleep(2000);
		if (rty_cnt > 0){
			rty_cnt--;
			goto wr_rtn_retry;
		}
	}

	rty_cnt = 3;
	if ((crc_exp != crc_cal) && (crc_cal != crc)){
		usleep(2000);
		if (rty_cnt > 0){
			rty_cnt--;
			goto wr_crc_err;	
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
		cnt = (res + (rem ? 1 : 0)) > GASWR_MAX_CRC_PAYLOAD ? GASWR_MAX_CRC_PAYLOAD : (res + (rem ? 1: 0));
		uart_gas_write(dev, dest, src, cnt*4, 0);
		dest += (cnt*4);
		src += (cnt*4);
		n -= (cnt*4) > n ? n : cnt*4;   	
	}
}

static void uart_gas_write8(struct switchtec_dev *dev, uint8_t val, uint8_t __gas *addr)
{
	uint32_t mask = 0;
	uint32_t tmpaddr;
	uint32_t tmpval = val;
	tmpaddr = (uint32_t)((void __gas *)addr - (void __gas *)0);
	switch (tmpaddr & 0x3){
		case 0:
		mask = 0xff;
		tmpval = tmpval << 0;
		break;
		case 1:
		mask = 0xff << 8;
		tmpval = tmpval << 8;
		break;
		case 2:
		mask = 0xff << 16;
		tmpval = tmpval << 16;
		break;
		case 3:
		mask = 0xff << 24;
		tmpval = tmpval << 24;
		break;
	}
	tmpaddr &= ~0x3;
	uart_gas_write(dev, tmpaddr + (void __gas *)0, &tmpval, sizeof(uint32_t), mask);
}

static void uart_gas_write16(struct switchtec_dev *dev, uint16_t val, uint16_t __gas *addr)
{
	uint32_t mask = 0, mask_sat = 0;
	uint32_t tmpaddr;
	uint32_t tmpval = val, tmpval_sat = 0;
	tmpaddr = (uint32_t)((void __gas *)addr - (void __gas *)0);
	switch (tmpaddr & 0x3){
		case 0:
		mask = 0xffff;
		tmpval = tmpval << 0;
		break;
		case 1:
		mask = 0xffff << 8;
		tmpval = tmpval << 8;
		break;
		case 2:
		mask = 0xffff << 16;
		tmpval = tmpval << 16;
		break;
		case 3:
		mask = 0xffff << 24;
		tmpval = tmpval << 24;
		mask_sat = 0xffff >> 8;
		tmpval_sat = val >> 8;
		break;
	}
	tmpaddr &= ~0x3;
	uart_gas_write(dev, tmpaddr + (void __gas *)0, &tmpval, sizeof(uint32_t), mask);
	if (mask_sat)
		uart_gas_write(dev, tmpaddr + (void __gas *)4, &tmpval_sat, sizeof(uint32_t), mask_sat);
}
static void uart_gas_write32(struct switchtec_dev *dev, uint32_t val, uint32_t __gas *addr)
{
	uint32_t mask = 0, mask_sat = 0;
	uint32_t tmpaddr;
	uint32_t tmpval = val, tmpval_sat = 0;
	tmpaddr = (uint32_t)((void __gas *)addr - (void __gas *)0);
	switch (tmpaddr & 0x3){
		case 0:
		mask = 0xffffffff;
		tmpval = tmpval << 0;
		break;
		case 1:
		mask = 0xffffffff << 8;
		tmpval = tmpval << 8;
		mask_sat = 0xffffffff >> 24;
		tmpval_sat = val >> 24;
		break;
		case 2:
		mask = 0xffffffff << 16;
		tmpval = tmpval << 16;
		mask_sat = 0xffffffff >> 16;
		tmpval_sat = val >> 16;
		break;
		case 3:
		mask = 0xffffffff << 24;
		tmpval = tmpval << 24;
		mask_sat = 0xffffffff >> 8;
		tmpval_sat = val >> 8;
		break;
	}
	tmpaddr &= ~0x3;
	uart_gas_write(dev, tmpaddr + (void __gas *)0, &tmpval, sizeof(uint32_t), mask);
	if (mask_sat)
		uart_gas_write(dev, tmpaddr + (void __gas *)4, &tmpval_sat, sizeof(uint32_t),mask_sat);
}
static void uart_gas_write64(struct switchtec_dev *dev, uint64_t val, uint64_t __gas *addr)
{
	uint32_t mask_low = 0, mask_middle = 0, mask_high = 0;
	uint32_t tmpaddr;
	uint32_t tmpval_low, tmpval_middle, tmpval_high;
	tmpaddr = (uint32_t)((void __gas *)addr - (void __gas *)0);
	switch (tmpaddr & 0x3){
		case 0:
		mask_low = 0xffffffff;
		tmpval_low = val;
		mask_middle = 0xffffffff;
		tmpval_middle = val >> 32;
		mask_high = 0;
		tmpval_high = 0;
		break;
		case 1:
		mask_low = 0xffffffff << 8;
		tmpval_low = val << 8;
		mask_middle = 0xffffffff;
		tmpval_middle = (val << 8) >> 32;
		mask_high = 0xffffffff >> 24;
		tmpval_high = (val >> 32) >> 24;
		break;
		case 2:
		mask_low = 0xffffffff << 16;
		tmpval_low = val << 16;
		mask_middle = 0xffffffff;
		tmpval_middle = (val << 16) >> 32;
		mask_high = 0xffffffff >> 16;
		tmpval_high = (val >> 32) >> 16;
		break;
		case 3:
		mask_low = 0xffffffff << 24;
		tmpval_low = val << 24;
		mask_middle = 0xffffffff;
		tmpval_middle = (val << 24) >> 32;
		mask_high = 0xffffffff >> 8;
		tmpval_high = (val >> 32) >> 8;
		break;
	}
	tmpaddr &= ~0x3;
	uart_gas_write(dev, tmpaddr + (void __gas *)0, &tmpval_low, sizeof(uint32_t), mask_low);
	if (mask_middle)
		uart_gas_write(dev, tmpaddr + (void __gas *)4, &tmpval_middle, sizeof(uint32_t), mask_middle);
	if (mask_high)
		uart_gas_write(dev, tmpaddr + (void __gas *)8, &tmpval_high, sizeof(uint32_t), mask_high);

}

static ssize_t uart_write_from_gas(struct switchtec_dev *dev, int fd,
				  const void __gas *src, size_t n)
{
	ssize_t ret;
	void *buf;

	buf = malloc(n);

	uart_memcpy_from_gas(dev, buf, src, n);
	ret = write(fd, buf, n);

	free(buf);

	return ret;
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
	.write_from_gas = uart_write_from_gas,
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

