/*
 * Microsemi Switchtec(tm) PCIe Management Library
 * Copyright (c) 2019, Microsemi Corporation
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

/**
 * @file
 * @brief Switchtec core library functions for mfg operations
 */

#include "switchtec_priv.h"
#include "switchtec/switchtec.h"
#include "switchtec/mfg.h"
#include "switchtec/errors.h"
#include "switchtec/endian.h"
#include "switchtec/mrpc.h"
#include "switchtec/errors.h"
#include <unistd.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "openssl/pem.h"


#include "lib/crc32.h"
#include "config.h"
#define SWITCHTEC_ACTV_IMG_ID_KMAN		1
#define SWITCHTEC_ACTV_IMG_ID_BL2		2
#define SWITCHTEC_ACTV_IMG_ID_CFG		3
#define SWITCHTEC_ACTV_IMG_ID_FW		4

#define SWITCHTEC_MB_MAX_ENTRIES		16
#define SWITCHTEC_ACTV_IDX_MAX_ENTRIES		32
#define SWITCHTEC_ACTV_IDX_SET_ENTRIES		4

#define SWITCHTEC_CLK_RATE_BITSHIFT		10
#define SWITCHTEC_RC_TMO_BITSHIFT		14
#define SWITCHTEC_I2C_PORT_BITSHIFT		18
#define SWITCHTEC_I2C_ADDR_BITSHIFT		22
#define SWITCHTEC_CMD_MAP_BITSHIFT		29

#if !HAVE_DECL_RSA_GET0_KEY
/**
*  openssl1.0 or older versions don't have this function, so copy
*  the code from openssl1.1 here
*/
static void RSA_get0_key(const RSA *r, const BIGNUM **n,
			 const BIGNUM **e, const BIGNUM **d)
{
	if (n != NULL)
		*n = r->n;
	if (e != NULL)
		*e = r->e;
	if (d != NULL)
		*d = r->d;
}
#endif

/**
 * @brief Get serial number and security version
 * @param[in]  dev	Switchtec device handle
 * @param[out] info	Serial number and security version info
 * @return 0 on success, error code on failure
 */
int switchtec_sn_ver_get(struct switchtec_dev *dev,
			 struct switchtec_sn_ver_info *info)
{
	int ret;

	ret = switchtec_cmd(dev, MRPC_SN_VER_GET, NULL, 0, info,
			    sizeof(struct switchtec_sn_ver_info));
	if (ret < 0)
		return ret;

	info->chip_serial = le32toh(info->chip_serial);
	info->ver_bl2 = le32toh(info->ver_bl2);
	info->ver_km = le32toh(info->ver_km);
	info->ver_main = le32toh(info->ver_main);
	info->ver_sec_unlock = le32toh(info->ver_sec_unlock);

	return 0;
}

/**
 * @brief Get secure boot configurations
 * @param[in]  dev	Switchtec device handle
 * @param[out] state	Current secure boot settings
 * @return 0 on success, error code on failure
 */
int switchtec_security_config_get(struct switchtec_dev *dev,
				  struct switchtec_security_cfg_stat *state)
{
	int ret;
	struct cfg_reply {
		uint32_t valid;
		uint32_t rsvd1;
		uint64_t cfg;
		uint32_t  public_key_exponent;
		uint8_t  rsvd2;
		uint8_t  public_key_num;
		uint8_t  public_key_ver;
		uint8_t  rsvd3;
		uint8_t  public_key[SWITCHTEC_KMSK_NUM][SWITCHTEC_KMSK_LEN];
		uint8_t  rsvd4[32];
	} reply;

	ret = switchtec_cmd(dev, MRPC_SECURITY_CONFIG_GET, NULL, 0,
			    &reply, sizeof(reply));
	if (ret < 0)
		return ret;

	reply.valid = le32toh(reply.valid);
	reply.cfg = le64toh(reply.cfg);
	reply.public_key_exponent = le32toh(reply.public_key_exponent);

	state->basic_setting_valid = !!(reply.valid & 0x01);
	state->public_key_exp_valid = !!(reply.valid & 0x02);
	state->public_key_num_valid = !!(reply.valid & 0x04);
	state->public_key_ver_valid = !!(reply.valid & 0x08);
	state->public_key_valid = !!(reply.valid & 0x10);

	state->debug_mode = reply.cfg & 0x03;
	state->secure_state = (reply.cfg>>2) & 0x03;

	state->jtag_lock_after_reset = !!(reply.cfg & 0x40);
	state->jtag_lock_after_bl1 = !!(reply.cfg & 0x80);
	state->jtag_bl1_unlock_allowed = !!(reply.cfg & 0x0100);
	state->jtag_post_bl1_unlock_allowed = !!(reply.cfg & 0x0200);

	state->spi_clk_rate =
		(reply.cfg >> SWITCHTEC_CLK_RATE_BITSHIFT) & 0x0f;
	if (state->spi_clk_rate == 0)
		state->spi_clk_rate = SWITCHTEC_SPI_RATE_25M;

	state->i2c_recovery_tmo =
		(reply.cfg >> SWITCHTEC_RC_TMO_BITSHIFT) & 0x0f;
	state->i2c_port = (reply.cfg >> SWITCHTEC_I2C_PORT_BITSHIFT) & 0xf;
	state->i2c_addr = (reply.cfg >> SWITCHTEC_I2C_ADDR_BITSHIFT) & 0x7f;
	state->i2c_cmd_map =
		(reply.cfg >> SWITCHTEC_CMD_MAP_BITSHIFT) & 0xfff;

	state->public_key_exponent = reply.public_key_exponent;
	state->public_key_num = reply.public_key_num;
	state->public_key_ver = reply.public_key_ver;
	memcpy(state->public_key, reply.public_key,
	       SWITCHTEC_KMSK_NUM * SWITCHTEC_KMSK_LEN);

	return 0;
}

/**
 * @brief Retrieve mailbox entries
 * @param[in]  dev	Switchtec device handle
 * @param[in]  fd	File handle to write the log data
 * @return 0 on success, error code on failure
 */
int switchtec_mailbox_to_file(struct switchtec_dev *dev, int fd)
{
	int ret;
	int num_to_read = htole32(SWITCHTEC_MB_MAX_ENTRIES);
	struct mb_reply {
		uint8_t num_returned;
		uint8_t num_remaining;
		uint8_t rsvd[2];
		uint8_t data[SWITCHTEC_MB_MAX_ENTRIES *
			     SWITCHTEC_MB_LOG_LEN];
	} reply;

	do {
		ret = switchtec_cmd(dev, MRPC_MAILBOX_GET, &num_to_read,
				    sizeof(int), &reply,  sizeof(reply));
		if (ret < 0)
			return ret;

		reply.num_remaining = le32toh(reply.num_remaining);
		reply.num_returned = le32toh(reply.num_returned);

		ret = write(fd, reply.data,
			    (reply.num_returned) * SWITCHTEC_MB_LOG_LEN);
		if (ret < 0)
			return ret;
	} while (reply.num_remaining > 0);

	return 0;
}

/**
 * @brief Get active image index
 * @param[in]  dev	Switchtec device handle
 * @param[out] index	Active images indices
 * @return 0 on success, error code on failure
 */
int switchtec_active_image_index_get(struct switchtec_dev *dev,
				     struct switchtec_active_index *index)
{
	int ret;
	struct active_indices {
		uint8_t index[SWITCHTEC_ACTV_IDX_MAX_ENTRIES];
	} reply;

	ret = switchtec_cmd(dev, MRPC_ACT_IMG_IDX_GET, NULL,
			    0, &reply, sizeof(reply));
	if (ret < 0)
		return ret;

	index->keyman = reply.index[SWITCHTEC_ACTV_IMG_ID_KMAN];
	index->bl2 = reply.index[SWITCHTEC_ACTV_IMG_ID_BL2];
	index->config = reply.index[SWITCHTEC_ACTV_IMG_ID_CFG];
	index->firmware = reply.index[SWITCHTEC_ACTV_IMG_ID_FW];

	return 0;
}

/**
 * @brief Set active image index
 * @param[in]  dev	Switchtec device handle
 * @param[in] index	Active image indices
 * @return 0 on success, error code on failure
 */
int switchtec_active_image_index_set(struct switchtec_dev *dev,
				     struct switchtec_active_index *index)
{
	int ret;
	int i = 0;
	struct active_idx {
		uint32_t count;
		struct entry {
			uint8_t image_id;
			uint8_t index;
		} idx[SWITCHTEC_ACTV_IDX_SET_ENTRIES];
	} set;

	memset(&set, 0, sizeof(set));

	if (index->keyman != SWITCHTEC_ACTIVE_INDEX_NOT_SET) {
		set.idx[i].image_id = SWITCHTEC_ACTV_IMG_ID_KMAN;
		set.idx[i].index = index->keyman;
		i++;
	}

	if (index->bl2 != SWITCHTEC_ACTIVE_INDEX_NOT_SET) {
		set.idx[i].image_id = SWITCHTEC_ACTV_IMG_ID_BL2;
		set.idx[i].index = index->bl2;
		i++;
	}

	if (index->config != SWITCHTEC_ACTIVE_INDEX_NOT_SET) {
		set.idx[i].image_id =  SWITCHTEC_ACTV_IMG_ID_CFG;
		set.idx[i].index = index->config;
		i++;
	}

	if (index->firmware != SWITCHTEC_ACTIVE_INDEX_NOT_SET) {
		set.idx[i].image_id = SWITCHTEC_ACTV_IMG_ID_FW;
		set.idx[i].index = index->firmware;
		i++;
	}

	if (i == 0)
		return 0;

	set.count = htole32(i);

	ret = switchtec_cmd(dev, MRPC_ACT_IMG_IDX_SET, &set,
			    sizeof(set), NULL, 0);
	return ret;
}

/**
 * @brief Execute the transferred firmware
 * @param[in]  dev		Switchtec device handle
 * @param[in]  recovery_mode	Recovery mode in case of a boot failure
 * @return 0 on success, error code on failure
 */
int switchtec_fw_exec(struct switchtec_dev *dev,
		      enum switchtec_bl2_recovery_mode recovery_mode)
{
	struct fw_exec_struct {
		uint8_t subcmd;
		uint8_t recovery_mode;
		uint8_t rsvd[2];
	} cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.subcmd = MRPC_FW_TX_EXEC;
	cmd.recovery_mode = recovery_mode;

	return switchtec_cmd(dev, MRPC_FW_TX, &cmd, sizeof(cmd), NULL, 0);
}

/**
 * @brief Resume device boot.
 *        Note that after calling this function, the current
 *        'dev' pointer is no longer valid. Before making further
 *        calls to switchtec library functions, be sure to close
 *        this pointer and get a new one by calling switchtec_open().
 * 	  Also be sure to check the return value of switchtec_open()
 * 	  for error, as the device might not be immediately 
 * 	  accessible after normal boot process.
 * @param[in]  dev	Switchtec device handle
 * @return 0 on success, error code on failure
 */
int switchtec_boot_resume(struct switchtec_dev *dev)
{
	return switchtec_cmd(dev, MRPC_BOOTUP_RESUME, NULL, 0,
			     NULL, 0);
}

/**
 * @brief Set KMSK entry
 * @param[in]  dev		Switchtec device handle
 * @param[in]  public_key	Public key
 * @param[in]  public_key_exp	Public key exponent
 * @param[in]  signature	Signature
 * @param[in]  kmsk_entry_data	KMSK entry data
 * @return 0 on success, error code on failure
 */
int switchtec_kmsk_set(struct switchtec_dev *dev,
		       uint8_t *public_key,
		       uint32_t public_key_exp,
		       uint8_t *signature,
		       uint8_t *kmsk_entry_data)
{
	int ret;
	struct kmsk_cmd1 {
		uint8_t subcmd;
		uint8_t reserved[3];
		uint8_t pub_key[SWITCHTEC_PUB_KEY_LEN];
		uint32_t pub_key_exponent;
	} cmd1;

	struct kmsk_cmd2 {
		uint8_t subcmd;
		uint8_t reserved[3];
		uint8_t signature[SWITCHTEC_SIG_LEN];
	} cmd2;

	struct kmsk_cmd3 {
		uint8_t subcmd;
		uint8_t num_entries;
		uint8_t reserved[2];
		uint8_t kmsk[SWITCHTEC_KMSK_LEN];
	} cmd3;

	if (public_key) {
		memset(&cmd1, 0, sizeof(cmd1));
		cmd1.subcmd = MRPC_KMSK_ENTRY_SET_PKEY;
		memcpy(cmd1.pub_key, public_key, SWITCHTEC_PUB_KEY_LEN);
		cmd1.pub_key_exponent = htole32(public_key_exp);

		ret = switchtec_cmd(dev, MRPC_KMSK_ENTRY_SET, &cmd1,
				    sizeof(cmd1), NULL, 0);
		if (ret < 0) {
			return ret;
		}
	}

	if (signature) {
		memset(&cmd2, 0, sizeof(cmd2));
		cmd2.subcmd = MRPC_KMSK_ENTRY_SET_SIG;
		memcpy(cmd2.signature, signature, SWITCHTEC_SIG_LEN);

		ret = switchtec_cmd(dev, MRPC_KMSK_ENTRY_SET, &cmd2,
			    	    sizeof(cmd2), NULL, 0);
		if (ret < 0) {
			return ret;
		}
	}

	memset(&cmd3, 0, sizeof(cmd3));
	cmd3.subcmd = MRPC_KMSK_ENTRY_SET_KMSK;
	cmd3.num_entries = 1;
	memcpy(cmd3.kmsk, kmsk_entry_data, SWITCHTEC_KMSK_LEN);

	ret = switchtec_cmd(dev, MRPC_KMSK_ENTRY_SET, &cmd3, sizeof(cmd3),
			    NULL, 0);
	return ret;
}


/**
 * @brief Set device secure state
 * @param[in]  dev	Switchtec device handle
 * @param[in]  state	Secure state
 * @return 0 on success, error code on failure
 */
int switchtec_secure_state_set(struct switchtec_dev *dev,
			       enum switchtec_secure_state state)
{
	uint32_t data;

	if ((state != SWITCHTEC_INITIALIZED_UNSECURED)
	   && (state != SWITCHTEC_INITIALIZED_SECURED)) {
		return ERR_PARAM_INVALID;
	}
	data = htole32(state);

	return switchtec_cmd(dev, MRPC_SECURE_STATE_SET, &data, sizeof(data),
			     NULL, 0);
}


/**
 * @brief Read public key from public key file
 * @param[in]  pubk_file Public key file
 * @param[out] pubk	 Public key
 * @param[out] exp	 Public key exponent
 * @return 0 on success, error code on failure
 */
int switchtec_read_pubk_file(FILE *pubk_file, uint8_t *pubk,
			     uint32_t *exp)
{
	RSA	     *RSAKey = NULL;
	const BIGNUM    *modulus_bn;
	const BIGNUM    *exponent_bn;
	uint32_t    exponent_tmp = 0;

	RSAKey = PEM_read_RSA_PUBKEY(pubk_file, NULL, NULL, NULL);
	if (RSAKey == NULL) {
		return -1;
	}

	RSA_get0_key(RSAKey, &modulus_bn, &exponent_bn, NULL);

	BN_bn2bin(modulus_bn, pubk);
	BN_bn2bin(exponent_bn, (uint8_t *)&exponent_tmp);

	*exp = be32toh(exponent_tmp);
	RSA_free(RSAKey);

	return 0;
}

/**
 * @brief Read KMSK data from KMSK file
 * @param[in]  kmsk_file KMSK file
 * @param[out] kmsk   	 KMSK entry data
 * @return 0 on success, error code on failure
 */
int switchtec_read_kmsk_file(FILE *kmsk_file, uint8_t *kmsk)
{
	ssize_t rlen;
	struct kmsk_struct {
		uint8_t magic[4];
		uint32_t version;
		uint32_t reserved;
		uint32_t crc32;
		uint8_t kmsk[SWITCHTEC_KMSK_LEN];
	} s;

	char magic[4] = {'K', 'M', 'S', 'K'};
	uint32_t crc;

	rlen = fread(&s, 1, sizeof(s), kmsk_file);

	if (rlen < sizeof(s))
		return -EBADF;

	if (memcmp(s.magic, magic, sizeof(magic)))
		return -EBADF;

	crc = crc32(s.kmsk, SWITCHTEC_KMSK_LEN, 0, 1, 1);
	if (crc != le32toh(s.crc32))
		return -EBADF;

	memcpy(kmsk, s.kmsk, SWITCHTEC_KMSK_LEN);

	return 0;
}
