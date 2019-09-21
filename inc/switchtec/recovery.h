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

#ifndef LIBSWITCHTEC_RECOVERY_H
#define LIBSWITCHTEC_RECOVERY_H

#define SWITCHTEC_MB_LOG_LEN	64

#define SWITCHTEC_PUB_KEY_LEN	512
#define SWITCHTEC_SIG_LEN	512
#define SWITCHTEC_KMSK_LEN	64
#define SWITCHTEC_UDS_LEN	32

#define SWITCHTEC_KMSK_NUM	4

struct switchtec_sn_ver_info {
	unsigned int chip_serial;
	unsigned int ver_km;
	unsigned int ver_bl2;
	unsigned int ver_main;
	unsigned int ver_sec_unlock;
};

enum switchtec_debug_mode {
	SWITCHTEC_DEBUG_MODE_ENABLED,
	SWITCHTEC_DEBUG_MODE_DISABLED_BUT_ENABLE_ALLOWED,
	SWITCHTEC_DEBUG_MODE_DISABLED
};

enum switchtec_secure_state {
	SWITCHTEC_UNINITIALIZED_UNSECURED,
	SWITCHTEC_INITIALIZED_UNSECURED,
	SWITCHTEC_INITIALIZED_SECURED
};

enum switchtec_attst_mode {
	SWITCHTEC_ATTST_MODE_NONE,
	SWITCHTEC_ATTST_MODE_DICE
};

enum switchtec_spi_clk_rate {
	SWITCHTEC_SPI_RATE_100M = 1,
	SWITCHTEC_SPI_RATE_67M,
	SWITCHTEC_SPI_RATE_50M,
	SWITCHTEC_SPI_RATE_40M,
	SWITCHTEC_SPI_RATE_33_33M,
	SWITCHTEC_SPI_RATE_28_57M,
	SWITCHTEC_SPI_RATE_25M,
	SWITCHTEC_SPI_RATE_22_22M,
	SWITCHTEC_SPI_RATE_20M,
	SWITCHTEC_SPI_RATE_18_18M
};

struct switchtec_security_cfg_stat {
	unsigned char basic_setting_valid;
	unsigned char public_key_exp_valid;
	unsigned char public_key_num_valid;
	unsigned char public_key_ver_valid;
	unsigned char public_key_valid;

	enum switchtec_debug_mode debug_mode;
	enum switchtec_secure_state secure_state;
	enum switchtec_attst_mode attest_mode;

	unsigned char jtag_lock_after_reset;
	unsigned char jtag_lock_after_bl1;
	unsigned char jtag_bl1_unlock_allowed;
	unsigned char jtag_post_bl1_unlock_allowed;

	enum switchtec_spi_clk_rate spi_clk_rate;
	unsigned int i2c_recovery_tmo;
	unsigned int i2c_port;
	unsigned int i2c_addr;
	unsigned int i2c_cmd_map;
	unsigned int public_key_exponent;
	unsigned int cdi_valid;
	unsigned int public_key_num;
	unsigned int public_key_ver;

	unsigned char public_key[SWITCHTEC_KMSK_NUM][SWITCHTEC_KMSK_LEN];
	unsigned char uds_key[SWITCHTEC_UDS_LEN];
};

struct switchtec_security_cfg_set {
	enum switchtec_attst_mode attest_mode;

	unsigned char jtag_lock_after_reset;
	unsigned char jtag_lock_after_bl1;
	unsigned char jtag_bl1_unlock_allowed;
	unsigned char jtag_post_bl1_unlock_allowed;

	unsigned int spi_clk_rate;
	unsigned int i2c_recovery_tmo;
	unsigned int i2c_port;
	unsigned int i2c_addr;
	unsigned int i2c_cmd_map;
	unsigned int public_key_exponent;
	unsigned int uds_valid;
	unsigned char uds[SWITCHTEC_UDS_LEN];
};

enum switchtec_active_index_id {
	SWITCHTEC_ACTIVE_INDEX_0 = 0,
	SWITCHTEC_ACTIVE_INDEX_1 = 1,
	SWITCHTEC_ACTIVE_INDEX_NOT_SET = 0xfe
};


struct switchtec_active_index {
	enum switchtec_active_index_id bl2;
	enum switchtec_active_index_id firmware;
	enum switchtec_active_index_id config;
	enum switchtec_active_index_id keyman;
};

enum switchtec_bl2_recovery_mode {
	SWITCHTEC_BL2_RECOVERY_I2C = 1,
	SWITCHTEC_BL2_RECOVERY_XMODEM = 2,
	SWITCHTEC_BL2_RECOVERY_I2C_AND_XMODEM = 3
};

enum switchtec_read_kmsk_error {
	SWITCHTEC_KMSK_FILE_ERROR_LEN = 1,
	SWITCHTEC_KMSK_FILE_ERROR_SIG,
	SWITCHTEC_KMSK_FILE_ERROR_CRC,
};

enum switchtec_parse_setting_error {
	SWITCHTEC_SETTING_FILE_ERROR_LEN = 1,
	SWITCHTEC_SETTING_FILE_ERROR_SIG,
	SWITCHTEC_SETTING_FILE_ERROR_CRC,
	SWITCHTEC_SETTING_FILE_ERROR_UDS
};

int switchtec_ping(struct switchtec_dev *dev,
		   unsigned int ping_dw,
		   unsigned int *reply_dw);
int switchtec_get_boot_phase(struct switchtec_dev *dev,
			     enum switchtec_boot_phase *phase_id);
int switchtec_sn_ver_get(struct switchtec_dev *dev,
			 struct switchtec_sn_ver_info *info);
int switchtec_security_config_get(struct switchtec_dev *dev,
			          struct switchtec_security_cfg_stat *state);
int switchtec_security_config_set(struct switchtec_dev *dev,
				  struct switchtec_security_cfg_set *setting);
int switchtec_mailbox_get(struct switchtec_dev *dev, int fd);
int switchtec_active_image_index_get(struct switchtec_dev *dev,
				     struct switchtec_active_index *index);
int switchtec_active_image_index_set(struct switchtec_dev *dev,
				     struct switchtec_active_index *index);
int switchtec_fw_exec(struct switchtec_dev *dev,
		      enum switchtec_bl2_recovery_mode recovery_mode);
int switchtec_kmsk_set(struct switchtec_dev *dev,
		       unsigned char *public_key,
		       unsigned int public_key_exp,
		       unsigned char *signature,
		       unsigned char *kmsk_entry_data);
int switchtec_secure_state_set(struct switchtec_dev *dev,
			       enum switchtec_secure_state state);
int switchtec_boot_resume(struct switchtec_dev *dev);
int switchtec_dport_unlock(struct switchtec_dev *dev,
			   unsigned int serial,
			   unsigned int ver_sec_unlock,
			   unsigned char *public_key,
			   unsigned int public_key_exp,
			   unsigned char *signatue);
int switchtec_secure_unlock_version_update(struct switchtec_dev *dev,
					   unsigned int serial,
					   unsigned int ver_sec_unlock,
					   unsigned char *public_key,
					   unsigned int public_key_exp,
					   unsigned char *signatue);
int switchtec_read_pubk_file(FILE *pubk_file, unsigned char *pubk,
			     unsigned int *exp);
int switchtec_read_kmsk_file(FILE *kmsk_file, unsigned char *kmsk);
int switchtec_read_sec_cfg_file(FILE *setting_file,
			       	struct switchtec_security_cfg_set *s);

#endif // LIBSWITCHTEC_RECOVERY_H
