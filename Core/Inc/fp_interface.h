/*
 * fp_interface.h
 *
 *  Created on: Nov 18, 2023
 *      Author: MSI
 */

#ifndef INC_FP_INTERFACE_H_
#define INC_FP_INTERFACE_H_

#include "main.h"
#include "usbd_cdc_if.h"
#include "string.h"

// Constants unless DEVICE_ADDRESS is changed by user
#define HEADER 0xEF01
#define DEVICE_ADDRESS 0xFFFFFFFF

#define CMD_PKG 0x01
#define PKG_SUBSEQUENT 0x02
#define RESP_PKG 0x07
#define END_PACKET 0x08

// Instruction codes
#define PS_GET_IMG 0x01
#define PS_GET_IMG_PKT_LEN 0x0003

#define PS_GEN_CHAR 0x02

#define PS_MATCH 0x03
#define PS_MATCH_LEN 0x0003

#define PS_WRITE_REG 0x0E
#define PS_WRITE_REG_DIR_LEN 0x0005

#define PS_READ_SYS_PARA 0x0F
#define PS_READ_SYS_PARA_DIR_LEN 0x0003

#define PS_READ_INF_PAGE 0x16
#define PS_READ_INF_PAGE_LEN 0x0003

#define PS_READ_VALID_TEMP 0x1D
#define PS_READ_VALID_TEMP_LEN 0x0003

#define PS_EMPTY_ALL 0x0D
#define PS_EMPTY_ALL_LEN 0x0003

#define PS_AUTO_ENROLL 0x31
#define PS_AUTO_ENROLL_LEN 0x0008

#define PS_AUTO_IDENT 0x32
#define PS_AUTO_IDENT_LEN 0x0008

// Diagnostic Messages
#define CONF_INSTRUCTION_OK 0x00
#define CONF_PACKET_RECEIVE_ERROR 0x01
#define CONF_NO_FINGER_DETECTED 0x02
#define CONF_FINGERPRINT_IMAGE_INPUT_FAIL 0x03
#define CONF_FINGERPRINT_TOO_DRY 0x04
#define CONF_FINGERPRINT_TOO_WET 0x05
#define CONF_FINGERPRINT_TOO_MESSY 0x06
#define CONF_FINGERPRINT_TOO_FEW_FEATURE_POINTS 0x07
#define CONF_FINGERPRINT_MISMATCH 0x08
#define CONF_NO_FINGERPRINT_FOUND 0x09
#define CONF_FEATURE_MERGE_FAIL 0x0A
#define CONF_ADDRESS_OUTSIDE_LIBRARY 0x0B
#define CONF_READ_TEMPLATE_ERROR 0x0C
#define CONF_UPLOAD_FEATURE_FAIL 0x0D
#define CONF_CANNOT_RECEIVE_PACKETS 0x0E
#define CONF_UPLOAD_IMAGE_FAIL 0x0F
#define CONF_TEMPLATE_DELETION_FAIL 0x10
#define CONF_EMPTY_LIBRARY_FAIL 0x11
#define CONF_CANNOT_ENTER_LOW_POWER 0x12
#define CONF_INCORRECT_PASSWORD 0x13
#define CONF_RESET_FAIL 0x14
#define CONF_NO_VALID_ORIGINAL_IMAGE 0x15
#define CONF_ONLINE_UPGRADE_FAIL 0x16
#define CONF_RESIDUAL_FINGERPRINT 0x17
#define CONF_READ_WRITE_FLASH_ERROR 0x18
#define CONF_RANDOM_NUMBER_GENERATION_FAIL 0x19
#define CONF_INVALID_REGISTER_NUMBER 0x1A
#define CONF_REGISTER_CONTENT_ERROR 0x1B
#define CONF_INVALID_NOTEPAD_PAGE_NUMBER 0x1C
#define CONF_PORT_OPERATION_FAIL 0x1D
#define CONF_AUTO_ENROLL_FAIL 0x1E
#define CONF_FINGERPRINT_LIBRARY_FULL 0x1F
#define CONF_DEVICE_ADDRESS_ERROR 0x20
#define CONF_INCORRECT_PASSWORD_DEVICE 0x21
#define CONF_TEMPLATE_NOT_EMPTY 0x22
#define CONF_TEMPLATE_EMPTY 0x23
#define CONF_LIBRARY_EMPTY 0x24
#define CONF_INCORRECT_ENTRY_TIMES 0x25
#define CONF_OVERTIME 0x26
#define CONF_ALREADY_EXISTS 0x27
#define CONF_FEATURES_ASSOCIATED 0x28
#define CONF_SENSOR_FAILED 0x29
#define CONF_MODULE_INFO_NOT_EMPTY 0x2A
#define CONF_MODULE_INFO_EMPTY 0x2B
#define CONF_OTP_FAILED 0x2C
#define CONF_SECRET_KEY_GEN_FAILED 0x2D
#define CONF_SECRET_KEY_MISSING 0x2E
#define CONF_ALGO_FAILED 0x2F
#define CONF_ENCRYPTION_DECRYPTION_WRONG 0x30
#define CONF_FUNCTIONALITY_MISMATCH 0x31
#define CONF_KEY_LOCKED 0x32
#define CONF_SMALL_IMAGE_AREA 0x33
#define CONF_IMAGES_NOT_AVAILABLE 0x34
#define CONF_ILLEGAL_DATA 0x35

uint8_t is_fp_detected(void);
void clear_fp_flag(void);

uint32_t PS_AutoEnroll(uint16_t id_nb, uint8_t entry_times, uint16_t parameters);
uint32_t PS_AutoIdentify(uint16_t id_nb, uint8_t score_grade);
int PS_ReadSysPara(uint8_t *rx_data, uint8_t len);
int PS_WriteReg(uint8_t regNb, uint8_t content);
uint16_t PS_ValidTempleteNum(void);
uint32_t PS_Match(void);
int PS_GetImage(void);
uint8_t PS_Empty(void);
void flush_buff(void);

void write_directive_command(uint8_t pkg_type, uint8_t regNb, uint16_t len,
		uint8_t *data);

/*
 * @Brief: decodes the received packet from the fingerprint module then extracts data bytes and save them in a buffer.
 * @Ret: confirm code from the FP module.
 * @Param: *rx_data: pointer to data buffer. N.B: not accessible if len < 3. data bytes recieved are saved here.
 * @Param: rx_data_len: the received packet length (mentioned in the datasheet for each register)
 */
int decode_rx_frame(uint8_t *rx_data, uint8_t rx_data_len);

void fp_init(void);

#endif /* INC_FP_INTERFACE_H_ */
