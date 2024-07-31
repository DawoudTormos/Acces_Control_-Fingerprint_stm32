/*
 * fp_interface.c
 *
 *  Created on: Nov 18, 2023
 *      Author: Raja
 */

#include "fp_interface.h"

UART_HandleTypeDef fp;

uint8_t fp_detected = 0;

static void MX_USART1_UART_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void fp_init(void) {
	MX_USART1_UART_Init();
}

int PS_ReadSysPara(uint8_t *rx_data, uint8_t len) {
	uint8_t tx_buff[1];
	write_directive_command(CMD_PKG, PS_READ_SYS_PARA, PS_READ_SYS_PARA_DIR_LEN,
			tx_buff);

	int conf_code = decode_rx_frame(rx_data, (len + 3));

	return conf_code;
}
uint16_t PS_ValidTempleteNum(void) {
	uint8_t data[1];
	uint8_t rx_data[2];

	write_directive_command(CMD_PKG, PS_READ_VALID_TEMP, PS_READ_VALID_TEMP_LEN,
			data);

	int conf_code = decode_rx_frame(rx_data, 5);
	uint16_t valid_temp_num = rx_data[0] << 8 | rx_data[1];

	return valid_temp_num;
}

void write_directive_command(uint8_t pkg_type, uint8_t regNb, uint16_t len,
		uint8_t *data) {
	uint8_t temp_tx[9 + len]; // preamble: 2 bytes, header: 4 bytes, pckg logo: 1 byte, len: 2 bytes --> sum = 9 bytes
	uint16_t checksum = 0;
	uint8_t data_count = 0;

	temp_tx[0] = HEADER >> 8;
	temp_tx[1] = HEADER;

	temp_tx[2] = DEVICE_ADDRESS >> 24;
	temp_tx[3] = DEVICE_ADDRESS >> 16;
	temp_tx[4] = DEVICE_ADDRESS >> 8;
	temp_tx[5] = DEVICE_ADDRESS;

	temp_tx[6] = pkg_type;

	temp_tx[7] = len >> 8;
	temp_tx[8] = len;

	temp_tx[9] = regNb;

	checksum += pkg_type + len + regNb;

	// if the directive command contains some parameters other than
	// checksum and instruction code
	if (len > 3) {
		while (data_count < (len - 3)) {
			temp_tx[10 + data_count] = data[data_count];
			checksum += data[data_count];

			data_count++;
		}

		temp_tx[10 + data_count] = (checksum >> 8) & 0xFF;
		temp_tx[11 + data_count] = (checksum & 0xFF);

	} else {
		temp_tx[10] = checksum >> 8;
		temp_tx[11] = checksum;
	}

	HAL_UART_Transmit(&fp, temp_tx, sizeof(temp_tx), 1000);
	//HAL_Delay(10);
}
int decode_rx_frame(uint8_t *rx_data, uint8_t rx_data_len) {

	uint8_t temp_rx[9 + rx_data_len];  // buffer to store
	uint8_t pkg_logo, confirm_code;
	uint16_t header, packet_len, checksum;
	uint32_t dev_address;
	uint8_t data_count = 0;

	const char *c = "500";

	memset(temp_rx, '\0', sizeof(temp_rx));
	HAL_StatusTypeDef ret = HAL_UART_Receive(&fp, temp_rx, sizeof(temp_rx),
			500);

	header = temp_rx[0] << 8 | temp_rx[1];
	if (header != HEADER) {
		c = "invalid header";
	}

	pkg_logo = temp_rx[6];
	packet_len = (temp_rx[7] << 8) | temp_rx[8];
	confirm_code = temp_rx[9];

	if (rx_data_len > 3) {
		while (data_count < (rx_data_len - 3)) {
			rx_data[data_count] = temp_rx[10 + data_count];
			data_count++;
		}
		checksum = (temp_rx[10 + data_count] << 8) | temp_rx[11 + data_count];

	} else {
		checksum = (temp_rx[10] << 8) | temp_rx[11];
	}

	switch (confirm_code) {
	case CONF_INSTRUCTION_OK:
		c = "Instruction is Performed";
		break;

	case CONF_PACKET_RECEIVE_ERROR:
		c = "Error receiving the package";
		break;

	case CONF_NO_FINGER_DETECTED:
		c = "No finger on the sensor";
		break;

	case CONF_FINGERPRINT_IMAGE_INPUT_FAIL:
		c = "Image input fails";
		break;

	case CONF_FINGERPRINT_TOO_FEW_FEATURE_POINTS:
		c = "Failed to generate features";
		break;

	case CONF_FINGERPRINT_MISMATCH:
		c = "Fingerprint Mismatch";
		break;

	case CONF_FEATURE_MERGE_FAIL:
		c = "Failed to merge templates";
		break;

	case CONF_ADDRESS_OUTSIDE_LIBRARY:
		c = "ID number out of range";
		break;

	case CONF_READ_WRITE_FLASH_ERROR:
		c = "Error in reading and writing the FLASH";
		break;

	case CONF_FINGERPRINT_LIBRARY_FULL:
		c = "The fingerprint library is full";
		break;

	case CONF_TEMPLATE_NOT_EMPTY:
		c = "Fingerprint template is not empty";
		break;

	case CONF_TEMPLATE_EMPTY:
		c = "Fingerprint template is empty";
		break;

	case CONF_INCORRECT_ENTRY_TIMES:
		c = "Entry times are set incorrect";
		break;

	case CONF_OVERTIME:
		c = "Overtime";
		break;

	case CONF_ALREADY_EXISTS:
		c = "The fingerprint already exists";
		break;

	case CONF_FUNCTIONALITY_MISMATCH:
		c = "The function does not match with the encryption level";
		break;

	case CONF_ILLEGAL_DATA:
		c = "Illegal data";
		break;
	}
	CDC_Transmit_FS(temp_rx, sizeof(temp_rx));
	HAL_Delay(50);
	CDC_Transmit_FS(c, strlen(c));

	return confirm_code;
}

uint32_t PS_AutoEnroll(uint16_t id_nb, uint8_t entry_times, uint16_t parameters) {
	uint8_t data[5];
	uint8_t rx_data[2];

	data[0] = id_nb >> 8;
	data[1] = id_nb;
	data[2] = entry_times;
	data[3] = parameters >> 8;
	data[4] = parameters;

	write_directive_command(CMD_PKG, PS_AUTO_ENROLL, PS_AUTO_ENROLL_LEN, data);

	int conf_code = decode_rx_frame(rx_data, 5);
	uint32_t params = conf_code << 16 | rx_data[0] << 8 | rx_data[1];

	memset(rx_data, '\0', sizeof(rx_data));
	return params;
}
uint32_t PS_AutoIdentify(uint16_t id_nb, uint8_t score_grade) {
	uint8_t data[5];
	uint8_t rx_data[5];

	data[0] = score_grade;
	data[1] = id_nb >> 8;
	data[2] = id_nb;
	data[3] = 0x00;
	data[4] = 0x07;

	write_directive_command(CMD_PKG, PS_AUTO_IDENT, PS_AUTO_IDENT_LEN, data);
	int conf_code = decode_rx_frame(rx_data, 8);

	uint32_t param = conf_code << 24 | rx_data[0] << 16 | rx_data[3] << 8
			| rx_data[4];

	memset(rx_data, '\0', sizeof(rx_data));
	return param;
}

int PS_GetImage(void) {
	uint8_t data[1];

	write_directive_command(CMD_PKG, PS_GET_IMG, PS_GET_IMG_PKT_LEN, data);

	uint8_t rx_data[1];
	int conf_code = decode_rx_frame(rx_data, 3);

	memset(rx_data, '\0', sizeof(rx_data));
	return conf_code;
}
uint32_t PS_Match(void) {
	uint8_t data[1];

	write_directive_command(CMD_PKG, PS_MATCH, PS_MATCH_LEN, data);

	uint8_t rx_data[2];
	int conf_code = decode_rx_frame(rx_data, 5);

	uint32_t score = conf_code << 16 | rx_data[0] << 8 | rx_data[1];

	memset(rx_data, '\0', sizeof(rx_data));
	return score;
}
uint8_t PS_Empty(void) {
	uint8_t data[1];

	write_directive_command(CMD_PKG, PS_EMPTY_ALL, PS_EMPTY_ALL_LEN, data);

	uint8_t rx_data[1];
	uint8_t conf_code = decode_rx_frame(rx_data, 3);
	memset(rx_data, '\0', sizeof(rx_data));

	return conf_code;
}

void flush_buff(void) {
	uint8_t dummyData;
	HAL_UART_Receive(&fp, &dummyData, 1, 1000);
}

uint8_t is_fp_detected(void) {
	return fp_detected;
}
void clear_fp_flag(void) {
	fp_detected = 0;
}

static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	fp.Instance = USART1;
	fp.Init.BaudRate = 57600;
	fp.Init.WordLength = UART_WORDLENGTH_8B;
	fp.Init.StopBits = UART_STOPBITS_1;
	fp.Init.Parity = UART_PARITY_NONE;
	fp.Init.Mode = UART_MODE_TX_RX;
	fp.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	fp.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&fp) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	fp_detected = 1;
}
