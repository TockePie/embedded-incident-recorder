#include "drivers/w25q64.h"

#include <string.h>

#include "main.h"

/* Standard SPI instruction set. */
#define W25Q64_CMD_WRITE_ENABLE       0x06U
#define W25Q64_CMD_WRITE_DISABLE      0x04U
#define W25Q64_CMD_READ_SR1           0x05U
#define W25Q64_CMD_READ_SR2           0x35U
#define W25Q64_CMD_PAGE_PROGRAM       0x02U
#define W25Q64_CMD_SECTOR_ERASE       0x20U
#define W25Q64_CMD_BLOCK32_ERASE      0x52U
#define W25Q64_CMD_BLOCK64_ERASE      0xD8U
#define W25Q64_CMD_READ_DATA          0x03U
#define W25Q64_CMD_READ_JEDEC_ID      0x9FU

#define W25Q64_IO_CHUNK_SIZE          256U

static const uint8_t s_dummy_tx[W25Q64_IO_CHUNK_SIZE] = { 0U };
static uint8_t s_crc_read_buffer[W25Q64_IO_CHUNK_SIZE];

static void W25Q64_Select(void) {
	HAL_GPIO_WritePin(FLASH_CS_N_GPIO_Port, FLASH_CS_N_Pin, GPIO_PIN_RESET);
}

static void W25Q64_Deselect(void) {
	HAL_GPIO_WritePin(FLASH_CS_N_GPIO_Port, FLASH_CS_N_Pin, GPIO_PIN_SET);
}

static void W25Q64_EncodeAddress(uint8_t header[4], uint8_t command,
		uint32_t address) {
	header[0] = command;
	header[1] = (uint8_t) (address >> 16U);
	header[2] = (uint8_t) (address >> 8U);
	header[3] = (uint8_t) address;
}

static W25Q64_Status_t W25Q64_TransmitCommand(SPI_HandleTypeDef *hspi,
		uint8_t command) {
	HAL_StatusTypeDef hal_status;

	if (hspi == NULL) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}

	W25Q64_Select();
	hal_status = HAL_SPI_Transmit(hspi, &command, 1U, W25Q64_SPI_TIMEOUT_MS);
	W25Q64_Deselect();

	return (hal_status == HAL_OK) ? W25Q64_STATUS_OK : W25Q64_STATUS_IO_ERROR;
}

static W25Q64_Status_t W25Q64_CheckAddressRange(uint32_t address,
		uint32_t length) {
	if (address > W25Q64_TOTAL_SIZE_BYTES) {
		return W25Q64_STATUS_OUT_OF_RANGE;
	}

	if (length > (W25Q64_TOTAL_SIZE_BYTES - address)) {
		return W25Q64_STATUS_OUT_OF_RANGE;
	}

	return W25Q64_STATUS_OK;
}

static W25Q64_Status_t W25Q64_CheckNotBusy(SPI_HandleTypeDef *hspi) {
	uint8_t status1 = 0U;
	W25Q64_Status_t result = W25Q64_ReadStatusRegister1(hspi, &status1);

	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	return ((status1 & W25Q64_STATUS1_BUSY_MASK) != 0U) ?
			W25Q64_STATUS_BUSY : W25Q64_STATUS_OK;
}

static W25Q64_Status_t W25Q64_WriteEnableAndVerify(SPI_HandleTypeDef *hspi) {
	uint8_t status1 = 0U;
	W25Q64_Status_t result = W25Q64_CheckNotBusy(hspi);

	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	result = W25Q64_TransmitCommand(hspi, W25Q64_CMD_WRITE_ENABLE);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	result = W25Q64_ReadStatusRegister1(hspi, &status1);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	return ((status1 & W25Q64_STATUS1_WEL_MASK) != 0U) ?
			W25Q64_STATUS_OK : W25Q64_STATUS_WRITE_ENABLE_FAILED;
}

static W25Q64_Status_t W25Q64_StartErase(SPI_HandleTypeDef *hspi,
		uint8_t command, uint32_t address, uint32_t alignment) {
	uint8_t header[4];
	HAL_StatusTypeDef hal_status;
	W25Q64_Status_t result;

	if (hspi == NULL) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}

	result = W25Q64_CheckAddressRange(address, 1U);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	if ((alignment == 0U) || ((address % alignment) != 0U)) {
		return W25Q64_STATUS_ALIGNMENT_ERROR;
	}

	result = W25Q64_WriteEnableAndVerify(hspi);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	W25Q64_EncodeAddress(header, command, address);
	W25Q64_Select();
	hal_status = HAL_SPI_Transmit(hspi, header, (uint16_t) sizeof(header),
	W25Q64_SPI_TIMEOUT_MS);
	W25Q64_Deselect();

	if (hal_status != HAL_OK) {
		(void) W25Q64_TransmitCommand(hspi, W25Q64_CMD_WRITE_DISABLE);
		return W25Q64_STATUS_IO_ERROR;
	}
	return W25Q64_STATUS_OK;
}

W25Q64_Status_t W25Q64_ReadJedecId(SPI_HandleTypeDef *hspi, uint8_t id[3]) {
	uint8_t tx[4] = { W25Q64_CMD_READ_JEDEC_ID, 0xFFU, 0xFFU, 0xFFU };
	uint8_t rx[4] = { 0U, 0U, 0U, 0U };
	HAL_StatusTypeDef hal_status;

	if ((hspi == NULL) || (id == NULL)) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}

	W25Q64_Select();
	hal_status = HAL_SPI_TransmitReceive(hspi, tx, rx, (uint16_t) sizeof(tx),
	W25Q64_SPI_TIMEOUT_MS);
	W25Q64_Deselect();

	if (hal_status != HAL_OK) {
		return W25Q64_STATUS_IO_ERROR;
	}

	id[0] = rx[1];
	id[1] = rx[2];
	id[2] = rx[3];
	return W25Q64_STATUS_OK;
}

W25Q64_Status_t W25Q64_Probe(SPI_HandleTypeDef *hspi, uint8_t id[3]) {
	W25Q64_Status_t result;

	if ((hspi == NULL) || (id == NULL)) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}

	result = W25Q64_WaitReady(hspi, W25Q64_DEFAULT_READY_TIMEOUT_MS);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	result = W25Q64_ReadJedecId(hspi, id);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	if ((id[0] != W25Q64_JEDEC_MANUFACTURER_ID)
			|| (id[1] != W25Q64_JEDEC_MEMORY_TYPE)
			|| (id[2] != W25Q64_JEDEC_CAPACITY)) {
		return W25Q64_STATUS_UNEXPECTED_ID;
	}

	return W25Q64_STATUS_OK;
}

W25Q64_Status_t W25Q64_ReadStatusRegister1(SPI_HandleTypeDef *hspi,
		uint8_t *status1) {
	uint8_t tx[2] = { W25Q64_CMD_READ_SR1, 0xFFU };
	uint8_t rx[2] = { 0U, 0U };
	HAL_StatusTypeDef hal_status;

	if ((hspi == NULL) || (status1 == NULL)) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}

	W25Q64_Select();
	hal_status = HAL_SPI_TransmitReceive(hspi, tx, rx, 2U,
	W25Q64_SPI_TIMEOUT_MS);
	W25Q64_Deselect();

	if (hal_status != HAL_OK) {
		return W25Q64_STATUS_IO_ERROR;
	}

	*status1 = rx[1];
	return W25Q64_STATUS_OK;
}

W25Q64_Status_t W25Q64_ReadStatusRegister2(SPI_HandleTypeDef *hspi,
		uint8_t *status2) {
	uint8_t tx[2] = { W25Q64_CMD_READ_SR2, 0xFFU };
	uint8_t rx[2] = { 0U, 0U };
	HAL_StatusTypeDef hal_status;

	if ((hspi == NULL) || (status2 == NULL)) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}

	W25Q64_Select();
	hal_status = HAL_SPI_TransmitReceive(hspi, tx, rx, 2U,
	W25Q64_SPI_TIMEOUT_MS);
	W25Q64_Deselect();

	if (hal_status != HAL_OK) {
		return W25Q64_STATUS_IO_ERROR;
	}

	*status2 = rx[1];
	return W25Q64_STATUS_OK;
}

W25Q64_Status_t W25Q64_ReadStatusRegisters(SPI_HandleTypeDef *hspi,
		W25Q64_StatusRegisters_t *status) {
	W25Q64_Status_t result;

	if ((hspi == NULL) || (status == NULL)) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}

	memset(status, 0, sizeof(*status));
	result = W25Q64_ReadStatusRegister1(hspi, &status->status1);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	result = W25Q64_ReadStatusRegister2(hspi, &status->status2);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	status->busy = ((status->status1 & W25Q64_STATUS1_BUSY_MASK) != 0U);
	status->write_enable_latch = ((status->status1 & W25Q64_STATUS1_WEL_MASK)
			!= 0U);
	status->block_protect_bits = (uint8_t) ((status->status1
			& W25Q64_STATUS1_BP_MASK) >> 2U);
	status->top_bottom_protect = ((status->status1 & W25Q64_STATUS1_TB_MASK)
			!= 0U);
	status->sector_protect =
			((status->status1 & W25Q64_STATUS1_SEC_MASK) != 0U);
	status->quad_enable = ((status->status2 & W25Q64_STATUS2_QE_MASK) != 0U);
	status->complement_protect = ((status->status2 & W25Q64_STATUS2_CMP_MASK)
			!= 0U);
	status->suspend = ((status->status2 & W25Q64_STATUS2_SUS_MASK) != 0U);

	return W25Q64_STATUS_OK;
}

W25Q64_Status_t W25Q64_IsBusy(SPI_HandleTypeDef *hspi, bool *busy) {
	uint8_t status1 = 0U;
	W25Q64_Status_t result;

	if ((hspi == NULL) || (busy == NULL)) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}

	result = W25Q64_ReadStatusRegister1(hspi, &status1);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	*busy = ((status1 & W25Q64_STATUS1_BUSY_MASK) != 0U);
	return W25Q64_STATUS_OK;
}

W25Q64_Status_t W25Q64_WaitReady(SPI_HandleTypeDef *hspi, uint32_t timeout_ms) {
	uint32_t start_tick;
	uint8_t status1;
	W25Q64_Status_t result;

	if (hspi == NULL) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}

	start_tick = HAL_GetTick();
	for (;;) {
		result = W25Q64_ReadStatusRegister1(hspi, &status1);
		if (result != W25Q64_STATUS_OK) {
			return result;
		}

		if ((status1 & W25Q64_STATUS1_BUSY_MASK) == 0U) {
			return W25Q64_STATUS_OK;
		}

		if ((uint32_t) (HAL_GetTick() - start_tick) >= timeout_ms) {
			return W25Q64_STATUS_TIMEOUT;
		}

		HAL_Delay(1U);
	}
}

W25Q64_Status_t W25Q64_WriteEnable(SPI_HandleTypeDef *hspi) {
	if (hspi == NULL) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}
	return W25Q64_WriteEnableAndVerify(hspi);
}

W25Q64_Status_t W25Q64_WriteDisable(SPI_HandleTypeDef *hspi) {
	uint8_t status1 = 0U;
	W25Q64_Status_t result = W25Q64_CheckNotBusy(hspi);

	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	result = W25Q64_TransmitCommand(hspi, W25Q64_CMD_WRITE_DISABLE);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	result = W25Q64_ReadStatusRegister1(hspi, &status1);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	return ((status1 & W25Q64_STATUS1_WEL_MASK) == 0U) ?
			W25Q64_STATUS_OK : W25Q64_STATUS_WRITE_ENABLE_FAILED;
}

W25Q64_Status_t W25Q64_Read(SPI_HandleTypeDef *hspi, uint32_t address,
		uint8_t *data, uint32_t length) {
	uint8_t header[4];
	uint32_t remaining;
	uint8_t *destination;
	HAL_StatusTypeDef hal_status;
	W25Q64_Status_t result;

	if ((hspi == NULL) || ((data == NULL) && (length != 0U))) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}

	result = W25Q64_CheckAddressRange(address, length);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}
	if (length == 0U) {
		return W25Q64_STATUS_OK;
	}

	result = W25Q64_CheckNotBusy(hspi);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	W25Q64_EncodeAddress(header, W25Q64_CMD_READ_DATA, address);
	W25Q64_Select();
	hal_status = HAL_SPI_Transmit(hspi, header, (uint16_t) sizeof(header),
	W25Q64_SPI_TIMEOUT_MS);
	if (hal_status != HAL_OK) {
		W25Q64_Deselect();
		return W25Q64_STATUS_IO_ERROR;
	}

	remaining = length;
	destination = data;
	while (remaining > 0U) {
		uint16_t chunk =
				(remaining > W25Q64_IO_CHUNK_SIZE) ?
						(uint16_t) W25Q64_IO_CHUNK_SIZE : (uint16_t) remaining;
		hal_status = HAL_SPI_TransmitReceive(hspi, (uint8_t*) s_dummy_tx,
				destination, chunk,
				W25Q64_SPI_TIMEOUT_MS);
		if (hal_status != HAL_OK) {
			W25Q64_Deselect();
			return W25Q64_STATUS_IO_ERROR;
		}
		destination += chunk;
		remaining -= chunk;
	}

	W25Q64_Deselect();
	return W25Q64_STATUS_OK;
}

W25Q64_Status_t W25Q64_StartPageProgram(SPI_HandleTypeDef *hspi,
		uint32_t address, const uint8_t *data, uint16_t length) {
	uint8_t header[4];
	uint32_t page_offset;
	HAL_StatusTypeDef hal_status;
	W25Q64_Status_t result;

	if ((hspi == NULL) || (data == NULL) || (length == 0U)
			|| (length > W25Q64_PAGE_SIZE_BYTES)) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}

	result = W25Q64_CheckAddressRange(address, length);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	page_offset = address % W25Q64_PAGE_SIZE_BYTES;
	if ((page_offset + length) > W25Q64_PAGE_SIZE_BYTES) {
		return W25Q64_STATUS_ALIGNMENT_ERROR;
	}

	result = W25Q64_WriteEnableAndVerify(hspi);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	W25Q64_EncodeAddress(header, W25Q64_CMD_PAGE_PROGRAM, address);
	W25Q64_Select();
	hal_status = HAL_SPI_Transmit(hspi, header, (uint16_t) sizeof(header),
	W25Q64_SPI_TIMEOUT_MS);
	if (hal_status == HAL_OK) {
		hal_status = HAL_SPI_Transmit(hspi, (uint8_t*) data, length,
		W25Q64_SPI_TIMEOUT_MS);
	}
	W25Q64_Deselect();

	if (hal_status != HAL_OK) {
		(void) W25Q64_TransmitCommand(hspi, W25Q64_CMD_WRITE_DISABLE);
		return W25Q64_STATUS_IO_ERROR;
	}
	return W25Q64_STATUS_OK;
}

W25Q64_Status_t W25Q64_StartSectorErase(SPI_HandleTypeDef *hspi,
		uint32_t address) {
	return W25Q64_StartErase(hspi, W25Q64_CMD_SECTOR_ERASE, address,
	W25Q64_SECTOR_SIZE_BYTES);
}

W25Q64_Status_t W25Q64_StartBlock32Erase(SPI_HandleTypeDef *hspi,
		uint32_t address) {
	return W25Q64_StartErase(hspi, W25Q64_CMD_BLOCK32_ERASE, address,
	W25Q64_BLOCK32_SIZE_BYTES);
}

W25Q64_Status_t W25Q64_StartBlock64Erase(SPI_HandleTypeDef *hspi,
		uint32_t address) {
	return W25Q64_StartErase(hspi, W25Q64_CMD_BLOCK64_ERASE, address,
	W25Q64_BLOCK64_SIZE_BYTES);
}

W25Q64_Status_t W25Q64_Program(SPI_HandleTypeDef *hspi, uint32_t address,
		const uint8_t *data, uint32_t length) {
	uint32_t remaining = length;
	uint32_t current_address = address;
	const uint8_t *source = data;
	W25Q64_Status_t result;

	if ((hspi == NULL) || ((data == NULL) && (length != 0U))) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}
	result = W25Q64_CheckAddressRange(address, length);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	while (remaining > 0U) {
		uint32_t page_offset = current_address % W25Q64_PAGE_SIZE_BYTES;
		uint32_t page_room = W25Q64_PAGE_SIZE_BYTES - page_offset;
		uint16_t chunk = (uint16_t) (
				(remaining < page_room) ? remaining : page_room);

		result = W25Q64_WaitReady(hspi, W25Q64_DEFAULT_READY_TIMEOUT_MS);
		if (result != W25Q64_STATUS_OK) {
			return result;
		}

		result = W25Q64_StartPageProgram(hspi, current_address, source, chunk);
		if (result != W25Q64_STATUS_OK) {
			return result;
		}

		result = W25Q64_WaitReady(hspi, W25Q64_PAGE_PROGRAM_TIMEOUT_MS);
		if (result != W25Q64_STATUS_OK) {
			return result;
		}

		current_address += chunk;
		source += chunk;
		remaining -= chunk;
	}

	return W25Q64_STATUS_OK;
}

W25Q64_Status_t W25Q64_EraseSector(SPI_HandleTypeDef *hspi, uint32_t address) {
	W25Q64_Status_t result = W25Q64_StartSectorErase(hspi, address);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}
	return W25Q64_WaitReady(hspi, W25Q64_SECTOR_ERASE_TIMEOUT_MS);
}

W25Q64_Status_t W25Q64_EraseBlock32(SPI_HandleTypeDef *hspi, uint32_t address) {
	W25Q64_Status_t result = W25Q64_StartBlock32Erase(hspi, address);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}
	return W25Q64_WaitReady(hspi, W25Q64_BLOCK32_ERASE_TIMEOUT_MS);
}

W25Q64_Status_t W25Q64_EraseBlock64(SPI_HandleTypeDef *hspi, uint32_t address) {
	W25Q64_Status_t result = W25Q64_StartBlock64Erase(hspi, address);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}
	return W25Q64_WaitReady(hspi, W25Q64_BLOCK64_ERASE_TIMEOUT_MS);
}

uint32_t W25Q64_CalculateCRC32(const uint8_t *data, uint32_t length) {
	uint32_t crc = 0xFFFFFFFFUL;
	uint32_t i;
	uint8_t bit;

	if ((data == NULL) && (length != 0U)) {
		return 0U;
	}

	for (i = 0U; i < length; ++i) {
		crc ^= data[i];
		for (bit = 0U; bit < 8U; ++bit) {
			const uint32_t mask = (uint32_t) -(int32_t) (crc & 1U);
			crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
		}
	}

	return crc ^ 0xFFFFFFFFUL;
}

W25Q64_Status_t W25Q64_ComputeCRC32(SPI_HandleTypeDef *hspi, uint32_t address,
		uint32_t length, uint32_t *crc32) {
	uint32_t crc = 0xFFFFFFFFUL;
	uint32_t remaining = length;
	uint32_t current_address = address;
	uint32_t i;
	uint8_t bit;
	W25Q64_Status_t result;

	if ((hspi == NULL) || (crc32 == NULL)) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}

	result = W25Q64_CheckAddressRange(address, length);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	while (remaining > 0U) {
		uint32_t chunk =
				(remaining > sizeof(s_crc_read_buffer)) ?
						(uint32_t) sizeof(s_crc_read_buffer) : remaining;
		result = W25Q64_Read(hspi, current_address, s_crc_read_buffer, chunk);
		if (result != W25Q64_STATUS_OK) {
			return result;
		}

		for (i = 0U; i < chunk; ++i) {
			crc ^= s_crc_read_buffer[i];
			for (bit = 0U; bit < 8U; ++bit) {
				const uint32_t mask = (uint32_t) -(int32_t) (crc & 1U);
				crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
			}
		}

		current_address += chunk;
		remaining -= chunk;
	}

	*crc32 = crc ^ 0xFFFFFFFFUL;
	return W25Q64_STATUS_OK;
}

W25Q64_Status_t W25Q64_Verify(SPI_HandleTypeDef *hspi, uint32_t address,
		const uint8_t *expected, uint32_t length) {
	uint32_t remaining = length;
	uint32_t current_address = address;
	const uint8_t *reference = expected;
	W25Q64_Status_t result;

	if ((hspi == NULL) || ((expected == NULL) && (length != 0U))) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}
	result = W25Q64_CheckAddressRange(address, length);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	while (remaining > 0U) {
		uint8_t readback[W25Q64_IO_CHUNK_SIZE];
		uint32_t chunk =
				(remaining > sizeof(readback)) ?
						(uint32_t) sizeof(readback) : remaining;
		result = W25Q64_Read(hspi, current_address, readback, chunk);
		if (result != W25Q64_STATUS_OK) {
			return result;
		}
		if (memcmp(readback, reference, chunk) != 0) {
			return W25Q64_STATUS_VERIFY_FAILED;
		}

		current_address += chunk;
		reference += chunk;
		remaining -= chunk;
	}

	return W25Q64_STATUS_OK;
}

W25Q64_Status_t W25Q64_VerifyCRC32(SPI_HandleTypeDef *hspi, uint32_t address,
		uint32_t length, uint32_t expected_crc32, uint32_t *actual_crc32) {
	uint32_t calculated = 0U;
	W25Q64_Status_t result = W25Q64_ComputeCRC32(hspi, address, length,
			&calculated);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	if (actual_crc32 != NULL) {
		*actual_crc32 = calculated;
	}

	return (calculated == expected_crc32) ?
			W25Q64_STATUS_OK : W25Q64_STATUS_VERIFY_FAILED;
}

W25Q64_Status_t W25Q64_VerifyErased(SPI_HandleTypeDef *hspi, uint32_t address,
		uint32_t length) {
	uint32_t remaining = length;
	uint32_t current_address = address;
	W25Q64_Status_t result;

	if (hspi == NULL) {
		return W25Q64_STATUS_INVALID_ARGUMENT;
	}
	result = W25Q64_CheckAddressRange(address, length);
	if (result != W25Q64_STATUS_OK) {
		return result;
	}

	while (remaining > 0U) {
		uint32_t i;
		uint32_t chunk =
				(remaining > sizeof(s_crc_read_buffer)) ?
						(uint32_t) sizeof(s_crc_read_buffer) : remaining;
		result = W25Q64_Read(hspi, current_address, s_crc_read_buffer, chunk);
		if (result != W25Q64_STATUS_OK) {
			return result;
		}
		for (i = 0U; i < chunk; ++i) {
			if (s_crc_read_buffer[i] != 0xFFU) {
				return W25Q64_STATUS_VERIFY_FAILED;
			}
		}
		current_address += chunk;
		remaining -= chunk;
	}

	return W25Q64_STATUS_OK;
}
