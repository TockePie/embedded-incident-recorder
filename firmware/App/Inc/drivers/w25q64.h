#ifndef W25Q64_H
#define W25Q64_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/* W25Q64 geometry: 64 Mbit = 8 MiB. */
#define W25Q64_TOTAL_SIZE_BYTES       (8UL * 1024UL * 1024UL)
#define W25Q64_PAGE_SIZE_BYTES        256UL
#define W25Q64_SECTOR_SIZE_BYTES      4096UL
#define W25Q64_BLOCK32_SIZE_BYTES    (32UL * 1024UL)
#define W25Q64_BLOCK64_SIZE_BYTES    (64UL * 1024UL)
#define W25Q64_SECTOR_COUNT           2048UL
#define W25Q64_PAGE_COUNT             32768UL

#define W25Q64_JEDEC_MANUFACTURER_ID  0xEFU
#define W25Q64_JEDEC_MEMORY_TYPE      0x40U
#define W25Q64_JEDEC_CAPACITY         0x17U

/* Conservative software timeouts; the datasheet maxima are lower for
 * the standard W25Q64FV family, but margin is intentional. */
#define W25Q64_SPI_TIMEOUT_MS                 100U
#define W25Q64_DEFAULT_READY_TIMEOUT_MS      1000U
#define W25Q64_PAGE_PROGRAM_TIMEOUT_MS        100U
#define W25Q64_SECTOR_ERASE_TIMEOUT_MS       1500U
#define W25Q64_BLOCK32_ERASE_TIMEOUT_MS      2500U
#define W25Q64_BLOCK64_ERASE_TIMEOUT_MS      3500U

#define W25Q64_STATUS1_BUSY_MASK              0x01U
#define W25Q64_STATUS1_WEL_MASK               0x02U
#define W25Q64_STATUS1_BP_MASK                0x1CU
#define W25Q64_STATUS1_TB_MASK                0x20U
#define W25Q64_STATUS1_SEC_MASK               0x40U
#define W25Q64_STATUS1_SRP0_MASK              0x80U

#define W25Q64_STATUS2_SRP1_MASK              0x01U
#define W25Q64_STATUS2_QE_MASK                0x02U
#define W25Q64_STATUS2_LB_MASK                0x38U
#define W25Q64_STATUS2_CMP_MASK               0x40U
#define W25Q64_STATUS2_SUS_MASK               0x80U

typedef enum {
	W25Q64_STATUS_OK = 0,
	W25Q64_STATUS_IO_ERROR,
	W25Q64_STATUS_UNEXPECTED_ID,
	W25Q64_STATUS_INVALID_ARGUMENT,
	W25Q64_STATUS_OUT_OF_RANGE,
	W25Q64_STATUS_ALIGNMENT_ERROR,
	W25Q64_STATUS_BUSY,
	W25Q64_STATUS_TIMEOUT,
	W25Q64_STATUS_WRITE_ENABLE_FAILED,
	W25Q64_STATUS_VERIFY_FAILED
} W25Q64_Status_t;

typedef struct {
	uint8_t status1;
	uint8_t status2;
	bool busy;
	bool write_enable_latch;
	uint8_t block_protect_bits;
	bool top_bottom_protect;
	bool sector_protect;
	bool complement_protect;
	bool quad_enable;
	bool suspend;
} W25Q64_StatusRegisters_t;

/** Read the three-byte JEDEC ID using command 0x9F. */
W25Q64_Status_t W25Q64_ReadJedecId(SPI_HandleTypeDef *hspi, uint8_t id[3]);

/** Check for the Winbond W25Q64 JEDEC ID EF 40 17. Read-only operation. */
W25Q64_Status_t W25Q64_Probe(SPI_HandleTypeDef *hspi, uint8_t id[3]);

/** Read Status Register-1 (command 0x05) and Status Register-2 (0x35). */
W25Q64_Status_t W25Q64_ReadStatusRegister1(SPI_HandleTypeDef *hspi,
		uint8_t *status1);
W25Q64_Status_t W25Q64_ReadStatusRegister2(SPI_HandleTypeDef *hspi,
		uint8_t *status2);
W25Q64_Status_t W25Q64_ReadStatusRegisters(SPI_HandleTypeDef *hspi,
		W25Q64_StatusRegisters_t *status);

/** Read the current BUSY bit without waiting (non-blocking poll primitive). */
W25Q64_Status_t W25Q64_IsBusy(SPI_HandleTypeDef *hspi, bool *busy);

/** Poll BUSY until clear. This function is blocking but always time-bounded. */
W25Q64_Status_t W25Q64_WaitReady(SPI_HandleTypeDef *hspi, uint32_t timeout_ms);

/** Explicit write-latch controls; neither function alters array contents. */
W25Q64_Status_t W25Q64_WriteEnable(SPI_HandleTypeDef *hspi);
W25Q64_Status_t W25Q64_WriteDisable(SPI_HandleTypeDef *hspi);

/**
 * Read arbitrary bytes. Reads are split into bounded SPI transfers while /CS
 * remains asserted. Returns BUSY if an internal program/erase is in progress.
 */
W25Q64_Status_t W25Q64_Read(SPI_HandleTypeDef *hspi, uint32_t address,
		uint8_t *data, uint32_t length);

/**
 * Start one page-program operation without waiting for its internal cycle.
 * Length must be 1..256 and must not cross a 256-byte page boundary.
 * Use W25Q64_WaitReady() or poll W25Q64_ReadStatusRegister1() afterwards.
 */
W25Q64_Status_t W25Q64_StartPageProgram(SPI_HandleTypeDef *hspi,
		uint32_t address, const uint8_t *data, uint16_t length);

/** Start sector/block erase without waiting for completion. Addresses must
 * be aligned to the corresponding erase size. */
W25Q64_Status_t W25Q64_StartSectorErase(SPI_HandleTypeDef *hspi,
		uint32_t address);
W25Q64_Status_t W25Q64_StartBlock32Erase(SPI_HandleTypeDef *hspi,
		uint32_t address);
W25Q64_Status_t W25Q64_StartBlock64Erase(SPI_HandleTypeDef *hspi,
		uint32_t address);

/** Blocking convenience wrappers. Program splits data at page boundaries. */
W25Q64_Status_t W25Q64_Program(SPI_HandleTypeDef *hspi, uint32_t address,
		const uint8_t *data, uint32_t length);
W25Q64_Status_t W25Q64_EraseSector(SPI_HandleTypeDef *hspi, uint32_t address);
W25Q64_Status_t W25Q64_EraseBlock32(SPI_HandleTypeDef *hspi, uint32_t address);
W25Q64_Status_t W25Q64_EraseBlock64(SPI_HandleTypeDef *hspi, uint32_t address);

/** Software CRC-32/ISO-HDLC (reflected polynomial 0xEDB88320). */
uint32_t W25Q64_CalculateCRC32(const uint8_t *data, uint32_t length);
W25Q64_Status_t W25Q64_ComputeCRC32(SPI_HandleTypeDef *hspi, uint32_t address,
		uint32_t length, uint32_t *crc32);
W25Q64_Status_t W25Q64_Verify(SPI_HandleTypeDef *hspi, uint32_t address,
		const uint8_t *expected, uint32_t length);
W25Q64_Status_t W25Q64_VerifyCRC32(SPI_HandleTypeDef *hspi, uint32_t address,
		uint32_t length, uint32_t expected_crc32, uint32_t *actual_crc32);
W25Q64_Status_t W25Q64_VerifyErased(SPI_HandleTypeDef *hspi, uint32_t address,
		uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* W25Q64_H */
