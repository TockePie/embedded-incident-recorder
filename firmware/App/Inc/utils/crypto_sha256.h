#ifndef CRYPTO_SHA256_H
#define CRYPTO_SHA256_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Crypto_Sha256(const uint8_t *data, size_t length, uint8_t digest[32]);
void Crypto_Pbkdf2HmacSha256(const uint8_t *password,
		size_t password_length, const uint8_t *salt, size_t salt_length,
		uint32_t iterations, uint8_t output[32]);

/* Return nonzero when the byte sequences match; comparison time is constant. */
uint8_t Crypto_ConstantTimeEqual(const uint8_t *a, const uint8_t *b,
		size_t length);

#ifdef __cplusplus
}
#endif

#endif /* CRYPTO_SHA256_H */
