#include "utils/crypto_sha256.h"

#include <string.h>

typedef struct {
	uint32_t h[8];
	uint64_t bit_count;
	uint8_t block[64];
	uint32_t used;
} Sha256Context_t;

static const uint32_t s_round_constants[64] = { 0x428a2f98U, 0x71374491U,
		0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U, 0x923f82a4U,
		0xab1c5ed5U, 0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
		0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U, 0xe49b69c1U,
		0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU,
		0x5cb0a9dcU, 0x76f988daU, 0x983e5152U, 0xa831c66dU, 0xb00327c8U,
		0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U, 0x06ca6351U, 0x14292967U,
		0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U, 0x650a7354U,
		0x766a0abbU, 0x81c2c92eU, 0x92722c85U, 0xa2bfe8a1U, 0xa81a664bU,
		0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U,
		0x106aa070U, 0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U,
		0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U, 0x748f82eeU,
		0x78a5636fU, 0x84c87814U, 0x8cc70208U, 0x90befffaU, 0xa4506cebU,
		0xbef9a3f7U, 0xc67178f2U };

static uint32_t Sha256_RotateRight(uint32_t value, uint32_t shift) {
	return (value >> shift) | (value << (32U - shift));
}

static void Sha256_Transform(Sha256Context_t *context, const uint8_t block[64]) {
	uint32_t words[64];
	uint32_t a, b, c, d, e, f, g, h;
	uint32_t temp1, temp2;
	unsigned index;

	for (index = 0U; index < 16U; index++) {
		words[index] = ((uint32_t) block[index * 4U] << 24U)
				| ((uint32_t) block[index * 4U + 1U] << 16U)
				| ((uint32_t) block[index * 4U + 2U] << 8U)
				| (uint32_t) block[index * 4U + 3U];
	}

	for (index = 16U; index < 64U; index++) {
		uint32_t x = words[index - 15U];
		uint32_t y = words[index - 2U];
		uint32_t sigma0 = Sha256_RotateRight(x, 7U) ^ Sha256_RotateRight(x, 18U)
				^ (x >> 3U);
		uint32_t sigma1 = Sha256_RotateRight(y, 17U)
				^ Sha256_RotateRight(y, 19U) ^ (y >> 10U);

		words[index] = words[index - 16U] + sigma0 + words[index - 7U] + sigma1;
	}

	a = context->h[0];
	b = context->h[1];
	c = context->h[2];
	d = context->h[3];
	e = context->h[4];
	f = context->h[5];
	g = context->h[6];
	h = context->h[7];

	for (index = 0U; index < 64U; index++) {
		uint32_t sum1 = Sha256_RotateRight(e, 6U) ^ Sha256_RotateRight(e, 11U)
				^ Sha256_RotateRight(e, 25U);
		uint32_t choose = (e & f) ^ ((~e) & g);
		uint32_t sum0 = Sha256_RotateRight(a, 2U) ^ Sha256_RotateRight(a, 13U)
				^ Sha256_RotateRight(a, 22U);
		uint32_t majority = (a & b) ^ (a & c) ^ (b & c);

		temp1 = h + sum1 + choose + s_round_constants[index] + words[index];
		temp2 = sum0 + majority;

		h = g;
		g = f;
		f = e;
		e = d + temp1;
		d = c;
		c = b;
		b = a;
		a = temp1 + temp2;
	}

	context->h[0] += a;
	context->h[1] += b;
	context->h[2] += c;
	context->h[3] += d;
	context->h[4] += e;
	context->h[5] += f;
	context->h[6] += g;
	context->h[7] += h;
}

static void Sha256_Init(Sha256Context_t *context) {
	static const uint32_t initial_state[8] = { 0x6a09e667U, 0xbb67ae85U,
			0x3c6ef372U, 0xa54ff53aU, 0x510e527fU, 0x9b05688cU, 0x1f83d9abU,
			0x5be0cd19U };

	memcpy(context->h, initial_state, sizeof(initial_state));
	context->bit_count = 0U;
	context->used = 0U;
}

static void Sha256_Update(Sha256Context_t *context, const uint8_t *data,
		size_t length) {
	while (length > 0U) {
		size_t chunk = 64U - context->used;
		if (chunk > length)
			chunk = length;

		memcpy(&context->block[context->used], data, chunk);
		context->used += (uint32_t) chunk;
		data += chunk;
		length -= chunk;
		context->bit_count += (uint64_t) chunk * 8U;

		if (context->used == 64U) {
			Sha256_Transform(context, context->block);
			context->used = 0U;
		}
	}
}

static void Sha256_Final(Sha256Context_t *context, uint8_t digest[32]) {
	uint64_t original_bit_count = context->bit_count;
	unsigned index;

	context->block[context->used++] = 0x80U;

	if (context->used > 56U) {
		memset(&context->block[context->used], 0, 64U - context->used);
		Sha256_Transform(context, context->block);
		context->used = 0U;
	}

	memset(&context->block[context->used], 0, 56U - context->used);
	for (index = 0U; index < 8U; index++) {
		context->block[63U - index] = (uint8_t) (original_bit_count
				>> (index * 8U));
	}

	Sha256_Transform(context, context->block);

	for (index = 0U; index < 8U; index++) {
		digest[index * 4U] = (uint8_t) (context->h[index] >> 24U);
		digest[index * 4U + 1U] = (uint8_t) (context->h[index] >> 16U);
		digest[index * 4U + 2U] = (uint8_t) (context->h[index] >> 8U);
		digest[index * 4U + 3U] = (uint8_t) context->h[index];
	}

	memset(context, 0, sizeof(*context));
}

static void HmacSha256(const uint8_t *key, size_t key_length,
		const uint8_t *message, size_t message_length, uint8_t digest[32]) {
	uint8_t key_block[64] = { 0U };
	uint8_t inner_digest[32];
	uint8_t inner_pad[64];
	uint8_t outer_pad[64];
	Sha256Context_t context;
	size_t index;

	if (key_length > sizeof(key_block)) {
		Sha256_Init(&context);
		Sha256_Update(&context, key, key_length);
		Sha256_Final(&context, key_block);
	} else if (key_length > 0U) {
		memcpy(key_block, key, key_length);
	}

	for (index = 0U; index < sizeof(key_block); index++) {
		inner_pad[index] = (uint8_t) (key_block[index] ^ 0x36U);
		outer_pad[index] = (uint8_t) (key_block[index] ^ 0x5CU);
	}

	Sha256_Init(&context);
	Sha256_Update(&context, inner_pad, sizeof(inner_pad));
	Sha256_Update(&context, message, message_length);
	Sha256_Final(&context, inner_digest);

	Sha256_Init(&context);
	Sha256_Update(&context, outer_pad, sizeof(outer_pad));
	Sha256_Update(&context, inner_digest, sizeof(inner_digest));
	Sha256_Final(&context, digest);

	memset(key_block, 0, sizeof(key_block));
	memset(inner_digest, 0, sizeof(inner_digest));
	memset(inner_pad, 0, sizeof(inner_pad));
	memset(outer_pad, 0, sizeof(outer_pad));
}

void Crypto_Sha256(const uint8_t *data, size_t length, uint8_t digest[32]) {
	Sha256Context_t context;

	if ((data == NULL) || (digest == NULL)) {
		if (digest != NULL)
			memset(digest, 0, 32U);
		return;
	}

	Sha256_Init(&context);
	Sha256_Update(&context, data, length);
	Sha256_Final(&context, digest);
}

void Crypto_Pbkdf2HmacSha256(const uint8_t *password, size_t password_length,
		const uint8_t *salt, size_t salt_length, uint32_t iterations,
		uint8_t output[32]) {
	uint8_t salt_block[128];
	uint8_t current[32];
	uint8_t accumulated[32];
	size_t effective_salt_length = salt_length;
	uint32_t iteration;
	size_t index;

	if ((password == NULL) || (salt == NULL) || (output == NULL)
			|| (iterations == 0U) || (salt_length > 124U)) {
		if (output != NULL)
			memset(output, 0, 32U);
		return;
	}

	if (effective_salt_length > 124U) {
		effective_salt_length = 124U;
	}

	memcpy(salt_block, salt, effective_salt_length);
	salt_block[effective_salt_length] = 0U;
	salt_block[effective_salt_length + 1U] = 0U;
	salt_block[effective_salt_length + 2U] = 0U;
	salt_block[effective_salt_length + 3U] = 1U;

	HmacSha256(password, password_length, salt_block,
			effective_salt_length + 4U, current);
	memcpy(accumulated, current, sizeof(accumulated));

	for (iteration = 1U; iteration < iterations; iteration++) {
		HmacSha256(password, password_length, current, sizeof(current),
				current);
		for (index = 0U; index < sizeof(accumulated); index++) {
			accumulated[index] ^= current[index];
		}
	}

	memcpy(output, accumulated, sizeof(accumulated));
	memset(salt_block, 0, sizeof(salt_block));
	memset(current, 0, sizeof(current));
	memset(accumulated, 0, sizeof(accumulated));
}

uint8_t Crypto_ConstantTimeEqual(const uint8_t *a, const uint8_t *b,
		size_t length) {
	uint8_t difference = 0U;
	size_t index;

	if ((a == NULL) || (b == NULL))
		return 0U;

	for (index = 0U; index < length; index++) {
		difference |= (uint8_t) (a[index] ^ b[index]);
	}

	return (uint8_t) (difference == 0U);
}
