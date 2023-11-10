#pragma once

#include <memory>


/** Get the hex representation of a range of bytes. Blocks are separated by a space,
 * and block endian-ness can be changed by the template parameter. */
template<
	typename T,
	size_t blocksize = sizeof(T),
	bool B_endian = true>
static std::unique_ptr<char[]> hexify(const T* data, const size_t items = 1U) {
	static_assert(blocksize > 0, "Blocksize must be at least 1!");
	static constexpr char const* _hex = "0123456789ABCDEF";
	static constexpr size_t unit_size = sizeof(T);
	const size_t
		raw_len = unit_size * items,
		blocks = raw_len / blocksize,
		chunksize = blocksize * 2 + 1,
		len = chunksize * blocks;	// raw space times 2 hex digits per byte + 1 char space for each block + null termination - 1 since last block doesnt need space
	char* hex = new char[len];
	const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data);
	for (size_t i = 0; i < blocks; i++) {
		for (size_t b = 0; b < blocksize; b++) {
			const uint8_t byte = bytes[i * blocksize + b];	// NOTE: bits within each byte are big-endian
			if constexpr (B_endian) {    // big endian byte order (in this case, per-block)
				hex[i * chunksize + (blocksize - b - 1) * 2 + 0] = _hex[(byte >> 4)];   // upper 4 bits
				hex[i * chunksize + (blocksize - b - 1) * 2 + 1] = _hex[(byte & 0xF)];	// lower 4 bits
			}
			else {
				hex[i * chunksize + b * 2 + 0] = _hex[(byte >> 4)];	    // upper 4 bits
				hex[i * chunksize + b * 2 + 1] = _hex[(byte & 0xF)];	// lower 4 bits
			}
		}
		hex[i * chunksize + blocksize * 2] = ' ';
	}
	hex[len - 1] = '\0';
	return std::unique_ptr<char[]>(hex);
}