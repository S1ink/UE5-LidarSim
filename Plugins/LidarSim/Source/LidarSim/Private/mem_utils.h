#pragma once

#include <cstdint>
#include <type_traits>
#include <memory>
#include <vector>


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

template<typename T>
std::unique_ptr<uint8_t[]> create_mem_snapshot(const T& val) {
	uint8_t* buff = new uint8_t[sizeof(T)];
	memcpy(buff, &val, sizeof(T));
	return std::unique_ptr<uint8_t[]>{buff};
}
template<typename T>
bool compare_mem_snapshot(const T& val, std::unique_ptr<uint8_t[]>& snap) {
	return !memcmp(&val, snap.get(), sizeof(T));
}

#ifdef WITH_ENGINE
#include <Containers/Array.h>
#include <pcl/point_cloud.h>

template<
	typename v_T,
	typename a_T,
	typename alloc_T = std::allocator<v_T>>
	static bool memSwap(std::vector<v_T, alloc_T>& std_vec, TArray<a_T>& ue_arr) {
	static constexpr bool
		valid_destruct = std::is_trivially_destructible<v_T>::value && std::is_trivially_destructible<a_T>::value,
		valid_memory = sizeof(v_T) == sizeof(a_T)/* && alignof(v_T) == alignof(a_T)*/;	// alignment only matters if it is larger than the size?
	static_assert(valid_destruct, "Base types must be trivially destructible or else they won't be cleaned up correctly after swapping!");
	static_assert(valid_memory, "Base types must have similar memory layouts for successful swapping!");
	if constexpr (valid_destruct && valid_memory) {
		uintptr_t* std_vec_active_ptr = reinterpret_cast<uintptr_t*>(&std_vec);
		uintptr_t* ue_arr_active_ptr = reinterpret_cast<uintptr_t*>(&ue_arr);
		uintptr_t	// uint64_t
			std_vec_buff_start = std_vec_active_ptr[0],
			std_vec_buff_elem_end = std_vec_active_ptr[1],
			std_vec_buff_cap_end = std_vec_active_ptr[2],
			ue_arr_buff_start = ue_arr_active_ptr[0];
		uint32_t
			ue_arr_buff_elem_num = reinterpret_cast<uint32_t*>(ue_arr_active_ptr + 1)[0],
			ue_arr_buff_cap_num = reinterpret_cast<uint32_t*>(ue_arr_active_ptr + 1)[1];
		std_vec_active_ptr[0] = ue_arr_buff_start;
		std_vec_active_ptr[1] = reinterpret_cast<uintptr_t>(reinterpret_cast<a_T*>(ue_arr_buff_start) + ue_arr_buff_elem_num);	// size
		std_vec_active_ptr[2] = reinterpret_cast<uintptr_t>(reinterpret_cast<a_T*>(ue_arr_buff_start) + ue_arr_buff_cap_num);	// capacity
		ue_arr_active_ptr[0] = std_vec_buff_start;
		reinterpret_cast<uint32_t*>(ue_arr_active_ptr + 1)[0] = reinterpret_cast<v_T*>(std_vec_buff_elem_end) - reinterpret_cast<v_T*>(std_vec_buff_start);
		reinterpret_cast<uint32_t*>(ue_arr_active_ptr + 1)[1] = reinterpret_cast<v_T*>(std_vec_buff_cap_end) - reinterpret_cast<v_T*>(std_vec_buff_start);
		return true;
	}
	return false;
}
template<
	typename a_T,
	typename v_T,
	typename alloc_T = std::allocator<v_T>>
	inline static bool memSwap(TArray<a_T>& ue_arr, std::vector<v_T, alloc_T>& std_vec) {
	return memSwap<v_T, a_T, alloc_T>(std_vec, ue_arr);
}

template<
	typename p_T,
	typename a_T>
static bool memSwap(pcl::PointCloud<p_T>& cloud, TArray<a_T>& ue_arr) {
	if (memSwap<p_T, a_T, Eigen::aligned_allocator<p_T>>(cloud.points, ue_arr)) {
		cloud.width = cloud.points.size();
		cloud.height = 1;
		return true;
	}
	return false;
}
template<
	typename a_T,
	typename p_T>
inline static bool memSwap(TArray<a_T>& ue_arr, pcl::PointCloud<p_T>& cloud) {
	return memSwap<p_T, a_T>(cloud, ue_arr);
}

template<
	typename T,
	typename Alloc_A,
	typename Alloc_B>
static bool memSwap(TArray<T, Alloc_A>& a, TArray<T, Alloc_B>& b) {
	if constexpr (
		UE4Array_Private::CanMoveTArrayPointersBetweenArrayTypes<TArray<T, Alloc_A>, TArray<Alloc_B>>() &&
		(sizeof(TArray<T>) == sizeof(TArray<T, Alloc_A>) && sizeof(TArray<T>) == sizeof(TArray<T, Alloc_B>))
		) {
		uint8_t* tmp = new uint8_t[sizeof(TArray<T>)];
		memcpy(tmp, &a, sizeof(TArray<T>));
		memcpy(&a, &b, sizeof(TArray<T>));
		memcpy(&b, tmp, sizeof(TArray<T>));
		delete[] tmp;
		return true;
	}
	return false;
}
#endif

template<
	typename T,
	typename Alloc_A,
	typename Alloc_B>
static bool memSwap(std::vector<T, Alloc_A>& a, std::vector<T, Alloc_B>& b) {
	if constexpr (std::is_same<Alloc_A, Alloc_B>::value) {
		std::swap(a, b);
		return true;
	}
	return false;
}
