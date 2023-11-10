// #include <type_traits>
// #include <memory>

// #include <CoreMinimal.h>
// #include <Components/ActorComponent.h>
// #include <Physics/PhysicsInterfaceCore.h>
// #include <PhysicalMaterials/PhysicalMaterial.h>

// #define PCL_NO_PRECOMPILE
// THIRD_PARTY_INCLUDES_START
// #include <pcl/memory.h>
// #include <pcl/pcl_macros.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/pcd_io.h>
// THIRD_PARTY_INCLUDES_END

// #include "PCLHelper.h"
// #include "LidarSimComponent.h"


// #define ASSERT_FP_TYPE(fp_T) static_assert(std::is_floating_point_v<fp_T>, "fp_T must be floating point type")

// /* Analogous to pcl::PointXYZI but memory aligned to match TVector4<> for fast reinterpretting */
// template<typename fp_T>
// struct alignas(UE::Math::TVector4<fp_T>) Sample_ {
// 	ASSERT_FP_TYPE(fp_T);
// 	union {
// 		struct {
// 			fp_T x;
// 			fp_T y;
// 			fp_T z;
// 			fp_T i;
// 		};
// 		fp_T xyzi[4];
// 	};

// 	inline UE::Math::TVector4<fp_T>* reinterpret()
// 		{ return reinterpret_cast<UE::Math::TVector4<fp_T>*>(this); }
// 	inline const UE::Math::TVector4<fp_T>* reinterpret() const
// 		{ return reinterpret_cast<const UE::Math::TVector4<fp_T>*>(this); }

// };
// using FSample = Sample_<double>;

// POINT_CLOUD_REGISTER_POINT_STRUCT(
// 	Sample_<double>,
// 	(double, x, x)
// 	(double, y, y)
// 	(double, z, z)
// 	(double, i, i)
// )
// POINT_CLOUD_REGISTER_POINT_STRUCT(
// 	Sample_<float>,
// 	(float, x, x)
// 	(float, y, y)
// 	(float, z, z)
// 	(float, i, i)
// )


// template<typename T, size_t blocksize = sizeof(T), bool B_endian = true>
// static std::unique_ptr<char[]> hexify(const T* data, const size_t items = 1U) {
// 	static_assert(blocksize > 0, "Blocksize must be at least 1!");
// 	static constexpr char const* _hex = "0123456789ABCDEF";
// 	static constexpr size_t unit_size = sizeof(T);
// 	const size_t
// 		raw_len = unit_size * items,
// 		blocks = raw_len / blocksize,
// 		chunksize = blocksize * 2 + 1,
// 		len = chunksize * blocks;	// raw space times 2 hex digits per byte + 1 char space for each block + null termination - 1 since last block doesnt need space
// 	char* hex = new char[len];
// 	const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data);
// 	for (size_t i = 0; i < blocks; i++) {
// 		for (size_t b = 0; b < blocksize; b++) {
// 			const uint8_t byte = bytes[i * blocksize + b];	// NOTE: bits within each byte are big-endian
// 			if constexpr (B_endian) {    // big endian byte order (in this case, per-block)
// 				hex[i * chunksize + (blocksize - b - 1) * 2 + 0] = _hex[(byte >> 4)];   // upper 4 bits
// 				hex[i * chunksize + (blocksize - b - 1) * 2 + 1] = _hex[(byte & 0xF)];	// lower 4 bits
// 			}
// 			else {
// 				hex[i * chunksize + b * 2 + 0] = _hex[(byte >> 4)];	    // upper 4 bits
// 				hex[i * chunksize + b * 2 + 1] = _hex[(byte & 0xF)];	// lower 4 bits
// 			}
// 		}
// 		hex[i * chunksize + blocksize * 2] = ' ';
// 	}
// 	hex[len - 1] = '\0';
// 	return std::unique_ptr<char[]>(hex);
// }
// //static inline char* hexify(const void* data, size_t len, char* hex) {
// //	static constexpr char const* _hex = "0123456789ABCDEF";
// //	for (size_t i = 0; i < len; i++) {
// //		const uint8_t v = reinterpret_cast<const uint8_t*>(data)[i];
// //		hex[2 * i + 0] = _hex[v & 0xF];
// //		hex[2 * i + 1] = _hex[v >> 4];
// //	}
// //	return hex;
// //}

// template<
// 	typename v_T,
// 	typename a_T,
// 	typename alloc_T = std::allocator<v_T>>
// static bool memSwap(std::vector<v_T, alloc_T>& std_vec, TArray<a_T>& ue_arr) {
// 	static constexpr bool valid_destruct = std::is_trivially_destructible<v_T>::value && std::is_trivially_destructible<a_T>::value;
// 	static constexpr bool valid_memory = sizeof(v_T) == sizeof(a_T)/* && alignof(v_T) == alignof(a_T)*/;	// alignment only matters if it is larger than the size?
// 	static_assert(valid_destruct, "Base types must be trivially destructible or else they won't be cleaned up correctly after swapping!");
// 	static_assert(valid_memory, "Base types must have similar memory layouts for successful swapping!");
// 	if constexpr (valid_destruct && valid_memory) {
// 		uintptr_t* std_vec_active_ptr = reinterpret_cast<uintptr_t*>(&std_vec);
// 		uintptr_t* ue_arr_active_ptr = reinterpret_cast<uintptr_t*>(&ue_arr);
// 		uintptr_t	// uint64_t
// 			std_vec_buff_start = std_vec_active_ptr[0],
// 			std_vec_buff_elem_end = std_vec_active_ptr[1],
// 			std_vec_buff_cap_end = std_vec_active_ptr[2],
// 			ue_arr_buff_start = ue_arr_active_ptr[0];
// 		uint32_t
// 			ue_arr_buff_elem_num = reinterpret_cast<uint32_t*>(ue_arr_active_ptr + 1)[0],
// 			ue_arr_buff_cap_num = reinterpret_cast<uint32_t*>(ue_arr_active_ptr + 1)[1];
// 		std_vec_active_ptr[0] = ue_arr_buff_start;
// 		std_vec_active_ptr[1] = reinterpret_cast<uintptr_t>(reinterpret_cast<a_T*>(ue_arr_buff_start) + ue_arr_buff_elem_num);	// size
// 		std_vec_active_ptr[2] = reinterpret_cast<uintptr_t>(reinterpret_cast<a_T*>(ue_arr_buff_start) + ue_arr_buff_cap_num);	// capacity
// 		ue_arr_active_ptr[0] = std_vec_buff_start;
// 		reinterpret_cast<uint32_t*>(ue_arr_active_ptr + 1)[0] = reinterpret_cast<v_T*>(std_vec_buff_elem_end) - reinterpret_cast<v_T*>(std_vec_buff_start);
// 		reinterpret_cast<uint32_t*>(ue_arr_active_ptr + 1)[1] = reinterpret_cast<v_T*>(std_vec_buff_cap_end) - reinterpret_cast<v_T*>(std_vec_buff_start);
// 		return true;
// 	}
// 	return false;
// }
// template<
// 	typename a_T,
// 	typename v_T,
// 	typename alloc_T = std::allocator<v_T>>
// inline static bool memSwap(TArray<a_T>& ue_arr, std::vector<v_T, alloc_T>& std_vec) {
// 	return memSwap<v_T, a_T, alloc_T>(std_vec, ue_arr);
// }
// template<typename p_T, typename a_T>
// static bool memSwap(pcl::PointCloud<p_T>& cloud, TArray<a_T>& ue_arr) {
// 	if (memSwap<p_T, a_T, Eigen::aligned_allocator<p_T>>(cloud.points, ue_arr)) {
// 		cloud.width = cloud.points.size();
// 		cloud.height = 1;
// 		return true;
// 	}
// 	return false;
// }
// template<typename a_T, typename p_T>
// inline static bool memSwap(TArray<a_T>& ue_arr, pcl::PointCloud<p_T>& cloud) {
// 	return memSwap<p_T, a_T>(cloud, ue_arr);
// }

// static FString run_tests(int i) {
// 	FString logs{};
// 	/*pcl::PointCloud<FSample> cloud;
// 	if(pcl::io::loadPCDFile<FSample>("C:\\Users\\Hoodi\\Downloads\\Untitled_Scan_15_55_17.pcd", cloud) == 0) {
// 		logs += FString::Printf(TEXT("Successfully loaded PCD file: %d total points.\n"), cloud.points.size());
// 	} else {
// 		logs += FString::Printf(TEXT("Failed to load PCD file :(\n"));
// 	}
// 	FSample* src = cloud.points.data();
// 	FVector4* cast = (FVector4*)src;
// 	size_t errors = 0, i = 0;
// 	for (; i < cloud.points.size(); i++) {
// 		FSample& a = src[i];
// 		FVector4& b = cast[i];
// 		errors += (
// 			(a.x == b.X) +
// 			(a.y == b.Y) +
// 			(a.z == b.Z) +
// 			(a.i == b.W)
// 		);
// 	}
// 	logs += FString::Printf(TEXT("Total reinterpretation equality: %d, total scanned: %d \n"), errors, i);*/



// 	////uint8_t* vars = new uint8_t[100];
// 	//std::vector<uint8_t> vec{ { 1, 2, 50, 45, 2, 6 } };
// 	////vec.assign(vars, vars + 100);
// 	//TArray<uint8_t> tarr{ { 7, 8, 9, 72, 4 } };
// 	////tarr.Append(vars, 100);

// 	///*std::uintptr_t
// 	//	vd = reinterpret_cast<std::uintptr_t>(vec.data()),
// 	//	td = reinterpret_cast<std::uintptr_t>(tarr.GetData());*/

// 	////std::unique_ptr<char[]> buff = hexify(vars, 100);
// 	//std::unique_ptr<char[]> vec_hex = hexify<std::vector<uint8_t>, 8, true>(&vec);
// 	//std::unique_ptr<char[]> tarr_hex = hexify<TArray<uint8_t>, 4, true>(&tarr);
// 	//memSwap(vec, tarr);
// 	//std::unique_ptr<char[]> vec_hex_swap = hexify<std::vector<uint8_t>, 8, true>(&vec);
// 	//std::unique_ptr<char[]> tarr_hex_swap = hexify<TArray<uint8_t>, 4, true>(&tarr);

// 	//logs += FString::Printf(
// 	//	TEXT("Vector (PRE): %s, \tTArray (PRE): %s, \tVector (POST): %s, \tTArray (POST): %s\t\t"),
// 	//	ANSI_TO_TCHAR(vec_hex.get()), ANSI_TO_TCHAR(tarr_hex.get()),
// 	//	ANSI_TO_TCHAR(vec_hex_swap.get()), ANSI_TO_TCHAR(tarr_hex_swap.get())
// 	//);
// 	//logs += FString::Printf(
// 	//	TEXT("Vector size: %d, TArray size: %d"),
// 	//	vec.size(),
// 	//	tarr.Num()
// 	//);

// 	std::vector<FSample> samples{ { FSample{4, 5, 3, 4}, FSample{1, 2, 4, 54}, FSample{7, 4, 7, 3} } };
// 	TArray<FVector4> vectors{ /*{ FVector4{1, 1, 1, 1}, FVector4{2, 2, 2, 2} }*/ };
// 	std::unique_ptr<char[]> vec_hex = hexify(&samples);
// 	std::unique_ptr<char[]> arr_hex = hexify(&vectors);
// 	memSwap(samples, vectors);
// 	std::unique_ptr<char[]> vec_hex_post = hexify(&samples);
// 	std::unique_ptr<char[]> arr_hex_post = hexify(&vectors);
// 	logs += FString::Printf(
// 		TEXT("Vector (PRE): %s, \tTArray (PRE): %s, \tVector (POST): %s, \tTArray (POST): %s\t\t"),
// 		ANSI_TO_TCHAR(vec_hex.get()), ANSI_TO_TCHAR(arr_hex.get()),
// 		ANSI_TO_TCHAR(vec_hex_post.get()), ANSI_TO_TCHAR(arr_hex_post.get())
// 	);
// 	logs += FString::Printf(
// 		TEXT("Vector size: %d, TArray size: %d"),
// 		samples.size(),
// 		vectors.Num()
// 	);

// 	/*std::unique_ptr<char[]> vd_hex = hexify(&vd);
// 	std::unique_ptr<char[]> td_hex = hexify(&td);
// 	logs += FString::Printf(
// 		TEXT("Raw Buffer: %s,\tVector >> Memory: %s, Data Ptr: %s, Size: %d, Cap: %d,\tTArray >> Memory: %s, Data Ptr: %s, Size: %d, Cap: %d"),
// 		ANSI_TO_TCHAR(buff.get()),
// 		ANSI_TO_TCHAR(vec_hex.get()), ANSI_TO_TCHAR(vd_hex.get()), vec.size(), vec.capacity(),
// 		ANSI_TO_TCHAR(tarr_hex.get()), ANSI_TO_TCHAR(td_hex.get()), tarr.Num(), tarr.Max()
// 	);*/

// 	return logs;
// }
// // From tests >>
// // Vector: U64(START "data()"), U64(END "end()"), U64(CAP_END)			--> total 24 bytes
// // TArray : U64(START "Data()"), U32(LEN "Num()"), U32(CAP_LEN "Max()") --> total 16 bytes

// //template<typename base_T>
// //static std::unique_ptr<std::vector<base_T>> genStdVec(const base_T* data, const size_t size) {
// //	uint64_t* buffer = new uint64_t[3];
// //	buffer[0] = reinterpret_cast<std::uintptr_t>(data);
// //	buffer[1] = buffer[2] = buffer[0] + size * sizeof(base_T);
// //	return std::unique_ptr<std::vector<base_T>>(buffer);
// //}
// //template<typename base_T, typename alloc_T = std::allocator<base_T>>
// //static size_t replaceData(std::vector<base_T, alloc_T>& std_vec, base_T** raw) {
// //	std::uintptr_t* ptr = reinterpret_cast<std::uintptr_t*>(&std_vec);
// //	*raw = reinterpret_cast<base_T*>(ptr[0]);
// //	size_t len = ptr[1];
// //	ptr[0] = ptr[1] = ptr[0] = 0;
// //	return len;
// //}
// //template<typename base_T>
// //static size_t replaceData(TArray<base_T>& tarr, base_T** raw) {
// //	std::uintptr_t* ptr = reinterpret_cast<std::uintptr_t*>(&tarr);
// //	*raw = reinterpret_cast<base_T*>(ptr[0]);
// //}



// //template<typename reinterpret_T>
// //struct ReinterpretBuffer {
// //
// //	inline ReinterpretBuffer() :
// //		data(new uint8_t[SIZE])
// //	{}
// //	template<typename T>
// //	inline ReinterpretBuffer(T* data) :
// //		data(reinterpret_cast<uint8_t*>(data))
// //	{}
// //	~ReinterpretBuffer() {
// //		delete[] this->data;
// //	}
// //
// //	template<typename _reinterpret_T = reinterpret_T>
// //	inline volatile const _reinterpret_T* get() {
// //		static_assert(sizeof(_reinterpret_T) == SIZE, "Reinterpreted type must match memory signature of base datatype");
// //		return reinterpret_cast<const _reinterpret_T*>(this->data);
// //	}
// //	template<typename _reinterpret_T = reinterpret_T>
// //	inline const _reinterpret_T& operator*() const {
// //		return *this->get<_reinterpret_T>();
// //	}
// //	template<typename _reinterpret_T = reinterpret_T>
// //	inline volatile const _reinterpret_T* operator->() const {
// //		return this->get<_reinterpret_T>();
// //	}
// //
// //	constexpr size_t SIZE{ sizeof(reinterpret_T) };
// //	const uint8_t* data;
// //
// //};


// struct Tracing {

// 	template<typename fp_T = double>
// 	static TArray<UE::Math::TVector<fp_T> > sphericalVectorProduct(const TArray<fp_T>& _theta, const TArray<fp_T>& _phi) {
// 		TArray<UE::Math::TVector<fp_T> > vecs;
// 		sphericalVectorProduct(_theta, _phi, vecs);
// 		return vecs;
// 	}
// 	/* angles should be in radians */
// 	template<typename fp_T = double, typename fpo_T = fp_T>
// 	static void sphericalVectorProduct(const TArray<fp_T>& _theta, const TArray<fp_T>& _phi, TArray<UE::Math::TVector<fpo_T> >& vecs) {
// 		ASSERT_FP_TYPE(fp_T);
// 		const size_t
// 			total_azimuth = _theta.Num(),
// 			total = total_azimuth * _phi.Num();
// 		vecs.SetNum(total);

// 		// precompute theta sine and cosine values since we loop through for each phi angle
// 		fp_T* theta_components{ new fp_T[total_azimuth * 2] };
// 		size_t index = 0;
// 		for (const fp_T theta : _theta) {
// 			theta_components[index++] = sin(theta);
// 			theta_components[index++] = cos(theta);
// 		}

// 		index = 0;
// 		// loop though each layer
// 		for (fp_T phi : _phi) {
// 			// precompute for all points in the layer
// 			const fp_T
// 				sin_phi = sin(phi),
// 				cos_phi = cos(phi);
// 			// loop though each theta
// 			for (int i = 0; i < total_azimuth; i++) {
// 				// easier access to the precomputed components using pointer arithmetic
// 				const fp_T* theta = theta_components + (i * 2);
// 				// form xyz based on spherical-cartesian conversion
// 				vecs[index++] = {
// 					cos_phi * theta[1],		// x = r * cos(phi) * cos(theta)
// 					cos_phi * theta[0],		// y = r * cos(phi) * sin(theta)
// 					sin_phi					// z = r * sin(phi)
// 				};
// 			}
// 		}

// 		delete[] theta_components;

// 	}

// 	/* generate the start and end bounds for a set of traces given the direction, range, and world transform */
// 	template<typename fp_T = double>
// 	static void genTraceBounds(
// 		const UE::Math::TTransform<fp_T>&			to_world,
// 		const TArray< UE::Math::TVector<fp_T> >&	vecs,
// 		const fp_T									range,
// 		TArray< UE::Math::TVector<fp_T> >&			start_vecs,
// 		TArray< UE::Math::TVector<fp_T> >&			end_vecs
// 	) {
// 		ASSERT_FP_TYPE(fp_T);
// 		const size_t len = vecs.Num();
// 		if (start_vecs.Num() < len) start_vecs.SetNum(len);
// 		if (end_vecs.Num() < len) end_vecs.SetNum(len);
// 		for (int i = 0; i < len; i++) {
// 			genTraceBounds<fp_T>(to_world, vecs[i], range, start_vecs[i], end_vecs[i]);
// 		}
// 	}
// 	/* generate the bounds for a single trace */
// 	template<typename fp_T = double>
// 	static void genTraceBounds(
// 		const UE::Math::TTransform<fp_T>& to_world,
// 		const UE::Math::TVector<fp_T>& vec,
// 		const fp_T range,
// 		UE::Math::TVector<fp_T>& start_vec,
// 		UE::Math::TVector<fp_T>& end_vec
// 	) {
// 		ASSERT_FP_TYPE(fp_T);
// 		// ray should extend from transformer's position in the rotated direction as far as the range
// 		start_vec = to_world.GetLocation();
// 		end_vec = to_world.TransformPositionNoScale(vec * range);
// 	}



// 	template<typename intensity_T = float>
// 	static void scan(const AActor* src, const TArray<FVector>& directions, const double range, std::function<void(FVector&&, intensity_T)> out) {
// 		ASSERT_FP_TYPE(intensity_T);

// 		TStatId stats{};
// 		FCollisionQueryParams trace_params = FCollisionQueryParams(TEXT("LiDAR Trace"), stats, true, src);
// 		trace_params.bReturnPhysicalMaterial = true;

// 		FHitResult result{};
// 		FVector start{}, end{};

// 		const FTransform& to_world = src->ActorToWorld();
// 		const size_t len = directions.Num();
// 		for (int i = 0; i < len; i++) {

// 			genTraceBounds<double>(to_world, directions[i], range, start, end);
// 			// in the future we may want to use a multi-trace and test materials for transparency
// 			src->GetWorld()->LineTraceSingleByObjectType(
// 				result, start, end,
// 				FCollisionObjectQueryParams::DefaultObjectQueryParam, trace_params
// 			);

// 			if (result.bBlockingHit) {
// 				// set W component to represent intensity --> determined from intersection material somehow...
// 				out(FVector(result.Location), (intensity_T)1.0);	// change to FSample or other PCL compatible vector type
// 			}

// 		}

// 	}

// 	/*static const int s = sizeof(FVector4);
// 	static const int s2 = sizeof(pcl::PointXYZI);

// 	static void convert(TArray<FVector4>& xyzi_array, pcl::PointCloud<pcl::PointXYZI>& cloud) {

// 	}*/


// };





// DECLARE_STATS_GROUP(TEXT("LidarSimulation"), STATGROUP_LidarSim, STATCAT_Advanced);
// DECLARE_CYCLE_STAT(TEXT("Bulk Scan"), STAT_BulkScan, STATGROUP_LidarSim);
// DECLARE_CYCLE_STAT(TEXT("Segment Plane"), STAT_SegmentPoip, STATGROUP_LidarSim);

// DEFINE_LOG_CATEGORY(LidarSimComponent);


// void ULidarSimulationComponent::ConvertToRadians(TArray<float>& thetas, TArray<float>& phis) {
// 	for (int i = 0; i < thetas.Num(); i++) {
// 		thetas[i] = FMath::DegreesToRadians(thetas[i]);
// 	}
// 	for (int i = 0; i < phis.Num(); i++) {
// 		phis[i] = FMath::DegreesToRadians(phis[i]);
// 	}
// }

// void ULidarSimulationComponent::GenerateDirections(const TArray<float>& thetas, const TArray<float>& phis, TArray<FVector>& directions) {
// 	Tracing::sphericalVectorProduct<float, double>(thetas, phis, directions);
// }

// void ULidarSimulationComponent::Scan_0(const TArray<FVector>& directions, TArray<FVector4>& hits, const float max_range) {
// 	SCOPE_CYCLE_COUNTER(STAT_BulkScan);
// 	if (hits.Num() != 0) {
// 		UE_LOG(LidarSimComponent, Warning, TEXT("Hits output array contains prexisting elements."));
// 	}
// 	Tracing::scan<float>(this->GetOwner(), directions, max_range,
// 		[&hits](FVector&& pos, float i) {
// 			hits.Emplace(std::move(pos), (double)i);
// 		}
// 	);
// }
// void ULidarSimulationComponent::Scan_1(const TArray<FVector>& directions, TArray<FVector>& positions, TArray<float>& intensities, const float max_range) {
// 	SCOPE_CYCLE_COUNTER(STAT_BulkScan);
// 	if (positions.Num() != 0) {					UE_LOG(LidarSimComponent, Warning, TEXT("Positions output array contains prexisting elements.")); }
// 	if (intensities.Num() != 0) {				UE_LOG(LidarSimComponent, Warning, TEXT("Intensities output array contains prexisting elements.")); }
// 	if (positions.Num() != intensities.Num()) { UE_LOG(LidarSimComponent, Error, TEXT("Output arrays have unequal initial sizes - outputs will be misaligned.")); }
// 	Tracing::scan<float>(this->GetOwner(), directions, max_range,
// 		[&positions, &intensities](FVector&& pos, float i) {
// 			positions.Emplace(std::move(pos));
// 			intensities.Add(i);
// 		}
// 	);
// }
// void ULidarSimulationComponent::Scan_2(const TArray<FVector>& directions, TArray<FLinearColor>& positions, TArray<uint8>& generated_colors, const float max_range, const FColor intensity_albedo) {
// 	SCOPE_CYCLE_COUNTER(STAT_BulkScan);
// 	if (positions.Num() != 0) {							UE_LOG(LidarSimComponent, Warning, TEXT("Positions output array contains prexisting elements.")); }
// 	if (generated_colors.Num() != 0) {					UE_LOG(LidarSimComponent, Warning, TEXT("Intensities output array contains prexisting elements.")); }
// 	if (positions.Num() != generated_colors.Num()) {	UE_LOG(LidarSimComponent, Error, TEXT("Output arrays have unequal initial sizes - outputs will be misaligned.")); }
// 	Tracing::scan<float>(this->GetOwner(), directions, max_range,
// 		[&positions, &generated_colors, &intensity_albedo](FVector&& pos, float i) {
// 			positions.Emplace(std::move(pos));
// 			generated_colors.Add(i * intensity_albedo.R);	// there is probably a more optimal way to add 4 units to the array
// 			generated_colors.Add(i * intensity_albedo.G);
// 			generated_colors.Add(i * intensity_albedo.B);
// 			generated_colors.Add(i * intensity_albedo.A);
// 		}
// 	);
// }

// double ULidarSimulationComponent::SavePointsToFile(const TArray<FVector4>& points, const FString& fname) {
// 	pcl::PointCloud<FSample> cloud;
// 	memSwap(cloud, const_cast<TArray<FVector4>&>(points));	// swap to point cloud
// 	const double a = FPlatformTime::Seconds();
// 	if (pcl::io::savePCDFile<FSample>(std::string(TCHAR_TO_UTF8(*fname)), cloud) != 0) {
// 		UE_LOG(LidarSimComponent, Warning, TEXT("Failed to save points to file: %s"), *fname);
// 	}
// 	memSwap(cloud, const_cast<TArray<FVector4>&>(points));	// swap back since the point cloud gets deleted
// 	return FPlatformTime::Seconds() - a;
// }


// //void ULidarSimulationComponent::SegmentPlane(
// //	const TArray<FLinearColor>& points, TArray<int32_t>& inlier_indices, FVector4& plane_fit,
// //	const FVector3f& target_plane_normal, double fit_distance_threshold, double fit_theta_threshold
// //) {
// //	SCOPE_CYCLE_COUNTER(STAT_SegmentPoip);
// //
// //	pcl::PointCloud<Sample_<float>>::Ptr cloud{ new pcl::PointCloud<Sample_<float>> };
// //	pcl::SACSegmentation<Sample_<float>> seg;
// //	pcl::PointIndices inliers{};
// //	pcl::ModelCoefficients fit_coeffs{};
// //
// //	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(points));
// //	PCLHelper<Sample_<float>>::segmodel_perpendicular(
// //		seg, fit_distance_threshold, fit_theta_threshold, reinterpret_cast<const Eigen::Vector3f&>(target_plane_normal));
// //	PCLHelper<Sample_<float>>::filter_single(cloud, seg, inliers, fit_coeffs);
// //	memSwap(inliers.indices, inlier_indices);
// //	plane_fit = FVector4{
// //		fit_coeffs.values[0],
// //		fit_coeffs.values[1],
// //		fit_coeffs.values[2],
// //		fit_coeffs.values[3],
// //	};
// //	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(points));
// //}
// //
// //void ULidarSimulationComponent::RecolorPoints(
// //	TArray<uint8_t>& point_colors, const TArray<int32_t>& inliers,
// //	const FColor inlier_color/*, const FColor outlier_color*/
// //) {
// //	uint32_t* color_bytes = reinterpret_cast<uint32_t*>(point_colors.GetData());
// //	/*if (outlier_color.Bits != 0) {
// //		const size_t total_colors = point_colors.Num() / 4;
// //		for (int i = 0; i < total_colors; i++) {
// //			color_bytes[i] = outlier_color.Bits;
// //		}
// //	}*/
// //	for (int i = 0; i < inliers.Num(); i++) {
// //		color_bytes[inliers[i]] = inlier_color.Bits;
// //	}
// //}