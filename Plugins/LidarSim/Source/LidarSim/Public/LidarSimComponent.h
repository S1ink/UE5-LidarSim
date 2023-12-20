#pragma once

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>

#include <fstream>
THIRD_PARTY_INCLUDES_START
#include <pcl/io/tar.h>
#include <pcl/io/pcd_io.h>
THIRD_PARTY_INCLUDES_END

#include "LidarSimComponent.generated.h"


DECLARE_LOG_CATEGORY_EXTERN(LidarSimComponent, Log, All);


UCLASS(ClassGroup = Simulation, meta = (BlueprintSpawnableComponent))
class LIDARSIM_API ULidarSimulationComponent : public USceneComponent {

	GENERATED_BODY()

public:
	ULidarSimulationComponent(const FObjectInitializer& init) {}
	~ULidarSimulationComponent() {}


	/** Blueprint callable bulk angle conversion to radians */
	UFUNCTION(BlueprintCallable, DisplayName = "LidarSim Convert To Radians (Array)")
	static void ConvertToRadians(UPARAM(ref) TArray<float>& thetas, UPARAM(ref) TArray<float>& phis);

	/** Blueprint callable direction vector generation for an array of spherical coordinate components (output is for each in the cartesian product of the inputs) */
	UFUNCTION(BlueprintCallable, DisplayName = "LidarSim Generate Direction Vectors From Spherical")
	static void GenerateDirections(const TArray<float>& thetas, const TArray<float>& phis, UPARAM(ref) TArray<FVector>& directions);

	/** Initiate a scan centered on the current parent actor instance */
	UFUNCTION(DisplayName = "LidarSim Initiate Scan (Vector4)", BlueprintCallable)
	void Scan_0(
		const TArray<FVector>& directions,
		UPARAM(ref) TArray<FVector4>& hits,
		const float max_range = 1e3f,
		const float noise_distance_scale = 5e-3f);

	/** Initiate a scan centered on the current parent actor instance */
	UFUNCTION(DisplayName = "LidarSim Initate Scan (Vec3 + Intensity)", BlueprintCallable)
	void Scan_1(
		const TArray<FVector>& directions,
		UPARAM(ref) TArray<FVector>& positions, UPARAM(ref) TArray<float>& intensities,
		const float max_range = 1e3f,
		const float noise_distance_scale = 5e-3f);

	/** Initiate a scan centered on the current parent actor instance - optimized output format for the render pipeline */
	UFUNCTION(DisplayName = "LidarSim Initiate Scan (LinearColor + uint8) >> For PCR", BlueprintCallable)
	void Scan_2(
		const TArray<FVector>& directions,
		UPARAM(ref) TArray<FLinearColor>& positions, UPARAM(ref) TArray<uint8>& generated_colors,
		const float max_range = 1e3f, const float noise_distance_scale = 5e-3f,
		const FColor intensity_albedo = FColor::White);

	/** Blueprint callable point saving to file */
	UFUNCTION(BlueprintCallable, DisplayName = "LidarSim Save Points To File")
	static double SavePointsToFile(const TArray<FLinearColor>& points, const FString& fname);


	/** Blueprint callable plane segment function where inlier indices and fit params are exported */
	UFUNCTION(DisplayName = "LidarSim Segment Plane", BlueprintCallable)
	static void SegmentPlane(
		UPARAM(ref) const TArray<FLinearColor>& points, UPARAM(ref) TArray<int32>& inlier_indices, UPARAM(ref) FVector4& plane_fit,
		UPARAM(ref) const FVector3f& target_plane_normal, const double fit_distance_threshold = 1.0, const double fit_theta_threshold = 0.1);

	/*UFUNCTION(DisplayName = "LidarSim Segment Plane", BlueprintCallable)
	void SegmentPlane(
		UPARAM(ref) const TArray<FVector4>& points, UPARAM(ref) TArray<int32_t>& inlier_indices, UPARAM(ref) FVector4& plane_fit,
		double fit_distance_threshold = 1.0, double fit_theta_threshold = 0.1, const FVector3f& target_plane_normal = FVector3f::UpVector);*/

	/** Blueprint callable recolor operation on specified inliers */
	UFUNCTION(DisplayName = "LidarSim Recolor Inliers/Outliers", BlueprintCallable)
	static void RecolorPoints(
		UPARAM(ref) TArray<uint8>& point_colors, UPARAM(ref) const TArray<int32>& inliers,
		const FColor inlier_color/*, const FColor outlier_color = FColor::Transparent*/);



	UFUNCTION(BlueprintCallable, DisplayName = "Sample and export with ring IDs")
	void RingExport(
		const TArray<FVector>& directions, const TArray<int>& ring_ids, const FString& fname,
		const float max_range = 1e3f, const float noise_distance_scale = 5e-3f);

};


UCLASS(Blueprintable, BlueprintType, ClassGroup = IO)
class LIDARSIM_API UPCDWriter : public UObject {

	GENERATED_BODY()

public:
	UPCDWriter() {}
	~UPCDWriter();

	UFUNCTION(BlueprintCallable, DisplayName = "Open TAR for cloud export")
	bool Open(const FString& fname);

	UFUNCTION(BlueprintCallable, DisplayName = "Close TAR")
	void Close();

	UFUNCTION(BlueprintCallable, DisplayName = "Is a file currently open?")
	bool IsOpen();

	UFUNCTION(BlueprintCallable, DisplayName = "Append cloud to TAR")
	bool Append(const FString& fname, const TArray<FLinearColor>& positions,
		const FVector3f origin, const FQuat4f orient,
		bool compress = true);


	bool setFile(const char* fname);
	void closeIO();
	void addCloud(
		const pcl::PCLPointCloud2& cloud, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orient,
		bool compress = true, const char* pcd_fname = nullptr);


protected:
	using spos_t = std::iostream::pos_type;

	pcl::io::TARHeader* head_buff{ nullptr };
	pcl::PointCloud<pcl::PointXYZ> swap_cloud{};
	pcl::PCLPointCloud2 write_cloud{};

	std::ofstream fio{};
	pcl::PCDWriter writer{};
	spos_t append_pos{ 0 };
	uint32_t status_bits{ 0 };

	constexpr static std::ios::openmode
		OPEN_MODES = (std::ios::binary | std::ios::in | std::ios::out);

};

//UCLASS(ClassGroup = Simulation, meta = (BlueprintSpawnableComponent))
//class LIDARSIM_API UPCFilterBuffer {
//
//	GENERATED_BODY();
//
//public:
//	UPCFilterBuffer(const FObjectInitializer& init) {}
//	~UPCFilterBuffer() {}
//
//
//private:
//
//
//};
