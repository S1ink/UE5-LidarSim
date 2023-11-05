#include <type_traits>

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>
#include <Physics/PhysicsInterfaceCore.h>
#include <PhysicalMaterials/PhysicalMaterial.h>

#define PCL_NO_PRECOMPILE
THIRD_PARTY_INCLUDES_START
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
THIRD_PARTY_INCLUDES_END

#include "PCLHelper.h"
#include "LidarSimComponent.h"


#define ASSERT_FP_TYPE(fp_T) static_assert(std::is_floating_point_v<fp_T>, "fp_T must be floating point type")

/* Analogous to pcl::PointXYZI but memory aligned to match TVector4<> for fast reinterpretting */
template<typename fp_T>
struct Sample_ {
	ASSERT_FP_TYPE(fp_T);
	union {
		struct {
			fp_T x;
			fp_T y;
			fp_T z;
			fp_T i;
		};
		fp_T xyzi[4];
	};

	inline UE::Math::TVector4<fp_T>* reinterpret()
		{ return reinterpret_cast<UE::Math::TVector4<fp_T>*>(this); }
	inline const UE::Math::TVector4<fp_T>* reinterpret() const
		{ return reinterpret_cast<const UE::Math::TVector4<fp_T>*>(this); }

};
using FSample = Sample_<double>;

POINT_CLOUD_REGISTER_POINT_STRUCT(
	Sample_<double>,
	(double, x, x)
	(double, y, y)
	(double, z, z)
	(double, i, i)
)



static FString run_tests() {
	FString logs{};
	pcl::PointCloud<FSample> cloud;
	if(pcl::io::loadPCDFile<FSample>("C:\\Users\\Hoodi\\Downloads\\Untitled_Scan_15_55_17.pcd", cloud) == 0) {
		logs += FString::Printf(TEXT("Successfully loaded PCD file: %d total points.\n"), cloud.points.size());
	} else {
		logs += FString::Printf(TEXT("Failed to load PCD file :(\n"));
	}
	FSample* src = cloud.points.data();
	FVector4* cast = (FVector4*)src;
	size_t errors = 0, i = 0;
	for (; i < cloud.points.size(); i++) {
		FSample& a = src[i];
		FVector4& b = cast[i];
		errors += (
			(a.x == b.X) +
			(a.y == b.Y) +
			(a.z == b.Z) +
			(a.i == b.W)
		);
	}
	logs += FString::Printf(TEXT("Total reinterpretation equality: %d, total scanned: %d \n"), errors, i);
	return logs;
}


struct Tracing {

	template<typename fp_T = double>
	static TArray<UE::Math::TVector<fp_T> > sphericalVectorProduct(const TArray<fp_T>& _theta, const TArray<fp_T>& _phi) {
		TArray<UE::Math::TVector<fp_T> > vecs;
		sphericalVectorProduct(_theta, _phi, vecs);
		return vecs;
	}
	/* angles should be in radians */
	template<typename fp_T = double, typename fpo_T = fp_T>
	static void sphericalVectorProduct(const TArray<fp_T>& _theta, const TArray<fp_T>& _phi, TArray<UE::Math::TVector<fpo_T> >& vecs) {
		ASSERT_FP_TYPE(fp_T);
		const size_t
			total_azimuth = _theta.Num(),
			total = total_azimuth * _phi.Num();
		vecs.SetNum(total);

		// precompute theta sine and cosine values since we loop through for each phi angle
		fp_T* theta_components{ new fp_T[total_azimuth * 2] };
		size_t index = 0;
		for (const fp_T theta : _theta) {
			theta_components[index++] = sin(theta);
			theta_components[index++] = cos(theta);
		}

		index = 0;
		// loop though each layer
		for (fp_T phi : _phi) {
			// precompute for all points in the layer
			const fp_T
				sin_phi = sin(phi),
				cos_phi = cos(phi);
			// loop though each theta
			for (int i = 0; i < total_azimuth; i++) {
				// easier access to the precomputed components using pointer arithmetic
				const fp_T* theta = theta_components + (i * 2);
				// form xyz based on spherical-cartesian conversion
				vecs[index++] = {
					cos_phi * theta[1],		// x = r * cos(phi) * cos(theta)
					cos_phi * theta[0],		// y = r * cos(phi) * sin(theta)
					sin_phi					// z = r * sin(phi)
				};
			}
		}

		delete[] theta_components;

	}

	/* generate the start and end bounds for a set of traces given the direction, range, and world transform */
	template<typename fp_T = double>
	static void genTraceBounds(
		const UE::Math::TTransform<fp_T>&			to_world,
		const TArray< UE::Math::TVector<fp_T> >&	vecs,
		const fp_T									range,
		TArray< UE::Math::TVector<fp_T> >&			start_vecs,
		TArray< UE::Math::TVector<fp_T> >&			end_vecs
	) {
		ASSERT_FP_TYPE(fp_T);
		const size_t len = vecs.Num();
		if (start_vecs.Num() < len) start_vecs.SetNum(len);
		if (end_vecs.Num() < len) end_vecs.SetNum(len);
		for (int i = 0; i < len; i++) {
			genTraceBounds<fp_T>(to_world, vecs[i], range, start_vecs[i], end_vecs[i]);
		}
	}
	/* generate the bounds for a single trace */
	template<typename fp_T = double>
	static void genTraceBounds(
		const UE::Math::TTransform<fp_T>& to_world,
		const UE::Math::TVector<fp_T>& vec,
		const fp_T range,
		UE::Math::TVector<fp_T>& start_vec,
		UE::Math::TVector<fp_T>& end_vec
	) {
		ASSERT_FP_TYPE(fp_T);
		// ray should extend from transformer's position in the rotated direction as far as the range
		start_vec = to_world.GetLocation();
		end_vec = to_world.TransformVectorNoScale(vec * range);
	}



	static void scan(const AActor* src, const TArray<FVector>& directions, const double range, TArray<FVector4>& results) {

		TStatId stats{};
		FCollisionQueryParams trace_params = FCollisionQueryParams(TEXT("LiDAR Trace"), stats, true, src);
		trace_params.bReturnPhysicalMaterial = true;

		FHitResult result{};
		FVector start{}, end{};

		const FTransform& to_world = src->ActorToWorld();
		const size_t len = directions.Num();
		for (int i = 0; i < len; i++) {

			genTraceBounds<double>(to_world, directions[i], range, start, end);
			// in the future we may want to use a multi-trace and test materials for transparency
			src->GetWorld()->LineTraceSingleByObjectType(
				result, start, end,
				FCollisionObjectQueryParams::DefaultObjectQueryParam, trace_params
			);

			if (result.bBlockingHit) {
				// set W component to represent intensity --> determined from intersection material somehow...
				results.Emplace(result.Location);	// change to FSample or other PCL compatible vector type
			}

		}

	}

	/*static const int s = sizeof(FVector4);
	static const int s2 = sizeof(pcl::PointXYZI);

	static void convert(TArray<FVector4>& xyzi_array, pcl::PointCloud<pcl::PointXYZI>& cloud) {

	}*/


};






DEFINE_LOG_CATEGORY(LidarSim);

void ULidarSimulationComponent::ConvertToRadians(TArray<float>& thetas, TArray<float>& phis) {
	for (int i = 0; i < thetas.Num(); i++) {
		thetas[i] = FMath::DegreesToRadians(thetas[i]);
	}
	for (int i = 0; i < phis.Num(); i++) {
		phis[i] = FMath::DegreesToRadians(phis[i]);
	}
}

void ULidarSimulationComponent::GenerateDirections(const TArray<float>& thetas, const TArray<float>& phis, TArray<FVector>& directions) {
	Tracing::sphericalVectorProduct<float, double>(thetas, phis, directions);
}

double ULidarSimulationComponent::Scan(const TArray<FVector>& directions, TArray<FVector4>& hits, const float max_range) {
	const double a = FPlatformTime::Seconds();
	Tracing::scan(this->GetOwner(), directions, max_range, hits);
	return FPlatformTime::Seconds() - a;
}

double ULidarSimulationComponent::SavePointsToFile(const TArray<FVector4>& points, const FString& fname) {
	pcl::PointCloud<FSample> cloud;
	const FSample* pts = reinterpret_cast<const FSample*>(points.GetData());
	cloud.points.assign(pts, pts + points.Num());
	cloud.width = points.Num();
	cloud.height = 1;
	const double a = FPlatformTime::Seconds();
	if (pcl::io::savePCDFile<FSample>(std::string(TCHAR_TO_UTF8(*fname)), cloud) != 0) {
		UE_LOG(LidarSim, Warning, TEXT("Failed to save points to file: %s"), *fname);
	}
	return FPlatformTime::Seconds() - a;
}