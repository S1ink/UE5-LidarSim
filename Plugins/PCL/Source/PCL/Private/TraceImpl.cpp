#include <type_traits>

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>
#include <Physics/PhysicsInterfaceCore.h>
#include <PhysicalMaterials/PhysicalMaterial.h>

#include "PCLHelper.h"


#define ASSERT_FP_TYPE(fp_T) static_assert(std::is_floating_point_v<fp_T>, "fp_T must be floating point type")


struct Tracing {

	template<typename fp_T = double>
	TArray<UE::Math::TVector<fp_T> > sphericalVectorProduct(const TArray<fp_T>& _theta, const TArray<fp_T>& _phi) {
		TArray<UE::Math::TVector<fp_T> > vecs;
		sphericalVectorProduct(_theta, _phi, vecs);
		return vecs;
	}
	/* angles should be in radians */
	template<typename fp_T = double>
	void sphericalVectorProduct(const TArray<fp_T>& _theta, const TArray<fp_T>& _phi, TArray<UE::Math::TVector<fp_T> >& vecs) {
		ASSERT_FP_TYPE(fp_T);
		const size_t
			total_azimuth = _theta.Num(),
			total = total_azimuth * _phi.Num();
		vecs.Reserve(total);

		// precompute theta sine and cosine values since we loop through for each phi angle
		TArray<fp_T> theta_components{};
		theta_components.Reserve(total_azimuth * 2);
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
				const fp_T* theta = theta_components.GetData() + (i * 2 * sizeof(fp_T));
				// form xyz based on spherical-cartesian conversion
				vecs[index++] = {
					cos_phi * theta[1],		// x = r * cos(phi) * cos(theta)
					cos_phi * theta[0],		// y = r * cos(phi) * sin(theta)
					sin_phi					// z = r * sin(phi)
				};
			}
		}

	}

	/* generate the start and end bounds for a set of traces given the direction, range, and world transform */
	template<typename fp_T = double>
	void genTraceBounds(
		const UE::Math::TTransform<fp_T>&			to_world,
		const TArray< UE::Math::TVector<fp_T> >&	vecs,
		const fp_T									range,
		TArray< UE::Math::TVector<fp_T> >&			start_vecs,
		TArray< UE::Math::TVector<fp_T> >&			end_vecs
	) {
		ASSERT_FP_TYPE(fp_T);
		const size_t len = vecs.Num();
		if (start_vecs.Num() < len) start_vecs.Reserve(len);
		if (end_vecs.Num() < len) end_vecs.Reserve(len);
		for (int i = 0; i < len; i++) {
			genTraceBounds<fp_T>(to_world, vecs[i], range, start_vecs[i], end_vecs[i]);
		}
	}
	/* generate the bounds for a single trace */
	template<typename fp_T = double>
	void genTraceBounds(
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



	void scan(const AActor* src, const TArray<FVector>& directions, const double range, TArray<FVector>& results) {

		TStatId stats{};
		FCollisionQueryParams trace_params = FCollisionQueryParams(TEXT("LiDAR Trace"), stats, true, src);
		trace_params.bReturnPhysicalMaterial = true;

		FHitResult result{};
		FVector start{}, end{};

		const FTransform& to_world = src->ActorToWorld();
		const size_t len = directions.Num();
		for (int i = 0; i < len; i++) {

			genTraceBounds<double>(to_world, directions[i], range, start, end);
			src->GetWorld()->LineTraceSingleByObjectType(
				result, start, end,
				FCollisionObjectQueryParams::DefaultObjectQueryParam, trace_params
			);

			if (result.bBlockingHit) {
				results.Add(result.Location);
			}

		}

	}

};