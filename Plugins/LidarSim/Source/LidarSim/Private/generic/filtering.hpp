#pragma once

#include <vector>
#include <limits>
#include <memory>
#include <span>

#include <Eigen/Core>

#include <pcl/types.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/impl/voxel_grid.hpp>				// includes <pcl/common/centroid.h> and <boost/sort/spreadsort/integer_sort.hpp> which we use
#include <pcl/filters/impl/morphological_filter.hpp>	// includes <pcl/octree/octree_search.h>



/** Get the min/max of first N dimensions for a selection of points. Does not properly handle non-dense clouds */
template<
	int Ndims = 3,
	typename PointT = pcl::PointXYZ,
	typename IntT = pcl::index_t,
	typename FloatT = float>
void minMaxND(
	const pcl::PointCloud<PointT>& cloud,
	const std::vector<IntT>& selection,
	Eigen::Vector<FloatT, Ndims>& min,
	Eigen::Vector<FloatT, Ndims>& max
) {
	static_assert(Ndims > 0 && Ndims <= 4, "");
	using VecT = Eigen::Vector<FloatT, Ndims>;

	min.setConstant( std::numeric_limits<float>::max() );
	max.setConstant( std::numeric_limits<float>::min() );

	if(selection.empty()) {
		for(const PointT& pt : cloud.points) {
			const VecT* pt2 = reinterpret_cast<const VecT*>(&pt);
			min = min.cwiseMin(*pt2);
			max = max.cwiseMax(*pt2);
		}
	} else {
		for(const IntT i : selection) {
			const VecT* pt2 = reinterpret_cast<const VecT*>(&cloud.points[i]);
			min = min.cwiseMin(*pt2);
			max = max.cwiseMax(*pt2);
		}
	}
}

/** minMaxND<>() alias for getting min/max for x and y */
template<
	typename PointT = pcl::PointXYZ,
	typename IntT = pcl::index_t,
	typename FloatT = float>
constexpr void(*minMaxXY)(
	const pcl::PointCloud<PointT>&, const std::vector<IntT>&,
	Eigen::Vector2<FloatT>&, Eigen::Vector2<FloatT>&
) = &minMaxND<2, PointT, IntT, FloatT>;

/** minMaxND<>() alias for getting min/max for x, y, and z */
template<
	typename PointT = pcl::PointXYZ,
	typename IntT = pcl::index_t,
	typename FloatT = float>
constexpr void(*minMaxXYZ)(
	const pcl::PointCloud<PointT>&, const std::vector<IntT>&,
	Eigen::Vector3<FloatT>&, Eigen::Vector3<FloatT>&
) = &minMaxND<3, PointT, IntT, FloatT>;






/** Voxelization static reimpl -- copied from VoxelGrid<>::applyFilter() and simplified */
template<
	typename PointT = pcl::PointXYZ,
	typename IntT = pcl::index_t,
	typename FloatT = float>
void voxel_filter(
	const pcl::PointCloud<PointT>& cloud,
	const std::vector<IntT>& selection,
	pcl::PointCloud<PointT>& voxelized,
	FloatT leaf_x, FloatT leaf_y, FloatT leaf_z,
	unsigned int min_points_per_voxel_ = 0,
	bool downsample_all_data_ = false
) {
	const bool use_selection = !selection.empty();

	const Eigen::Vector3<FloatT>
		leaf_size_{ leaf_x, leaf_y, leaf_z };
	const Eigen::Array3<FloatT>
		inverse_leaf_size_{ Eigen::Array3<FloatT>::Ones() / leaf_size_.array() };

	// Copy the header (and thus the frame_id) + allocate enough space for points
	voxelized.height       = 1;                    // downsampling breaks the organized structure
	voxelized.is_dense     = true;                 // we filter out invalid points

	// Eigen::Vector4f min_p, max_p;
	// // Get the minimum and maximum dimensions
	// if(use_selection) {
	// 	pcl::getMinMax3D<PointT>(cloud, selection, min_p, max_p);
	// } else {
	// 	pcl::getMinMax3D<PointT>(cloud, min_p, max_p);
	// }
	Eigen::Vector3<FloatT> min_p, max_p;
	minMaxXYZ<PointT>(cloud, selection, min_p, max_p);

	// Check that the leaf size is not too small, given the size of the data
	std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
	std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
	std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

	if( (dx * dy * dz) > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) ) {
		voxelized.clear();
		return;
	}

	Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

	// Compute the minimum and maximum bounding box values
	min_b_[0] = static_cast<int> ( std::floor(min_p[0] * inverse_leaf_size_[0]) );
	max_b_[0] = static_cast<int> ( std::floor(max_p[0] * inverse_leaf_size_[0]) );
	min_b_[1] = static_cast<int> ( std::floor(min_p[1] * inverse_leaf_size_[1]) );
	max_b_[1] = static_cast<int> ( std::floor(max_p[1] * inverse_leaf_size_[1]) );
	min_b_[2] = static_cast<int> ( std::floor(min_p[2] * inverse_leaf_size_[2]) );
	max_b_[2] = static_cast<int> ( std::floor(max_p[2] * inverse_leaf_size_[2]) );

	// Compute the number of divisions needed along all axis
	div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
	div_b_[3] = 0;

	// Set up the division multiplier
	divb_mul_ = Eigen::Vector4i{ 1, div_b_[0], div_b_[0] * div_b_[1], 0 };

	// Storage for mapping leaf and pointcloud indexes
	std::vector<cloud_point_index_idx> index_vector;

	// First pass: go over all points and insert them into the index_vector vector
	// with calculated idx. Points with the same idx value will contribute to the
	// same point of resulting CloudPoint
	if(use_selection) {
		index_vector.reserve(selection.size());
		for(const auto& index : selection) {
			if(!cloud.is_dense && !pcl::isXYZFinite(cloud[index])) continue;

			int ijk0 = static_cast<int>( std::floor(cloud[index].x * inverse_leaf_size_[0]) - static_cast<float>(min_b_[0]) );
			int ijk1 = static_cast<int>( std::floor(cloud[index].y * inverse_leaf_size_[1]) - static_cast<float>(min_b_[1]) );
			int ijk2 = static_cast<int>( std::floor(cloud[index].z * inverse_leaf_size_[2]) - static_cast<float>(min_b_[2]) );

			// Compute the centroid leaf index
			int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
			index_vector.emplace_back( static_cast<unsigned int>(idx), index );
		}
	} else {
		index_vector.reserve(cloud.size());
		for(IntT index = 0; index < cloud.size(); index++) {
			if(!cloud.is_dense && !pcl::isXYZFinite(cloud[index])) continue;

			int ijk0 = static_cast<int>( std::floor(cloud[index].x * inverse_leaf_size_[0]) - static_cast<float>(min_b_[0]) );
			int ijk1 = static_cast<int>( std::floor(cloud[index].y * inverse_leaf_size_[1]) - static_cast<float>(min_b_[1]) );
			int ijk2 = static_cast<int>( std::floor(cloud[index].z * inverse_leaf_size_[2]) - static_cast<float>(min_b_[2]) );

			// Compute the centroid leaf index
			int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
			index_vector.emplace_back( static_cast<unsigned int>(idx), index );
		}
	}

	// Second pass: sort the index_vector vector using value representing target cell as index
	// in effect all points belonging to the same output cell will be next to each other
	auto rightshift_func = [](const cloud_point_index_idx &x, const unsigned offset) { return x.idx >> offset; };
	boost::sort::spreadsort::integer_sort(index_vector.begin(), index_vector.end(), rightshift_func);

	// Third pass: count output cells
	// we need to skip all the same, adjacent idx values
	unsigned int total = 0;
	unsigned int index = 0;
	// first_and_last_indices_vector[i] represents the index in index_vector of the first point in
	// index_vector belonging to the voxel which corresponds to the i-th output point,
	// and of the first point not belonging to.
	std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
	// Worst case size
	first_and_last_indices_vector.reserve (index_vector.size());
	while(index < index_vector.size()) {
		unsigned int i = index + 1;
		for(; i < index_vector.size() && index_vector[i].idx == index_vector[index].idx; ++i);
		if (i - index >= min_points_per_voxel_) {
			++total;
			first_and_last_indices_vector.emplace_back(index, i);
		}
		index = i;
	}

	// Fourth pass: compute centroids, insert them into their final position
	voxelized.resize(total);

	index = 0;
	for (const auto &cp : first_and_last_indices_vector) {
		// calculate centroid - sum values from all input points, that have the same idx value in index_vector array
		unsigned int first_index = cp.first;
		unsigned int last_index = cp.second;

		//Limit downsampling to coords
		if (!downsample_all_data_) {
			Eigen::Vector4f centroid{ Eigen::Vector4f::Zero() };

			for (unsigned int li = first_index; li < last_index; ++li) {
				centroid += cloud[index_vector[li].cloud_point_index].getVector4fMap();
			}
			centroid /= static_cast<float> (last_index - first_index);
			voxelized[index].getVector4fMap() = centroid;
		}
		else {
			pcl::CentroidPoint<PointT> centroid;

			// fill in the accumulator with leaf points
			for (unsigned int li = first_index; li < last_index; ++li) {
				centroid.add( cloud[index_vector[li].cloud_point_index] );
			}
			centroid.get(voxelized[index]);
		}
		++index;
	}
	voxelized.width = voxelized.size ();

}





/** Cartesian "crop box" filter reimpl -- copied from pcl::CropBox<>::applyFilter() and simplified */
template<
	typename PointT = pcl::PointXYZ,
	typename IntT = pcl::index_t,
	bool negative_ = false>
void cropbox_filter(
	const pcl::PointCloud<PointT>& cloud,
	const std::vector<IntT>& selection,
	std::vector<IntT>& filtered,
	const Eigen::Vector3f min_pt_ = Eigen::Vector3f{ -1.f, -1.f, -1.f },
	const Eigen::Vector3f max_pt_ = Eigen::Vector3f{ 1.f, 1.f, 1.f }
	// full CropBox<> also allows additional transformation of box and cloud
) {
	const bool use_selection = !selection.empty();

	filtered.clear();
	filtered.reserve(use_selection ? selection.size() : cloud.size());	// reserve maximum size

	if(use_selection) {
		for (const IntT index : selection) {
			const PointT& pt = cloud[index];
			if( !cloud.is_dense && !isFinite(pt) ) continue;

			if( (pt.x < min_pt_[0] || pt.y < min_pt_[1] || pt.z < min_pt_[2]) ||
				(pt.x > max_pt_[0] || pt.y > max_pt_[1] || pt.z > max_pt_[2]) )
			{
				if constexpr(negative_) {
					filtered.push_back(index);	// outside the cropbox --> push on negative
				}
			} else if constexpr(!negative_) {
				filtered.push_back(index);		// inside the cropbox and not negative
			}
		}
	} else {
		for (size_t index = 0; index < cloud.points.size(); index++) {
			const PointT& pt = cloud[index];
			if( !cloud.is_dense && !isFinite(pt) ) continue;

			if( (pt.x < min_pt_[0] || pt.y < min_pt_[1] || pt.z < min_pt_[2]) ||
				(pt.x > max_pt_[0] || pt.y > max_pt_[1] || pt.z > max_pt_[2]) )
			{
				if constexpr(negative_) {
					filtered.push_back(index);	// outside the cropbox --> push on negative
				}
			} else if constexpr(!negative_) {
				filtered.push_back(index);		// inside the cropbox and not negative
			}
		}
	}

}



/** cropbox_filter<>() specialization for sorting exclusively using the z-coordinate */
template<
	typename PointT = pcl::PointXYZ,
	typename IntT = pcl::index_t,
	typename FloatT = float,
	bool negative_ = false>
void carteZ_filter(
	const pcl::PointCloud<PointT>& cloud,
	const std::vector<IntT>& selection,
	std::vector<IntT>& filtered,
	const FloatT min_z = -1.f,
	const FloatT max_z = 1.f
) {
	const bool use_selection = !selection.empty();

	filtered.clear();
	filtered.reserve(use_selection ? selection.size() : cloud.size());	// reserve maximum size

	if(use_selection) {
		for (const IntT index : selection) {
			const PointT& pt = cloud[index];
			if( !cloud.is_dense && !isFinite(pt) ) continue;
			if( pt.z < min_z || pt.z > max_z ) {
				if constexpr(negative_) {
					filtered.push_back(index);	// outside the cropbox --> push on negative
				}
			} else if constexpr(!negative_) {
				filtered.push_back(index);		// inside the cropbox and not negative
			}
		}
	} else {
		for (size_t index = 0; index < cloud.points.size(); index++) {
			const PointT& pt = cloud[index];
			if( !cloud.is_dense && !isFinite(pt) ) continue;
			if( pt.z < min_z || pt.z > max_z ) {
				if constexpr(negative_) {
					filtered.push_back(index);	// outside the cropbox --> push on negative
				}
			} else if constexpr(!negative_) {
				filtered.push_back(index);		// inside the cropbox and not negative
			}
		}
	}

}





/** PMF filter reimpl -- See <pcl/filters/progressive_morphological_filter.h> */
template<
	// bool mirror_z = false,
	typename PointT = pcl::PointXYZ,
	typename IntT = pcl::index_t>
void progressive_morph_filter(
	const pcl::PointCloud<PointT>& cloud_,
	const std::vector<IntT>& selection,
	pcl::Indices& ground,
	const float base_,
	const float max_window_size_,
	const float cell_size_,
	const float initial_distance_,
	const float max_distance_,
	const float slope_,
	const bool exponential_
) {
	// Compute the series of window sizes and height thresholds
	std::vector<float> height_thresholds;
	std::vector<float> window_sizes;
	int iteration = 0;
	float window_size = 0.0f;
	float height_threshold = 0.0f;

	while (window_size < max_window_size_)
	{
		// Determine the initial window size.
		if (exponential_)
			window_size = cell_size_ * (2.0f * std::pow(base_, iteration) + 1.0f);		// << this becomes an issue when base_ is less than 0 since the loop never exits! :O
		else
			window_size = cell_size_ * (2.0f * (iteration + 1) * base_ + 1.0f);

		// Calculate the height threshold to be used in the next iteration.
		if (iteration == 0)
			height_threshold = initial_distance_;
		else
			height_threshold = slope_ * (window_size - window_sizes[iteration - 1]) * cell_size_ + initial_distance_;

		// Enforce max distance on height threshold
		if (height_threshold > max_distance_)
			height_threshold = max_distance_;

		window_sizes.push_back(window_size);
		height_thresholds.push_back(height_threshold);

		iteration++;
	}

	// Ground indices are initially limited to those points in the input cloud we
	// wish to process
	if (selection.size() > 0 && selection.size() <= cloud_.size()) {
		ground = selection;
	} else {
		ground.resize(cloud_.size());
		for (std::size_t i = 0; i < cloud_.size(); i++) {
			ground[i] = i;
		}
	}

	pcl::octree::OctreePointCloudSearch<PointT> tree{ 1.f };
	const std::shared_ptr< const pcl::PointCloud<PointT> >
		cloud_shared_ref{ &cloud_, [](const pcl::PointCloud<PointT>*) {} };
	const std::shared_ptr< const pcl::Indices >
		ground_shared_ref{ &ground, [](const pcl::Indices*) {} };

	// reused buffers
	std::vector<pcl::Indices> pt_window_indices{};
	std::vector<float>
		zp_temp{}, zp_final{}, zn_temp{}, zn_final{};
	zp_temp.resize(cloud_.size());
	zp_final.resize(cloud_.size());
	zn_temp.resize(cloud_.size());
	zn_final.resize(cloud_.size());

	// Progressively filter ground returns using morphological open
	for(size_t i = 0; i < window_sizes.size(); i++) {

		// reset tree and reinit to new window size and narrowed selection of points
		tree.deleteTree();
		tree.setResolution(window_sizes[i]);
		tree.setInputCloud(cloud_shared_ref, ground_shared_ref);	// points in the tree will be in the domain of the full cloud
		tree.addPointsFromInputCloud();

		pt_window_indices.resize(ground.size());

		const float half_res = window_sizes[i] / 2.0f;
		// calculate points within each window (for each point in the selection)
		for(size_t _idx = 0; _idx < ground.size(); _idx++) {
			const PointT& _pt = cloud_[ground[_idx]];	// retrieve source (x, y) for each pt in selection
			tree.boxSearch(
				Eigen::Vector3f{
					_pt.x - half_res,
					_pt.y - half_res,
					-std::numeric_limits<float>::max()
				},
				Eigen::Vector3f{
					_pt.x + half_res,
					_pt.y + half_res,
					std::numeric_limits<float>::max()
				},
				pt_window_indices[_idx]		// output into the cache
			);
		}

		// morph op stage 1
		for(size_t p_idx = 0; p_idx < ground.size(); p_idx++) {

			const pcl::Indices& pt_indices = pt_window_indices[p_idx];
			float& _zp_temp = zp_temp[ground[p_idx]];
			float& _zn_temp = zn_temp[ground[p_idx]];
			_zp_temp = _zn_temp = cloud_[ground[p_idx]].z;

			for (const pcl::index_t window_idx : pt_indices) {
				const float _z = cloud_[window_idx].z;
				if (_z < _zp_temp) {
					_zp_temp = _z;
				}
				if (_z > _zn_temp) {
					_zn_temp = _z;
				}
			}

		}

		// morph op stage 2
		for(size_t p_idx = 0; p_idx < ground.size(); p_idx++) {

			const pcl::Indices& pt_indices = pt_window_indices[p_idx];
			float& _zp_final = zp_final[ground[p_idx]];
			float& _zn_final = zn_final[ground[p_idx]];
			_zp_final = _zn_final = zp_temp[ground[p_idx]];

			for (const pcl::index_t window_idx : pt_indices) {
				const float
					_zp = zp_temp[window_idx],
					_zn = zn_temp[window_idx];
				if (_zp > _zp_final) {
					_zp_final = _zp;
				}
				if (_zn < _zn_final) {
					_zn_final = _zn;
				}
			}

		}

		// Find indices of the points whose difference between the source and
		// filtered point clouds is less than the current height threshold.
		int64_t _end = static_cast<int64_t>(ground.size()) - 1;
		for(int64_t p_idx = 0; p_idx <= _end; p_idx++) {

			const float
				diff_p = cloud_[ground[p_idx]].z - zp_final[ground[p_idx]],
				diff_n = zn_final[ground[p_idx]] - cloud_[ground[p_idx]].z;

			if(diff_p >= height_thresholds[i] || diff_n >= height_thresholds[i]) {
				ground[p_idx] = ground[_end];	// we no longer care about the current index so replace it with the current last index and shorten the range
				_end--;
			}

		}
		ground.resize(_end + 1);

	}

}





/** Generate a set of ranges for each point in the provided cloud */
template<
	typename PointT = pcl::PointXYZ,
	typename AllocT = typename pcl::PointCloud<PointT>::VectorType::allocator_type,
	typename IntT = pcl::index_t,
	typename FloatT = float>
void pc_generate_ranges(
	const std::vector<PointT, AllocT>& points,
	const std::vector<IntT>& selection,
	std::vector<FloatT>& out_ranges,
	const Eigen::Vector3<FloatT> origin = Eigen::Vector3<FloatT>::Zero()
) {
	if (!selection.empty()) {
		out_ranges.resize(selection.size());
		for (int i = 0; i < selection.size(); i++) {
			out_ranges[i] = (origin - *reinterpret_cast<const Eigen::Vector3<FloatT>*>(&points[selection[i]])).norm();
		}
	}
	else {
		out_ranges.resize(points.size());
		for (int i = 0; i < points.size(); i++) {
			out_ranges[i] = (origin - *reinterpret_cast<const Eigen::Vector3<FloatT>*>(&points[i])).norm();
		}
	}
}

/** Remove the points at the each index in the provided set. Prereq: selection indices must be sorted in non-descending order! */
template<
	typename PointT = pcl::PointXYZ,
	typename AllocT = typename pcl::PointCloud<PointT>::VectorType::allocator_type,
	typename IntT = pcl::index_t>
void pc_remove_selection(
	std::vector<PointT, AllocT>& points,
	const std::vector<IntT>& selection
) {
	// assert sizes
	size_t last = points.size() - 1;
	for(size_t i = 0; i < selection.size(); i++) {
		memcpy(&points[selection[i]], &points[last], sizeof(PointT));
		last--;
	}
	points.resize(last + 1);
}
/** Normalize the set of points to only include the selected indices. Prereq: selection indices must be sorted in non-descending order! */
template<
	typename PointT = pcl::PointXYZ,
	typename AllocT = typename pcl::PointCloud<PointT>::VectorType::allocator_type,
	typename IntT = pcl::index_t>
inline void pc_normalize_selection(
	std::vector<PointT, AllocT>& points,
	const std::vector<IntT>& selection
) {
	for(size_t i = 0; i < selection.size(); i++) {
		memcpy(&points[i], &points[selection[i]], sizeof(PointT));
	}
	points.resize(selection.size());
}

/** Filter a set of ranges to an inclusive set of indices */
template<
	typename FloatT = float,
	typename IntT = pcl::index_t>
void pc_filter_ranges(
	const std::vector<FloatT>& ranges,
	const std::vector<IntT>& selection,
	std::vector<IntT>& filtered,
	const FloatT min, const FloatT max
) {
	filtered.clear();
	if(!selection.empty()) {
		filtered.reserve(selection.size());
		for(size_t i = 0; i < selection.size(); i++) {
			const IntT idx = selection[i];
			const FloatT r = ranges[idx];
			if(r <= max && r >= min) {
				filtered.push_back(idx);
			}
		}
	} else {
		filtered.reserve(ranges.size());
		for(size_t i = 0; i < ranges.size(); i++) {
			const FloatT r = ranges[i];
			if(r <= max && r >= min) {
				filtered.push_back(i);
			}
		}
	}
}
/** Filter a set of points by their distance from a specified origin point (<0, 0, 0> by default) */
template<
	typename PointT = pcl::PointXYZ,
	typename AllocT = typename pcl::PointCloud<PointT>::VectorType::allocator_type,
	typename IntT = pcl::index_t,
	typename FloatT = float>
void pc_filter_distance(
	const std::vector<PointT, AllocT>& points,
	const std::vector<IntT>& selection,
	std::vector<IntT>& filtered,
	const FloatT min, const FloatT max,
	const Eigen::Vector3<FloatT> origin = Eigen::Vector3<FloatT>::Zero()
) {
	filtered.clear();
	if(!selection.empty()) {
		filtered.reserve(selection.size());
		for(size_t i = 0; i < selection.size(); i++) {
			const IntT idx = selection[i];
			const FloatT r = (origin - *reinterpret_cast<const Eigen::Vector3<FloatT>*>(&points[idx])).norm();
			if(r <= max && r >= min) {
				filtered.push_back(idx);
			}
		}
	} else {
		filtered.reserve(points.size());
		for(size_t i = 0; i < points.size(); i++) {
			const FloatT r = (origin - *reinterpret_cast<const Eigen::Vector3<FloatT>*>(&points[i])).norm();
			if(r <= max && r >= min) {
				filtered.push_back(i);
			}
		}
	}
}

/** Given a base set of indices A and a subset of indices B, get (A - B).
 * prereq: selection indices must be in ascending order */
template<typename IntT = pcl::index_t>
void pc_negate_selection(
	const std::vector<IntT>& base,
	const std::vector<IntT>& selection,
	std::vector<IntT>& negated
) {
	if(base.size() <= selection.size()) {
		return;
	}
	negated.resize(base.size() - selection.size());
	size_t
		_base = 0,
		_select = 0,
		_negate = 0;
	for(; _base < base.size() && _negate < negated.size(); _base++) {
		if(_select < selection.size() && base[_base] == selection[_select]) {
			_select++;
		} else {
			negated[_negate] = base[_base];
			_negate++;
		}
	}
}
/** Given a base set of indices A and a subset of indices B, get (A - B).
 * prereq: selection indices must be in ascending order */
template<typename IntT = pcl::index_t>
void pc_negate_selection(
	const IntT base_range,
	const std::vector<IntT>& selection,
	std::vector<IntT>& negated
) {
	if (base_range <= selection.size()) {
		return;
	}
	negated.resize(base_range - selection.size());
	size_t
		_base = 0,
		_select = 0,
		_negate = 0;
	for (; _base < base_range && _negate < negated.size(); _base++) {
		if (_select < selection.size() && _base == selection[_select]) {
			_select++;
		} else /*if (_base < selection[_select])*/ {
			negated[_negate] = _base;
			_negate++;
		}
	}
}

/** Merge two presorted selections into a single sorted selection (non-descending)
 * prereq: both selections must be sorted in non-descending order */
template<typename IntT = pcl::index_t>
void pc_combine_sorted(
	const std::vector<IntT>& sel1,
	const std::vector<IntT>& sel2,
	std::vector<IntT>& out
) {
	out.clear();
	out.reserve(sel1.size() + sel2.size());
	size_t
		_p1 = 0,
		_p2 = 0;
	for(; _p1 < sel1.size() && _p2 < sel2.size();) {
		const IntT
			_a = sel1[_p1],
			_b = sel2[_p2];
		if(_a <= _b) {
			out.push_back(_a);
			_p1++;
			_p2 += (_a == _b);
		} else {
			out.push_back(_b);
			_p2++;
		}
	}
	for(; _p1 < sel1.size(); _p1++) out.push_back( sel1[_p1] );
	for(; _p2 < sel2.size(); _p2++) out.push_back( sel2[_p2] );
}

/**  */
template<
	size_t interlace_rep = 4,
	size_t interlace_off = 3,
	typename IntT = pcl::index_t,
	typename ElemT = int32_t>
void write_interlaced_selection_bytes(
	std::span<ElemT> buffer,
	const std::vector<IntT>& selection,
	const ElemT selected, const ElemT unselected
) {
	static_assert(interlace_off < interlace_rep, "");
	size_t
		_buff = 0,
		_sel = 0;
	for(; _buff < buffer.size() / interlace_rep; _buff++) {
		const size_t idx = interlace_rep * _buff + interlace_off;
		if(_sel < selection.size() && _buff == selection[_sel]) {
			buffer[idx] = selected;
			_sel++;
		} else {
			buffer[idx] = unselected;
		}
	}
}
