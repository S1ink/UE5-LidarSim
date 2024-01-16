#pragma once

#include <cstdint>
#include <type_traits>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>


class WeightMapBase_ {
protected:
	template<typename weight_T, typename stat_T>
	struct CellBase_ {
		using Weight_T = weight_T;
		using Stat_T = stat_T;

		inline static constexpr bool
			IsDense = std::is_same<weight_T, stat_T>::value;
	};

public:
	template<typename weight_T, typename stat_T = float>
	struct MapCell_ : CellBase_<weight_T, stat_T> {
		weight_T w;
		union {
			struct { stat_T min_z, max_z, avg_z; };
			stat_T stat[3];
		};
	};
	template<typename data_T>
	struct MapCell_Aligned_ : CellBase_<data_T, data_T> {
		union {
			struct {
				data_T w;
				union {
					struct { data_T min_z, max_z, avg_z; };
					data_T stat[3];
				};
			};
			data_T data[4];
		};
	};

	template<typename weight_T, typename stat_T = float>
	using MapCell = typename std::conditional<
		std::is_same<weight_T, stat_T>::value,
		MapCell_Aligned_<weight_T>,
		MapCell_<weight_T, stat_T>
	>::type;


public:
	/** Align a point to a box grid of the given resolution and offset origin. Result may be negative if lower than current offset. */
	static Eigen::Vector2i gridAlign(float x, float y, const Eigen::Vector2f& off, float res) {
		return Eigen::Vector2i{
			std::floorf((x - off.x()) / res),	// always floor since grid cells are indexed by their "bottom left" corner's raw position
			std::floorf((y - off.y()) / res)
		};
	}
	static Eigen::Vector2i gridAlign(const Eigen::Vector4f& pt, const Eigen::Vector2f& off, float res) {
		return gridAlign(pt.x(), pt.y(), off, res);
	}

	/** Get a raw buffer idx from a 2d index and buffer size (Row~X, Col~Y order) */
	static int64_t gridIdx(const Eigen::Vector2i& loc, const Eigen::Vector2i& size) {
		return gridIdx(loc.x(), loc.y(), size);
	}
	static int64_t gridIdx(const int x, const int y, const Eigen::Vector2i& size) {
		return (int64_t)x * size.y() + y;	// rows along x-axis, cols along y-axis, thus (x, y) --> x * #cols + y
	}
	static Eigen::Vector2i gridLoc(std::size_t idx, const Eigen::Vector2i& size) {
		return Eigen::Vector2i{
			idx / size.y(),
			idx % size.y()
		};
	}


};

template<typename weight_T = float>
class WeightMap : public WeightMapBase_ {
	//static_assert(std::is_arithmetic<weight_T>::value, "");
	//friend class UAccumulatorMap;
public:
	using Weight_T = weight_T;
	using Scalar_T = float;
	using Cell_T = MapCell<Weight_T, Scalar_T>;
	using This_T = WeightMap<Weight_T>;
	using Base_T = WeightMapBase_;

	inline static constexpr bool
		Cell_IsDense = Cell_T::IsDense;
	inline static constexpr size_t
		Cell_Size = sizeof(Cell_T),
		Max_Alloc = (1Ui64 << 30) / Cell_Size;		// 1 << 30 ~ 1bn --> limit to ~1 gigabyte

	template<int64_t val>
	inline static constexpr Weight_T Weight() { return static_cast<Weight_T>(val); }

public:
	WeightMap() {}
	~WeightMap() {
		if (map_data) delete[] map_data;
	}


	void reset(float grid_res = 1.f, const Eigen::Vector2f grid_origin = Eigen::Vector2f::Zero()) {
		if (map_data) delete[] map_data;
		map_size = Eigen::Vector2i::Zero();
		max_weight = Weight<0>();
		resolution = grid_res <= 0.f ? 1.f : grid_res;
		map_origin = grid_origin;
	}

	inline const Eigen::Vector2i& size() const {
		return this->map_size;
	}
	inline const int64_t area() const {
		return (int64_t)this->map_size.x() * this->map_size.y();
	}
	inline typename std::conditional<(sizeof(Weight_T) > 8),
		const Weight_T&, const Weight_T
	>::type max() const {
		return this->max_weight;
	}
	inline const Eigen::Vector2f& origin() const {
		return this->map_origin;
	}
	inline const float gridRes() const {
		return this->resolution;
	}

	inline const Eigen::Vector2i boundingCell(const float x, const float y) const {
		return Base_T::gridAlign(x, y, this->map_origin, this->resolution);
	}
	inline const int64_t cellIdxOf(const float x, const float y) const {
		return Base_T::gridIdx(this->boundingCell(x, y), this->map_size);
	}


	/** Returns false if an invalid realloc was skipped */
	bool resizeToBounds(const Eigen::Vector4f& min, const Eigen::Vector4f& max, bool rebase = false) {

		static const Eigen::Vector2i
			_zero = Eigen::Vector2i::Zero();
		const Eigen::Vector2i
			_min = this->boundingCell(min.x(), min.y()),	// grid cell locations containing min and max, aligned with current offsets
			_max = this->boundingCell(max.x(), max.y());

		if (_min.cwiseLess(_zero).any() || _max.cwiseGreater(this->map_size).any()) {
			const Eigen::Vector2i
				_low = _min.cwiseMin(_zero),		// new high and low bounds for the map
				_high = _max.cwiseMax(this->map_size),
				_size = _high - _low;				// new map size

			const int64_t _area = (int64_t)_size.x() * _size.y();
			if (_area > This_T::Max_Alloc || _area < 0) return false;		// less than a gigabyte of allocated buffer is ideal

			Cell_T* _map = new Cell_T[_area];
			memset(_map, 0x00, _area * This_T::Cell_Size);	// :O don't forget this otherwise the map will start with all garbage data

			const Eigen::Vector2i _diff = _zero - _low;	// by how many grid cells did the origin shift
			if (this->map_data) {
				for (int r = 0; r < this->map_size.x(); r++) {		// for each row in existing...
					memcpy(									// remap to new buffer
						_map + ((r + _diff.x()) * _size.y() + _diff.y()),	// (row + offset rows) * new row size + offset cols
						this->map_data + (r * this->map_size.y()),
						this->map_size.y() * This_T::Cell_Size
					);
				}
				delete[] this->map_data;
			}
			this->map_data = _map;
			this->map_size = _size;
			this->map_origin -= (_diff.cast<float>() * this->resolution);
		}
		return true;

	}

	template<typename PointT>
	void insertPoints(const pcl::PointCloud<PointT>& cloud, const pcl::Indices& selection, bool rebase = false) {

		const bool _use_selection = !selection.empty();
		Eigen::Vector4f _min, _max;
		if (_use_selection) {
			pcl::getMinMax3D<PointT>(cloud, selection, _min, _max);
		} else {
			pcl::getMinMax3D<PointT>(cloud, _min, _max);
		}
		if (this->resizeToBounds(_min, _max, rebase)) {
			if (_use_selection) {
				for (const pcl::index_t idx : selection) {
					this->insert<PointT>(cloud.points[idx]);
				}
			} else {
				for (const PointT& pt : cloud.points) {
					this->insert<PointT>(pt);
				}
			}
		}

	}

protected:
	/** returns false if accumulation failed (invalid index) */
	template<typename PointT>
	bool insert(const PointT& pt) {
		const int64_t i = this->cellIdxOf(pt.x, pt.y);
#ifndef SKIP_MAP_BOUND_CHECKS
		if (i >= 0 && i < this->area())
#endif
		{
			Cell_T& _cell = this->map_data[i];
			_cell.w += Weight<1>();
			if (_cell.w > this->max_weight) this->max_weight = _cell.w;

			if (pt.z < _cell.min_z) _cell.min_z = pt.z;
			else if (pt.z > _cell.max_z) _cell.max_z = pt.z;
			_cell.avg_z = ((_cell.w - Weight<1>()) * _cell.avg_z + pt.z) / _cell.w;

			return true;
		}
		return false;
	}

protected:
	Scalar_T resolution{ 1.f };
	Eigen::Vector2f map_origin{};
	Eigen::Vector2i map_size{};
	Cell_T* map_data{ nullptr };

	Weight_T max_weight{ Weight<0>() };

};