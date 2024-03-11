#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

#include <Eigen/Core>
#include <pcl/point_cloud.h>

#include "filtering.hpp"


/** Static base for all grids */
class StaticGridBase {
public:

	/** Align a point to a box grid of the given resolution and offset origin. Result may be negative if lower than current offset. */
	template<typename IntT = int, typename FloatT = float>
	inline static Eigen::Vector2<IntT> gridAlign(FloatT x, FloatT y, const Eigen::Vector2<FloatT>& off, FloatT res) {
		return Eigen::Vector2<IntT>{
			static_cast<IntT>( std::floor((x - off.x()) / res) ),	// always floor since grid cells are indexed by their "bottom left" corner's raw position
			static_cast<IntT>( std::floor((y - off.y()) / res) )
		};
	}
	template<typename IntT = int, typename FloatT = float>
	inline static Eigen::Vector2<IntT> gridAlign(const Eigen::Vector4<FloatT>& pt, const Eigen::Vector2<FloatT>& off, FloatT res) {
		return gridAlign<IntT, FloatT>(pt.x(), pt.y(), off, res);
	}

	/** Get a raw buffer idx from a 2d index and buffer size (Row~X, Col~Y order) */
	template<typename IntT = int>
	inline static int64_t gridIdx(const IntT x, const IntT y, const Eigen::Vector2<IntT>& size) {
		return static_cast<int64_t>(x) * size.y() + y;	// rows along x-axis, cols along y-axis, thus (x, y) --> x * #cols + y
	}
	template<typename IntT = int>
	inline static int64_t gridIdx(const Eigen::Vector2<IntT>& loc, const Eigen::Vector2<IntT>& size) {
		return gridIdx<IntT>(loc.x(), loc.y(), size);
	}
	template<typename IntT = int>
	inline static Eigen::Vector2<IntT> gridLoc(size_t idx, const Eigen::Vector2<IntT>& size) {
		return Eigen::Vector2<IntT>{
			static_cast<IntT>(idx / size.y()),
			static_cast<IntT>(idx % size.y())
		};
	}

};

/** GridBase contains all generic functinality for a dynamic grid cells (template type) */
template<
	typename Cell_t,
	typename Int_t = int,
	typename Float_t = float,
	size_t Max_Alloc_Bytes = (1ULL << 30)>
class GridBase : public StaticGridBase {
	static_assert(std::is_integral_v<Int_t>, "");
	static_assert(std::is_floating_point_v<Float_t>, "");
	static_assert(Max_Alloc_Bytes >= sizeof(Cell_t), "");
public:
	using Cell_T = Cell_t;
	using IntT = Int_t;
	using FloatT = Float_t;
	using This_T = GridBase< Cell_T, IntT, FloatT, Max_Alloc_Bytes >;

	static constexpr size_t
		Cell_Size = sizeof(Cell_T),
		Max_Alloc_NCells = Max_Alloc_Bytes / Cell_Size;

public:
	GridBase() = default;
	~GridBase() {
		if (this->grid) delete[] this->grid;
	}


	void reset(FloatT resolution = (FloatT)1, const Eigen::Vector2<FloatT> origin = Eigen::Vector2<FloatT>::Zero()) {
		if (this->grid) {
			delete[] this->grid;
			this->grid = nullptr;
		}
		this->grid_size = Eigen::Vector2<IntT>::Zero();
		this->cell_res = resolution <= (FloatT)0 ? (FloatT)1 : resolution;
		this->grid_origin = origin;
	}

	inline const Eigen::Vector2<IntT>& size() const {
		return this->grid_size;
	}
	inline const int64_t area() const {
		return static_cast<int64_t>(this->grid_size.x()) * this->grid_size.y();
	}
	inline const Eigen::Vector2<FloatT>& origin() const {
		return this->grid_origin;
	}
	inline const FloatT cellRes() const {
		return this->cell_res;
	}
	inline const Cell_T* gridData() const {
		return this->grid;
	}

	inline const Eigen::Vector2<IntT> boundingLoc(const FloatT x, const FloatT y) const {
		return StaticGridBase::gridAlign<IntT, FloatT>(x, y, this->grid_origin, this->cell_res);
	}
	inline const int64_t cellIdxOf(const FloatT x, const FloatT y) const {
		return StaticGridBase::gridIdx<IntT>(this->boundingLoc(x, y), this->grid_size);
	}


	/** Returns false if an invalid realloc was skipped */
	bool resizeToBounds(const Eigen::Vector2<FloatT>& min, const Eigen::Vector2<FloatT>& max) {

		static const Eigen::Vector2<IntT>
			_zero = Eigen::Vector2<IntT>::Zero(),
			_one = Eigen::Vector2<IntT>::Ones();
		const Eigen::Vector2<IntT>
			_min = this->boundingLoc(min.x(), min.y()),	// grid cell locations containing min and max, aligned with current offsets
			_max = this->boundingLoc(max.x(), max.y());

		if (_min.cwiseLess(_zero).any() || _max.cwiseGreater(this->grid_size).any()) {
			const Eigen::Vector2<IntT>
				_low = _min.cwiseMin(_zero),		// new high and low bounds for the map
				_high = _max.cwiseMax(this->grid_size) + _one,	// need to add an additional row + col since indexing happens in the corner, thus by default the difference between corners will not cover the entire size
				_size = _high - _low;				// new map size

			const int64_t _area = static_cast<int64_t>(_size.x()) * _size.y();
			if (_area > This_T::Max_Alloc_NCells || _area < 0LL) return false;		// less than a gigabyte of allocated buffer is ideal

			Cell_T* _grid = new Cell_T[_area];
			memset(_grid, 0x00, _area * This_T::Cell_Size);	// :O don't forget this otherwise the map will start with all garbage data

			const Eigen::Vector2<IntT> _diff = _zero - _low;	// by how many grid cells did the origin shift
			if (this->grid) {
				for (IntT r = 0; r < this->grid_size.x(); r++) {		// for each row in existing...
					memcpy(									// remap to new buffer
						_grid + ((static_cast<int64_t>(r) + _diff.x()) * _size.y() + _diff.y()),	// (row + offset rows) * new row size + offset cols
						this->grid + (static_cast<int64_t>(r) * this->grid_size.y()),
						this->grid_size.y() * This_T::Cell_Size
					);
				}
				delete[] this->grid;
			}
			this->grid_origin -= (_diff.template cast<FloatT>() * this->cell_res);
			this->grid_size = _size;
			this->grid = _grid;
		}
		return true;

	}


protected:
	Eigen::Vector2<FloatT> grid_origin{};
	Eigen::Vector2<IntT> grid_size{};
	FloatT cell_res{ static_cast<FloatT>(1) };
	Cell_T* grid{ nullptr };


};





template<typename Data_t = float>
struct RatioGridCell {
public:
	using DataT = Data_t;
	union {
		struct {
			DataT
				accum,	// accumulated [numerator] value
				base;	// base [denomenator] value
		};
		DataT data[2];
	};

};

template<
	typename Ratio_t = float,
	typename Int_t = int,
	typename Float_t = float,
	size_t Max_Alloc_Bytes = (1ULL << 30)>
class RatioGrid : public GridBase< RatioGridCell<Ratio_t>, Int_t, Float_t, Max_Alloc_Bytes > {
public:
	using This_T = RatioGrid< Ratio_t, Int_t, Float_t >;
	using Base_T = GridBase< RatioGridCell<Ratio_t>, Int_t, Float_t, Max_Alloc_Bytes >;
	using RatioT = Ratio_t;
	using typename Base_T::IntT;
	using typename Base_T::FloatT;
	using typename Base_T::Cell_T;
	using PreciseFloatT = typename std::conditional<
		std::is_floating_point<RatioT>::value,	// check if the ratio internal type is fp
		typename std::conditional<
			(sizeof(RatioT) > sizeof(FloatT)),	// get the fp type with highest precision
			RatioT,
			FloatT
		>::type,
		FloatT		// use default fp type
	>::type;

	template<long long int V>
	inline static constexpr RatioT RatioVal() { return static_cast<RatioT>(V); }

public:
	RatioGrid() = default;
	~RatioGrid() = default;

	/** The accumulation selection must be a subset of the base selection! If not, simply call the individual methods. */
	// template<typename PointT>
	// void incrementRatio(
	// 	const pcl::PointCloud<PointT>& cloud,
	// 	const pcl::Indices& accum_selection,
	// 	const pcl::Indices& base_selection
	// ) {

	// 	if(accum_selection.size() > base_selection.size() && !base_selection.empty()) return;	// definitely not a subset - don't add since we don't want to risk cells having a base of 0 (div by 0 on export)

	// 	Eigen::Vector2<FloatT> _min, _max;
	// 	minMaxXY<PointT>(cloud, base_selection, _min, _max);

	// 	if(this->resizeToBounds(_min, _max)) {
	// 		if(accum_selection.size() == base_selection.size()) {
	// 			this->incrementAllInternal(cloud, base_selection);
	// 		} else {

	// 		}
	// 	}

	// 	// accum:all, base:all --> shortcut
	// 	// accum:some, base:all --> iternative
	// 	// accum:some, base:some --> check for shortcut : iterative
	// 	// accum:all, base:some --> NOT VALID

	// }

public:
	template<typename ExportT = RatioT>
	inline ExportT exportRatio(size_t idx, ExportT normalization = (ExportT)1) {
		const Cell_T& _cell = this->grid[idx];
		const PreciseFloatT _ratio = static_cast<PreciseFloatT>(_cell.accum) / _cell.base;
		return std::isnan(_ratio) ?
			static_cast<ExportT>(0) :
			static_cast<ExportT>(normalization * _ratio)
		;
	}

	inline RatioT operator[](size_t idx) {
		return this->exportRatio(idx);
	}

	template<typename PointT>
	inline void incrementAccum(
		const pcl::PointCloud<PointT>& cloud,
		const pcl::Indices& selection
	) {
		this->increment<0>(cloud, selection);
	}
	template<typename PointT>
	inline void incrementBase(
		const pcl::PointCloud<PointT>& cloud,
		const pcl::Indices& selection
	) {
		this->increment<1>(cloud, selection);
	}


protected:
	/** Does not implement any safety checks! */
	template<typename PointT>
	inline Cell_T* boundingCell(const PointT& pt) {
		const int64_t i = this->cellIdxOf(static_cast<FloatT>(pt.x), static_cast<FloatT>(pt.y));
#ifndef GRID_IMPL_SKIP_BOUND_CHECKING
		if (i >= 0 && i < this->area())
#endif
		{
			return this->grid + i;
		}
		return nullptr;
	}

	template<size_t data_off, typename PointT>
	void increment(
		const pcl::PointCloud<PointT>& cloud,
		const pcl::Indices& selection
	) {
		static_assert(data_off < (sizeof(Cell_T) / sizeof(RatioT)), "");

		Eigen::Vector2<FloatT> _min, _max;
		minMaxXY<PointT>(cloud, selection, _min, _max);

		if(this->resizeToBounds(_min, _max)) {
			this->incrementInternal<data_off, PointT>(cloud, selection);
		}

	}

	/** Do not use before checking bounds and resizing the grid!!! */
	template<size_t data_off, typename PointT>
	void incrementInternal(
		const pcl::PointCloud<PointT>& cloud,
		const pcl::Indices& selection
	) {
		Cell_T* _cell;
		if(selection.empty()) {
			for(const PointT& pt : cloud.points) {
				if(_cell = this->boundingCell(pt)) {
					_cell->data[data_off] += RatioVal<1>();
				}
			}
		} else {
			for(const pcl::index_t idx : selection) {
				if(_cell = this->boundingCell(cloud.points[idx])) {
					_cell->data[data_off] += RatioVal<1>();
				}
			}
		}
	}

	template<typename PointT>
	void incrementAllInternal(
		const pcl::PointCloud<PointT>& cloud,
		const pcl::Indices& selection
	) {
		Cell_T* _cell;
		if(selection.empty()) {
			for(const PointT& pt : cloud.points) {
				if((_cell = this->boundingCell(pt))) {
					_cell->accum += RatioVal<1>();
					_cell->base += RatioVal<1>();
				}
			}
		} else {
			for(const pcl::index_t idx : selection) {
				if((_cell = this->boundingCell(cloud.points[idx]))) {
					_cell->accum += RatioVal<1>();
					_cell->base += RatioVal<1>();
				}
			}
		}
	}


};





/** constexpr conditional value (v1 = true val, v2 = false val) */
template<bool _test, typename T, T tV, T fV>
struct conditional_literal {
	static constexpr T value = tV;
};
template<typename T, T tV, T fV>
struct conditional_literal<false, T, tV, fV> {
	static constexpr T value = fV;
};

/** RatioGrid that also stores the result buffer, and updates it inline with ratio updates */
template<
	typename Buff_t = uint8_t,
	typename Ratio_t = float,
	typename Int_t = int,
	typename Float_t = float,
	size_t Max_Alloc_Bytes = (1ULL << 30)>
class QuantizedRatioGrid : public RatioGrid< Ratio_t, Int_t, Float_t, Max_Alloc_Bytes > {
public:
	using This_T = QuantizedRatioGrid< Buff_t, Ratio_t, Int_t, Float_t, Max_Alloc_Bytes >;
	using Super_T = RatioGrid< Ratio_t, Int_t, Float_t, Max_Alloc_Bytes >;
	using Buff_T = Buff_t;
	using typename Super_T::Base_T;
	using typename Super_T::RatioT;
	using typename Super_T::IntT;
	using typename Super_T::FloatT;
	using typename Super_T::Cell_T;
	using typename Super_T::PreciseFloatT;

	static constexpr size_t
		Buff_Cell_Size = sizeof(Buff_T),
		Max_Alloc_NBlocks = Max_Alloc_Bytes / Buff_Cell_Size;
	static constexpr Buff_T
		Buff_Norm_Val = conditional_literal<
			std::is_integral<Buff_T>::value, Buff_T,
			std::numeric_limits<Buff_T>::max(),		// Use the full range for integral types
			static_cast<Buff_T>(1)					// Buff_T is floating point or a special type. Normalize to 1 (or equivalent)
		>::value;

public:
	QuantizedRatioGrid() = default;
	~QuantizedRatioGrid() {
		if(this->buffer) delete[] this->buffer;
	}


	/** Calls super reset() and resets buffer */
	void reset(FloatT resolution = (FloatT)1, const Eigen::Vector2<FloatT> origin = Eigen::Vector2<FloatT>::Zero()) {
		if(this->buffer) {
			delete[] this->buffer;
			this->buffer = nullptr;
		}
		this->Base_T::reset(resolution, origin);
	}

	/** The quantized ratio buffer */
	inline const Buff_T* buffData() {
		return this->buffer;
	}


	/** Resize all buffers and copy old data into new viewport */
	bool resizeToBounds(const Eigen::Vector2<FloatT>& min, const Eigen::Vector2<FloatT>& max) {

		static const Eigen::Vector2<IntT>
			_zero = Eigen::Vector2<IntT>::Zero(),
			_one = Eigen::Vector2<IntT>::Ones();
		const Eigen::Vector2<IntT>
			_min = this->boundingLoc(min.x(), min.y()),	// grid cell locations containing min and max, aligned with current offsets
			_max = this->boundingLoc(max.x(), max.y());

		if (_min.cwiseLess(_zero).any() || _max.cwiseGreater(this->grid_size).any()) {
			const Eigen::Vector2<IntT>
				_low = _min.cwiseMin(_zero),		// new high and low bounds for the map
				_high = _max.cwiseMax(this->grid_size) + _one,	// need to add an additional row + col since indexing happens in the corner, thus by default the difference between corners will not cover the entire size
				_size = _high - _low;				// new map size

			const int64_t _area = static_cast<int64_t>(_size.x()) * _size.y();
			if (_area < 0LL || _area > Base_T::Max_Alloc_NCells || _area > This_T::Max_Alloc_NBlocks) return false;		// less than a gigabyte of allocated buffer is ideal

			Cell_T* _grid = new Cell_T[_area];
			Buff_T* _buffer = new Buff_T[_area];
			memset(_grid, 0x00, _area * Base_T::Cell_Size);	// :O don't forget this otherwise the map will start with all garbage data
			memset(_buffer, 0x00, _area * This_T::Buff_Cell_Size);

			const Eigen::Vector2<IntT> _diff = _zero - _low;	// by how many grid cells did the origin shift
			if(this->grid || this->buffer) {	// should be synchronized :?
				for (IntT r = 0; r < this->grid_size.x(); r++) {		// for each row in existing...
					memcpy(									// remap to new buffer
						_grid + ((static_cast<int64_t>(r) + _diff.x()) * _size.y() + _diff.y()),	// (row + offset rows) * new row size + offset cols
						this->grid + (static_cast<int64_t>(r) * this->grid_size.y()),
						this->grid_size.y() * Base_T::Cell_Size
					);
					memcpy(
						_buffer + ((static_cast<int64_t>(r) + _diff.x()) * _size.y() + _diff.y()),
						this->buffer + (static_cast<int64_t>(r) * this->grid_size.y()),
						this->grid_size.y() * This_T::Buff_Cell_Size
					);
				}
				delete[] this->grid;
				delete[] this->buffer;
			}
			this->grid_origin -= (_diff.template cast<FloatT>() * this->cell_res);
			this->grid_size = _size;
			this->grid = _grid;
			this->buffer = _buffer;
		}
		return true;

	}


	template<typename PointT>
	void incrementRatio(
		const pcl::PointCloud<PointT>& cloud,
		const pcl::Indices& base_selection,
		const pcl::Indices& accum_selection
	) {

		Eigen::Vector2<FloatT> min, max;
		minMaxXY<PointT>(cloud, base_selection, min, max);

		if(!this->This_T::resizeToBounds(min, max)) return;

		if(base_selection.empty()) {

			if(accum_selection.empty()) {	// CASE 1: increment both for all points
				for(const PointT& pt : cloud) {
					const int64_t i = this->cellIdxOf(static_cast<FloatT>(pt.x), static_cast<FloatT>(pt.y));
#ifndef GRID_IMPL_SKIP_BOUND_CHECKING
					if (i >= 0 && i < this->area())
#endif
					{
						Cell_T* _cell = this->grid + i;
						_cell->accum += Super_T::template RatioVal<1>();
						_cell->base += Super_T::template RatioVal<1>();
						this->buffer[i] = static_cast<Buff_T>( This_T::Buff_Norm_Val * (static_cast<PreciseFloatT>(_cell->accum) / _cell->base) );
					}
				}
			} else {						// CASE 2: increment all base + selected accumulator indices
				size_t
					_pt = 0,
					_sel = 0;
				for(; _pt < cloud.points.size(); _pt++) {
					const PointT& pt = cloud.points[_pt];
					const int64_t i = this->cellIdxOf(static_cast<FloatT>(pt.x), static_cast<FloatT>(pt.y));
#ifndef GRID_IMPL_SKIP_BOUND_CHECKING
					if (i < 0 || i >= this->area()) {	// opposite the usual so that we only need one macro test!
						if(_sel < accum_selection.size() && accum_selection[_sel] == _pt) _sel++;	// need to test for increment even if point is invalid
					} else	// else since we tested the opposite!
#endif
					{
						Cell_T* _cell = this->grid + i;
						if(_sel < accum_selection.size() && accum_selection[_sel] == _pt) {
							_cell->accum += Super_T::template RatioVal<1>();
							_sel++;
						}
						_cell->base += Super_T::template RatioVal<1>();
						this->buffer[i] = static_cast<Buff_T>( This_T::Buff_Norm_Val * (static_cast<PreciseFloatT>(_cell->accum) / _cell->base) );
					}
				}
			}

		} else {

			if(accum_selection.empty()) {	// CASE 3: increment both for entire base selection
				for(const pcl::index_t _pt : base_selection) {
					const PointT& pt = cloud.points[_pt];
					const int64_t i = this->cellIdxOf(static_cast<FloatT>(pt.x), static_cast<FloatT>(pt.y));
#ifndef GRID_IMPL_SKIP_BOUND_CHECKING
					if (i >= 0 && i < this->area())
#endif
					{
						Cell_T* _cell = this->grid + i;
						_cell->accum += Super_T::template RatioVal<1>();
						_cell->base += Super_T::template RatioVal<1>();
						this->buffer[i] = static_cast<Buff_T>( This_T::Buff_Norm_Val * (static_cast<PreciseFloatT>(_cell->accum) / _cell->base) );
					}
				}
			} else {						// CASE 4: increment through base and accum selections (in case subset is invalid)
				size_t _accum = 0;
				for(const pcl::index_t _pt : base_selection) {
					const PointT& pt = cloud.points[_pt];
					const int64_t i = this->cellIdxOf(static_cast<FloatT>(pt.x), static_cast<FloatT>(pt.y));
#ifndef GRID_IMPL_SKIP_BOUND_CHECKING
					if (i < 0 || i >= this->area()) {	// opposite the usual so that we only need one macro test!
						if(_accum < accum_selection.size() && accum_selection[_accum] == _pt) _accum++;	// need to test for increment even if point is invalid
					} else	// else since we tested the opposite!
#endif
					{
						Cell_T* _cell = this->grid + i;
						if(_accum < accum_selection.size() && accum_selection[_accum] == _pt) {
							_cell->accum += Super_T::template RatioVal<1>();
							_accum++;
						}
						_cell->base += Super_T::template RatioVal<1>();
						this->buffer[i] = static_cast<Buff_T>( This_T::Buff_Norm_Val * (static_cast<PreciseFloatT>(_cell->accum) / _cell->base) );
					}
				}
			}

		}

	}


protected:
	Buff_T* buffer{ nullptr };


};









/** OLD */

/** Statistical grid cell types */
struct StatCellTypes {
protected:
	template<typename Acc_t, typename Stat_t>
	struct CellBase_ {
		using Acc_T = Acc_t;
		using Stat_T = Stat_t;

		inline static constexpr bool
			IsDense = std::is_same<Acc_T, Stat_T>::value;
	};

public:
	template<typename Acc_t, typename Stat_t = float>
	struct GridCell_ : CellBase_<Acc_t, Stat_t> {
		using typename CellBase_<Acc_t, Stat_t>::Acc_T;
		using typename CellBase_<Acc_t, Stat_t>::Stat_T;
		Acc_T val;
		union {
			struct { Stat_T min_z, max_z, avg_z; };
			Stat_T stat[3];
		};
	};
	template<typename Data_t>
	struct GridCell_Aligned_ : CellBase_<Data_t, Data_t> {
		using Data_T = Data_t;
		union {
			struct {
				Data_T val;
				union {
					struct { Data_T min_z, max_z, avg_z; };
					Data_T stat[3];
				};
			};
			Data_T data[4];
		};
	};

	/** If datatypes are the same, use a "dense" allocation, otherwise use standard layout */
	template<typename Acc_t, typename Stat_t>
	using GridCell = typename std::conditional<
		CellBase_<Acc_t, Stat_t>::IsDense,
			GridCell_Aligned_<Acc_t>,
			GridCell_<Acc_t, Stat_t>
		>::type;

};

/** Renamed version of the old grid strategy */
template<
	typename Acc_t = float,
	typename Int_t = int,
	typename Float_t = float,
	size_t Max_Alloc_Bytes = (1ULL << 30)>
class StatisticGrid : public GridBase< StatCellTypes::GridCell<Acc_t, Float_t>, Int_t, Float_t, Max_Alloc_Bytes > {

public:
	using This_T = StatisticGrid< Acc_t, Int_t, Float_t >;
	using Base_T = GridBase< StatCellTypes::GridCell<Acc_t, Float_t>, Int_t, Float_t, Max_Alloc_Bytes >;
	using Accumulation_T = Acc_t;
	using typename Base_T::IntT;
	using typename Base_T::FloatT;
	using typename Base_T::Cell_T;

	inline static constexpr bool
		Cell_IsDense = Cell_T::IsDense;

	template<int64_t val>
	inline static constexpr Accumulation_T AccVal() { return static_cast<Accumulation_T>(val); }

public:
	StatisticGrid() = default;
	~StatisticGrid() = default;


	template<typename PointT>
	void insertPoints(const pcl::PointCloud<PointT>& cloud, const pcl::Indices& selection) {

		Eigen::Vector2<FloatT> _min, _max;
		minMaxXY<PointT>(cloud, selection, _min, _max);

		if (this->resizeToBounds(_min, _max)) {
			if (!selection.empty()) {
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
		const int64_t i = this->cellIdxOf(static_cast<FloatT>(pt.x), static_cast<FloatT>(pt.y));
#ifndef GRID_IMPL_SKIP_BOUND_CHECKING
		if (i >= 0 && i < this->area())
#endif
		{
			Cell_T& _cell = this->grid[i];
			_cell.val += AccVal<1>();
			if (_cell.val > this->max_weight) this->max_weight = _cell.val;

			const FloatT _z = static_cast<FloatT>(pt.z);
			if (_z < _cell.min_z) _cell.min_z = _z;
			else if (_z > _cell.max_z) _cell.max_z = _z;
			_cell.avg_z = (static_cast<FloatT>(_cell.val - AccVal<1>()) * _cell.avg_z + _z) / static_cast<FloatT>(_cell.val);

			return true;
		}
		return false;
	}

protected:
	Accumulation_T max_weight{ AccVal<0>() };


};
