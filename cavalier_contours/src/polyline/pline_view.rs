use crate::{
    core::{math::Vector2, traits::Real},
    polyline::seg_split_at_point,
};

use super::{seg_closest_point, PlineSource, PlineVertex, Polyline};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A [PlineView] represents a partial selection or subpart of a source polyline without copying.
/// This structure borrows a source polyline to access vertex data for iteration and operations.
///
/// See [PlineViewData] for how to create different types of views/selections.
#[derive(Debug, Clone, Copy)]
pub struct PlineView<'a, P>
where
    P: PlineSource + ?Sized,
{
    /// Reference to the source polyline for this view.
    pub source: &'a P,
    /// View data used for indexing into the `source` polyline.
    pub data: PlineViewData<P::Num>,
}

impl<'a, P> PlineView<'a, P>
where
    P: PlineSource + ?Sized,
{
    /// Create a new view with the given source and data.
    #[inline]
    pub fn new(source: &'a P, data: PlineViewData<P::Num>) -> Self {
        Self { source, data }
    }

    /// Consume the view (releasing the borrow on the source polyline) and returning the associated
    /// view data.
    #[inline]
    pub fn detach(self) -> PlineViewData<P::Num> {
        self.data
    }
}

impl<'a, P> PlineSource for PlineView<'a, P>
where
    P: PlineSource + ?Sized,
{
    type Num = P::Num;

    type OutputPolyline = Polyline<P::Num>;

    #[inline]
    fn vertex_count(&self) -> usize {
        self.data.vertex_count()
    }

    #[inline]
    fn is_closed(&self) -> bool {
        false
    }

    #[inline]
    fn get(&self, index: usize) -> Option<PlineVertex<Self::Num>> {
        self.data.get_vertex(self.source, index)
    }

    #[inline]
    fn at(&self, index: usize) -> PlineVertex<Self::Num> {
        self.data.get_vertex(self.source, index).unwrap()
    }
}

#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(rename_all = "camelCase")
)]
/// Structure to hold the minimum data required to create view as a partial selection over a source
/// polyline. This structure is detached from the source polyline unlike [PlineView].
///
/// A [PlineViewData] has all the information required to construct a complete polyline that
/// represents the contiguous subpart of a source polyline (which optionally may be inverted).
///
/// [PlineViewData::view] is called to form an active view (using a reference to the source polyline
/// to then iterate over or perform operations on).
///
/// # Examples
///
/// ## Creating view over polyline from slice points
///
/// ```
/// # use cavalier_contours::core::math::Vector2;
/// # use cavalier_contours::polyline::{PlineCreation, PlineSource, PlineSourceMut, PlineVertex, PlineViewData, Polyline};
/// # use cavalier_contours::assert_fuzzy_eq;
///
/// let mut polyline = Polyline::new_closed();
/// polyline.add(0.0, 0.0, 0.0);
/// polyline.add(5.0, 0.0, 0.0);
/// polyline.add(5.0, 5.0, 0.0);
/// polyline.add(0.0, 5.0, 0.0);
/// // construction view data from slice points, view data represents a slice of the source polyline
/// // starting at (2.5, 0.0) on the first segment (index 0) and ending at (2.5, 5.0) on the third
/// // segment (index 2)
/// let view_data =
///     PlineViewData::from_slice_points(
///         // source polyline
///         &polyline,
///         // start point
///         Vector2::new(2.5, 0.0),
///         // segment index start point lies on
///         0,
///         // end point
///         Vector2::new(2.5, 5.0),
///         // segment index end point lies on
///         2,
///         // position equal epsilon
///         1e-5).expect("slice not collapsed");
///
/// // construct the view (which implements polyline traits) from the view data and source
/// let view = view_data.view(&polyline);
///
/// // we can now use common trait methods on the slice
/// // note we never had to copy the source polyline
/// let slice_length = view.path_length();
/// assert_fuzzy_eq!(view.path_length(), 10.0);
/// let slice_vertex_count = view.vertex_count();
/// assert_eq!(slice_vertex_count, 4);
/// let slice_extents = view.extents().unwrap();
/// assert_fuzzy_eq!(slice_extents.min_x, 2.5);
/// assert_fuzzy_eq!(slice_extents.min_y, 0.0);
/// assert_fuzzy_eq!(slice_extents.max_x, 5.0);
/// assert_fuzzy_eq!(slice_extents.max_y, 5.0);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct PlineViewData<T = f64> {
    /// Source polyline start segment index.
    pub start_index: usize,
    /// Wrapping offset from `start_index` to reach the last segment index in the source polyline.
    pub end_index_offset: usize,
    /// First vertex of the view (positioned somewhere along the `start_index` segment with bulge
    /// and position updated).
    pub updated_start: PlineVertex<T>,
    /// Updated bulge value to be used in the end_index segment.
    pub updated_end_bulge: T,
    /// Final end point of the view.
    pub end_point: Vector2<T>,
    /// Whether the view direction is inverted or not, note this just affects the way vertexes are
    /// constructed from the source polyline, all properties stay oriented/defined the same.
    pub inverted_direction: bool,
}

impl<T> PlineViewData<T>
where
    T: Real,
{
    /// Create a [PlineView] by giving a reference to be borrowed as the source polyline.
    #[inline]
    pub fn view<'a, P>(&self, source: &'a P) -> PlineView<'a, P>
    where
        P: PlineSource<Num = T> + ?Sized,
    {
        debug_assert_eq!(
            self.validate_for_source(source),
            ViewDataValidation::IsValid
        );

        PlineView {
            source,
            data: *self,
        }
    }

    #[inline]
    fn vertex_count(&self) -> usize {
        self.end_index_offset + 2
    }

    /// Get vertex at given `index` position based on this view data and a `source`. Note this
    /// method is private since [PlineViewData::view] should be called to get a [PlineView] to
    /// access the underlying data through the view.
    fn get_vertex<P>(&self, source: &P, index: usize) -> Option<PlineVertex<T>>
    where
        P: PlineSource<Num = T> + ?Sized,
        T: Real,
    {
        if index >= self.vertex_count() {
            return None;
        }

        if self.inverted_direction {
            // inverted direction example
            // |0123456789| <-- source
            // |----    ^-| <-- view selected range (start_index = 8, offset = 5)
            // index = 0 --> end_point on seg starting at 3, -updated_end_bulge
            // index = 1 --> vert 3 with negative bulge from vert 2
            // index = 2 --> vert 2 with negative bulge from vert 1
            // index = 3 --> vert 1 with negative bulge from vert 0
            // index = 4 --> vert 0 with negative bulge from vert 9
            // index = 5 (offset) --> vert 9 with negative updated start bulge
            // index = 6 (offset + 1) --> updated start with 0 bulge

            if index == 0 {
                let v = PlineVertex::from_vector2(self.end_point, -self.updated_end_bulge);
                return Some(v);
            }

            if index < self.end_index_offset {
                let bulge_i =
                    source.fwd_wrapping_index(self.start_index, self.end_index_offset - index);
                let i = source.next_wrapping_index(bulge_i);
                return Some(source.at(i).with_bulge(-source.at(bulge_i).bulge));
            }

            if index == self.end_index_offset {
                let i =
                    source.fwd_wrapping_index(self.start_index, self.end_index_offset - index + 1);

                let v = source.at(i);
                return Some(v.with_bulge(-self.updated_start.bulge));
            }

            if index == self.end_index_offset + 1 {
                return Some(self.updated_start.with_bulge(T::zero()));
            }
        } else {
            if index == 0 {
                return Some(self.updated_start);
            }

            if index < self.end_index_offset {
                let i = source.fwd_wrapping_index(self.start_index, index);
                return Some(source.at(i));
            }

            if index == self.end_index_offset {
                let i = source.fwd_wrapping_index(self.start_index, self.end_index_offset);
                let v = source.at(i);
                return Some(v.with_bulge(self.updated_end_bulge));
            }

            if index == self.end_index_offset + 1 {
                return Some(PlineVertex::from_vector2(self.end_point, T::zero()));
            }
        }

        None
    }

    /// Create view data from source polyline that selects over a single segment.
    ///
    /// Returns `None` if `updated_start` is on top of `end_intersect` (collapsed selection).
    pub fn create_on_single_segment<P>(
        source: &P,
        start_index: usize,
        updated_start: PlineVertex<T>,
        end_intersect: Vector2<T>,
        pos_equal_eps: T,
    ) -> Option<Self>
    where
        P: PlineSource<Num = T> + ?Sized,
    {
        if updated_start
            .pos()
            .fuzzy_eq_eps(end_intersect, pos_equal_eps)
        {
            return None;
        }
        let view_data = Self {
            start_index,
            end_index_offset: 0,
            updated_start,
            updated_end_bulge: updated_start.bulge,
            end_point: end_intersect,
            inverted_direction: false,
        };

        debug_assert_eq!(
            view_data.validate_for_source(source),
            ViewDataValidation::IsValid
        );

        Some(view_data)
    }

    /// Create view data from source polyline and parameters.
    ///
    /// # Panics
    ///
    /// This function panics if `traverse_count == 0` or indexes out of range for `source`. Use
    /// [PlineViewData::create_on_single_segment] if view selects over only a single segment.
    pub fn create<P>(
        source: &P,
        start_index: usize,
        end_intersect: Vector2<T>,
        intersect_index: usize,
        updated_start: PlineVertex<T>,
        traverse_count: usize,
        pos_equal_eps: T,
    ) -> Self
    where
        P: PlineSource<Num = T> + ?Sized,
    {
        assert!(traverse_count != 0,
            "traverse_count must be greater than 1, use different constructor if view is all on one segment"
        );

        let current_vertex = source.at(intersect_index);
        let (end_index_offset, updated_end_bulge) =
            if end_intersect.fuzzy_eq_eps(current_vertex.pos(), pos_equal_eps) {
                // intersect lies on top of vertex at start of segment
                let offset = traverse_count - 1;
                let updated_end_bulge = if offset != 0 {
                    source.at(source.prev_wrapping_index(intersect_index)).bulge
                } else {
                    updated_start.bulge
                };
                (offset, updated_end_bulge)
            } else {
                // trim bulge to intersect position
                let next_index = source.next_wrapping_index(intersect_index);
                let split = seg_split_at_point(
                    current_vertex,
                    source.at(next_index),
                    end_intersect,
                    pos_equal_eps,
                );
                (traverse_count, split.updated_start.bulge)
            };

        let view_data = Self {
            start_index,
            end_index_offset,
            updated_start,
            updated_end_bulge,
            end_point: end_intersect,
            inverted_direction: false,
        };

        debug_assert_eq!(
            view_data.validate_for_source(source),
            ViewDataValidation::IsValid
        );

        view_data
    }

    /// Construct view representing an entire polyline. The view is always considered an open
    /// polyline even if the source given is closed (but the view will geometrically follow the same
    /// closed path).
    ///
    /// # Panics
    ///
    /// This function panics if `source` has less than 2 vertexes or indexes out of range for
    /// `source`.
    pub fn from_entire_pline<P>(source: &P) -> Self
    where
        P: PlineSource<Num = T> + ?Sized,
    {
        let vc = source.vertex_count();
        assert!(
            vc >= 2,
            "source must have at least 2 vertexes to form view data"
        );

        let view_data = if source.is_closed() {
            Self {
                start_index: 0,
                end_index_offset: vc - 1,
                updated_start: source.at(0),
                updated_end_bulge: source.last().unwrap().bulge,
                end_point: source.at(0).pos(),
                inverted_direction: false,
            }
        } else {
            Self {
                start_index: 0,
                end_index_offset: vc - 2,
                updated_start: source.at(0),
                updated_end_bulge: source.at(vc - 2).bulge,
                end_point: source.at(vc - 1).pos(),
                inverted_direction: false,
            }
        };

        debug_assert_eq!(
            view_data.validate_for_source(source),
            ViewDataValidation::IsValid
        );

        view_data
    }

    /// Construct view which changes the start point of a polyline. If the polyline is open this
    /// will trim the polyline up to the start point. If the polyline is closed then the entire
    /// polyline path is retained with just the start point changed. Returns `None` if polyline is
    /// open and start point equals the final vertex position for the polyline.
    ///
    /// # Panics
    ///
    /// This function panics if `source` has less than 2 vertexes or `start_index` out of range for
    /// `source`.
    pub fn from_new_start<P>(
        source: &P,
        start_point: Vector2<T>,
        start_index: usize,
        pos_equal_eps: T,
    ) -> Option<Self>
    where
        P: PlineSource<Num = T> + ?Sized,
    {
        // check if open polyline then just delegate to slice points method
        if !source.is_closed() {
            return Self::from_slice_points(
                source,
                start_point,
                start_index,
                source.last()?.pos(),
                source.vertex_count() - 1,
                pos_equal_eps,
            );
        }

        let vc = source.vertex_count();
        assert!(
            vc >= 2,
            "source must have at least 2 vertexes to form view data"
        );

        // catch where start point is at very end of start index segment (and adjust forward)
        let start_index = {
            let next_index = source.next_wrapping_index(start_index);
            if source
                .at(next_index)
                .pos()
                .fuzzy_eq_eps(start_point, pos_equal_eps)
            {
                next_index
            } else {
                start_index
            }
        };

        let start_v1 = source.at(start_index);
        let start_v2 = source.at(source.next_wrapping_index(start_index));
        let split = seg_split_at_point(start_v1, start_v2, start_point, pos_equal_eps);

        let end_index_offset = if start_v1.pos().fuzzy_eq_eps(start_point, pos_equal_eps) {
            vc - 1
        } else {
            vc
        };

        let view_data = Self {
            start_index,
            end_index_offset,
            updated_start: split.split_vertex,
            updated_end_bulge: split.updated_start.bulge,
            end_point: start_point,
            inverted_direction: false,
        };

        debug_assert_eq!(
            view_data.validate_for_source(source),
            ViewDataValidation::IsValid
        );

        Some(view_data)
    }

    /// Construct view that is contiguous between two points on a source polyline (start and end of
    /// source polyline are trimmed).
    ///
    /// # Panics
    ///
    /// This function panics if `source` has less than 2 vertexes or indexes out of range for
    /// `source`.
    pub fn from_slice_points<P>(
        source: &P,
        start_point: Vector2<T>,
        start_index: usize,
        end_point: Vector2<T>,
        end_index: usize,
        pos_equal_eps: T,
    ) -> Option<Self>
    where
        P: PlineSource<Num = T> + ?Sized,
    {
        debug_assert!(
            start_index <= end_index || source.is_closed(),
            "start index should be less than or equal to end index if polyline is open"
        );

        // catch if start_point is at end of first segment
        let (start_index, start_point_at_seg_end) = {
            if !source.is_closed() && start_index >= end_index {
                // not possible to wrap index forward
                (start_index, false)
            } else {
                let next_index = source.next_wrapping_index(start_index);
                if source
                    .at(next_index)
                    .pos()
                    .fuzzy_eq_eps(start_point, pos_equal_eps)
                {
                    (next_index, true)
                } else {
                    (start_index, false)
                }
            }
        };

        let traverse_count = source.fwd_wrapping_dist(start_index, end_index);

        // compute updated start vertex
        let updated_start = {
            let start_v1 = source.at(start_index);
            let start_v2 = source.at(source.next_wrapping_index(start_index));
            if start_point_at_seg_end {
                // start point on top of vertex no need to split using start_point
                if traverse_count == 0 {
                    // start and end point on same segment, split at end point
                    let split = seg_split_at_point(start_v1, start_v2, end_point, pos_equal_eps);
                    split.updated_start
                } else {
                    start_v1
                }
            } else {
                // split at start point
                let start_split =
                    seg_split_at_point(start_v1, start_v2, start_point, pos_equal_eps);
                let updated_for_start = start_split.split_vertex;
                if traverse_count == 0 {
                    // start and end point on same segment, split at end point
                    let split =
                        seg_split_at_point(updated_for_start, start_v2, end_point, pos_equal_eps);
                    split.updated_start
                } else {
                    updated_for_start
                }
            }
        };

        if traverse_count == 0 {
            Self::create_on_single_segment(
                source,
                start_index,
                updated_start,
                end_point,
                pos_equal_eps,
            )
        } else {
            Some(Self::create(
                source,
                start_index,
                end_point,
                end_index,
                updated_start,
                traverse_count,
                pos_equal_eps,
            ))
        }
    }

    /// Epsilon value to be used by [PlineViewData::validate_for_source].
    const VALIDATION_EPS: f64 = 1e-5;

    /// Epsilon value to be used by [PlineViewData::validate_for_source] when testing if positions
    /// are fuzzy equal.
    const VALIDATION_POINT_ON_SEG_EPS: f64 = 1e-3;

    /// Function mostly used for debugging and asserts, checks that this slice's properties are
    /// valid for the source polyline provided.
    pub fn validate_for_source<P>(&self, source: &P) -> ViewDataValidation<T>
    where
        P: PlineSource<Num = T> + ?Sized,
    {
        if source.vertex_count() < 2 {
            return ViewDataValidation::SourceHasNoSegments;
        }

        if self.end_index_offset > source.vertex_count() {
            return ViewDataValidation::OffsetOutOfRange {
                offset: self.end_index_offset,
                source_length: source.vertex_count(),
            };
        }

        let point_is_on_segment = |seg_index, point: Vector2<T>| {
            let on_seg_eps = T::from(Self::VALIDATION_POINT_ON_SEG_EPS).unwrap();
            let v1 = source.at(seg_index);
            let v2 = source.at(source.next_wrapping_index(seg_index));
            if point.fuzzy_eq_eps(v1.pos(), on_seg_eps) || point.fuzzy_eq_eps(v2.pos(), on_seg_eps)
            {
                return true;
            }
            let closest_point = seg_closest_point(v1, v2, point);
            closest_point.fuzzy_eq_eps(point, on_seg_eps)
        };
        // check that updated start lies on the source polyline according to start index segment
        if !point_is_on_segment(self.start_index, self.updated_start.pos()) {
            return ViewDataValidation::UpdatedStartNotOnSegment {
                start_point: self.updated_start.pos(),
            };
        }

        // check that end point lies on the source polyline according to end index segment
        let end_index = source.fwd_wrapping_index(self.start_index, self.end_index_offset);
        if !point_is_on_segment(end_index, self.end_point) {
            return ViewDataValidation::EndPointNotOnSegment {
                end_point: self.end_point,
            };
        }

        let validation_eps = T::from(Self::VALIDATION_EPS).unwrap();
        // end point should never lie directly on top of end index segment start
        if self
            .end_point
            .fuzzy_eq_eps(source.at(end_index).pos(), validation_eps)
        {
            return ViewDataValidation::EndPointOnFinalOffsetVertex {
                end_point: self.end_point,
                final_offset_vertex: source.at(end_index),
            };
        }

        if self.end_index_offset == 0 {
            // end point on start index segment, check that updated bulge matches updated start
            // bulge
            if !self
                .updated_end_bulge
                .fuzzy_eq_eps(self.updated_start.bulge, validation_eps)
            {
                return ViewDataValidation::UpdatedBulgeDoesNotMatch {
                    updated_bulge: self.updated_end_bulge,
                    expected: self.updated_start.bulge,
                };
            }
        }

        ViewDataValidation::IsValid
    }
}

/// Enum used for view data validation debugging and asserting.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum ViewDataValidation<T> {
    SourceHasNoSegments,
    OffsetOutOfRange {
        offset: usize,
        source_length: usize,
    },
    UpdatedStartNotOnSegment {
        start_point: Vector2<T>,
    },
    EndPointNotOnSegment {
        end_point: Vector2<T>,
    },
    EndPointOnFinalOffsetVertex {
        end_point: Vector2<T>,
        final_offset_vertex: PlineVertex<T>,
    },
    UpdatedBulgeDoesNotMatch {
        updated_bulge: T,
        expected: T,
    },
    IsValid,
}
