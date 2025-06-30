use super::{PlineCreation, PlineSource, PlineSourceMut, PlineVertex};
use crate::core::traits::Real;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};
use std::ops::{Index, IndexMut};

/// Basic polyline data representation that implements the core polyline traits:
/// [PlineSource], [PlineSourceMut], and [PlineCreation]. See the traits documentation for all
/// the polyline methods/operations available.
#[cfg_attr(
    feature = "serde",
    derive(Serialize, Deserialize),
    serde(rename_all = "camelCase")
)]
#[derive(Debug, Clone)]
pub struct Polyline<T = f64> {
    #[cfg_attr(feature = "serde", serde(rename = "vertexes"))]
    /// Contiguous sequence of vertexes.
    pub vertex_data: Vec<PlineVertex<T>>,
    /// Bool to indicate whether the polyline is closed or open.
    pub is_closed: bool,
    // Vec of user-provided u64 values. Preserved across offset calls. Note that a Polyline that was
    // composed out of multiple slices of other Polylines will have userdata values from each source
    // polyline, and as such userdata values may appear repeatedly.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Vec::is_empty")
    )]
    pub userdata: Vec<u64>,
}

impl<T> Default for Polyline<T>
where
    T: Real,
{
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl<T> Polyline<T>
where
    T: Real,
{
    /// Create a new empty [Polyline] with `is_closed` set to false.
    #[inline]
    pub fn new() -> Self {
        Polyline {
            vertex_data: Vec::new(),
            is_closed: false,
            userdata: Vec::new(),
        }
    }

    /// Create a new empty [Polyline] with `is_closed` set to true.
    #[inline]
    pub fn new_closed() -> Self {
        Polyline {
            vertex_data: Vec::new(),
            is_closed: true,
            userdata: Vec::new(),
        }
    }

    #[inline]
    pub fn get_userdata_count(&self) -> usize {
        self.userdata.len()
    }

    #[inline]
    pub fn get_userdata_values(&self) -> impl Iterator<Item = u64> + '_ {
        self.userdata.iter().copied()
    }

    #[inline]
    pub fn set_userdata_values(&mut self, values: impl IntoIterator<Item = u64>) {
        self.userdata.clear();
        self.userdata.extend(values);
    }

    #[inline]
    pub fn add_userdata_values(&mut self, values: impl IntoIterator<Item = u64>) {
        self.userdata.extend(values);
    }
}

impl<T> Index<usize> for Polyline<T> {
    type Output = PlineVertex<T>;

    #[inline]
    fn index(&self, index: usize) -> &Self::Output {
        &self.vertex_data[index]
    }
}

impl<T> IndexMut<usize> for Polyline<T> {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.vertex_data[index]
    }
}

impl<T> PlineSource for Polyline<T>
where
    T: Real,
{
    type Num = T;
    type OutputPolyline = Polyline<T>;

    #[inline]
    fn get_userdata_count(&self) -> usize {
        self.userdata.len()
    }

    #[inline]
    fn get_userdata_values(&self) -> impl Iterator<Item = u64> + '_ {
        self.userdata.iter().copied()
    }

    #[inline]
    fn vertex_count(&self) -> usize {
        self.vertex_data.len()
    }

    #[inline]
    fn is_closed(&self) -> bool {
        self.is_closed
    }

    #[inline]
    fn get(&self, index: usize) -> Option<PlineVertex<Self::Num>> {
        self.vertex_data.get(index).copied()
    }

    #[inline]
    fn at(&self, index: usize) -> PlineVertex<Self::Num> {
        self[index]
    }
}

impl<T> PlineSourceMut for Polyline<T>
where
    T: Real,
{
    #[inline]
    fn set_userdata_values(&mut self, values: impl IntoIterator<Item = u64>) {
        self.userdata.clear();
        self.userdata.extend(values);
    }

    #[inline]
    fn add_userdata_values(&mut self, values: impl IntoIterator<Item = u64>) {
        self.userdata.extend(values);
    }

    #[inline]
    fn set_vertex(&mut self, index: usize, vertex: PlineVertex<Self::Num>) {
        self.vertex_data[index] = vertex;
    }

    #[inline]
    fn insert_vertex(&mut self, index: usize, vertex: PlineVertex<Self::Num>) {
        self.vertex_data.insert(index, vertex);
    }

    #[inline]
    fn remove(&mut self, index: usize) -> PlineVertex<Self::Num> {
        self.vertex_data.remove(index)
    }

    #[inline]
    fn add_vertex(&mut self, vertex: PlineVertex<Self::Num>) {
        self.vertex_data.push(vertex)
    }

    #[inline]
    fn reserve(&mut self, additional: usize) {
        self.vertex_data.reserve(additional);
    }

    #[inline]
    fn set_is_closed(&mut self, is_closed: bool) {
        self.is_closed = is_closed;
    }

    #[inline]
    fn clear(&mut self) {
        self.vertex_data.clear()
    }

    #[inline]
    fn extend_vertexes<I>(&mut self, vertexes: I)
    where
        I: IntoIterator<Item = PlineVertex<Self::Num>>,
    {
        self.vertex_data.extend(vertexes);
    }
}

impl<T> PlineCreation for Polyline<T>
where
    T: Real,
{
    #[inline]
    fn with_capacity(capacity: usize, is_closed: bool) -> Self {
        Polyline {
            vertex_data: Vec::with_capacity(capacity),
            is_closed,
            userdata: Vec::new(),
        }
    }

    #[inline]
    fn from_iter<I>(iter: I, is_closed: bool) -> Self
    where
        I: Iterator<Item = PlineVertex<Self::Num>>,
    {
        Polyline {
            vertex_data: iter.collect(),
            is_closed,
            userdata: Vec::new(),
        }
    }
}
