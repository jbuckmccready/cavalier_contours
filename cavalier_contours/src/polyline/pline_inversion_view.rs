use crate::polyline::{PlineSource, PlineVertex};

/// A read-only "view" of another polyline but with inverted (reversed) direction
/// so that you do not need to clone and call `invert_direction_mut`.
///
/// NOTE: for both open and closed polylines the index position of each vertex is shifted by one, so
/// the last vertex position in the original polyline will be the first vertex in the inverted polyline.
///
/// This type implements `PlineSource`, so you can pass it to boolean ops, offsets, etc.
pub struct PlineInversionView<'a, P>
where
    P: PlineSource + ?Sized,
{
    original: &'a P,
}

impl<'a, P> PlineInversionView<'a, P>
where
    P: PlineSource + ?Sized,
{
    pub fn new(original: &'a P) -> Self {
        Self { original }
    }
}

impl<P> PlineSource for PlineInversionView<'_, P>
where
    P: PlineSource + ?Sized,
{
    type Num = P::Num;
    type OutputPolyline = P::OutputPolyline;

    #[inline]
    fn vertex_count(&self) -> usize {
        self.original.vertex_count()
    }

    #[inline]
    fn is_closed(&self) -> bool {
        self.original.is_closed()
    }

    #[inline]
    fn get(&self, index: usize) -> Option<PlineVertex<Self::Num>> {
        if self.vertex_count() == 0 || index >= self.original.vertex_count() {
            return None;
        }

        if self.original.vertex_count() == 1 {
            return self.original.get(0);
        }

        let rev_index = self.original.vertex_count() - 1 - index;
        let bulge_index = self.original.prev_wrapping_index(rev_index);
        let bulge = -self.original.get(bulge_index)?.bulge;
        Some(self.original.get(rev_index)?.with_bulge(bulge))
    }

    #[inline]
    fn at(&self, index: usize) -> PlineVertex<Self::Num> {
        self.get(index)
            .expect("PlineInversionView: `index` out of range")
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::polyline::{PlineSourceMut, Polyline};

    #[test]
    fn test_open_pline_inversion_view() {
        let mut pline = Polyline::new();
        pline.add(0.0, 1.1, 0.1);
        pline.add(1.0, 2.2, 0.2);
        pline.add(2.0, 3.3, 0.3);
        pline.add(3.0, 4.4, 0.4);
        pline.add(4.0, 5.5, 0.5);

        let inv_view = PlineInversionView::new(&pline);

        assert_eq!(inv_view.vertex_count(), 5);
        assert_eq!(inv_view.is_closed(), false);

        assert_eq!(inv_view.at(0), PlineVertex::new(4.0, 5.5, -0.4));
        assert_eq!(inv_view.at(1), PlineVertex::new(3.0, 4.4, -0.3));
        assert_eq!(inv_view.at(2), PlineVertex::new(2.0, 3.3, -0.2));
        assert_eq!(inv_view.at(3), PlineVertex::new(1.0, 2.2, -0.1));
        assert_eq!(inv_view.at(4), PlineVertex::new(0.0, 1.1, -0.5));
    }

    #[test]
    fn test_closed_pline_inversion_view() {
        let mut pline = Polyline::new_closed();
        pline.add(0.0, 1.1, 0.1);
        pline.add(1.0, 2.2, 0.2);
        pline.add(2.0, 3.3, 0.3);
        pline.add(3.0, 4.4, 0.4);
        pline.add(4.0, 5.5, 0.5);

        let inv_view = PlineInversionView::new(&pline);

        assert_eq!(inv_view.vertex_count(), 5);
        assert_eq!(inv_view.is_closed(), true);

        assert_eq!(inv_view.at(0), PlineVertex::new(4.0, 5.5, -0.4));
        assert_eq!(inv_view.at(1), PlineVertex::new(3.0, 4.4, -0.3));
        assert_eq!(inv_view.at(2), PlineVertex::new(2.0, 3.3, -0.2));
        assert_eq!(inv_view.at(3), PlineVertex::new(1.0, 2.2, -0.1));
        assert_eq!(inv_view.at(4), PlineVertex::new(0.0, 1.1, -0.5));
    }
}
