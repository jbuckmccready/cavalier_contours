use crate::polyline::{PlineSource, PlineVertex};

/// A read-only "view" of another polyline but with inverted (reversed) direction
/// so that you do not need to clone and call `invert_direction_mut`.
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

impl<'a, P> PlineSource for PlineInversionView<'a, P>
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
        let vc = self.vertex_count();
        if vc == 0 || index >= vc {
            return None;
        }
        // reversed index
        // e.g. reversed_i = (vc - 1) - index
        let reversed_i = (vc - 1).saturating_sub(index);
        let maybe_v = self.original.get(reversed_i)?;
        // invert bulge sign, except for final segment if closed
        if vc > 1 {
            // The bulge for segment in reversed direction is that of the "next" in original,
            // which is the "previous" in reversed order.
            let _next_reversed = reversed_i.saturating_sub(1);
            let is_last_in_closed = self.is_closed() && reversed_i == 0;
            if !is_last_in_closed {
                let neg_bulge = -maybe_v.bulge;
                return Some(maybe_v.with_bulge(neg_bulge));
            }
        }
        Some(maybe_v.with_bulge(-maybe_v.bulge))
    }

    #[inline]
    fn at(&self, index: usize) -> PlineVertex<Self::Num> {
        self.get(index)
            .expect("PlineInversionView: `index` out of range")
    }
}
