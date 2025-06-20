/// Macro used for fuzzy equal assertions.
#[macro_export]
macro_rules! assert_fuzzy_eq {
    ($left:expr, $right:expr) => {{
        #[allow(unused_imports)]
        use $crate::core::traits::*;
        match (&$left, &$right) {
            (left_val, right_val) => {
                if !(left_val.fuzzy_eq(*right_val)) {
                    panic!(
                        r#"assertion failed: `left.fuzzy_eq(right)`
  left: `{:?}`,
 right: `{:?}`"#,
                        &*left_val, &*right_val
                    )
                }
            }
        }
    }};
    ($left:expr, $right:expr, $eps:expr) => {{
        #[allow(unused_imports)]
        use $crate::core::traits::*;
        match (&$left, &$right, &$eps) {
            (left_val, right_val, eps_val) => {
                if !(left_val.fuzzy_eq_eps(*right_val, *eps_val)) {
                    panic!(
                        r#"assertion failed: `left.fuzzy_eq_eps(right, eps)`
  left: `{:?}`,
 right: `{:?}`
 eps: `{:?}`"#,
                        &*left_val, &*right_val, &*eps_val
                    )
                }
            }
        }
    }};
}

/// Macro used for implementing pline macros. Used for extracting macro repetition count for
/// reserving capacity up front.
#[doc(hidden)]
#[macro_export]
macro_rules! replace_expr {
    ($_t:tt $sub:expr) => {
        $sub
    };
}

/// Construct a open polyline with the vertexes given as a list of (x, y, bulge) tuples.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::pline_open;
/// # use cavalier_contours::polyline::*;
/// let polyline = pline_open![(0.0, 1.0, 1.0), (2.0, 0.0, 0.0)];
/// assert!(!polyline.is_closed());
/// assert_eq!(polyline[0], PlineVertex::new(0.0, 1.0, 1.0));
/// assert_eq!(polyline[1], PlineVertex::new(2.0, 0.0, 0.0));
/// ```
#[macro_export]
macro_rules! pline_open {
    ($( $x:expr ),* $(,)?) => {
        {
            #[allow(unused_imports)]
            use $crate::polyline::*;
            let size = <[()]>::len(&[$(cavalier_contours::replace_expr!(($x) ())),*]);
            let mut pl = Polyline::with_capacity(size, false);
            $(
                pl.add($x.0, $x.1, $x.2);
            )*
            pl
        }
    };
}

/// Construct a open polyline with userdata and the vertexes given as a list of (x, y, bulge) tuples.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::pline_open_userdata;
/// # use cavalier_contours::polyline::*;
/// let polyline = pline_open_userdata![vec![4, 117], (0.0, 1.0, 1.0), (2.0, 0.0, 0.0)];
/// assert!(!polyline.is_closed());
/// assert_eq!(polyline[0], PlineVertex::new(0.0, 1.0, 1.0));
/// assert_eq!(polyline[1], PlineVertex::new(2.0, 0.0, 0.0));
/// assert_eq!(polyline.get_userdata_count(), 2);
/// assert!(polyline.get_userdata_values().any(|x| x == 4));
/// assert!(polyline.get_userdata_values().any(|x| x == 117));
/// ```
#[macro_export]
macro_rules! pline_open_userdata {
    ($u:expr, $( $x:expr ),* $(,)?) => {
        {
            #[allow(unused_imports)]
            use $crate::polyline::*;
            let size = <[()]>::len(&[$(cavalier_contours::replace_expr!(($x) ())),*]);
            let mut pl = Polyline::with_capacity(size, false);
            $(
                pl.add($x.0, $x.1, $x.2);
            )*
            pl.set_userdata_values($u.iter().copied());
            pl
        }
    };
}

/// Construct a closed polyline with the vertexes given as a list of (x, y, bulge) tuples.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::pline_closed;
/// # use cavalier_contours::polyline::*;
/// let polyline = pline_closed![(0.0, 1.0, 1.0), (2.0, 0.0, 0.0)];
/// assert!(polyline.is_closed());
/// assert_eq!(polyline[0], PlineVertex::new(0.0, 1.0, 1.0));
/// assert_eq!(polyline[1], PlineVertex::new(2.0, 0.0, 0.0));
/// ```
#[macro_export]
macro_rules! pline_closed {
    ($( $x:expr ),* $(,)?) => {
        {
            #[allow(unused_imports)]
            use $crate::polyline::*;
            let size = <[()]>::len(&[$(cavalier_contours::replace_expr!(($x) ())),*]);
            let mut pl = Polyline::with_capacity(size, true);
            $(
                pl.add($x.0, $x.1, $x.2);
            )*
            pl
        }
    };
}

/// Construct a closed polyline with userdata and the vertexes given as a list of (x, y, bulge) tuples.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::pline_closed_userdata;
/// # use cavalier_contours::polyline::*;
/// let polyline = pline_closed_userdata![vec![4, 117], (0.0, 1.0, 1.0), (2.0, 0.0, 0.0)];
/// assert!(polyline.is_closed());
/// assert_eq!(polyline[0], PlineVertex::new(0.0, 1.0, 1.0));
/// assert_eq!(polyline[1], PlineVertex::new(2.0, 0.0, 0.0));
/// assert_eq!(polyline.get_userdata_count(), 2);
/// assert!(polyline.get_userdata_values().any(|x| x == 4));
/// assert!(polyline.get_userdata_values().any(|x| x == 117));
/// ```
#[macro_export]
macro_rules! pline_closed_userdata {
    ($u:expr, $( $x:expr ),* $(,)?) => {
        {
            #[allow(unused_imports)]
            use $crate::polyline::*;
            let size = <[()]>::len(&[$(cavalier_contours::replace_expr!(($x) ())),*]);
            let mut pl = Polyline::with_capacity(size, true);
            $(
                pl.add($x.0, $x.1, $x.2);
            )*
            pl.set_userdata_values($u.iter().copied());
            pl
        }
    };
}
