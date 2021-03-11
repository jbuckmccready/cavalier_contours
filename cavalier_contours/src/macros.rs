#[macro_export]
macro_rules! assert_fuzzy_eq {
    ($left:expr, $right:expr) => {{
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

#[macro_export]
macro_rules! pline_open {
    ( $( $x:expr ),* ) => {
        {
            let mut pl = Polyline::new();
            $(
                pl.add($x.0, $x.1, $x.2);
            )*
            pl
        }
    };
}

#[macro_export]
macro_rules! pline_closed {
    ( $( $x:expr ),* ) => {
        {
            let mut pl = Polyline::new_closed();
            $(
                pl.add($x.0, $x.1, $x.2);
            )*
            pl
        }
    };
}
