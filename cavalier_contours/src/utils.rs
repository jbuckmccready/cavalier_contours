#[cfg(test)]
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
