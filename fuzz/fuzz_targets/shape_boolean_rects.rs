#![no_main]

use libfuzzer_sys::fuzz_target;

mod support;

fuzz_target!(|data: &[u8]| {
    let mut reader = support::ByteReader::new(data);
    let a = support::rectangle_shape(&mut reader, 4);
    let b = support::rectangle_shape(&mut reader, 4);
    let op = support::boolean_op(&mut reader);
    support::assert_boolean_semantics(&a, &b, op);
});
