#![no_main]

use libfuzzer_sys::fuzz_target;

mod support;

fuzz_target!(|data: &[u8]| {
    let mut reader = support::ByteReader::new(data);
    let a = support::donut_shape(&mut reader);
    let b = if reader.bool() {
        support::donut_shape(&mut reader)
    } else {
        support::rectangle_shape(&mut reader, 3)
    };
    let op = support::boolean_op(&mut reader);
    support::assert_boolean_semantics(&a, &b, op);
});
