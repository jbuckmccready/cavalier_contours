## Summary

This project is a continuation of the C++ [CavalierContours](https://github.com/jbuckmccready/CavalierContours) library rewritten in Rust with the goal of building out more functionality, better documentation, and creating a stable C FFI. This project is still in early development, with no APIs yet solidified, the code is not fully tested, and some functions still need to be ported from C++. For tracking progress and contributing checkout the project GitHub issues.

## Why go to Rust?

* All the same benefits of using C or C++ (great performance/optimizations, native compile, no garbage collection, no run time) for creating fast portable libraries with a C FFI
* Great builtin tooling around builds and packages (cargo + crates)
* Great builtin tooling for writing and maintaining tests
* All of the great builtin tooling makes open source contribution and participation easier to facilitate
* Borrow checker + lifetimes allow for more advanced memory allocation optimizations without the risk of memory errors/corruption bugs
* Type system allows for leaning heavily on threads/concurrency without the risk of memory errors/corruption bugs
* Discriminated unions and pattern matching as first class language features
* Great tooling for targeting wasm

## New to Rust but still want to contribute?

I recommend the official rust book [here](https://doc.rust-lang.org/book/ch01-00-getting-started.html). Visual Studio Code + [rust-analyzer](https://rust-analyzer.github.io/) extension work great for editing. If you're looking for something specific to work on check the project issues labeled `good first issue`.

## License

Licensed under either of

 * Apache License, Version 2.0
   ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license
   ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
