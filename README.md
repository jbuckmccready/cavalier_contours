[![Build Status](https://github.com/jbuckmccready/cavalier_contours/actions/workflows/ci.yml/badge.svg)](https://github.com/jbuckmccready/cavalier_contours/actions)
[![Crates.io](https://img.shields.io/crates/v/cavalier_contours.svg)](https://crates.io/crates/cavalier_contours)
[![Docs.rs](https://docs.rs/cavalier_contours/badge.svg)](https://docs.rs/cavalier_contours)
[![MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE-MIT)
[![Apache](https://img.shields.io/badge/license-Apache-blue.svg)](LICENSE-APACHE)

## Summary

This project is a continuation of the C++
[CavalierContours](https://github.com/jbuckmccready/CavalierContours) library rewritten in Rust with
the goal of building out more functionality, better documentation, and creating a stable C FFI.
This project has all of the functionality of the C++ repository with more code documentation, test
coverage, and some additional functions for working with polylines. For tracking progress and
contributing checkout the project GitHub issues. For more information about the parallel offset
algorithm and background information see the old C++ repository `README.md`
[here](https://github.com/jbuckmccready/CavalierContours).


## Main Features

- Polylines defined with line and arc segments (fixed radius, arcs are not approximated as line segments)
- Polyline parallel offsetting (works on open, closed, and self-intersecting polylines)
- Boolean operations between two closed polylines (union, intersection, difference)
- Polyline containment and intersection tests
- 2D spatial indexing to speed up alogorithms on high vertex count polylines
- Multi-polyline parallel offsetting ("shapes" defined with islands)
- No unsafe code in core crate
- C FFI for integration with other languages
- WebAssembly support and interactive web demo

[👉 Click to run the WASM web demo 👈](https://jbuckmccready.github.io/cavalier_contours/)

<img src="https://github.com/jbuckmccready/CavalierContoursDoc/blob/master/gifs/PolylineOffsets.gif" width="400"/> <img src="https://github.com/jbuckmccready/CavalierContoursDoc/blob/master/gifs/PolylineCombines.gif" width="400"/>

<img src="https://raw.githubusercontent.com/jbuckmccready/CavalierContoursDoc/master/images/pretty_examples/example1.png" width="400"/> <img src="https://raw.githubusercontent.com/jbuckmccready/CavalierContoursDoc/master/images/pretty_examples/islands_example1.png" width="400"/>


## Workspace Structure

- **cavalier_contours**: Core Rust library and API for polyline algorithms.
- **cavalier_contours_ffi**: C FFI bindings for use from C/C++ and other languages. [`cavalier_contours_ffi` README](cavalier_contours_ffi/README.md)
- **cavalier_contours_ui**: Web-based UI demo (WASM, using [egui](https://github.com/emilk/egui)). [`cavalier_contours_ui` README](cavalier_contours_ui/README.md)


## Requirements

- Rust 1.88+ (MSRV)
- Tested with CI on Linux, macOS, and Windows


## Interactive Web Page for Visualizing and Testing

> [!NOTE]
> This is the old web demo ui page. The new page built with egui is [here](https://jbuckmccready.github.io/cavalier_contours/).

This project is compiled to WASM to create an interactive web demo page for visualizing and testing.

- Interactive web demo page: https://cavaliercontours.dev/
- Interactive web demo page repo: https://github.com/jbuckmccready/cavalier_contours_web_demo


## Why go to Rust?

- All the same benefits of using C or C++ (great performance/optimizations, native compile, no
  garbage collection, no run time) for creating fast portable libraries with a C FFI
- Great builtin tooling around builds and packages (cargo + crates)
- Great builtin tooling for writing and maintaining tests
- All of the great builtin tooling makes open source contribution and participation easier to
  facilitate
- Borrow checker + lifetimes allow for more advanced memory allocation optimizations without the
  risk of memory errors/corruption bugs
- Type system allows for leaning heavily on threads/concurrency without the risk of memory
  errors/corruption bugs
- Discriminated unions and pattern matching as first class language features
- Great tooling for targeting wasm


## License

Licensed under either of

- Apache License, Version 2.0
  ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license
  ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.


## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
