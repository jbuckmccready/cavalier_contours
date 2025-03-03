## Summary

This is the ui app for visualizing, testing, and demonstrating the cavalier_contours crate.

Github CI publishes latest on master branch to github pages.

[👉 Click to run the web demo 👈](https://jbuckmccready.github.io/cavalier_contours/).

The app is built with:
- [egui](https://github.com/emilk/egui) (immediate mode GUI library)
- [egui_plot](https://github.com/emilk/egui_plot) (plotting widget for egui)
- [lyon](https://github.com/nical/lyon) (path tessellation library for filling concave and complex polygons)


## Running Native Locally

```sh
cargo run
```

## Run Web Locally

> [!IMPORTANT]  
> You need to have trunk installed to run the web ui locally (install docs [here](https://trunkrs.dev/guide/getting-started/installation.html)).
> ```sh
> cargo install trunk --locked
> ```

```sh 
trunk serve
```
