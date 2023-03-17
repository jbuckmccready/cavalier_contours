use cavalier_contours::{polyline::Polyline, shape_algorithms::Shape};

#[test]
fn test1() {
    let json = r#"[
  {
    "isClosed": true, 
    "vertexes": [
      [100, 100, -0.5], 
      [80, 90, 0.374794619217547], 
      [210, 0, 0], 
      [230, 0, 1], 
      [320, 0, -0.5], 
      [280, 0, 0.5], 
      [390, 210, 0], 
      [280, 120, 0.5]
    ]
  }, 
  {
    "isClosed": true, 
    "vertexes": [
      [150, 50, 0], 
      [146.32758944101474, 104.13867601941358, 0], 
      [200, 100, 0], 
      [200, 50, 0]
    ]
  }
]"#;

    let plines: Vec<Polyline<f64>> = serde_json::from_str(json).unwrap();
    let shape = Shape::from_plines(plines);
    let result = shape.parallel_offset(17.0).unwrap();
}

#[test]
fn test2() {
    let json = r#"[
  {
    "isClosed": true, 
    "vertexes": [
      [160.655879768138, 148.75471430537402, -0.5], 
      [80, 90, 0.374794619217547], 
      [210, 0, 0], 
      [230, 0, 1], 
      [320, 0, -0.5], 
      [280, 0, 0.5], 
      [390, 210, 0], 
      [280, 120, 0.5]
    ]
  }, 
  {
    "isClosed": true, 
    "vertexes": [
      [150, 50, 0], 
      [192.62381977774953, 130.82800839110848, 0], 
      [200, 100, 0], 
      [200, 50, 0]
    ]
  }
]"#;

    let plines: Vec<Polyline<f64>> = serde_json::from_str(json).unwrap();
    let shape = Shape::from_plines(plines);
    let result = shape.parallel_offset(17.0).unwrap();
}
