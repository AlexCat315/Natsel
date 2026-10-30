//! Soft contact cell grid.

use crate::math::types::{AABB, Vec3};
use crate::soft_body::soft_body_contacts::CellGrid;

fn visit(grid: &CellGrid, point: Vec3) -> Vec<i32> {
    let mut result: Vec<i32> = Vec::new();
    grid.visit_point(point, |item| result.push(item));
    result
}

#[test]
// "cell visits preserve ascending feature order with repeated and unordered inserts"
fn ordered_features() {
    let mut grid = CellGrid::new(1.0);
    let point = Vec3::new(0.25, 0.25, 0.25);
    for item in [7i32, 2, -3, 2] {
        grid.insert(AABB::new(point, point), item);
    }
    grid.finish();
    let mut visited: Vec<i32> = Vec::new();
    grid.visit_point(point, |item| visited.push(item));
    assert_eq!(visited, [-3, 2, 2, 7]);
}

#[test]
// "negative coordinates and exact cell boundaries keep all spanning features"
fn boundaries() {
    let mut grid = CellGrid::new(1.0);
    grid.insert(
        AABB::new(Vec3::new(-1.0, -1.0, -1.0), Vec3::new(1.0, 1.0, 1.0)),
        4,
    );
    grid.insert(
        AABB::new(Vec3::new(-0.001, -0.001, -0.001), Vec3::ZERO),
        2,
    );
    grid.insert(AABB::new(Vec3::ZERO, Vec3::ZERO), 1);
    grid.finish();

    assert_eq!(visit(&grid, Vec3::new(-0.5, -0.5, -0.5)), [2, 4]);
    assert_eq!(visit(&grid, Vec3::ZERO), [1, 2, 4]);
    assert_eq!(visit(&grid, Vec3::new(1.0, 1.0, 1.0)), [4]);
    assert!(visit(&grid, Vec3::new(2.0, 2.0, 2.0)).is_empty());
}

#[test]
// "finishing again after insertion retains previous cells and feature order"
fn rebuilding() {
    let mut grid = CellGrid::new(1.0);
    grid.insert(AABB::new(Vec3::ZERO, Vec3::ZERO), 5);
    grid.finish();
    grid.insert(AABB::new(Vec3::ZERO, Vec3::ZERO), 3);
    let remote = Vec3::new(4.0, 5.0, 6.0);
    grid.insert(AABB::new(remote, remote), 1);
    grid.finish();

    let origin = visit(&grid, Vec3::ZERO);
    let distant = visit(&grid, remote);
    assert_eq!(origin, [3, 5]);
    assert_eq!(distant, [1]);
}
