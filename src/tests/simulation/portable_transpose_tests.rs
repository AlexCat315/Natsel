//! Ported from Tests/NatselTests/Simulation/PortableTransposeTests.swift.

use crate::simulation::wide_types::{portable_transpose4, transpose4, FloatW};

// "portable transpose preserves lane bits and agrees with the platform implementation"
#[test]
fn lane_bits() {
    let rows = [
        FloatW::new(0.0, -0.0, f32::INFINITY, f32::NEG_INFINITY),
        FloatW::new(1.0, -2.0, f32::MIN_POSITIVE, f32::MAX),
        FloatW::new(f32::from_bits(0x7fc0_0001), 4.0, 5.0, 6.0),
        FloatW::new(7.0, 8.0, 9.0, 10.0),
    ];
    let actual = portable_transpose4(rows[0], rows[1], rows[2], rows[3]);
    let platform = transpose4(rows[0], rows[1], rows[2], rows[3]);
    let columns = [actual.0, actual.1, actual.2, actual.3];
    let expected = [platform.0, platform.1, platform.2, platform.3];
    for column in 0..4 {
        for row in 0..4 {
            assert_eq!(columns[column][row].to_bits(), rows[row][column].to_bits());
            assert_eq!(columns[column][row].to_bits(), expected[column][row].to_bits());
        }
    }
}
