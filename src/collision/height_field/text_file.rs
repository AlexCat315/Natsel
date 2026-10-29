//! Authored height-field interchange in the plain-text sample format.

use crate::collision::height_field::HeightField;
use crate::math::types::Vec3;
use crate::types::geometry::HeightFieldDef;

/// Failure to read, write or parse a height-field text file.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeightFieldFileError {
    InvalidData,
}

impl HeightFieldDef {
    /// Write authored samples in the height-data text format.
    ///
    /// Swift rethrew Foundation's write error; the public surface only has `invalidData`, so an
    /// I/O failure is reported as that case. `std::fs::write` matches Swift's `atomically: false`.
    pub fn write(&self, path: &str) -> Result<(), HeightFieldFileError> {
        let text = HeightFieldText::encode(self).ok_or(HeightFieldFileError::InvalidData)?;
        std::fs::write(path, text.as_bytes()).map_err(|_| HeightFieldFileError::InvalidData)
    }
}

impl HeightField {
    /// Build shared terrain from a text file in the height-data format.
    pub fn load(path: &str) -> Result<HeightField, HeightFieldFileError> {
        let text = std::fs::read_to_string(path).map_err(|_| HeightFieldFileError::InvalidData)?;
        let definition = HeightFieldText::decode(&text).ok_or(HeightFieldFileError::InvalidData)?;
        HeightField::new(&definition).ok_or(HeightFieldFileError::InvalidData)
    }
}

/// The text format codec; internal in Swift, so nothing here is `pub`.
struct HeightFieldText;

impl HeightFieldText {
    /// A locale-independent `en_US_POSIX` fixed decimal, exactly nine fraction digits.
    fn decimal(value: f32) -> String {
        format!("{:.9}", value as f64)
    }

    fn encode(definition: &HeightFieldDef) -> Option<String> {
        let columns = definition.count_x;
        let rows = definition.count_z;
        if columns < 2
            || rows < 2
            || definition.heights.len() != (columns as usize) * (rows as usize)
            || (!definition.material_indices.is_empty()
                && definition.material_indices.len()
                    != (columns as usize - 1) * (rows as usize - 1))
        {
            return None;
        }

        let mut lines: Vec<String> = Vec::with_capacity(4 + definition.heights.len());
        lines.push(format!("{} {}", columns, rows));
        lines.push(format!(
            "{} {} {}",
            Self::decimal(definition.scale.x),
            Self::decimal(definition.scale.y),
            Self::decimal(definition.scale.z)
        ));
        lines.push(format!(
            "{} {}",
            Self::decimal(definition.global_minimum_height),
            Self::decimal(definition.global_maximum_height)
        ));
        lines.push(if definition.clockwise_winding {
            "1".to_owned()
        } else {
            "0".to_owned()
        });
        for value in &definition.heights {
            lines.push(Self::decimal(*value));
        }

        let cell_count = (columns as usize - 1) * (rows as usize - 1);
        if definition.material_indices.is_empty() {
            for _ in 0..cell_count {
                lines.push("0".to_owned());
            }
        } else {
            for value in &definition.material_indices {
                lines.push(format!("{}", value));
            }
        }

        Some(lines.join("\n") + "\n")
    }

    fn decode(text: &str) -> Option<HeightFieldDef> {
        let tokens: Vec<&str> = text.split_whitespace().collect();
        if tokens.len() < 8 {
            return None;
        }
        let columns = tokens[0].parse::<i32>().ok()?;
        let rows = tokens[1].parse::<i32>().ok()?;
        if columns < 2 || rows < 2 {
            return None;
        }
        let x = tokens[2].parse::<f32>().ok()?;
        let y = tokens[3].parse::<f32>().ok()?;
        let z = tokens[4].parse::<f32>().ok()?;
        let minimum = tokens[5].parse::<f32>().ok()?;
        let maximum = tokens[6].parse::<f32>().ok()?;
        let clockwise = tokens[7].parse::<i32>().ok()?;

        let height_count = columns as usize * rows as usize;
        let cell_count = (columns as usize - 1) * (rows as usize - 1);
        if tokens.len() < 8 + height_count + cell_count {
            return None;
        }

        let mut definition = HeightFieldDef::default();
        definition.count_x = columns;
        definition.count_z = rows;
        definition.scale = Vec3::new(x, y, z);
        definition.global_minimum_height = minimum;
        definition.global_maximum_height = maximum;
        definition.clockwise_winding = clockwise != 0;
        definition.heights.reserve(height_count);
        definition.material_indices.reserve(cell_count);
        for i in 0..height_count {
            let value = tokens[8 + i].parse::<f32>().ok()?;
            definition.heights.push(value);
        }
        for i in 0..cell_count {
            let value = tokens[8 + height_count + i].parse::<i32>().ok()?;
            definition.material_indices.push(value as u8);
        }
        Some(definition)
    }
}
