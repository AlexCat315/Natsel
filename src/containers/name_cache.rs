//! Hash cache for debug names.

use std::collections::HashMap;

use crate::base::core::b3_log;

pub const NULL_NAME: u32 = 0;

/// Hash small strings, avoiding collisions for a 32-bit hash.
pub fn hash32(bytes: &[u8]) -> u32 {
    let mut h: u32 = 0x811c_9dc5;
    for byte in bytes {
        h ^= *byte as u32;
        h = h.wrapping_mul(0x0100_0193);
    }

    h ^= h >> 16;
    h = h.wrapping_mul(0x85eb_ca6b);
    h ^= h >> 13;
    h = h.wrapping_mul(0xc2b2_ae35);
    h ^= h >> 16;
    h
}

#[derive(Clone, Debug)]
pub struct NameEntry {
    pub hash: u32,
    pub name: String,
}

#[derive(Clone, Debug, Default)]
pub struct NameCache {
    pub entries: Vec<NameEntry>,
    pub map: HashMap<u32, i32>,
}

impl NameCache {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add(&mut self, name: Option<&str>) -> u32 {
        let Some(name) = name else {
            return NULL_NAME;
        };
        if name.is_empty() {
            return NULL_NAME;
        }

        let mut id = hash32(name.as_bytes());
        if id == 0 {
            id = 1;
        }

        if let Some(&index) = self.map.get(&id) {
            if self.entries[index as usize].name != name {
                b3_log(&format!("Hash collision on {name}"));
            }
            return id;
        }

        let index = self.entries.len() as i32;
        self.entries.push(NameEntry {
            hash: id,
            name: name.to_string(),
        });
        self.map.insert(id, index);
        id
    }

    pub fn find(&self, id: u32) -> Option<&str> {
        self.map
            .get(&id)
            .map(|&index| self.entries[index as usize].name.as_str())
    }

    pub fn find_or<'a>(&'a self, id: u32, default_name: &'a str) -> &'a str {
        self.find(id).unwrap_or(default_name)
    }

    /// Load a name from a recording.
    pub fn load(&mut self, id: u32, name: Option<&str>) {
        let Some(name) = name else {
            return;
        };
        if name.is_empty() || id == NULL_NAME {
            return;
        }
        if self.map.contains_key(&id) {
            return;
        }
        let index = self.entries.len() as i32;
        self.entries.push(NameEntry {
            hash: id,
            name: name.to_string(),
        });
        self.map.insert(id, index);
    }
}
