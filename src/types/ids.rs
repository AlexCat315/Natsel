//! Opaque handles to internal objects. All ids are null when zero initialized.

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(C)]
pub struct WorldID {
    pub index1: u16,
    pub generation: u16,
}

impl WorldID {
    pub fn new(index1: u16, generation: u16) -> Self {
        Self { index1, generation }
    }

    pub const NULL: WorldID = WorldID {
        index1: 0,
        generation: 0,
    };

    pub fn is_null(&self) -> bool {
        self.index1 == 0
    }

    /// Store a world id into a `u32`.
    pub fn stored(&self) -> u32 {
        ((self.index1 as u32) << 16) | self.generation as u32
    }

    /// Load a world id from a `u32`.
    pub fn from_stored(x: u32) -> Self {
        Self {
            index1: (x >> 16) as u16,
            generation: x as u16,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(C)]
pub struct BodyID {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

impl BodyID {
    pub fn new(index1: i32, world0: u16, generation: u16) -> Self {
        Self {
            index1,
            world0,
            generation,
        }
    }

    pub const NULL: BodyID = BodyID {
        index1: 0,
        world0: 0,
        generation: 0,
    };

    pub fn is_null(&self) -> bool {
        self.index1 == 0
    }

    pub fn stored(&self) -> u64 {
        ((self.index1 as u32 as u64) << 32)
            | ((self.world0 as u64) << 16)
            | self.generation as u64
    }

    pub fn from_stored(x: u64) -> Self {
        Self {
            index1: (x >> 32) as u32 as i32,
            world0: (x >> 16) as u16,
            generation: x as u16,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(C)]
pub struct ShapeID {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

impl ShapeID {
    pub fn new(index1: i32, world0: u16, generation: u16) -> Self {
        Self {
            index1,
            world0,
            generation,
        }
    }

    pub const NULL: ShapeID = ShapeID {
        index1: 0,
        world0: 0,
        generation: 0,
    };

    pub fn is_null(&self) -> bool {
        self.index1 == 0
    }

    pub fn stored(&self) -> u64 {
        ((self.index1 as u32 as u64) << 32)
            | ((self.world0 as u64) << 16)
            | self.generation as u64
    }

    pub fn from_stored(x: u64) -> Self {
        Self {
            index1: (x >> 32) as u32 as i32,
            world0: (x >> 16) as u16,
            generation: x as u16,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(C)]
pub struct JointID {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

impl JointID {
    pub fn new(index1: i32, world0: u16, generation: u16) -> Self {
        Self {
            index1,
            world0,
            generation,
        }
    }

    pub const NULL: JointID = JointID {
        index1: 0,
        world0: 0,
        generation: 0,
    };

    pub fn is_null(&self) -> bool {
        self.index1 == 0
    }

    pub fn stored(&self) -> u64 {
        ((self.index1 as u32 as u64) << 32)
            | ((self.world0 as u64) << 16)
            | self.generation as u64
    }

    pub fn from_stored(x: u64) -> Self {
        Self {
            index1: (x >> 32) as u32 as i32,
            world0: (x >> 16) as u16,
            generation: x as u16,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(C)]
pub struct ContactID {
    pub index1: i32,
    pub world0: u16,
    pub padding: i16,
    pub generation: u32,
}

impl ContactID {
    pub fn new(index1: i32, world0: u16, generation: u32) -> Self {
        Self {
            index1,
            world0,
            padding: 0,
            generation,
        }
    }

    pub const NULL: ContactID = ContactID {
        index1: 0,
        world0: 0,
        padding: 0,
        generation: 0,
    };

    pub fn is_null(&self) -> bool {
        self.index1 == 0
    }

    /// Store a contact id into three `u32` values.
    pub fn stored(&self) -> (u32, u32, u32) {
        (self.index1 as u32, self.world0 as u32, self.generation)
    }

    /// Load a contact id from three `u32` values.
    pub fn from_stored(values: (u32, u32, u32)) -> Self {
        Self {
            index1: values.0 as i32,
            world0: values.1 as u16,
            padding: 0,
            generation: values.2,
        }
    }
}
