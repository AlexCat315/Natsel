//! Ported from Tests/NatselTests/Simulation/Human.swift.
//!
//! A scene builder rather than a test module: it constructs the humanoid ragdoll that the
//! determinism, parallelism and world-state suites drive. The bone table below is a captured
//! asset bake, so every literal is load-bearing.

use crate::base::constants as b3;
use crate::math::functions::{normalize_quat, offset_pos};
use crate::math::types::{Pos, Quat, Transform, Vec2, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Capsule;
use crate::types::ids::{BodyID, JointID};
use crate::types::joint_defs::{
    FilterJointDef, JointType, RevoluteJointDef, SphericalJointDef,
};
use crate::types::shape_def::ShapeDef;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i32)]
pub enum BoneId {
    Pelvis = 0,
    Spine01,
    Spine03,
    Neck,
    ThighL,
    CalfL,
    ThighR,
    CalfR,
    UpperArmL,
    LowerArmL,
    UpperArmR,
    LowerArmR,
}

impl BoneId {
    /// Swift's `BoneId.allCases`, in declaration order.
    pub const ALL: [BoneId; 12] = [
        BoneId::Pelvis,
        BoneId::Spine01,
        BoneId::Spine03,
        BoneId::Neck,
        BoneId::ThighL,
        BoneId::CalfL,
        BoneId::ThighR,
        BoneId::CalfR,
        BoneId::UpperArmL,
        BoneId::LowerArmL,
        BoneId::UpperArmR,
        BoneId::LowerArmR,
    ];

    pub const COUNT: usize = 12;

    #[inline]
    pub fn raw_value(self) -> i32 {
        self as i32
    }
}

#[derive(Clone, Copy, Debug)]
pub struct HumanBone {
    pub body_id: BodyID,
    pub joint_id: JointID,
    pub parent_index: i32,
    pub joint_type: JointType,
    pub local_frame_a: Transform,
    pub local_frame_b: Transform,
    pub reference_frame: Transform,
    pub swing_limit: f32,
    pub twist_limit: Vec2,
    pub joint_friction: f32,
}

impl Default for HumanBone {
    #[inline]
    fn default() -> Self {
        Self {
            body_id: BodyID::NULL,
            joint_id: JointID::NULL,
            parent_index: -1,
            joint_type: JointType::FilterJoint,
            local_frame_a: Transform::default(),
            local_frame_b: Transform::default(),
            reference_frame: Transform::default(),
            swing_limit: 0.0,
            twist_limit: Vec2::ZERO,
            joint_friction: 1.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Human {
    pub bones: Vec<HumanBone>,
    pub filter_joints: Vec<JointID>,
    pub friction_torque: f32,
    pub is_spawned: bool,
}

impl Default for Human {
    #[inline]
    fn default() -> Self {
        Self {
            bones: vec![HumanBone::default(); BoneId::COUNT],
            filter_joints: Vec::new(),
            friction_torque: 0.0,
            is_spawned: false,
        }
    }
}

impl Human {
    pub fn new() -> Self {
        Self::default()
    }
}

/// One `addBone` call's optional arguments, with Swift's declared defaults.
struct BoneSpec {
    name: &'static str,
    reference: Transform,
    group: i32,
    friction: f32,
    rolling: f32,
    capsules: &'static [(Vec3, Vec3, f32)],
    joint_type: JointType,
    local_frame_a: Transform,
    local_frame_b: Transform,
    swing_limit: f32,
    twist_limit: Vec2,
    joint_friction: f32,
    second_capsule_density: Option<f32>,
}

impl BoneSpec {
    #[inline]
    fn new(
        name: &'static str,
        reference: Transform,
        group: i32,
        friction: f32,
        rolling: f32,
        capsules: &'static [(Vec3, Vec3, f32)],
    ) -> Self {
        Self {
            name,
            reference,
            group,
            friction,
            rolling,
            capsules,
            joint_type: JointType::FilterJoint,
            local_frame_a: Transform::default(),
            local_frame_b: Transform::default(),
            swing_limit: 0.0,
            twist_limit: Vec2::ZERO,
            joint_friction: 1.0,
            second_capsule_density: None,
        }
    }
}

#[inline]
fn bone_transform(p: Vec3, q: Quat) -> Transform {
    Transform::new(p, q)
}

#[inline]
fn make_capsule(center1: Vec3, center2: Vec3, radius: f32) -> Capsule {
    Capsule::new(center1, center2, radius)
}

#[allow(clippy::too_many_arguments)]
fn add_bone(
    world: &mut World,
    human: &mut Human,
    body_def: &mut BodyDef,
    shape_def: &mut ShapeDef,
    default_density: f32,
    position: Pos,
    bone_id: BoneId,
    parent: Option<BoneId>,
    spec: BoneSpec,
) {
    let index = bone_id.raw_value() as usize;
    human.bones[index].parent_index = match parent {
        Some(parent) => parent.raw_value(),
        None => -1,
    };
    body_def.name = Some(spec.name.to_string());
    human.bones[index].reference_frame = spec.reference;
    body_def.rotation = spec.reference.q;
    body_def.position = offset_pos(position, spec.reference.p);
    human.bones[index].body_id = world.create_body(body_def);

    shape_def.filter.group_index = spec.group;
    shape_def.base_material.friction = spec.friction;
    shape_def.base_material.rolling_resistance = spec.rolling;

    for (capsule_index, capsule) in spec.capsules.iter().enumerate() {
        if capsule_index == 1 {
            if let Some(density) = spec.second_capsule_density {
                shape_def.density = density;
            }
        }
        let _ = world.create_capsule_shape(
            human.bones[index].body_id,
            shape_def,
            make_capsule(capsule.0, capsule.1, capsule.2),
        );
    }
    shape_def.density = default_density;

    human.bones[index].joint_type = spec.joint_type;
    human.bones[index].local_frame_a = spec.local_frame_a;
    human.bones[index].local_frame_b = spec.local_frame_b;
    human.bones[index].swing_limit = spec.swing_limit;
    human.bones[index].twist_limit = spec.twist_limit;
    human.bones[index].joint_friction = spec.joint_friction;
}

#[allow(clippy::too_many_arguments)]
pub fn create_human(
    human: &mut Human,
    world: &mut World,
    position: Pos,
    friction_torque: f32,
    hertz: f32,
    damping_ratio: f32,
    group_index: i32,
    user_data: Option<*mut core::ffi::c_void>,
    _colorize: bool,
) {
    assert!(!human.is_spawned);

    for i in 0..human.bones.len() {
        human.bones[i].body_id = BodyID::NULL;
        human.bones[i].joint_id = JointID::NULL;
        human.bones[i].joint_friction = 1.0;
        human.bones[i].parent_index = -1;
    }
    human.filter_joints.clear();
    human.friction_torque = friction_torque;

    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.user_data = user_data;

    let mut shape_def = ShapeDef::default();
    let default_density = shape_def.density;

    add_bone(
        world,
        human,
        &mut body_def,
        &mut shape_def,
        default_density,
        position,
        BoneId::Pelvis,
        None,
        BoneSpec::new(
            "pelvis",
            bone_transform(
                Vec3::new(0.000000, 0.996219, -0.023868),
                Quat::new(Vec3::new(1.0, 0.0, 0.0), 0.000000),
            ),
            0,
            0.6,
            0.1,
            &[(Vec3::new(0.040000, 0.000001, 0.000000), Vec3::new(-0.040000, -0.000001, 0.000000), 0.150000)],
        ),
    );

    let mut spec = BoneSpec::new(
        "spine_01",
        bone_transform(
            Vec3::new(0.000000, 1.017288, -0.024882),
            Quat::new(Vec3::new(1.0, 0.0, 0.0), 0.000000),
        ),
        -group_index,
        0.5,
        0.1,
        &[(Vec3::new(0.029876, -0.146581, 0.006260), Vec3::new(-0.029876, -0.146574, 0.006260), 0.145663)],
    );
    spec.joint_type = JointType::SphericalJoint;
    spec.local_frame_a = bone_transform(
        Vec3::new(0.000000, -0.021069, 0.001014),
        Quat::new(Vec3::new(-0.642737, 0.0, 0.0), -0.766087),
    );
    spec.local_frame_b = bone_transform(
        Vec3::new(0.0, 0.0, 0.0),
        Quat::new(Vec3::new(-0.707107, 0.0, 0.0), -0.707107),
    );
    spec.swing_limit = 35.0 * b3::DEG_TO_RAD;
    spec.twist_limit = Vec2::new(-17.5 * b3::DEG_TO_RAD, 17.5 * b3::DEG_TO_RAD);
    add_bone(
        world,
        human,
        &mut body_def,
        &mut shape_def,
        default_density,
        position,
        BoneId::Spine01,
        Some(BoneId::Pelvis),
        spec,
    );

    let mut spec = BoneSpec::new(
        "spine_03",
        bone_transform(
            Vec3::new(0.000000, 1.267766, -0.022320),
            Quat::new(Vec3::new(1.0, 0.0, 0.0), 0.000000),
        ),
        0,
        0.5,
        0.1,
        &[(Vec3::new(0.063996, -0.117434, -0.040199), Vec3::new(-0.063996, -0.117432, -0.040199), 0.165004)],
    );
    spec.joint_type = JointType::SphericalJoint;
    spec.local_frame_a = bone_transform(
        Vec3::new(0.000000, -0.250478, -0.002562),
        Quat::new(Vec3::new(-0.642766, 0.0, 0.0), -0.766063),
    );
    spec.local_frame_b = bone_transform(
        Vec3::new(0.0, 0.0, 0.0),
        Quat::new(Vec3::new(-0.707107, 0.0, 0.0), -0.707107),
    );
    spec.swing_limit = 35.0 * b3::DEG_TO_RAD;
    spec.twist_limit = Vec2::new(-17.5 * b3::DEG_TO_RAD, 17.5 * b3::DEG_TO_RAD);
    add_bone(
        world,
        human,
        &mut body_def,
        &mut shape_def,
        default_density,
        position,
        BoneId::Spine03,
        Some(BoneId::Spine01),
        spec,
    );

    let mut spec = BoneSpec::new(
        "neck_01",
        bone_transform(
            Vec3::new(0.000000, 1.498783, 0.024462),
            Quat::new(Vec3::new(0.987879, 0.0, 0.0), 0.155228),
        ),
        0,
        0.2,
        0.05,
        &[(Vec3::new(0.000000, -0.212340, 0.000000), Vec3::new(0.000000, -0.087340, 0.000000), 0.110000)],
    );
    spec.joint_type = JointType::SphericalJoint;
    spec.local_frame_a = bone_transform(
        Vec3::new(0.000000, -0.231017, -0.046781),
        Quat::new(Vec3::new(-0.516157, -0.000002, 0.000003), -0.856494),
    );
    spec.local_frame_b = bone_transform(
        Vec3::new(0.0, 0.0, 0.0),
        Quat::new(Vec3::new(-0.707108, -0.000002, 0.000002), -0.707106),
    );
    spec.swing_limit = 30.0 * b3::DEG_TO_RAD;
    spec.twist_limit = Vec2::new(-17.5 * b3::DEG_TO_RAD, 17.5 * b3::DEG_TO_RAD);
    add_bone(
        world,
        human,
        &mut body_def,
        &mut shape_def,
        default_density,
        position,
        BoneId::Neck,
        Some(BoneId::Spine03),
        spec,
    );

    let mut spec = BoneSpec::new(
        "thigh_l",
        bone_transform(
            Vec3::new(0.092175, 0.971562, -0.011177),
            Quat::new(Vec3::new(0.471412, 0.529510, -0.499888), -0.497495),
        ),
        -group_index,
        0.6,
        0.1,
        &[(Vec3::new(-0.047269, 0.000001, 0.000000), Vec3::new(-0.379769, 0.000005, 0.000000), 0.091539)],
    );
    spec.joint_type = JointType::SphericalJoint;
    spec.local_frame_a = bone_transform(
        Vec3::new(0.092175, 0.024656, -0.012691),
        Quat::new(Vec3::new(-0.460186, -0.365368, -0.637662), 0.498118),
    );
    spec.local_frame_b = bone_transform(
        Vec3::new(0.0, 0.0, 0.0),
        Quat::new(Vec3::new(-0.499997, -0.500003, 0.500002), 0.499998),
    );
    spec.swing_limit = 30.0 * b3::DEG_TO_RAD;
    spec.twist_limit = Vec2::new(-10.0 * b3::DEG_TO_RAD, 10.0 * b3::DEG_TO_RAD);
    add_bone(
        world,
        human,
        &mut body_def,
        &mut shape_def,
        default_density,
        position,
        BoneId::ThighL,
        Some(BoneId::Pelvis),
        spec,
    );

    let mut spec = BoneSpec::new(
        "calf_l",
        bone_transform(
            Vec3::new(0.118720, 0.534541, -0.035362),
            Quat::new(Vec3::new(0.521188, 0.480597, -0.546380), -0.445935),
        ),
        0,
        0.1,
        0.0,
        &[
            (Vec3::new(-0.445000, 0.000001, 0.000000), Vec3::new(0.005000, -0.000001, 0.000000), 0.080000),
            (Vec3::new(-0.456371, 0.129321, 0.000013), Vec3::new(-0.359977, 0.014394, 0.000013), 0.070000),
        ],
    );
    spec.joint_type = JointType::RevoluteJoint;
    spec.local_frame_a = bone_transform(
        Vec3::new(-0.438494, -0.000175, 0.000000),
        Quat::new(Vec3::new(0.344866, -0.938652, 0.000002), -0.000008),
    );
    spec.local_frame_b = bone_transform(
        Vec3::new(0.0, 0.0, 0.0),
        Quat::new(Vec3::new(-0.008182, -0.999967, 0.000001), -0.000007),
    );
    spec.twist_limit = Vec2::new(-30.0 * b3::DEG_TO_RAD, 30.0 * b3::DEG_TO_RAD);
    spec.second_capsule_density = Some(0.5 * default_density);
    add_bone(
        world,
        human,
        &mut body_def,
        &mut shape_def,
        default_density,
        position,
        BoneId::CalfL,
        Some(BoneId::ThighL),
        spec,
    );

    let mut spec = BoneSpec::new(
        "thigh_r",
        bone_transform(
            Vec3::new(-0.092175, 0.971562, -0.011177),
            Quat::new(Vec3::new(-0.497495, 0.499888, 0.529510), -0.471412),
        ),
        -group_index,
        0.6,
        0.1,
        &[(Vec3::new(0.047269, -0.000001, 0.000000), Vec3::new(0.379769, -0.000005, 0.000000), 0.092587)],
    );
    spec.joint_type = JointType::SphericalJoint;
    spec.local_frame_a = bone_transform(
        Vec3::new(-0.092175, 0.024656, -0.012691),
        Quat::new(Vec3::new(0.380666, 0.491846, 0.488644), -0.611889),
    );
    spec.local_frame_b = bone_transform(
        Vec3::new(0.0, 0.0, 0.0),
        Quat::new(Vec3::new(-0.500002, 0.499998, -0.499998), 0.500002),
    );
    spec.swing_limit = 30.0 * b3::DEG_TO_RAD;
    spec.twist_limit = Vec2::new(-10.0 * b3::DEG_TO_RAD, 10.0 * b3::DEG_TO_RAD);
    add_bone(
        world,
        human,
        &mut body_def,
        &mut shape_def,
        default_density,
        position,
        BoneId::ThighR,
        Some(BoneId::Pelvis),
        spec,
    );

    let mut spec = BoneSpec::new(
        "calf_r",
        bone_transform(
            Vec3::new(-0.118720, 0.534541, -0.035362),
            Quat::new(Vec3::new(-0.445935, 0.546380, 0.480597), -0.521188),
        ),
        0,
        0.1,
        0.0,
        &[
            (Vec3::new(0.445000, -0.000002, 0.000003), Vec3::new(-0.005000, 0.000000, 0.000003), 0.080000),
            (Vec3::new(0.369113, -0.006830, 0.000017), Vec3::new(0.465507, -0.121757, 0.000017), 0.070000),
        ],
    );
    spec.joint_type = JointType::RevoluteJoint;
    spec.local_frame_a = bone_transform(
        Vec3::new(0.438494, 0.000175, 0.000000),
        Quat::new(Vec3::new(0.000007, 0.000005, -0.344860), -0.938654),
    );
    spec.local_frame_b = bone_transform(
        Vec3::new(0.0, 0.0, 0.0),
        Quat::new(Vec3::new(0.000006, 0.000005, 0.008188), -0.999966),
    );
    spec.twist_limit = Vec2::new(-30.0 * b3::DEG_TO_RAD, 30.0 * b3::DEG_TO_RAD);
    spec.second_capsule_density = Some(0.5 * default_density);
    add_bone(
        world,
        human,
        &mut body_def,
        &mut shape_def,
        default_density,
        position,
        BoneId::CalfR,
        Some(BoneId::ThighR),
        spec,
    );

    let mut spec = BoneSpec::new(
        "upperarm_l",
        bone_transform(
            Vec3::new(0.185817, 1.443630, 0.031306),
            Quat::new(Vec3::new(0.560220, -0.307179, 0.366233), -0.676512),
        ),
        0,
        0.6,
        0.1,
        &[(Vec3::new(0.296038, 0.004386, -0.000794), Vec3::new(-0.016086, 0.004384, -0.016127), 0.065000)],
    );
    spec.joint_type = JointType::SphericalJoint;
    spec.local_frame_a = bone_transform(
        Vec3::new(0.185817, -0.175865, -0.053625),
        Quat::new(Vec3::new(-0.559466, -0.213784, -0.670199), 0.438323),
    );
    spec.local_frame_b = bone_transform(
        Vec3::new(0.0, 0.0, 0.0),
        Quat::new(Vec3::new(-0.500953, -0.502617, -0.509318), -0.486844),
    );
    spec.swing_limit = 45.0 * b3::DEG_TO_RAD;
    spec.twist_limit = Vec2::new(-20.0 * b3::DEG_TO_RAD, 20.0 * b3::DEG_TO_RAD);
    spec.joint_friction = 0.8;
    add_bone(
        world,
        human,
        &mut body_def,
        &mut shape_def,
        default_density,
        position,
        BoneId::UpperArmL,
        Some(BoneId::Spine03),
        spec,
    );

    let mut spec = BoneSpec::new(
        "lowerarm_l",
        bone_transform(
            Vec3::new(0.347683, 1.193455, 0.029378),
            Quat::new(Vec3::new(0.406736, -0.493012, 0.089135), -0.763911),
        ),
        0,
        0.3,
        0.1,
        &[(Vec3::new(0.312329, -0.008256, -0.016454), Vec3::new(0.033153, 0.008259, -0.002741), 0.060000)],
    );
    spec.joint_type = JointType::RevoluteJoint;
    spec.local_frame_a = bone_transform(
        Vec3::new(0.297979, 0.000361, 0.000000),
        Quat::new(Vec3::new(0.916726, 0.399352, 0.000839), -0.011456),
    );
    spec.local_frame_b = bone_transform(
        Vec3::new(0.0, 0.0, 0.0),
        Quat::new(Vec3::new(0.999922, -0.003857, 0.004659), -0.010908),
    );
    spec.twist_limit = Vec2::new(-35.0 * b3::DEG_TO_RAD, 35.0 * b3::DEG_TO_RAD);
    spec.joint_friction = 0.06;
    add_bone(
        world,
        human,
        &mut body_def,
        &mut shape_def,
        default_density,
        position,
        BoneId::LowerArmL,
        Some(BoneId::UpperArmL),
        spec,
    );

    let mut spec = BoneSpec::new(
        "upperarm_r",
        bone_transform(
            Vec3::new(-0.185817, 1.443630, 0.031306),
            Quat::new(Vec3::new(0.676512, 0.366233, 0.307179), 0.560220),
        ),
        0,
        0.6,
        0.1,
        &[(Vec3::new(-0.300237, -0.000001, -0.000319), Vec3::new(0.011887, 0.000001, 0.015013), 0.065000)],
    );
    spec.joint_type = JointType::SphericalJoint;
    spec.local_frame_a = bone_transform(
        Vec3::new(-0.185817, -0.175865, -0.053625),
        Quat::new(Vec3::new(0.213793, 0.559449, 0.438334), -0.670204),
    );
    spec.local_frame_b = bone_transform(
        Vec3::new(0.0, 0.0, 0.0),
        Quat::new(Vec3::new(-0.509319, 0.486843, 0.500954), -0.502617),
    );
    spec.swing_limit = 45.0 * b3::DEG_TO_RAD;
    spec.twist_limit = Vec2::new(-20.0 * b3::DEG_TO_RAD, 20.0 * b3::DEG_TO_RAD);
    spec.joint_friction = 0.8;
    add_bone(
        world,
        human,
        &mut body_def,
        &mut shape_def,
        default_density,
        position,
        BoneId::UpperArmR,
        Some(BoneId::Spine03),
        spec,
    );

    let mut spec = BoneSpec::new(
        "lowerarm_r",
        bone_transform(
            Vec3::new(-0.347683, 1.193455, 0.029378),
            Quat::new(Vec3::new(0.763911, 0.089135, 0.493012), 0.406736),
        ),
        0,
        0.3,
        0.1,
        &[(Vec3::new(-0.312649, -0.000003, 0.016471), Vec3::new(-0.032986, 0.000000, 0.002732), 0.060000)],
    );
    spec.joint_type = JointType::RevoluteJoint;
    spec.local_frame_a = bone_transform(
        Vec3::new(-0.297979, -0.000361, 0.000000),
        Quat::new(Vec3::new(-0.405444, 0.914059, 0.010541), 0.000852),
    );
    spec.local_frame_b = bone_transform(
        Vec3::new(0.0, 0.0, 0.0),
        Quat::new(Vec3::new(-0.002794, 0.999936, 0.010042), 0.004364),
    );
    spec.twist_limit = Vec2::new(-35.0 * b3::DEG_TO_RAD, 35.0 * b3::DEG_TO_RAD);
    spec.joint_friction = 0.06;
    add_bone(
        world,
        human,
        &mut body_def,
        &mut shape_def,
        default_density,
        position,
        BoneId::LowerArmR,
        Some(BoneId::UpperArmR),
        spec,
    );

    for i in 1..BoneId::COUNT {
        let mut bone = human.bones[i];
        let parent = human.bones[bone.parent_index as usize];

        bone.local_frame_a.q = normalize_quat(bone.local_frame_a.q);
        bone.local_frame_b.q = normalize_quat(bone.local_frame_b.q);

        if bone.joint_type == JointType::RevoluteJoint {
            let mut def = RevoluteJointDef::default();
            def.base.body_id_a = parent.body_id;
            def.base.body_id_b = bone.body_id;
            def.base.local_frame_a = bone.local_frame_a;
            def.base.local_frame_b = bone.local_frame_b;
            def.enable_limit = true;
            def.lower_angle = bone.twist_limit.x;
            def.upper_angle = bone.twist_limit.y;
            def.enable_spring = hertz > 0.0;
            def.hertz = hertz;
            def.damping_ratio = damping_ratio;
            def.enable_motor = true;
            def.max_motor_torque = bone.joint_friction * friction_torque;
            bone.joint_id = world.create_revolute_joint(&def);
        } else if bone.joint_type == JointType::SphericalJoint {
            let mut def = SphericalJointDef::default();
            def.base.body_id_a = parent.body_id;
            def.base.body_id_b = bone.body_id;
            def.base.local_frame_a = bone.local_frame_a;
            def.base.local_frame_b = bone.local_frame_b;
            def.enable_cone_limit = true;
            def.cone_angle = bone.swing_limit;
            def.enable_twist_limit = true;
            def.lower_twist_angle = bone.twist_limit.x;
            def.upper_twist_angle = bone.twist_limit.y;
            def.enable_spring = hertz > 0.0;
            def.hertz = hertz;
            def.damping_ratio = damping_ratio;
            def.enable_motor = true;
            def.max_motor_torque = bone.joint_friction * friction_torque;
            bone.joint_id = world.create_spherical_joint(&def);
        }
        human.bones[i] = bone;
    }

    // Disable some collisions.
    let mut filter_def = FilterJointDef::default();
    filter_def.base.body_id_a = human.bones[BoneId::ThighL.raw_value() as usize].body_id;
    filter_def.base.body_id_b = human.bones[BoneId::ThighR.raw_value() as usize].body_id;
    human.filter_joints.push(world.create_filter_joint(&filter_def));
    filter_def.base.body_id_a = human.bones[BoneId::Neck.raw_value() as usize].body_id;
    filter_def.base.body_id_b = human.bones[BoneId::UpperArmL.raw_value() as usize].body_id;
    human.filter_joints.push(world.create_filter_joint(&filter_def));
    filter_def.base.body_id_a = human.bones[BoneId::Neck.raw_value() as usize].body_id;
    filter_def.base.body_id_b = human.bones[BoneId::UpperArmR.raw_value() as usize].body_id;
    human.filter_joints.push(world.create_filter_joint(&filter_def));

    human.is_spawned = true;
}
