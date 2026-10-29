//! Keyframed skeletal animation (Jolt's SkeletalAnimation). Each track animates one joint, found by
//! name, with keyframes of local translation and rotation. Sampling interpolates linearly and
//! spherically and blends into a pose with a weight, so several animations can be layered.
//!
//! Translated from `Sources/Natsel/Skeleton/SkeletalAnimation.swift`.

use crate::math::functions::{clamp_float, lerp};
use crate::math::types::{Quat, Vec3};
use crate::skeleton::skeleton::{slerp_quat, SkeletonJointState, SkeletonPose};
use std::cmp::Ordering;

/// Swift's nested `SkeletalAnimation.Keyframe`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Keyframe {
    pub time: f32,
    pub translation: Vec3,
    pub rotation: Quat,
}

impl Keyframe {
    pub fn new(time: f32, translation: Vec3, rotation: Quat) -> Self {
        Self {
            time,
            translation,
            rotation,
        }
    }
}

/// Swift's nested `SkeletalAnimation.Track`.
#[derive(Clone, Debug)]
pub struct Track {
    pub joint_name: String,
    /// Sorted by time. Swift's `private(set) var keyframes`; mutated only in [`Track::new`].
    keyframes: Vec<Keyframe>,
}

impl Track {
    pub fn new(joint_name: String, keyframes: Vec<Keyframe>) -> Self {
        // Swift's `keyframes.sorted { $0.time < $1.time }`. The comparator mirrors `<`: equal
        // times compare Equal (Swift's unstable sort leaves their order unspecified; Rust's
        // stable sort keeps the input order, which is within that contract) and unordered
        // (NaN) times compare Equal like `<` does.
        let mut keyframes = keyframes;
        keyframes.sort_by(|a, b| {
            if a.time < b.time {
                Ordering::Less
            } else if b.time < a.time {
                Ordering::Greater
            } else {
                Ordering::Equal
            }
        });
        Self {
            joint_name,
            keyframes,
        }
    }

    /// Swift's `public private(set) var keyframes`.
    #[inline]
    pub fn keyframes(&self) -> &[Keyframe] {
        &self.keyframes
    }

    /// The joint state at `time`, clamped to the track's first and last keyframes.
    pub fn sample(&self, time: f32) -> Option<SkeletonJointState> {
        let first = self.keyframes.first()?;
        let last = self.keyframes.last()?;
        if time <= first.time {
            return Some(SkeletonJointState::new(first.translation, first.rotation));
        }
        if time >= last.time {
            return Some(SkeletonJointState::new(last.translation, last.rotation));
        }
        // Binary search for the last keyframe at or before `time`.
        let mut low = 0usize;
        let mut high = self.keyframes.len() - 1;
        while high - low > 1 {
            let mid = (low + high) / 2;
            if self.keyframes[mid].time <= time {
                low = mid;
            } else {
                high = mid;
            }
        }
        let a = self.keyframes[low];
        let b = self.keyframes[high];
        let span = b.time - a.time;
        let t = if span > 0.0 {
            (time - a.time) / span
        } else {
            0.0
        };
        Some(SkeletonJointState::new(
            lerp(a.translation, b.translation, t),
            slerp_quat(a.rotation, b.rotation, t),
        ))
    }
}

#[derive(Clone, Debug)]
pub struct SkeletalAnimation {
    pub tracks: Vec<Track>,
    /// A looping animation wraps the sample time into [0, duration).
    pub is_looping: bool,
}

impl Default for SkeletalAnimation {
    fn default() -> Self {
        Self {
            tracks: Vec::new(),
            is_looping: false,
        }
    }
}

impl SkeletalAnimation {
    pub fn new(tracks: Vec<Track>, is_looping: bool) -> Self {
        Self { tracks, is_looping }
    }

    /// Time of the last keyframe.
    pub fn duration(&self) -> f32 {
        // Swift's `tracks.reduce(0) { max($0, $1.keyframes.last?.time ?? 0) }`; stdlib `max(x, y)`
        // returns `y` only when `y > x`, else `x`.
        let mut duration: f32 = 0.0;
        for track in &self.tracks {
            let last = match track.keyframes.last() {
                Some(keyframe) => keyframe.time,
                None => 0.0,
            };
            duration = if last > duration { last } else { duration };
        }
        duration
    }

    /// Sample at `time` and blend into the pose's joint states: weight 1 replaces them, smaller
    /// weights interpolate toward the sample. Joints without a track keep their state. Call
    /// `calculate_joint_matrices` afterwards to update model space. Swift's `weight: Float = 1`
    /// default is passed explicitly by callers.
    pub fn sample(&self, time: f32, pose: &mut SkeletonPose, weight: f32) {
        let mut t = time;
        let length = self.duration();
        if self.is_looping && length > 0.0 {
            t %= length;
            if t < 0.0 {
                t += length;
            }
        }
        let w = clamp_float(weight, 0.0, 1.0);
        for track in &self.tracks {
            let (Some(joint), Some(state)) = (
                pose.skeleton.joint_index(&track.joint_name),
                track.sample(t),
            ) else {
                continue;
            };
            if w >= 1.0 {
                pose.joint_states[joint as usize] = state;
            } else {
                let current = pose.joint_states[joint as usize];
                pose.joint_states[joint as usize] = SkeletonJointState::new(
                    lerp(current.translation, state.translation, w),
                    slerp_quat(current.rotation, state.rotation, w),
                );
            }
        }
    }
}
