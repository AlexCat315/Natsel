// Keyframed skeletal animation (Jolt's SkeletalAnimation). Each track animates one joint, found by
// name, with keyframes of local translation and rotation. Sampling interpolates linearly and
// spherically and blends into a pose with a weight, so several animations can be layered.

public struct SkeletalAnimation: Sendable {
    public struct Keyframe: Sendable, Equatable {
        public var time: Float
        public var translation: Vec3
        public var rotation: Quat

        public init(time: Float, translation: Vec3, rotation: Quat) {
            self.time = time
            self.translation = translation
            self.rotation = rotation
        }
    }

    public struct Track: Sendable {
        public var jointName: String
        /// Sorted by time.
        public private(set) var keyframes: [Keyframe]

        public init(jointName: String, keyframes: [Keyframe]) {
            self.jointName = jointName
            self.keyframes = keyframes.sorted { $0.time < $1.time }
        }

        /// The joint state at `time`, clamped to the track's first and last keyframes.
        public func sample(_ time: Float) -> SkeletonJointState? {
            guard let first = keyframes.first, let last = keyframes.last else { return nil }
            if time <= first.time { return SkeletonJointState(translation: first.translation, rotation: first.rotation) }
            if time >= last.time { return SkeletonJointState(translation: last.translation, rotation: last.rotation) }
            // Binary search for the last keyframe at or before `time`.
            var low = 0
            var high = keyframes.count - 1
            while high - low > 1 {
                let mid = (low + high) / 2
                if keyframes[mid].time <= time { low = mid } else { high = mid }
            }
            let a = keyframes[low]
            let b = keyframes[high]
            let span = b.time - a.time
            let t = span > 0 ? (time - a.time) / span : 0
            return SkeletonJointState(translation: lerp(a.translation, b.translation, t),
                                      rotation: slerpQuat(a.rotation, b.rotation, t))
        }
    }

    public var tracks: [Track]
    /// A looping animation wraps the sample time into [0, duration).
    public var isLooping: Bool

    public init(tracks: [Track] = [], isLooping: Bool = false) {
        self.tracks = tracks
        self.isLooping = isLooping
    }

    /// Time of the last keyframe.
    public var duration: Float {
        tracks.reduce(0) { max($0, $1.keyframes.last?.time ?? 0) }
    }

    /// Sample at `time` and blend into the pose's joint states: weight 1 replaces them, smaller
    /// weights interpolate toward the sample. Joints without a track keep their state. Call
    /// `calculateJointMatrices` afterwards to update model space.
    public func sample(_ time: Float, _ pose: inout SkeletonPose, weight: Float = 1) {
        var t = time
        let length = duration
        if isLooping && length > 0 {
            t = t.truncatingRemainder(dividingBy: length)
            if t < 0 { t += length }
        }
        let w = clampFloat(weight, 0, 1)
        for track in tracks {
            guard let joint = pose.skeleton.jointIndex(track.jointName), let state = track.sample(t) else { continue }
            if w >= 1 {
                pose.jointStates[joint] = state
            } else {
                let current = pose.jointStates[joint]
                pose.jointStates[joint] = SkeletonJointState(
                    translation: lerp(current.translation, state.translation, w),
                    rotation: slerpQuat(current.rotation, state.rotation, w))
            }
        }
    }
}
