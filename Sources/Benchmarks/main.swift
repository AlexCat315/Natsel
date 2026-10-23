// Swift benchmark harness mirroring benchmark/main.c and shared/benchmarks.c.

import Foundation
import Natsel

struct Benchmark {
    let name: String
    let create: (World) -> Void
    let step: ((World, Int) -> Void)?
    let destroy: (() -> Void)?
    let totalStepCount: Int
}

var benchmarks: [Benchmark] = [
    Benchmark(name: "large_pyramid", create: createLargePyramid, step: nil, destroy: nil, totalStepCount: 200),
    Benchmark(name: "many_pyramids", create: createManyPyramids, step: nil, destroy: nil, totalStepCount: 100),
    Benchmark(name: "joint_grid", create: createJointGrid, step: nil, destroy: nil, totalStepCount: 100),
    Benchmark(name: "spinner", create: createSpinner, step: nil, destroy: destroySpinner, totalStepCount: 800),
    Benchmark(name: "sleep", create: createSleep, step: stepSleep, destroy: nil, totalStepCount: 300),
    Benchmark(name: "trees25", create: createTrees25, step: nil, destroy: destroyTrees, totalStepCount: 500),
    Benchmark(name: "trees50", create: createTrees50, step: nil, destroy: destroyTrees, totalStepCount: 500),
    Benchmark(name: "trees100", create: createTrees100, step: nil, destroy: destroyTrees, totalStepCount: 500),
    Benchmark(name: "washer", create: createWasher, step: nil, destroy: nil, totalStepCount: 1000),
    Benchmark(name: "convex_pile", create: createConvexPile, step: nil, destroy: nil, totalStepCount: 500),
]

var singleBenchmark: Int = -1
var runCount = 4
var recordSteps = false

var args = CommandLine.arguments
var i = 1
while i < args.count {
    let arg = args[i]
    if arg.hasPrefix("-b=") || arg.hasPrefix("--benchmark=") {
        let value = String(arg.split(separator: "=", maxSplits: 1)[1])
        if let index = Int(value) {
            singleBenchmark = index
        } else if let index = benchmarks.firstIndex(where: { $0.name.lowercased() == value.lowercased() }) {
            singleBenchmark = index
        } else {
            print("No benchmark named \"\(value)\"")
            exit(1)
        }
    } else if arg.hasPrefix("-r=") || arg.hasPrefix("--repeats=") {
        runCount = Int(String(arg.split(separator: "=", maxSplits: 1)[1])) ?? runCount
    } else if arg == "-s" || arg == "--record-steps" {
        recordSteps = true
    } else if arg == "-l" || arg == "--list" {
        for (index, benchmark) in benchmarks.enumerated() {
            print("\(index) \(benchmark.name)")
        }
        exit(0)
    }
    i += 1
}

print("Starting benchmarks")
print("======================================")

for (index, benchmark) in benchmarks.enumerated() {
    if singleBenchmark != -1 && index != singleBenchmark {
        continue
    }

    let stepCount = benchmark.totalStepCount
    print("benchmark: \(benchmark.name), steps = \(stepCount)")

    var minTime = Double.greatestFiniteMagnitude

    for runIndex in 0..<runCount {
        let world = World()
        if ProcessInfo.processInfo.environment["NATSEL_WIDE"] == "1" { world.enableWideSolver = true }
        benchmark.create(world)

        if benchmark.step != nil {
            benchmark.step!(world, 0)
        }

        world.step(1.0 / 60.0, subStepCount: 4)

        var acc = Profile()
        let start = DispatchTime.now().uptimeNanoseconds
        for stepIndex in 1..<stepCount {
            if benchmark.step != nil {
                benchmark.step!(world, stepIndex)
            }
            world.step(1.0 / 60.0, subStepCount: 4)
            let pr = world.profile
            acc.pairs += pr.pairs; acc.collide += pr.collide
            acc.prepareConstraints += pr.prepareConstraints; acc.warmStart += pr.warmStart
            acc.solveImpulses += pr.solveImpulses; acc.relaxImpulses += pr.relaxImpulses
            acc.storeImpulses += pr.storeImpulses
            acc.integrateVelocities += pr.integrateVelocities; acc.integratePositions += pr.integratePositions
            acc.transforms += pr.transforms; acc.hitEvents += pr.hitEvents
            acc.sleepIslands += pr.sleepIslands; acc.constraints += pr.constraints
        }
        let end = DispatchTime.now().uptimeNanoseconds
        let n = Float(stepCount - 1)
        if ProcessInfo.processInfo.environment["NATSEL_PHASE"] == "1" {
            print(String(format: "AVG %@ pairs=%.2f collide=%.2f prepare=%.2f warm=%.2f push=%.2f solve=%.2f store=%.2f intV=%.2f intP=%.2f finalize=%.2f hit=%.2f sleep=%.2f substeps=%.2f",
                benchmark.name, acc.pairs/n, acc.collide/n, acc.prepareConstraints/n, acc.warmStart/n,
                acc.solveImpulses/n, acc.relaxImpulses/n, acc.storeImpulses/n,
                acc.integrateVelocities/n, acc.integratePositions/n, acc.transforms/n,
                acc.hitEvents/n, acc.sleepIslands/n, acc.constraints/n))
        }

        let ms = Double(end - start) / 1_000_000.0
        if ProcessInfo.processInfo.environment["NATSEL_PHASE"] == "1" {
            let p = world.profile
            print("PHASE steps=\(stepCount) pairs=\(p.pairs) collide=\(p.collide) prepare=\(p.prepareConstraints) warm=\(p.warmStart) push=\(p.solveImpulses) solve=\(p.relaxImpulses) store=\(p.storeImpulses) intVel=\(p.integrateVelocities) intPos=\(p.integratePositions) substeps=\(p.constraints) hit=\(p.hitEvents) jnt=\(p.jointEvents) finalize=\(p.transforms) refit=\(p.refit) sleep=\(p.sleepIslands) total=\(ms)")
        }
        print("run \(runIndex) : \(ms) (ms)")
        minTime = Swift.min(minTime, ms)

        if runIndex == runCount - 1 {
            worldCounters = (world.bodyCount, 0, world.contactCount)
        }
        benchmark.destroy?()
    }

    let counters = worldCounters
    print("body \(counters.body) / shape \(counters.shape) / contact \(counters.contact)")
    print("")
    print("\(benchmark.name).csv: threads,ms 1,\(minTime)")
}

print("======================================")
print("All benchmarks complete!")
