// swift-tools-version: 6.4
import PackageDescription

let package = Package(
    name: "Natsel",
    platforms: [.macOS(.v26), .iOS(.v26)],
    products: [
        .library(name: "Natsel", targets: ["Natsel"]),
    ],
    targets: [
        .target(
            name: "Natsel",
            swiftSettings: [
                .enableUpcomingFeature("ApproachableConcurrency"),
                .unsafeFlags(["-Ounchecked", "-enforce-exclusivity=unchecked"], .when(configuration: .release)),
            ]
        ),
        .testTarget(
            name: "NatselTests",
            dependencies: ["Natsel"],
            swiftSettings: [
                .enableUpcomingFeature("ApproachableConcurrency"),
            ]
        ),
        .executableTarget(
            name: "Benchmarks",
            dependencies: ["Natsel"],
            swiftSettings: [
                .enableUpcomingFeature("ApproachableConcurrency"),
            ]
        ),
    ]
)
