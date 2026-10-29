private import Foundation

public enum HeightFieldFileError: Error {
    case invalidData
}

extension HeightFieldDef {
    /// Write authored samples in the height-data text format.
    public func write(to path: String) throws {
        guard let text = HeightFieldText.encode(self) else { throw HeightFieldFileError.invalidData }
        try text.write(toFile: path, atomically: false, encoding: .utf8)
    }
}

extension HeightField {
    public static func load(from path: String) throws -> HeightField {
        let text = try String(contentsOfFile: path, encoding: .utf8)
        guard let definition = HeightFieldText.decode(text), let field = HeightField(definition) else {
            throw HeightFieldFileError.invalidData
        }
        return field
    }
}

enum HeightFieldText {
    static func encode(_ definition: HeightFieldDef) -> String? {
        let columns = Int(definition.countX), rows = Int(definition.countZ)
        guard columns >= 2, rows >= 2, definition.heights.count == columns * rows,
              definition.materialIndices.isEmpty || definition.materialIndices.count == (columns - 1) * (rows - 1) else { return nil }
        let locale = Locale(identifier: "en_US_POSIX")
        func decimal(_ value: Float) -> String { String(format: "%.9f", locale: locale, Double(value)) }
        var lines = ["\(columns) \(rows)",
                     "\(decimal(definition.scale.x)) \(decimal(definition.scale.y)) \(decimal(definition.scale.z))",
                     "\(decimal(definition.globalMinimumHeight)) \(decimal(definition.globalMaximumHeight))",
                     definition.clockwiseWinding ? "1" : "0"]
        lines.append(contentsOf: definition.heights.map(decimal))
        let materials = definition.materialIndices.isEmpty
            ? [UInt8](repeating: 0, count: (columns - 1) * (rows - 1)) : definition.materialIndices
        lines.append(contentsOf: materials.map { String($0) })
        return lines.joined(separator: "\n") + "\n"
    }

    static func decode(_ text: String) -> HeightFieldDef? {
        let tokens = text.split(whereSeparator: \.isWhitespace)
        guard tokens.count >= 8, let columns = Int32(tokens[0]), let rows = Int32(tokens[1]), columns >= 2, rows >= 2,
              let x = Float(tokens[2]), let y = Float(tokens[3]), let z = Float(tokens[4]),
              let minimum = Float(tokens[5]), let maximum = Float(tokens[6]), let clockwise = Int32(tokens[7]) else { return nil }
        let heightCount = Int(columns) * Int(rows)
        let cellCount = Int(columns - 1) * Int(rows - 1)
        guard tokens.count >= 8 + heightCount + cellCount else { return nil }
        var definition = HeightFieldDef()
        definition.countX = columns
        definition.countZ = rows
        definition.scale = Vec3(x, y, z)
        definition.globalMinimumHeight = minimum
        definition.globalMaximumHeight = maximum
        definition.clockwiseWinding = clockwise != 0
        for i in 0..<heightCount {
            guard let value = Float(tokens[8 + i]) else { return nil }
            definition.heights.append(value)
        }
        for i in 0..<cellCount {
            guard let value = Int32(tokens[8 + heightCount + i]) else { return nil }
            definition.materialIndices.append(UInt8(truncatingIfNeeded: value))
        }
        return definition
    }
}
