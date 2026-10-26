import Testing
@testable import Natsel

@Suite("Portable wide transpose")
struct PortableTransposeTests {
    @Test("portable transpose preserves lane bits and agrees with the platform implementation")
    func laneBits() {
        let rows = [
            FloatW(0, -0.0, .infinity, -.infinity),
            FloatW(1, -2, .leastNonzeroMagnitude, .greatestFiniteMagnitude),
            FloatW(Float(bitPattern: 0x7fc00001), 4, 5, 6),
            FloatW(7, 8, 9, 10),
        ]
        let actual = portableTranspose4(rows[0], rows[1], rows[2], rows[3])
        let platform = transpose4(rows[0], rows[1], rows[2], rows[3])
        let columns = [actual.0, actual.1, actual.2, actual.3]
        let expected = [platform.0, platform.1, platform.2, platform.3]
        for column in 0..<4 {
            for row in 0..<4 {
                #expect(columns[column][row].bitPattern == rows[row][column].bitPattern)
                #expect(columns[column][row].bitPattern == expected[column][row].bitPattern)
            }
        }
    }
}
