import XCTest
import SwiftTreeSitter
import TreeSitterModelica

final class TreeSitterModelicaTests: XCTestCase {
    func testCanLoadGrammar() throws {
        let parser = Parser()
        let language = Language(language: tree_sitter_modelica())
        XCTAssertNoThrow(try parser.setLanguage(language),
                         "Error loading Modelica grammar")
    }
}
