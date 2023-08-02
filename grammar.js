/*
 * OMFrontend.js
 * Copyright (C) 2022 Perpetual Labs, Ltd.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @author Mohamad Omar Nachawati <omar@perpetuallabs.io>
 */

module.exports = grammar({
    name: "modelica",
    conflicts: $ => [
        [$.component_list],
        [$.equation_list],
        [$.function_arguments]
    ],
    extras: $ => [
        $.BLOCK_COMMENT,
        $.LINE_COMMENT,
        $._SPACE
    ],
    word: $ => $.IDENT,
    rules: {

        // A.2.1 Stored Definition – Within

        stored_definitions: $ => seq(
            optional($.BOM),
            optional(seq("within", optional(field("within", $.name)), ";")),
            repeat(field("storedDefinitions", $.stored_definition))
        ),

        stored_definition: $ => seq(
            optional(field("final", "final")),
            field("classDefinition", $.class_definition), ";"
        ),

        // A.2.2 Class Definition

        class_definition: $ => seq(
            optional(field("encapsulated", "encapsulated")),
            field("classPrefixes", $.class_prefixes),
            field("classSpecifier", $._class_specifier)
        ),

        class_prefixes: $ => seq(
            optional(field("partial", "partial")),
            choice(
                field("block", "block"),
                field("class", "class"),
                seq(
                    optional(field("expandable", "expandable")),
                    field("connector", "connector")),
                seq(
                    optional(choice(field("impure", "impure"), field("pure", "pure"))),
                    optional(field("operator", "operator")),
                    field("function", "function")),
                field("model", "model"),
                field("operator", "operator"),
                field("package", "package"),
                seq(
                    optional(field("operator", "operator")),
                    field("record", "record")),
                field("type", "type")
            )
        ),

        _class_specifier: $ => choice(
            $.derivative_class_specifier,
            $.enumeration_class_specifier,
            $.extends_class_specifier,
            $.long_class_specifier,
            $.short_class_specifier
        ),

        derivative_class_specifier: $ => seq(
            field("identifier", $.IDENT), "=", "der",
            "(", field("typeSpecifier", $.type_specifier), ",",
            field("argument", $.IDENT), optional(seq(",", field("argument", $.IDENT))), ")",
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        enumeration_class_specifier: $ => seq(
            field("identifier", $.IDENT), "=", "enumeration", "(",
            choice(
                optional(field("enumerationLiterals", $.enum_list)),
                field("unspecified", ":")
            ),
            ")",
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        enum_list: $ => seq(
            field("enumerationLiteral", $.enumeration_literal), repeat(seq(",", field("enumerationLiteral", $.enumeration_literal)))
        ),

        enumeration_literal: $ => seq(
            field("identifier", $.IDENT),
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        extends_class_specifier: $ => seq(
            "extends",
            field("identifier", $.IDENT),
            optional(field("classModification", $.class_modification)),
            optional(field("descriptionString", $.description_string)),
            optional($.element_list),
            repeat(choice(
                $.public_element_list,
                $.protected_element_list,
                $.algorithm_section,
                $.equation_section)),
            optional(field("externalClause", $.external_clause)),
            optional(seq(field("annotationClause", $.annotation_clause), ";")),
            "end",
            field("endIdentifier", $.IDENT)
        ),

        long_class_specifier: $ => seq(
            field("identifier", $.IDENT),
            optional(field("descriptionString", $.description_string)),
            optional($.element_list),
            repeat(choice(
                $.public_element_list,
                $.protected_element_list,
                $.algorithm_section,
                $.equation_section
            )),
            optional(field("externalClause", $.external_clause)),
            optional(seq(field("annotationClause", $.annotation_clause), ";")),
            "end",
            field("endIdentifier", $.IDENT)
        ),

        short_class_specifier: $ => seq(
            field("identifier", $.IDENT),
            "=",
            optional(field("basePrefix", $.base_prefix)),
            field("typeSpecifier", $.type_specifier),
            optional(field("subscripts", $.array_subscripts)),
            optional(field("classModification", $.class_modification)),
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        external_clause: $ => seq(
            "external",
            optional(field("languageSpecification", $.language_specification)),
            optional(field("externalFunction", $.external_function)),
            optional(field("annotationClause", $.annotation_clause)),
            ";"
        ),

        language_specification: $ => field("value", $.STRING),

        external_function: $ => seq(
            optional(seq(field("componentReference", $.component_reference), "=")),
            field("identifier", $.IDENT), "(", optional(field("expressions", $.expression_list)), ")"
        ),

        base_prefix: $ => choice("input", "output"),

        element_list: $ => seq(
            repeat1(seq(field("element", $._element), ";"))
        ),

        public_element_list: $ => seq(
            "public", repeat(seq(field("element", $._element), ";"))
        ),

        protected_element_list: $ => seq(
            "protected", repeat(seq(field("element", $._element), ";"))
        ),

        _element: $ => choice(
            $.extends_clause,
            $.import_clause,
            $.named_element
        ),

        named_element: $ => seq(
            optional(field("redeclare", "redeclare")),
            optional(field("final", "final")),
            optional(field("inner", "inner")),
            optional(field("outer", "outer")),
            optional(field("replaceable", "replaceable")),
            choice(
                field("classDefinition", $.class_definition),
                field("componentClause", $.component_clause)
            ),
            optional(seq(
                field("constrainingClause", $.constraining_clause),
                optional(field("descriptionString", $.description_string)),
                optional(field("annotationClause", $.annotation_clause))
            )),
        ),

        import_clause: $ => seq(
            "import",
            choice(
                seq(field("alias", $.IDENT), "=", field("name", $.name)),
                seq(
                    field("name", $.name),
                    optional(
                        seq(".",
                            choice(
                                field("wildcard", "*"),
                                seq("{", field("imports", $.import_list), "}")
                            )
                        )
                    )
                )
            ),
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause)),
        ),

        import_list: $ => seq(
            field("import", $.IDENT), repeat(seq(",", field("import", $.IDENT)))
        ),

        // A.2.3 Extends

        extends_clause: $ => seq(
            "extends",
            field("typeSpecifier", $.type_specifier),
            optional(field("classModification", $.class_modification)),
            optional(field("annotationClause", $.annotation_clause)),
        ),

        constraining_clause: $ => seq(
            "constrainedby",
            field("typeSpecifier", $.type_specifier),
            optional(field("classModification", $.class_modification))
        ),

        // A.2.4 Component Clause

        component_clause: $ => seq(
            optional(choice(field("flow", "flow"), field("stream", "stream"))),
            optional(choice(field("constant", "constant"), field("discrete", "discrete"), field("parameter", "parameter"))),
            optional(choice(field("input", "input"), field("output", "output"))),
            field("typeSpecifier", $.type_specifier),
            optional(field("subscripts", $.array_subscripts)),
            field("componentDeclarations", $.component_list)
        ),

        component_list: $ => seq(
            field("componentDeclaration", $.component_declaration),
            repeat(seq(",", field("componentDeclaration", $.component_declaration)))
        ),

        component_declaration: $ => seq(
            field("declaration", $.declaration),
            optional(seq("if", field("condition", $._expression))),
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        declaration: $ => seq(
            field("identifier", $.IDENT),
            optional(field("subscripts", $.array_subscripts)),
            optional(field("modification", $.modification))
        ),

        // A.2.5 Modification

        modification: $ => choice(
            seq(field("classModification", $.class_modification), optional(seq("=", field("expression", $._expression)))),
            seq(choice("=", ":="), field("expression", $._expression))
        ),

        class_modification: $ => seq(
            "(", optional(field("arguments", $.argument_list)), ")"
        ),

        argument_list: $ => seq(
            field("argument", $._argument), repeat(seq(",", field("argument", $._argument)))
        ),

        _argument: $ => choice(
            $.element_modification,
            $._element_redeclaration
        ),

        element_modification: $ => seq(
            optional(field("each", "each")),
            optional(field("final", "final")),
            field("name", $.name),
            optional(field("modification", $.modification)),
            optional(field("descriptionString", $.description_string))
        ),

        _element_redeclaration: $ => choice(
            $.class_redeclaration,
            $.component_redeclaration
        ),

        class_redeclaration: $ => seq(
            optional(field("redeclare", "redeclare")),
            optional(field("each", "each")),
            optional(field("final", "final")),
            optional(field("replaceable", "replaceable")),
            field("classDefinition", $.short_class_definition),
            optional(field("constrainingClause", $.constraining_clause))
        ),

        component_redeclaration: $ => seq(
            optional(field("redeclare", "redeclare")),
            optional(field("each", "each")),
            optional(field("final", "final")),
            optional(field("replaceable", "replaceable")),
            field("componentClause", $.component_clause),
            optional(field("constrainingClause", $.constraining_clause))
        ),

        short_class_definition: $ => seq(
            field("classPrefixes", $.class_prefixes),
            field("classSpecifier", choice(
                $.enumeration_class_specifier,
                $.short_class_specifier
            ))
        ),

        // A.2.6 Equations

        equation_section: $ => prec.right(seq(
            optional(field("initial", "initial")), "equation",
            optional(field("equations", $.equation_list))
        )),

        algorithm_section: $ => prec.right(seq(
            optional(field("initial", "initial")), "algorithm",
            optional(field("statements", $.statement_list))
        )),

        equation_list: $ => repeat1(seq(field("equation", $._equation), ";")),

        _equation: $ => choice(
            $.connect_clause,
            $.for_equation,
            $.function_application_equation,
            $.if_equation,
            $.simple_equation,
            $.when_equation
        ),

        statement_list: $ => repeat1(seq(field("statement", $._statement), ";")),

        _statement: $ => choice(
            $.assignment_statement,
            $.break_statement,
            $.for_statement,
            $.function_application_statement,
            $.if_statement,
            $.multiple_output_function_application_statement,
            $.return_statement,
            $.when_statement,
            $.while_statement
        ),

        assignment_statement: $ => seq(
            field("targetExpression", $.component_reference), ":=", field("sourceExpression", $._expression),
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        break_statement: $ => seq(
            "break",
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        connect_clause: $ => seq(
            "connect", "(", field("component1", $.component_reference), ",", field("component2", $.component_reference), ")",
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        for_equation: $ => seq(
            "for", field("indices", $.for_indices),
            "loop", optional(field("equations", $.equation_list)),
            "end", "for",
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        for_statement: $ => seq(
            "for", field("indices", $.for_indices),
            "loop", optional(field("statements", $.statement_list)),
            "end", "for",
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        for_indices: $ => seq(field("index", $.for_index), repeat(seq(",", field("index", $.for_index)))),

        for_index: $ => seq(field("identifier", $.IDENT), optional(seq("in", field("expression", $._expression)))),

        function_application_equation: $ => seq(
            field("functionReference", $.component_reference),
            field("arguments", $.function_call_args),
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        function_application_statement: $ => seq(
            field("functionReference", $.component_reference),
            field("arguments", $.function_call_args),
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        if_equation: $ => seq(
            "if", field("condition", $._expression),
            "then", optional(field("thenEquations", $.equation_list)),
            optional(field("elseIfClauses", $.else_if_equation_clause_list)),
            optional(seq("else", field("elseEquations", $.equation_list))),
            "end", "if",
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        else_if_equation_clause_list: $ => repeat1($.else_if_equation_clause),

        else_if_equation_clause: $ => prec.right(seq(
            "elseif", field("condition", $._expression),
            "then", optional(field("equations", $.equation_list))
        )),

        if_statement: $ => seq(
            "if", field("condition", $._expression),
            "then", optional(field("thenStatements", $.statement_list)),
            optional(field("elseIfClauses", $.else_if_statement_clause_list)),
            optional(seq("else", field("elseStatements", $.statement_list))),
            "end", "if",
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        else_if_statement_clause_list: $ => repeat1($.else_if_statement_clause),

        else_if_statement_clause: $ => prec.right(seq(
            "elseif", field("condition", $._expression),
            "then", optional(field("statements", $.statement_list))
        )),

        multiple_output_function_application_statement: $ => seq(
            field("targetExpression", $.parenthesized_expression), ":=",
            field("functionReference", $.component_reference),
            field("arguments", $.function_call_args),
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        return_statement: $ => seq(
            "return",
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        simple_equation: $ => seq(
            field("expression1", $._simple_expression), "=", field("expression2", $._expression),
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause)),
        ),

        when_equation: $ => seq(
            "when", field("condition", $._expression),
            "then", optional(field("equations", $.equation_list)),
            optional(field("elseWhenClauses", $.else_when_equation_clause_list)),
            "end", "when",
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        else_when_equation_clause_list: $ => repeat1($.else_when_equation_clause),

        else_when_equation_clause: $ => prec.right(seq(
            "elsewhen", field("condition", $._expression),
            "then", optional(field("equations", $.equation_list))
        )),

        when_statement: $ => seq(
            "when", field("condition", $._expression),
            "then", optional(field("statements", $.statement_list)),
            optional(field("elseWhenClauses", $.else_when_statement_clause_list)),
            "end", "when",
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        else_when_statement_clause_list: $ => repeat1($.else_when_statement_clause),

        else_when_statement_clause: $ => prec.right(seq(
            "elsewhen", field("condition", $._expression),
            "then", optional(field("statements", $.statement_list))
        )),

        while_statement: $ => seq(
            "while", field("condition", $._expression),
            "loop", optional(field("statements", $.statement_list)),
            "end", "while",
            optional(field("descriptionString", $.description_string)),
            optional(field("annotationClause", $.annotation_clause))
        ),

        // A.2.7 Expressions

        _expression: $ => choice(
            $.if_expression,
            $.range_expression,
            $._simple_expression
        ),

        if_expression: $ => seq(
            "if", field("condition", $._expression),
            "then", field("thenExpression", $._expression),
            repeat(field("elseIfClause", $.else_if_clause)),
            "else", field("elseExpression", $._expression)
        ),

        else_if_clause: $ => seq(
            "elseif", field("condition", $._expression),
            "then", field("expression", $._expression)
        ),

        range_expression: $ => choice(
            seq(field("startExpression", $._simple_expression), ":",
                field("stepExpression", $._simple_expression), ":",
                field("stopExpression", $._simple_expression)),
            seq(field("startExpression", $._simple_expression), ":",
                field("stopExpression", $._simple_expression))
        ),

        _simple_expression: $ => choice(
            $.unary_expression,
            $.binary_expression,
            $._primary_expression
        ),

        binary_expression: $ => choice(
            prec.left(1, seq(field("operand1", $._simple_expression), field("operator", "or"), field("operand2", $._simple_expression))),
            prec.left(2, seq(field("operand1", $._simple_expression), field("operator", "and"), field("operand2", $._simple_expression))),
            prec.right(3, seq(field("operand1", $._simple_expression), field("operator", "<"), field("operand2", $._simple_expression))),
            prec.right(3, seq(field("operand1", $._simple_expression), field("operator", "<="), field("operand2", $._simple_expression))),
            prec.right(3, seq(field("operand1", $._simple_expression), field("operator", ">"), field("operand2", $._simple_expression))),
            prec.right(3, seq(field("operand1", $._simple_expression), field("operator", ">="), field("operand2", $._simple_expression))),
            prec.right(3, seq(field("operand1", $._simple_expression), field("operator", "=="), field("operand2", $._simple_expression))),
            prec.right(3, seq(field("operand1", $._simple_expression), field("operator", "<>"), field("operand2", $._simple_expression))),
            prec.left(4, seq(field("operand1", $._simple_expression), field("operator", "+"), field("operand2", $._simple_expression))),
            prec.left(4, seq(field("operand1", $._simple_expression), field("operator", "-"), field("operand2", $._simple_expression))),
            prec.left(4, seq(field("operand1", $._simple_expression), field("operator", ".+"), field("operand2", $._simple_expression))),
            prec.left(4, seq(field("operand1", $._simple_expression), field("operator", ".-"), field("operand2", $._simple_expression))),
            prec.left(5, seq(field("operand1", $._simple_expression), field("operator", "*"), field("operand2", $._simple_expression))),
            prec.left(5, seq(field("operand1", $._simple_expression), field("operator", "/"), field("operand2", $._simple_expression))),
            prec.left(5, seq(field("operand1", $._simple_expression), field("operator", ".*"), field("operand2", $._simple_expression))),
            prec.left(5, seq(field("operand1", $._simple_expression), field("operator", "./"), field("operand2", $._simple_expression))),
            prec.right(6, seq(field("operand1", $._primary_expression), field("operator", "^"), field("operand2", $._primary_expression))),
            prec.right(6, seq(field("operand1", $._primary_expression), field("operator", ".^"), field("operand2", $._primary_expression))),
        ),

        unary_expression: $ => prec(7, choice(
            seq(field("operator", "not"), field("operand", $._simple_expression)),
            seq(field("operator", "+"), field("operand", $._simple_expression)),
            seq(field("operator", "-"), field("operand", $._simple_expression))
        )),

        _primary_expression: $ => choice(
            $.array_comprehension,
            $.array_concatenation,
            $.array_constructor,
            $.component_reference,
            $.end_expression,
            $.function_application,
            $._literal_expression,
            $.parenthesized_expression
        ),

        end_expression: $ => "end",

        array_comprehension: $ => seq(
            "{", field("expression", $._expression), "for", field("indices", $.for_indices), "}"
        ),

        array_concatenation: $ => seq(
            "[", field("expressions", $.expression_list), repeat(seq(";", field("expressions", $.expression_list))), "]"
        ),

        array_constructor: $ => seq(
            "{", field("arguments", $.array_arguments), "}"
        ),

        array_arguments: $ => seq(
            field("argument", $._expression), repeat(seq(",", field("argument", $._expression)))
        ),

        parenthesized_expression: $ => seq(
            "(", optional(field("expressions", $.output_expression_list)), ")"
        ),

        function_application: $ => seq(
            choice(
                field("functionReference", $.component_reference),
                field("der", "der"), field("initial", "initial"), field("pure", "pure")
            ),
            field("arguments", $.function_call_args)
        ),

        _literal_expression: $ => choice(
            $.logical_literal_expression,
            $.string_literal_expression,
            $._unsigned_number_literal_expression,
        ),

        logical_literal_expression: $ => choice("false", "true"),

        string_literal_expression: $ => $.STRING,

        _unsigned_number_literal_expression: $ => choice(
            $.unsigned_integer_literal_expression,
            $.unsigned_real_literal_expression,
        ),

        unsigned_integer_literal_expression: $ => $.UNSIGNED_INTEGER,

        unsigned_real_literal_expression: $ => $.UNSIGNED_REAL,

        type_specifier: $ => seq(
            optional(field("global", ".")), field("name", $.name)
        ),

        name: $ => prec.left(choice(
            seq(field("qualifier", $.name), ".", field("identifier", $.IDENT)),
            field("identifier", $.IDENT)
        )),

        component_reference: $ => prec.left(choice(
            seq(
                field("qualifier", $.component_reference), ".", field("identifier", $.IDENT), optional(field("subscripts", $.array_subscripts))
            ),
            seq(
                optional(field("global", ".")), field("identifier", $.IDENT), optional(field("subscripts", $.array_subscripts))
            )
        )),

        function_call_args: $ => choice(
            seq("(", field("arguments", $.function_arguments), optional(seq(",", field("namedArguments", $.named_arguments))), ")"),
            seq("(", optional(field("namedArguments", $.named_arguments)), ")"),
            seq("(", field("index", $._expression), "for", field("indices", $.for_indices), ")")
        ),

        function_arguments: $ => seq(
            field("argument", $._function_argument), repeat(seq(",", field("argument", $._function_argument)))
        ),

        named_arguments: $ => seq(
            field("namedArgument", $.named_argument), repeat(seq(",", field("namedArgument", $.named_argument)))
        ),

        named_argument: $ => seq(
            field("identifier", $.IDENT), "=", field("expression", $._function_argument)
        ),

        _function_argument: $ => choice(
            $.function_partial_application,
            $._expression
        ),

        function_partial_application: $ => seq(
            "function", field("typeSpecifier", $.type_specifier), "(", optional(field("namedArguments", $.named_arguments)), ")"
        ),

        output_expression_list: $ => choice(
            seq(field("expression", $._expression), repeat(seq(field("comma", ","), optional(field("expression", $._expression))))),
            repeat1(seq(field("comma", ","), optional(field("expression", $._expression))))
        ),

        expression_list: $ => seq(
            field("expression", $._expression), repeat(seq(",", field("expression", $._expression)))
        ),

        array_subscripts: $ => seq(
            "[", field("subscript", $.subscript), repeat(seq(",", field("subscript", $.subscript))), "]"
        ),

        subscript: $ => choice(
            ":", field("expression", $._expression)
        ),

        description_string: $ => seq(
            field("value", $.STRING), repeat(seq("+", field("value", $.STRING)))
        ),

        annotation_clause: $ => seq(
            "annotation", field("classModification", $.class_modification)
        ),

        // A.1 Lexical conventions

        BOM: $ => /\u00EF\u00BB\u00BF/,

        IDENT: $ => token(choice(
            seq(/[_a-zA-Z]/, repeat(choice(/[0-9]/, /[_a-zA-Z]/))),
            seq("’", repeat(choice(
                /[_a-zA-Z]/, /[0-9]/, "!", "#", "$", "%", "&", "(", ")",
                "*", "+", ",", "-", ".", "/", ":", ";", "<", ">", "=",
                "?", "@", "[", "]", "^", "{", "}", "|", "~", " ", "\"",
                seq("\\", choice("’", "'", "\"", "?", "\\", "a", "b", "f", "n", "r", "t", "v")))), "’"))),

        STRING: $ => token(seq("\"", repeat(choice(/[^"\\]/,
            seq("\\", choice("’", "'", "\"", "?", "\\", "a", "b", "f", "n", "r", "t", "v")))), "\"")),

        UNSIGNED_INTEGER: $ => /[0-9]+/,

        UNSIGNED_REAL: $ => token(choice(
            seq(/[0-9]+/, ".", optional(/[0-9]+/)),
            seq(/[0-9]+/, optional(seq(".", optional(/[0-9]+/))), choice("e", "E"), optional(choice("+", "-")), /[0-9]+/),
            seq(".", /[0-9]+/, optional(seq(choice("e", "E"), optional(choice("+", "-")), /[0-9]+/)))
        )),

        BLOCK_COMMENT: $ => token(
            seq("/*", /.*/, "*/")
        ),

        LINE_COMMENT: $ => token(
            seq("//", /[^\r\n]*/)
        ),

        _SPACE: $ => /\s+/

    }

});
