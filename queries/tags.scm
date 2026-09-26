;;;;; Definitions
;;
;; See https://tree-sitter.github.io/tree-sitter/4-code-navigation.html for
;; the standard `@definition.*` / `@reference.*` vocabulary used here.

;;; Function definitions (including operator functions)
;; e.g. function >computeForce< ... end computeForce;
;; e.g. operator function >'+'< ... end '+';
(class_definition
  classPrefixes: (class_prefixes function: "function")
  classSpecifier: [
    (long_class_specifier identifier: (IDENT) @name)
    (short_class_specifier identifier: (IDENT) @name)
  ]) @definition.function

;;; Package definitions (namespaces)
;; e.g. package >Modelica< ... end Modelica;
(class_definition
  classPrefixes: (class_prefixes package: "package")
  classSpecifier: (long_class_specifier identifier: (IDENT) @name)) @definition.module

;;; Every other class-like definition: model, block, class, connector,
;;; record, type (including enumerations and derivative types) and plain
;;; operator declarations.
;; e.g. model >Resistor< ... end Resistor;
;; e.g. connector >Pin< ... end Pin;
;; e.g. type >Voltage< = Real(unit = "V");
;; e.g. type >Angle< = enumeration(small, large);
(class_definition
  classPrefixes: (class_prefixes !function !package)
  classSpecifier: [
    (long_class_specifier identifier: (IDENT) @name)
    (short_class_specifier identifier: (IDENT) @name)
    (derivative_class_specifier identifier: (IDENT) @name)
    (enumeration_class_specifier identifier: (IDENT) @name)
    (extends_class_specifier identifier: (IDENT) @name)
  ]) @definition.class

;;; Enumeration literals
;; e.g. type Angle = enumeration(>small<, >large<);
(enumeration_literal
  identifier: (IDENT) @name) @definition.constant

;;;;; References

;;; Function/method calls
;; e.g. R.v = >sin<(time);
;; e.g. y = >Modelica.Math.sin<(x);
(function_application
  functionReference: (component_reference identifier: (IDENT) @name)) @reference.call
(function_application_statement
  functionReference: (component_reference identifier: (IDENT) @name)) @reference.call
(function_application_equation
  functionReference: (component_reference identifier: (IDENT) @name)) @reference.call
(multiple_output_function_application_statement
  functionReference: (component_reference identifier: (IDENT) @name)) @reference.call

;;; Class references: extending a class, typing a component, constraining a
;;; replaceable declaration, or a short class definition's base type.
;; e.g. extends >Modelica.Icons.Example<;
;; e.g. >Resistor< R1;
;; e.g. type Voltage = >Real<(unit = "V");
;; e.g. replaceable Real x constrainedby >Real<;
(extends_clause
  typeSpecifier: (type_specifier name: (name identifier: (IDENT) @name))) @reference.class
(component_clause
  typeSpecifier: (type_specifier name: (name identifier: (IDENT) @name))) @reference.class
(short_class_specifier
  typeSpecifier: (type_specifier name: (name identifier: (IDENT) @name))) @reference.class
(constraining_clause
  typeSpecifier: (type_specifier name: (name identifier: (IDENT) @name))) @reference.class
