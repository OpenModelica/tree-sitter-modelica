;;;;; Highlights in the examples happen between these symbols: >...<


(ERROR) @error                                                      ;; when something is placed wrongfully
; error has to be added to the config.json file to be shown correctly ;;"error": {"color": "red", "bold": true, "underline": true}

(string_literal_expression) @string                                 ;; R(unit=>"Ohm"<)


;;; Comments
[
  (BLOCK_COMMENT)                                                   ;; >/* comment */<
  (comment)                                                         ;; >// comment<
  (description_string)                                              ;; model foo >"description"<
] @comment


;;; Numbers
[
  (UNSIGNED_INTEGER)                                                ;; >220<
  (UNSIGNED_REAL)                                                   ;; >3.14159<
] @number


;;; Types
(type_specifier) @type.builtin                                      ;; >Real< x


;;; Variables
(enumeration_literal identifier: (IDENT) @variable.builtin)         ;; type foo = enumeration(>bar<, >test<);
(declaration identifier: (IDENT) @variable.builtin)                 ;; Resistor >R1< (R=10);
(element_modification name: (name) @variable.parameter)             ;; Resistor R1( >R< =10);
(modification (expression (simple_expression (primary_expression (component_reference) @variable.parameter))))
;; e.g. annotation(derivative = >tsat_der<);
;; e.g. Resistor R1(R= >S<);


;;; Function Builtins
(within_clause name: (name)) @function.builtin                      ;; within >Modelica<
(extends_class_specifier identifier: (IDENT) @function.builtin)     ;; extends >foo<
(long_class_specifier identifier: (IDENT) @function.builtin)        ;; function >foo<
(short_class_specifier identifier: (IDENT) @function.builtin)       ;; function >foo<
(enumeration_class_specifier identifier: (IDENT) @function.builtin) ;; function >foo<
(extends_class_specifier endIdentifier: (IDENT) @function.builtin)  ;; end >foo<
(long_class_specifier endIdentifier: (IDENT) @function.builtin)     ;; end >foo<


;;; Imports
(import_clause name: (name) @module)                                ;; import C = >Modelica.Units.SI<;
(import_clause alias: (IDENT) @variable.builtin)                    ;; import >C< = Modelica.Units.SI;


;;; Constants
(logical_literal_expression) @constant.builtin                      ;; >true< / >false<


;;; Function Application
(function_application_statement functionReference: (component_reference) @function.builtin)
;; e.g. >assert<(ref==res, "unary minus failed");
(function_application_equation functionReference: (component_reference) @function.builtin)
;; e.g. >assert<(ref==res, "unary minus failed");
(function_application functionReference: (component_reference) @function.builtin)
;; e.g. Integer nX = >size<(X_boundary, 1);
;; e.g. final constant Integer Integer_inf = >OpenModelica.Internal<
;; e.g. Res:= >Modelica.ComplexMath.real<(c1)
(multiple_output_function_application_statement (component_reference) @function.builtin)
;; e.g. (a, b, nextEventScaled, last) := >getInterpolationCoefficients<(table, offset, startTime/timeScale)


;;; Function Call Arguments
(function_call_args (named_arguments (named_argument (IDENT) @variable.builtin)))
;; e.g. y = homotopy(>actual< = smooth(0));
;; e.g. y = homotopy(>simplified< = simplifiedExpr);

;;; Assignment Statements
(assignment_statement (component_reference) @variable.builtin)
;; e.g. >Kelvin< := Celsius - Modelica.Constants.T_zero;

;;; For Loop
(for_index (IDENT)) @variable.builtin
;; e.g. for >i< in >1:nPort< loop

;;; Connect Clause
(connect_clause (component_reference) @variable.builtin)
;; e.g. connect(furnace.port, evaporator.heatPort);

;;; External Functions
(language_specification value: (STRING) @module)                    ;; external >"builtin"< y = log(u);
(external_function identifier: (IDENT) @function.builtin)           ;; external "builtin" y = >log<(u);
(external_function (component_reference) @variable.builtin)         ;; external "builtin" >y< = log(u);





;;; Variables for Binary Expressions:
(simple_expression (binary_expression (primary_expression (component_reference) @variable.builtin)))

;-------------------------------------

;;; binary expressions:
;(expression (simple_expression (binary_expression (primary_expression (component_reference))))) @variable.builtin
;; e.g. o[1] := >pi<^0.25;

;;; binary expressions within a binary_expression that has a component_references/text:
;(binary_expression (simple_expression (binary_expression (primary_expression (component_reference) @variable.builtin))))
;; e.g. h := (n[1] + n[2]*pi + n[3]*>pi<^2 + n[4]*>pi<^3)*hstar;
;; e.g. T := sum(n[i]*(pi + 0.240)^>I<[i]*(eta - 0.615)^>J<[i] for i in 1:31)*Tstar;
;; e.g. S := C[6]/>deltaTREL<^(3/5);

;-------------------------------------



;;; Variables:
(simple_expression (primary_expression (component_reference) @variable.builtin))

;----------------------------------------

;;; every expression that starts with 'not', '+'' or '-'
;(unary_expression (simple_expression (primary_expression (component_reference) @variable.parameter)))
;; e.g. parameter Real uMin = ->uMax<;

;;; function call args:
;(function_call_args (function_arguments (expression (simple_expression (binary_expression (simple_expression (primary_expression (component_reference) @variable.builtin)))))))
;; e.g. assert(>ref<==>res<, "unary minus failed");

;(function_call_args (function_arguments (expression (simple_expression (primary_expression (component_reference) @variable.builtin)))))
;; e.g. Integer nX = size(>X_boundary<, 1);
;; e.g. if initType == Init.SteadyState then der(>x<) = 0;

;(function_call_args (named_arguments (named_argument (expression (simple_expression (primary_expression (component_reference) @variable.builtin))))))
;; e.g. y = homotopy(simplified = >simplifiedExpr<);

;;; assignment statements:
;(assignment_statement (expression (simple_expression (primary_expression (component_reference) @variable.builtin))))
;; algorithm  b := >offset<;

;;; if_equation with binary expression that has a component_references/text (both sides):
;(if_equation (expression (simple_expression (binary_expression (simple_expression (primary_expression (component_reference) @variable.builtin))))))
;; e.g. if >initType< == >Init.SteadyState< then

;;; if_equation that has a component_reference/text:
;(if_equation (expression (simple_expression (primary_expression (component_reference) @variable.builtin))))

;;; elseif_clause that has a component_reference/text (both sides):
;(else_if_equation_clause (expression (simple_expression (binary_expression (simple_expression (primary_expression (component_reference) @variable.builtin))))))
;; e.g. elseif >initType< == >Init.InitialState< then

;;; then-expression that has a component_reference/text:
;(if_expression (expression (simple_expression (primary_expression (component_reference) @variable.builtin))))

;;; binary expressions that has a component_references/text:
;(binary_expression (simple_expression (primary_expression (component_reference) @variable.builtin)))
;; e.g. Real mu_0 = 4*>pi<;
;; e.g. der(x) = >u</>T<;
;; e.g. y = >k<*(>x< + >u<);

;;; simple_equations:
;(simple_equation (simple_expression (primary_expression (component_reference) @variable.builtin)))
;; e.g. elseif initType == Init.InitialOutput then >y< = y_start;

;(simple_equation (expression (simple_expression (primary_expression (component_reference) @variable.builtin))))
;; e.g. elseif initType == Init.InitialOutput then y = >y_start<;

;(expression (simple_expression (primary_expression (component_reference) @variable.builtin)))
;; e.g. Blocks.Interfaces.RealInput q_F(unit = "MW") if >use_inputs<;

;;; expressions in parentheses:
;(output_expression_list (expression (simple_expression (primary_expression (component_reference) @variable.builtin))))
;; e.g. gamma := if aux.region == 3 then aux.T/(>aux.cv<) else -1/aux;
;; e.g. (>a<, >b<, >nextEventScaled<, >last<) := getInterpolationCoefficients

;----------------------------------------



;; KEYWORDS

[
  (class_prefixes)  ; >partial< / >class< / >connector< / >model< etc.
  (base_prefix)     ; >input< / >output<

  "within"
  "encapsulated"
  "external"

  ; component clause
  "flow"
  "stream"
  "constant"
  "discrete"
  "parameter"
  "input"
  "output"

  ; element modification
  "each"
  "final"
  "inner"
  "outer"

  ; redecleration
  "redeclare"
  "replaceable"

  ; expression

  "initial"
  "pure"
  "public"
  "protected"
  "import"
  "der"
  "enumeration"
  "extends"
  "annotation"
  "function"
  "equation"
  "algorithm"
  "end"
  "constrainedby"
] @keyword [

  ;; equation and statement

  "or"
  "and"
  "not"
  "break"
  "return"
  "connect"
  "for"
  "in"
  "loop"
  "if"
  "elseif"
  "then"
  "else"
  "when"
  "elsewhen"
  "while"
] @constant

;; PUNCTUATION BRACKET

[
"("
")"
"{"
"}"
"["
"]"
] @punctuation.bracket

;; PUNCTUATION DELIMITER

;[ "," ";" ":"] @punctuation.delimiter

;; OPERATOR

[
":"
"="
":="

"<"
"<="
">"
">="
"=="
"<>"
"+"
"-"
".+"
".-"
"*"
"/"
".*"
"./"
"^"
".^"
] @operator
