within Modelica;
// <- keyword
//     ^ function.builtin
class SimpleMath
// <- keyword
//    ^ function.builtin
  "Simple Model"
//^ comment
  extends Modelica.Icons.Example;
//^ keyword
//        ^ type.builtin

  parameter Real a "Parameter a";
//^ keyword ^ type.builtin
//               ^ variable.builtin
//                 ^ comment
  parameter Real b "Parameter b";
//^ keyword ^ type.builtin
//               ^ variable.builtin
//                 ^ comment
  Real result "The Result";
//^ type.builtin
//     ^ variable.builtin
//            ^ comment

equation
// <- keyword
  result = a * b; // Multiplication of the Parameters
//^ variable.builtin
//       ^ operator
//         ^ variable.builtin
//           ^ operator
//             ^ variable.builtin
//                ^ comment
end SimpleMath;
// <- keyword
//  ^ function.builtin