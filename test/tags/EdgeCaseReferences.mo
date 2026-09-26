model RefEdgeCases
  replaceable Real x constrainedby Real;
//            ^ reference.class
//                                 ^ reference.class
  Foo f(redeclare Real y);
//^ reference.class
//                ^ reference.class
equation
  y = Modelica.Math.sin(x);
//                  ^ reference.call
end RefEdgeCases;
