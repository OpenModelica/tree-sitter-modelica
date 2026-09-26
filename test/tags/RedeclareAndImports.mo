package P
//      ^ definition.module
  redeclare function extends someFunction
//                           ^ definition.function
  algorithm
    y := x;
  end someFunction;
end P;

model M
//    ^ definition.class
  redeclare package extends SomePackage
//                          ^ definition.module
  end SomePackage;

  import Modelica.Blocks.Interfaces;
//                       ^ reference.class
  import C = Modelica.Units.SI;
//                          ^ reference.class
  import Modelica.Blocks.Interfaces.{A, B};
//                       ^ reference.class
//                                   ^ reference.class
//                                      ^ reference.class
end M;

package Q = SomeOtherPackage;
//      ^ definition.module
//          ^ reference.class
