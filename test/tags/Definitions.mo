package MyPackage
//      ^ definition.module

  connector Pin
  //        ^ definition.class
    Real v;
    flow Real i;
  end Pin;

  type Angle = enumeration(small, large);
  //   ^ definition.class
  //                       ^ definition.constant
  //                              ^ definition.constant

  function computeForce
  //       ^ definition.function
    input Real mass;
    output Real force;
  algorithm
    force := mass * 9.81;
  end computeForce;

  model Resistor
  //    ^ definition.class
    parameter Real R;
    Pin p, n;
  equation
    p.v - n.v = p.i * R;
  end Resistor;
end MyPackage;
