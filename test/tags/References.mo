model Resistor
  extends Modelica.Icons.Example;
//                       ^ reference.class
  parameter Real R;
//          ^ reference.class
equation
  force = computeForce(R);
//        ^ reference.call
end Resistor;
