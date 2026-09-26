package EdgeCases
//      ^ definition.module

  type AngularVelocity = der(Angle, time);
//     ^ definition.class

  model extends BaseModel
//              ^ definition.class
  end BaseModel;

  operator function add
//                  ^ definition.function
    input Real a;
    input Real b;
    output Real c;
  algorithm
    c := a + b;
  end add;

  class Helper
//      ^ definition.class
  end Helper;

  block Controller
//      ^ definition.class
  end Controller;

  record Point
//       ^ definition.class
    Real x;
    Real y;
  end Point;
end EdgeCases;
