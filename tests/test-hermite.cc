
#include <boost/test/unit_test.hpp>
#include <hpp/constraints/generic-transformation.hh>
#include <hpp/constraints/locked-joint.hh>
#include <hpp/constraints/relative-com.hh>
#include <hpp/core/path-validation-report.hh>
#include <hpp/core/steering-method/straight.hh>
#include <hpp/manipulation/constraint-set.hh>
#include <hpp/manipulation/steering-method/graph.hh>
#include <hpp/pinocchio/liegroup-element.hh>
#include <hpp/pinocchio/urdf/util.hh>
#include "hpp/manipulation/device.hh"
#include "hpp/manipulation/graph-path-validation.hh"
#include "hpp/manipulation/graph/edge.hh"
#include "hpp/manipulation/graph/graph.hh"
#include "hpp/manipulation/graph/state-selector.hh"
#include "hpp/manipulation/graph/state.hh"
#include "hpp/manipulation/problem.hh"
#include "hpp/constraints/differentiable-function.hh"
#include "hpp/core/path/hermite.hh"
//#include "hpp/corbaserver/problem.hh"

using hpp::core::SteeringMethodPtr_t;
using hpp::core::steeringMethod::Straight;

typedef std::vector<hpp::manipulation::graph::GraphComponentPtr_t>
    GraphComponents_t;

namespace hpp_test {
using hpp::core::Configuration_t;

hpp::manipulation::DevicePtr_t robot;

Configuration_t q1, q2;

void initialize(bool ur5) {
  robot = hpp::manipulation::Device::create("test-robot");
  hpp::manipulation::ProblemPtr_t problem(
      hpp::manipulation::Problem::create(robot));
  if (ur5) {
    hpp::pinocchio::urdf::loadModel(
        robot, 0, "ur5/", "anchor",
        "package://example-robot-data/robots/ur_description/urdf/"
        "ur5_joint_limited_robot.urdf",
        "package://example-robot-data/robots/ur_description/srdf/"
        "ur5_joint_limited_robot.srdf");
  }

  q1 = Configuration_t::Zero(6);
  q2 = Configuration_t::Zero(6);
  q1 << 0.0,0.0,0.0,0.0,0.0,0.0;
  q2 << 1.0,1.0,1.0,1.0,1.0,1.0;
}
}  // namespace hpp_test


BOOST_AUTO_TEST_CASE(Initialization) {
  using namespace hpp_test;
  using hpp::constraints::ComparisonTypes_t;
  using hpp::constraints::Equality;
  using hpp::constraints::EqualToZero;
  using hpp::constraints::ImplicitPtr_t;
  using hpp::constraints::LockedJoint;
  using hpp::manipulation::graph::Edge;
  using hpp::manipulation::graph::EdgePtr_t;
  using hpp::pinocchio::LiegroupElement;
  using hpp::pinocchio::LiegroupSpace;
  using hpp_test::robot;
  using hpp::constraints::DifferentiableFunctionPtr_t;

  initialize(true);
  hpp::core::ConstraintSetPtr_t constraint = hpp::core::ConstraintSet::create(robot, "contrainte vide");
  hpp::core::interval_t timeRange1(0,1);
  hpp::core::interval_t timeRange2(0,0.5);
  hpp::core::interval_t timeRange3(0,2);
  /*
  std::vector<int, std::allocator<int>> frame1 = {0,0,0,0,0,0,1};
  std::vector<int> frame2 = {0,0,0,0,0,0,1};
  vector<bool> mask (true, 6);
  std::string name("test-hermite-constraint");
  DifferentiableFunctionPtr_t func =
      buildGenericFunc<hpp::constraints::OutputR3xSO3Bit |
                        hpp::constraints::PositionBit |
                        hpp::constraints::OrientationBit>(
          robot, name, "", "ur_robot/tool0",
          hpp::corbaServer::toTransform3f(frame1), hpp::corbaServer::toTransform3f(frame2),
          mask);

  hpp::constraints::ImplicitPtr_t constraint = hpp::constraints::Implicit::create(func, 6 * hpp::constraints::EqualToZero);

  // Check that states refuse parameterizable constraints
  constraint->comparisonType(ComparisonTypes_t(6, Equality));
  */
  hpp::core::path::HermitePtr_t path1 = hpp::core::path::Hermite::create_with_timeRange(robot, q1, q2, constraint, timeRange1);
  hpp::core::path::HermitePtr_t path2 = hpp::core::path::Hermite::create_with_timeRange(robot, q1, q2, constraint, timeRange2);
  hpp::core::path::HermitePtr_t path3 = hpp::core::path::Hermite::create_with_timeRange(robot, q1, q2, constraint, timeRange3);
  hpp::core::path::HermitePtr_t path_no_TR = hpp::core::path::Hermite::create(robot, q1, q2, constraint);


  hpp::core::Configuration_t q_NO_TR (path_no_TR->outputSize());
  bool suc_NO_TR = path1->impl_compute(q_NO_TR, 1);
  BOOST_CHECK(suc_NO_TR);
  cout << q_NO_TR << endl << path1->end() << endl;
  BOOST_CHECK(q_NO_TR == path_no_TR->end());

  hpp::core::Configuration_t q_1_ (path1->outputSize());
  bool suc1_ = path1->impl_compute(q_1_, timeRange1.second);
  BOOST_CHECK(suc1_);
  cout << q_1_ << endl << path1->end() << endl;
  BOOST_CHECK(q_1_ == path1->end());

  hpp::core::Configuration_t q_1 (path1->outputSize());
  bool suc1 = path1->impl_compute(q_1, timeRange1.second /2);
  BOOST_CHECK(suc1);
  cout << q_1 << endl << path1->end() << endl;
  BOOST_CHECK(q_1 == path1->end());
  
  hpp::core::Configuration_t q_2(path2->outputSize());
  bool suc2 = path2->impl_compute(q_2, timeRange2.second /2);
  BOOST_CHECK(suc2);
  cout << q_2 << endl << path2->end() << endl;
  BOOST_CHECK(q_2 == path2->end());

  hpp::core::Configuration_t q_3(path3->outputSize());
  bool suc3 = path3->impl_compute(q_3, timeRange3.second /2);
  BOOST_CHECK(suc3);
  cout << q_3 << endl << path3->end() << endl;
  BOOST_CHECK(q_3 == path3->end());
  
}
/*

DevicePtr_t createRobot() {
  std::string urdf(
  "<link name=" + quote + "base_link" + quote + ">"
  "<link name=" + quote + "base_link_inertia" + quote + ">"
    "<visual>"
      "<origin rpy=" + quote + "0 0 3.141592653589793" + quote + " xyz=" + quote + "0 0 0" + quote + "/>"
      "<geometry>"
        "<mesh filename=" + quote + "package://ur_description/meshes/ur3/visual/base.dae" + quote + "/>"
      "</geometry>"
      "<material name=" + quote + "LightGrey" + quote + ">"
        "<color rgba=" + quote + "0.7 0.7 0.7 1.0" + quote + "/>"
      "</material>"
    "</visual>"
  "</link>"
  "<link name=" + quote + "shoulder_link" + quote + ">"
    "<visual>"
      "<origin rpy=" + quote + "0 0 3.141592653589793" + quote + " xyz=" + quote + "0 0 0" + quote + "/>"
      "<geometry>"
        "<mesh filename=" + quote + "package://ur_description/meshes/ur3/visual/shoulder.dae" + quote + "/>"
      "</geometry>"
      "<material name=" + quote + "LightGrey" + quote + ">"
        "<color rgba=" + quote + "0.7 0.7 0.7 1.0" + quote + "/>"
      "</material>"
    "</visual>"
  "</link>"
  "<link name=" + quote + "upper_arm_link" + quote + ">"
    "<visual>"
      "<origin rpy=" + quote + "1.5707963267948966 0 -1.5707963267948966" + quote + " xyz=" + quote + "0 0 0.1198" + quote + "/>"
      "<geometry>"
        "<mesh filename=" + quote + "package://ur_description/meshes/ur3/visual/upperarm.dae" + quote + "/>"
      "</geometry>"
      "<material name=" + quote + "LightGrey" + quote + ">"
        "<color rgba=" + quote + "0.7 0.7 0.7 1.0" + quote + "/>"
      "</material>"
    "</visual>"
  "</link>"
  "<link name=" + quote + "forearm_link" + quote + ">"
    "<visual>"
      "<origin rpy=" + quote + "1.5707963267948966 0 -1.5707963267948966" + quote + " xyz=" + quote + "0 0 0.0275" + quote + "/>"
      "<geometry>"
        "<mesh filename=" + quote + "package://ur_description/meshes/ur3/visual/forearm.dae" + quote + "/>"
      "</geometry>"
      "<material name=" + quote + "LightGrey" + quote + ">"
        "<color rgba=" + quote + "0.7 0.7 0.7 1.0" + quote + "/>"
      "</material>"
    "</visual>"
  "</link>"
  "<link name=" + quote + "wrist_1_link" + quote + ">"
    "<visual>"
      "<origin rpy=" + quote + "1.5707963267948966 0 0" + quote + " xyz=" + quote + "0 0 -0.085" + quote + "/>"
      "<geometry>"
        "<mesh filename=" + quote + "package://ur_description/meshes/ur3/visual/wrist1.dae" + quote + "/>"
      "</geometry>"
      "<material name=" + quote + "LightGrey" + quote + ">"
        "<color rgba=" + quote + "0.7 0.7 0.7 1.0" + quote + "/>"
      "</material>"
    "</visual>"
  "</link>"
  "<link name=" + quote + "wrist_2_link" + quote + ">"
    "<visual>"
      "<origin rpy=" + quote + "0 0 0" + quote + " xyz=" + quote + "0 0 -0.085" + quote + "/>"
      "<geometry>"
        "<mesh filename=" + quote + "package://ur_description/meshes/ur3/visual/wrist2.dae" + quote + "/>"
      "</geometry>"
      "<material name=" + quote + "LightGrey" + quote + ">"
        "<color rgba=" + quote + "0.7 0.7 0.7 1.0" + quote + "/>"
      "</material>"
    "</visual>"
  "</link>"
  "<link name=" + quote + "wrist_3_link" + quote + ">"
    "<visual>"
      "<origin rpy=" + quote + "1.5707963267948966 0 0" + quote + " xyz=" + quote + "0 0 -0.082" + quote + "/>"
      "<geometry>"
        "<mesh filename=" + quote + "package://ur_description/meshes/ur3/visual/wrist3.dae" + quote + "/>"
      "</geometry>"
      "<material name=" + quote + "LightGrey" + quote + ">"
        "<color rgba=" + quote + "0.7 0.7 0.7 1.0" + quote + "/>"
      "</material>"
    "</visual>"
  "</link>"
  "<!-- joints: main serial chain -->"
  "<joint name=" + quote + "base_link-base_link_inertia" + quote + " type=" + quote + "fixed" + quote + ">"
    "<parent link=" + quote + "base_link" + quote + "/>"
    "<child link=" + quote + "base_link_inertia" + quote + "/>"
    "<origin rpy=" + quote + "0 0 3.141592653589793" + quote + " xyz=" + quote + "0 0 0" + quote + "/>"
  "</joint>"
  "<joint name=" + quote + "shoulder_pan_joint" + quote + " type=" + quote + "revolute" + quote + ">"
    "<parent link=" + quote + "base_link_inertia" + quote + "/>"
    "<child link=" + quote + "shoulder_link" + quote + "/>"
    "<origin rpy=" + quote + "0 0 0" + quote + " xyz=" + quote + "0 0 0.1519" + quote + "/>"
    "<axis xyz=" + quote + "0 0 1" + quote + "/>"
    "<limit effort=" + quote + "56.0" + quote + " lower=" + quote + "-6.283185307179586" + quote + " upper=" + quote + "6.283185307179586" + quote + " velocity=" + quote + "3.141592653589793" + quote + "/>"
    "<dynamics damping=" + quote + "0" + quote + " friction=" + quote + "0" + quote + "/>"
  "</joint>"
  "<joint name=" + quote + "shoulder_lift_joint" + quote + " type=" + quote + "revolute" + quote + ">"
    "<parent link=" + quote + "shoulder_link" + quote + "/>"
    "<child link=" + quote + "upper_arm_link" + quote + "/>"
    "<origin rpy=" + quote + "1.570796327 0 0" + quote + " xyz=" + quote + "0 0 0" + quote + "/>"
    "<axis xyz=" + quote + "0 0 1" + quote + "/>"
    "<limit effort=" + quote + "56.0" + quote + " lower=" + quote + "-6.283185307179586" + quote + " upper=" + quote + "6.283185307179586" + quote + " velocity=" + quote + "3.141592653589793" + quote + "/>"
    "<dynamics damping=" + quote + "0" + quote + " friction=" + quote + "0" + quote + "/>"
  "</joint>"
  "<joint name=" + quote + "elbow_joint" + quote + " type=" + quote + "revolute" + quote + ">"
    "<parent link=" + quote + "upper_arm_link" + quote + "/>"
    "<child link=" + quote + "forearm_link" + quote + "/>"
    "<origin rpy=" + quote + "0 0 0" + quote + " xyz=" + quote + "-0.24365 0 0" + quote + "/>"
    "<axis xyz=" + quote + "0 0 1" + quote + "/>"
    "<limit effort=" + quote + "28.0" + quote + " lower=" + quote + "-3.141592653589793" + quote + " upper=" + quote + "3.141592653589793" + quote + " velocity=" + quote + "3.141592653589793" + quote + "/>"
    "<dynamics damping=" + quote + "0" + quote + " friction=" + quote + "0" + quote + "/>"
  "</joint>"
  "<joint name=" + quote + "wrist_1_joint" + quote + " type=" + quote + "revolute" + quote + ">"
    "<parent link=" + quote + "forearm_link" + quote + "/>"
    "<child link=" + quote + "wrist_1_link" + quote + "/>"
    "<origin rpy=" + quote + "0 0 0" + quote + " xyz=" + quote + "-0.21325 0 0.11235" + quote + "/>"
    "<axis xyz=" + quote + "0 0 1" + quote + "/>"
    "<limit effort=" + quote + "12.0" + quote + " lower=" + quote + "-6.283185307179586" + quote + " upper=" + quote + "6.283185307179586" + quote + " velocity=" + quote + "6.283185307179586" + quote + "/>"
    "<dynamics damping=" + quote + "0" + quote + " friction=" + quote + "0" + quote + "/>"
  "</joint>"
  "<joint name=" + quote + "wrist_2_joint" + quote + " type=" + quote + "revolute" + quote + ">"
    "<parent link=" + quote + "wrist_1_link" + quote + "/>"
    "<child link=" + quote + "wrist_2_link" + quote + "/>"
    "<origin rpy=" + quote + "1.570796327 0 0" + quote + " xyz=" + quote + "0 -0.08535 -1.750557762378351e-11" + quote + "/>"
    "<axis xyz=" + quote + "0 0 1" + quote + "/>"
    "<limit effort=" + quote + "12.0" + quote + " lower=" + quote + "-6.283185307179586" + quote + " upper=" + quote + "6.283185307179586" + quote + " velocity=" + quote + "6.283185307179586" + quote + "/>"
    "<dynamics damping=" + quote + "0" + quote + " friction=" + quote + "0" + quote + "/>"
  "</joint>"
  "<joint name=" + quote + "wrist_3_joint" + quote + " type=" + quote + "revolute" + quote + ">"
    "<parent link=" + quote + "wrist_2_link" + quote + "/>"
    "<child link=" + quote + "wrist_3_link" + quote + "/>"
    "<origin rpy=" + quote + "1.570796326589793 3.141592653589793 3.141592653589793" + quote + " xyz=" + quote + "0 0.0819 -1.679797079540562e-11" + quote + "/>"
    "<axis xyz=" + quote + "0 0 1" + quote + "/>"
    "<limit effort=" + quote + "12.0" + quote + " lower=" + quote + "-6.283185307179586" + quote + " upper=" + quote + "6.283185307179586" + quote + " velocity=" + quote + "6.283185307179586" + quote + "/>"
    "<dynamics damping=" + quote + "0" + quote + " friction=" + quote + "0" + quote + "/>"
  "</joint>"
  "<link name=" + quote + "base" + quote + "/>"
  "<joint name=" + quote + "base_link-base_fixed_joint" + quote + " type=" + quote + "fixed" + quote + ">"

    "<origin rpy=" + quote + "0 0 3.141592653589793" + quote + " xyz=" + quote + "0 0 0" + quote + "/>"
    "<parent link=" + quote + "base_link" + quote + "/>"
    "<child link=" + quote + "base" + quote + "/>"
  "</joint>"
  "<link name=" + quote + "flange" + quote + "/>"
  "<joint name=" + quote + "wrist_3-flange" + quote + " type=" + quote + "fixed" + quote + ">"
    "<parent link=" + quote + "wrist_3_link" + quote + "/>"
    "<child link=" + quote + "flange" + quote + "/>"
    "<origin rpy=" + quote + "0 -1.5707963267948966 -1.5707963267948966" + quote + " xyz=" + quote + "0 0 0" + quote + "/>"
  "</joint>"
  "<link name=" + quote + "tool0" + quote + "/>"
  "<joint name=" + quote + "flange-tool0" + quote + " type=" + quote + "fixed" + quote + ">"
    "<origin rpy=" + quote + "1.5707963267948966 0 1.5707963267948966" + quote + " xyz=" + quote + "0 0 0" + quote + "/>"
    "<parent link=" + quote + "flange" + quote + "/>"
    "<child link=" + quote + "tool0" + quote + "/>"
  "</joint>"
"</robot>"

      );
    DevicePtr_t robot = Device::create("test");
    urdf::loadModelFromString(robot, 0, "test", "anchor", urdf, "");
    return robot;

}
*/