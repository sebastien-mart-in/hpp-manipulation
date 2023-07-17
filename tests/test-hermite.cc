
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
  q1 << 0.0, 1.0, 0.0, 2.0, 0.0, 0;
  q2 << 1,2,3,4,5,6;

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

  hpp::core::path::HermitePtr_t path1 = hpp::core::path::Hermite::create_with_timeRange(robot, q1, q2, constraint, timeRange1);
  hpp::core::path::HermitePtr_t path2 = hpp::core::path::Hermite::create_with_timeRange(robot, q1, q2, constraint, timeRange2);
  hpp::core::path::HermitePtr_t path3 = hpp::core::path::Hermite::create_with_timeRange(robot, q1, q2, constraint, timeRange3);
  hpp::core::path::HermitePtr_t path_no_TR = hpp::core::path::Hermite::create(robot, q1, q2, constraint);


  

  hpp::core::Configuration_t q_NO_TR (path_no_TR->outputSize());
  bool suc_NO_TR = path_no_TR->impl_compute(q_NO_TR, 1);
  std::cout << "Without using timeRange :" << std::endl; 
  std::cout << "Expected configuration (using end() method): " << std::endl;
  std::cout << path_no_TR->end() << std::endl;
  std::cout << std::endl << "Calculated configuration (using impl_compute at timeRange.second): " << std::endl;
  std::cout << q_NO_TR <<std::endl << std::endl;
  BOOST_CHECK(suc_NO_TR);
  BOOST_CHECK(q_NO_TR == path_no_TR->end());

  hpp::core::Configuration_t q_1_ (path1->outputSize());
  bool suc1_ = path1->impl_compute(q_1_, timeRange1.second);
  std::cout << "Using timeRange method (timeRange = (0,1)): " << std::endl;
  std::cout << "Expected configuration (using end() method): " << std::endl;
  std::cout << path1->end() << std::endl;
  std::cout << std::endl << "Calculated configuration (using impl_compute at timeRange.second): " << std::endl;
  std::cout << q_1_ <<std::endl << std::endl;
  BOOST_CHECK(suc1_);
  BOOST_CHECK(q_1_ == path1->end());
  
  hpp::core::Configuration_t q_2(path2->outputSize());
  bool suc2 = path2->impl_compute(q_2, timeRange2.second);
  std::cout << "Using timeRange method (timeRange = (0, 0.5)) : " << std::endl;
  std::cout << "Expected configuration (using end() method): " << std::endl;
  std::cout << path2->end() << std::endl;
  std::cout << std::endl << "Calculated configuration (using impl_compute at timeRange.second): " << std::endl;
  std::cout << q_2 <<std::endl << std::endl;
  BOOST_CHECK(suc2);
  BOOST_CHECK(q_2 == path2->end());

  hpp::core::Configuration_t q_3(path3->outputSize());
  bool suc3 = path3->impl_compute(q_3, timeRange3.second);
  std::cout << "Using timeRange method (timeRange = (0,2)): " << std::endl;
  std::cout << "Expected configuration (using end() method): " << std::endl;
  std::cout << path3->end() << std::endl;
  std::cout << std::endl << "Calculated configuration (using impl_compute at timeRange.second): " << std::endl;
  std::cout << q_3 <<std::endl << std::endl;
  BOOST_CHECK(suc3);
  BOOST_CHECK(q_3 == path3->end());
  
}
