
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
  q1 << 0,3,0,-3,0,0;
  q2 << 1,1,2,1,1,1;

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

  hpp::core::path::HermitePtr_t path1 = hpp::core::path::Hermite::create_with_timeRange(robot, q1, q2, constraint, timeRange1);
  
  hpp::core::vector_t rrr(path1->outputDerivativeSize());
  path1->derivative(rrr,path1->timeRange().second, 1); 
  cout << endl << "rrr :\n" << rrr << endl;
  BOOST_CHECK(rrr == path1->v1());
  


}


