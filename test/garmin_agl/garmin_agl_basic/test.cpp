#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
  }

  bool test(void);
};

bool Tester::test(void) {

  const std::string uav_name = "uav1";

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }

  {
    auto [success, message] = uh->activateMidAir();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "midair activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

  if (uh->getActiveEstimator() != "gps_garmin") {
    RCLCPP_ERROR(node_->get_logger(), "gps_garmin estimator not active");
    return false;
  }

  const double desired_height = 2.5;

  {
    auto [success, message] = uh->gotoAbs(0, 0, desired_height, 0);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "goto failed with message: '%s'", message.c_str());
      return false;
    }
  }

  this->sleep(1.0);

  {

    auto height_agl = uh->getHeightAgl();

    if (!height_agl) {
      RCLCPP_ERROR(node_->get_logger(), "could not obtain height AGL");
      return false;
    }

    if (abs(height_agl.value() - desired_height) > 0.3) {
      RCLCPP_ERROR(node_->get_logger(), "reached the wrong AGL height (desired %.2f, current %.2f)", desired_height, height_agl.value());
      return false;
    }
  }

  this->sleep(5.0);

  if (uh->isFlyingNormally()) {
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "not flying normally");
    return false;
  }
}

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  tester.sleep(2.0);

  std::cout << "Test: reporting test results" << std::endl;

  tester.reportTestResult(test_result);

  tester.join();
}
