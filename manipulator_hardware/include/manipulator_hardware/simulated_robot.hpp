// This is a "header guard" — prevents the file from being included twice
// during compilation, which would cause duplicate definition errors.
#pragma once

// Including external libraries — like Python's "import"
// but you must be explicit about every single thing you use.

// std::vector — a resizable array, equivalent to Python's list
#include <vector>
// std::string — C++ string type
#include <string>

// The base class your hardware interface inherits from.
// This is part of the ros2_control framework.
#include "hardware_interface/system_interface.hpp"
// Types used in the return values of lifecycle methods.
#include "hardware_interface/types/hardware_interface_return_values.hpp"
// Provides CallbackReturn enum (SUCCESS, ERROR, FAILURE).
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// A "namespace" groups your code under a name to avoid collisions.
// Like if two people both wrote a class called "Robot",
// namespaces keep them separate.
namespace manipulator_hardware
{

// This is your hardware interface class.
// "class X : public Y" means X inherits from Y.
// Your class IS a SystemInterface, with extra behavior you define.
class SimulatedRobot : public hardware_interface::SystemInterface //hardware_interface is a namespace
{
public:
  // This macro sets up some boilerplate that ros2_control expects.
  // Don't worry about what it does internally — it's required.
  RCLCPP_SHARED_PTR_DEFINITIONS(SimulatedRobot)

  // --- Lifecycle methods you must override ---
  // "override" tells the compiler: "I intend to replace the base class
  // version of this method. If I misspelled the name, give me an error."

  // Called once when the hardware is first loaded.
  // Reads configuration (joint names, parameters) from the URDF.
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // Called when transitioning to the "active" state.
  // Resets joint positions to initial values.
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Called when transitioning to the "inactive" state.
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Tells ros2_control what state data this hardware provides.
  // Returns a list of StateInterface objects (read-only access
  // to joint position values).
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  // Tells ros2_control what command data this hardware accepts.
  // Returns a list of CommandInterface objects (write access
  // for controllers to send position commands).
  std::vector<hardware_interface::CommandInterface> //return type of the method
  export_command_interfaces() override;

  // Called every control cycle. Simulates reading sensor data.
  // "const rclcpp::Time &" — a reference (alias) to a Time object.
  // "const" means this function promises not to modify it.
  // "const rclcpp::Duration &" — same idea, for the time step.
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Called every control cycle. Receives commands from controllers.
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- Member variables ---
  // "private" means only methods inside this class can access these.
  // This is encapsulation — external code can't mess with your internals.

  // Number of joints (will be 4 for your robot).
  std::size_t num_joints_;

  // These vectors store the actual data that controllers read/write.
  // hw_commands_positions_: what the controller wants each joint to be
  // hw_states_positions_:   what the "sensor" currently reports
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_states_positions_;

  // --- Simulation parameters ---
  // Time constant for first-order lag (seconds).
  // Smaller = faster response, larger = more sluggish.
  // A real servo might have tau ~ 0.05–0.2s.
  double tau_;

  // Standard deviation of Gaussian noise added to position readback.
  // A real encoder might have noise ~ 0.001 rad.
  double noise_stddev_;

  // Joint position limits (lower and upper for each joint).
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
};

}  // namespace manipulator_hardware