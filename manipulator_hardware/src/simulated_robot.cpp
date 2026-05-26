#include "manipulator_hardware/simulated_robot.hpp"

// For random number generation (encoder noise simulation).
#include <random>
// For std::clamp — restricts a value to a range. Like numpy.clip.
#include <algorithm>
// For logging.
#include "rclcpp/rclcpp.hpp"
// For string-to-double conversion from URDF parameters.
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace manipulator_hardware
{ // this bracket is defining the scope of the namespace
    // for example, on_init is a method of a class called simulatedRobot, which is inside
    // the namespace manipulator_hardware

// ============================================================
// on_init: called once when the hardware interface is loaded.
// The HardwareInfo struct contains everything from the
// <ros2_control> block in your URDF: joint names, parameters,
// interface types.
// ============================================================
hardware_interface::CallbackReturn SimulatedRobot::on_init(
  const hardware_interface::HardwareInfo & info) //this HardwareInfo struct has all the information from the ros2_control urdf
{
  // Call the parent class on_init first. This parses the URDF
  // <ros2_control> block and populates info_.
  // "if (... != CallbackReturn::SUCCESS)" is a common error-check pattern.
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  //in this if statement, the output of the on_init was used to compare, but all the other 
  //things inside the method were all executed and effective, e.g. this->info_ = info

  // info_ is now populated (set by the parent class).
  // info_.joints is a vector of joint descriptions from the URDF.
  num_joints_ = info_.joints.size(); //info_ is a member variable of the parent class SystemInterface
  //inside the on_init method of the parent class, the code this->info_ = info; happens

  // Resize vectors to hold data for each joint.
  // .resize(n, value) sets the vector to n elements, all initialized
  // to "value". Like [0.0] * n in Python.
  // these are safety numbers just incase i forgot to specify my joint limits in my urdf file
  hw_commands_positions_.resize(num_joints_, 0.0); // the variable has been declared in the header file, at that time the default size is 0
  hw_states_positions_.resize(num_joints_, 0.0);
  joint_lower_limits_.resize(num_joints_, -3.15);
  joint_upper_limits_.resize(num_joints_, 3.15);


  // Read simulation parameters from the URDF <param> tags.
  // info_.hardware_parameters is a std::map<std::string, std::string>. (equivalent to a python dictionary)
  // it has key-value pairs from the urdf, ee.g. "tau" "0.1"
  // .find() returns an iterator; if it equals .end(), the key wasn't found.
  // "auto" lets the compiler figure out the iterator type automatically —
  // without it, you'd write:
  //   std::map<std::string, std::string>::iterator it = ...
  // which is painfully long.
  auto it_tau = info_.hardware_parameters.find("tau"); //info_ is a member from the parent class systeminterface
  //.find("tau") returns a pointer-like object that either points to the found entry or to a special 
  //"pass the end" position if the key doesnt exist
  // the following if statment means if the key exists...
  if (it_tau != info_.hardware_parameters.end()) {
    // std::stod converts a string to a double.
    // it_tau->second accesses the value (the key is it_tau->first).
    // the iterator it_tau points to a std::pair,std:string, std::string>>
    // a pair has two members .first (the key, e.g. "'tau" ) and .second (the value, e.g. "0.1")
    // -> works with an iterator or a pointer, it is equivalent to dereferencing and then access member
    tau_ = std::stod(it_tau->second); //std::stod turns a string into a double

  } else {
    tau_ = 0.1;  // default: 100ms time constant
  }

  auto it_noise = info_.hardware_parameters.find("noise_stddev");
  if (it_noise != info_.hardware_parameters.end()) {
    noise_stddev_ = std::stod(it_noise->second);
  } else {
    noise_stddev_ = 0.001;  // default: ~0.06 degrees
  }

  // Read joint limits from the URDF joint descriptions.
  // Range-based for loop: "for (type variable : collection)"
  // is like Python's "for variable in collection".
  // "const auto &" means: let the compiler figure out the type,
  // don't copy the element, and don't modify it.
  for (std::size_t i = 0; i < num_joints_; i++) {
    // std::size_t type vs int, the former only hold zero or positive numbers
    // info_.joints[i].command_interfaces is a vector of interface descriptions.
    // We check that each joint has exactly one command interface (position).
    if (info_.joints[i].command_interfaces.size() != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("SimulatedRobot"),
        "Joint '%s' must have exactly 1 command interface.",
        info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    
    }
    //RCLCPP_ERROR is a macro not a function, its a text substituion that happens before compilation
    //when i write the macro, the message will just be printed out in the terminal


    if (info_.joints[i].state_interfaces.size() != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("SimulatedRobot"),
        "Joint '%s' must have exactly 1 state interface.",
        info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Read joint limits from URDF parameters if provided.
    auto it_min = info_.joints[i].parameters.find("min_position");
    auto it_max = info_.joints[i].parameters.find("max_position");
    if (it_min != info_.joints[i].parameters.end()) {
      joint_lower_limits_[i] = std::stod(it_min->second);
    }
    if (it_max != info_.joints[i].parameters.end()) {
      joint_upper_limits_[i] = std::stod(it_max->second);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("SimulatedRobot"),
    "Initialized with %zu joints, tau=%.3f, noise=%.4f",
    num_joints_, tau_, noise_stddev_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// on_activate: called when the controller_manager transitions
// this hardware to the "active" state. Reset positions.
// ============================================================
hardware_interface::CallbackReturn SimulatedRobot::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/) // you are overriding a method in the parent class, which takes the input of type...
  //the method of your derived class needs to have the same signature of the base class function, and you do that with that thing with the comment
  //even though the derived method body doesnt care about the input, 
{
  // Set current state = current command so there's no initial jump.
  for (std::size_t i = 0; i < num_joints_; i++) {
    hw_states_positions_[i] = hw_commands_positions_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("SimulatedRobot"), "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// on_deactivate: called when transitioning to inactive.
// ============================================================
hardware_interface::CallbackReturn SimulatedRobot::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SimulatedRobot"), "Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================
// export_state_interfaces: tells ros2_control what data the
// controllers can READ from this hardware.
//
// Each StateInterface is a named, read-only reference to a
// double in your hw_states_positions_ vector.
// ============================================================
std::vector<hardware_interface::StateInterface>
SimulatedRobot::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < num_joints_; i++) {
    // StateInterface constructor takes:
    //   joint_name, interface_type, pointer_to_data
    // The controller reads from this pointer every cycle.
    state_interfaces.emplace_back( //state_interfaces is a vector, the emplace_back just adds a thing to the end of the vector, after the for loop, the vector would have one slot per joint
      hardware_interface::StateInterface( //this is a constructor call, you create and initialize an object of a class by calling its constructor
        info_.joints[i].name, //the joint name from the urdf file
        hardware_interface::HW_IF_POSITION, //it literally means "'position", its a constant string defined by ros2_control
        &hw_states_positions_[i])); //this thing is the address/pointer of the ith element of ...
  }

  return state_interfaces; //when this method returns the tstate_interfaces, the controller_manager hands these pointers to the active controllers
  //from this point on, whenever a controller reads, its reading directly from the memory address
}
// just setting up the memory addresses aka state_interfaces, so that the controllers can read from



// ============================================================
// export_command_interfaces: tells ros2_control what data the
// controllers can WRITE to this hardware.
// ============================================================
std::vector<hardware_interface::CommandInterface>
SimulatedRobot::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < num_joints_; i++) {
    // CommandInterface: same idea, but the controller writes here.
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_commands_positions_[i]));
  }

  return command_interfaces;
}

// ============================================================
// read: called every control cycle BEFORE the controller runs.
// This is where you'd read real encoders on a physical robot.
// Here we simulate: apply first-order lag + add noise.
//
// First-order lag model:
//   state_new = state_old + (dt / tau) * (command - state_old)
//
// When dt/tau is small, the state moves slowly toward the command.
// When dt/tau >= 1, it essentially jumps (instant response).
// This simulates a motor that can't instantly reach the target.
// ============================================================
hardware_interface::return_type SimulatedRobot::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // period.seconds() gives the time since the last read() call,
  // as a double in seconds.
  double dt = period.seconds();

  // alpha is the smoothing factor. Clamped to [0, 1].
  // If tau_ is very small or dt is large, alpha approaches 1 (instant).
  double alpha = std::clamp(dt / tau_, 0.0, 1.0); 

  // Set up random noise generator.
  // "static" means these are created once and persist across calls —
  // not recreated every cycle. This is important for performance
  // and for the random sequence to be proper.
  static std::mt19937 gen(42);  // Mersenne Twister RNG, seed=42
  std::normal_distribution<double> noise(0.0, noise_stddev_);

  for (std::size_t i = 0; i < num_joints_; i++) {
    // First-order lag: smoothly approach the commanded position.
    double target = hw_commands_positions_[i];
    hw_states_positions_[i] += alpha * (target - hw_states_positions_[i]);

    // Add simulated encoder noise.
    hw_states_positions_[i] += noise(gen);
  }

  return hardware_interface::return_type::OK;
}

// ============================================================
// write: called every control cycle AFTER the controller runs.
// This is where you'd send commands to real motors.
// Here we enforce joint limits.
// ============================================================
hardware_interface::return_type SimulatedRobot::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (std::size_t i = 0; i < num_joints_; i++) {
    // std::clamp restricts the value to [lower, upper].
    // If the controller commands a position outside the joint limits,
    // it gets clamped to the limit.
    hw_commands_positions_[i] = std::clamp(
      hw_commands_positions_[i],
      joint_lower_limits_[i],
      joint_upper_limits_[i]);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace manipulator_hardware

// ============================================================
// PLUGIN REGISTRATION MACRO
//
// This is the magic line that makes your class discoverable by
// pluginlib at runtime. Without it, controller_manager can't
// find your hardware interface even if the code compiles fine.
//
// It says: "SimulatedRobot is a plugin that implements
// SystemInterface."
// ============================================================
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  manipulator_hardware::SimulatedRobot,
  hardware_interface::SystemInterface)