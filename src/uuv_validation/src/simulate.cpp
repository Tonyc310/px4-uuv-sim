#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "uuv_dynamics/motion.hpp"
#include "uuv_dynamics/vehicle.hpp"

namespace
{

constexpr char kUsage[] =
  "usage: simulate --seconds <s> --step <s> [--velocity <u v w p q r>]\n"
  "\n"
  "Prints the BlueROV2 Heavy's free motion from a level start as CSV, one row per step: the time\n"
  "and the body velocity (m/s and rad/s, forward-right-down). The time must be a whole number of\n"
  "steps.\n";

constexpr int kUsageError = 2;
// How far the time may be from a whole number of steps, as a fraction of it: a decimal step such
// as 0.001 isn't exact in binary.
constexpr double kWholeStepsTolerance = 1e-9;

struct Scenario
{
  std::int64_t steps = 0;
  double step_seconds = 0.0;
  uuv_dynamics::Vector6d velocity = uuv_dynamics::Vector6d::Zero();
};

[[nodiscard]] std::optional<double> number(const std::string & text)
{
  char * end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (text.empty() || *end != '\0' || !std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

bool is_option(const std::string & argument) { return argument.rfind("--", 0) == 0; }

// The scenario the arguments describe. None if an option is unknown, repeated, has the wrong number
// of values or a value that isn't a number, or if the time and the step aren't both given, positive
// and a whole number of steps apart.
[[nodiscard]] std::optional<Scenario> parse(const std::vector<std::string> & arguments)
{
  Scenario scenario;
  double seconds = 0.0;
  std::set<std::string> options_seen;
  std::size_t next = 0;
  while (next < arguments.size()) {
    const std::string & option = arguments[next++];
    if (!options_seen.insert(option).second) {
      return std::nullopt;
    }
    std::vector<double> values;
    while (next < arguments.size() && !is_option(arguments[next])) {
      const auto value = number(arguments[next++]);
      if (!value) {
        return std::nullopt;
      }
      values.push_back(*value);
    }
    if (option == "--seconds" && values.size() == 1) {
      seconds = values[0];
    } else if (option == "--step" && values.size() == 1) {
      scenario.step_seconds = values[0];
    } else if (option == "--velocity" && values.size() == 6) {
      scenario.velocity = Eigen::Map<const uuv_dynamics::Vector6d>(values.data());
    } else {
      return std::nullopt;
    }
  }
  if (seconds <= 0.0 || scenario.step_seconds <= 0.0) {
    return std::nullopt;
  }
  const double steps = seconds / scenario.step_seconds;
  if (std::abs(steps - std::round(steps)) > kWholeStepsTolerance * steps) {
    return std::nullopt;
  }
  scenario.steps = std::llround(steps);
  return scenario;
}

void print_row(double seconds, const uuv_dynamics::Vector6d & velocity)
{
  std::cout << seconds;
  for (const double value : velocity) {
    std::cout << ',' << value;
  }
  std::cout << '\n';
}

}  // namespace

int main(int argc, char ** argv)
{
  const auto scenario = parse({argv + 1, argv + argc});
  if (!scenario) {
    std::cerr << kUsage;
    return kUsageError;
  }
  const uuv_dynamics::Vehicle heavy = uuv_dynamics::bluerov2_heavy();
  uuv_dynamics::State state{
    Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), scenario->velocity};

  std::cout << std::setprecision(12) << "t,u,v,w,p,q,r\n";
  print_row(0.0, state.nu);
  for (std::int64_t i = 1; i <= scenario->steps; ++i) {
    state =
      uuv_dynamics::step(heavy, state, uuv_dynamics::Vector6d::Zero(), scenario->step_seconds);
    print_row(static_cast<double>(i) * scenario->step_seconds, state.nu);
  }
  std::cout.flush();
  if (!std::cout) {
    std::cerr << "simulate: couldn't write the trajectory\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
