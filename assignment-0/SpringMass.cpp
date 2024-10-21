#include "SpringMass.hpp"

// define gravity constant
const double SpringMass::GRAVITY = 10;
const double SpringMass::SPRING_CONST = 7;
const double SpringMass::MASS = 30;

SpringMass::SpringMass(
    double pos_init,
    double vel_init,
    double pos_eqm,
    double vel_eqm
)
    : equilibrium_position(pos_eqm), equilibrium_velocity(vel_eqm),
      current_timestep(0) {
  Vec2d initial_state = {pos_init, vel_init};
  motion_states.push_back(initial_state);
}

int SpringMass::step() {
  Vec2d current_motion_state = motion_states.back();
  double next_velocity =
      current_motion_state.y -
      (SPRING_CONST / MASS) * (current_motion_state.x - equilibrium_position);
  double next_position = current_motion_state.x + next_velocity;
  Vec2d next_state = {next_position, next_velocity};
  motion_states.push_back(next_state);
  current_timestep++;
  return current_timestep;
}

bool SpringMass::getConfiguration(int t, Vec2d &state) const {
  if (current_timestep < t) {
    return false;
  }
  state = motion_states[t];
  return true;
}

int SpringMass::getCurrentSimulationTime() const { return current_timestep; }

SpringMass::~SpringMass() = default;
