#include "SpringDamperMass.hpp"

SpringDamperMass::SpringDamperMass(
    double pos_init,
    double vel_init,
    double pos_eqm,
    double vel_eqm,
    double _damping_coeff
)
    : SpringMass(pos_init, vel_init, pos_eqm, vel_eqm),
      damping_coeff(_damping_coeff) {}

int SpringDamperMass::step() {
  Vec2d current_motion_state = motion_states.back();
  double next_velocity =
      current_motion_state.y - (damping_coeff / MASS) * current_motion_state.y -
      (SPRING_CONST / MASS) * (current_motion_state.x - equilibrium_position);
  double next_position = current_motion_state.x + next_velocity;
  Vec2d next_state = {next_position, next_velocity};
  motion_states.push_back(next_state);
  current_timestep++;
  return current_timestep;
}

SpringDamperMass::~SpringDamperMass() = default;
