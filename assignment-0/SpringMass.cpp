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
    : initial_position(pos_init), initial_velocity(vel_init),
      equilibrium_position(pos_eqm), equilibrium_velocity(vel_eqm) {}

// TODO SpringMass simulation step
int SpringMass::step() {}

// TODO SpringMass configuration getter
bool SpringMass::getConfiguration(int t, Vec2d &state) const {}

// TODO SpringMass current simulation time getter
int SpringMass::getCurrentSimulationTime() const {}
