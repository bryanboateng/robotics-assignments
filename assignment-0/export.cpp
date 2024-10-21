#include "SpringDamperMass.hpp"
#include "SpringMass.hpp"
#include <fstream>
#include <iostream>
#include <vector>

void normal() {
  std::string filename = "out/normal-data.csv";
  std::ofstream outfile(filename);
  if (!outfile.is_open()) {
    std::cerr << "Error opening file " << filename << " for writing.\n";
  }
  outfile << "x,y\n";
  double pos_init = 200;
  double vel_init = 0;
  outfile << pos_init << "," << vel_init << "\n";
  SpringMass mass(pos_init, vel_init, 161., 0.);
  int t;
  do {
    t = mass.step();
    Vec2d s;
    mass.getConfiguration(t, s);
    outfile << s.x << "," << s.y << "\n";
    std::cout << "written vec2d: (" << s.x << ", " << s.y << ")\n";
  } while (t <= 500 && t > 0);
  outfile.close();
  std::cout << "Data successfully written to " << filename << "\n";
}

void damper() {
  std::string filename = "out/damper-data.csv";
  std::ofstream outfile(filename);
  if (!outfile.is_open()) {
    std::cerr << "Error opening file " << filename << " for writing.\n";
  }
  outfile << "x,y\n";
  double pos_init = 200;
  double vel_init = 0;
  outfile << pos_init << "," << vel_init << "\n";
  SpringDamperMass mass(pos_init, vel_init, 161., 0., 1);
  int t;
  do {
    t = mass.step();
    Vec2d s;
    mass.getConfiguration(t, s);
    outfile << s.x << "," << s.y << "\n";
    std::cout << "written vec2d: (" << s.x << ", " << s.y << ")\n";
  } while (t <= 500 && t > 0);
  outfile.close();
  std::cout << "Data successfully written to " << filename << "\n";
}

int main() {
  normal();
  damper();
  return 0;
}
