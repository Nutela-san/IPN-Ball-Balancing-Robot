#ifndef IK_3RPS
#define IK_3RPS
#include <Arduino.h>

enum eslabon
{
  A,
  B,
  C
};

class MachineIK
{
public:
  // Global User Defined Constants
  double d; // distance from the center of the base to any of its corners
  double e; // distance from the center of the platform to any of its corners
  double f; // length of link #1
  double g; // length of link #2

  // Calculation Variables
  double nmag, nz; // magnitude and z component of the normal vector
  double x, y, z;  // generic variables for the components of legs A, B, and C
  double mag;      // generic magnitude of the leg vector
  double angle;    // generic angle for legs A, B, and C

  MachineIK(double d, double e, double l1, double l2);
  double get_IK_Theta(eslabon leg, double heigth_platfom,
                      double X_inclination, double Y_inclination);
};

MachineIK::MachineIK(double d, double e, double l1, double l2){
  this->d = d;
  this->e = e;
  this->f = l1;
  this->g = l2;
}

double MachineIK::get_IK_Theta(eslabon leg, double heigth_platfom,
                      double nx, double ny){
  //create unit normal vector
  nmag = sqrt(pow(nx, 2) + pow(ny, 2) + 1);  //magnitude of the normal vector
  nx /= nmag;
  ny /= nmag;
  nz = 1 / nmag;
  //calculates angle A, B, or C
  switch(leg){
    case eslabon::A :{
      y = d + (e / 2) * (1 - (pow(nx, 2) + 3 * pow(nz, 2) + 3 * nz) / (nz + 1 - pow(nx, 2) + (pow(nx, 4) - 3 * pow(nx, 2) * pow(ny, 2)) / ((nz + 1) * (nz + 1 - pow(nx, 2)))));
      z = heigth_platfom + e * ny;
      mag = sqrt(pow(y, 2) + pow(z, 2));
      angle = acos(y / mag) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
      break;
    }
    case eslabon::B:{
      x = (sqrt(3) / 2) * (e * (1 - (pow(nx, 2) + sqrt(3) * nx * ny) / (nz + 1)) - d);
      y = x / sqrt(3);
      z = heigth_platfom - (e / 2) * (sqrt(3) * nx + ny);
      mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
      angle = acos((sqrt(3) * x + y) / (-2 * mag)) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
      break;
    }
    case eslabon::C:{
      x = (sqrt(3) / 2) * (d - e * (1 - (pow(nx, 2) - sqrt(3) * nx * ny) / (nz + 1)));
      y = -x / sqrt(3);
      z = heigth_platfom + (e / 2) * (sqrt(3) * nx - ny);
      mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
      angle = acos((sqrt(3) * x - y) / (2 * mag)) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
      break;
    }
  }
  return (angle * (180/PI));
}


#endif