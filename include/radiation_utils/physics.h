#ifndef RADIATION_PHYSICS_H
#define RADIATION_PHYSICS_H

#include <vector>
#include <string>
#include <sstream>
#include <exception>
#include <cmath>

enum AttenuationType
{
  MASS        = 1,
  MASS_ENERGY = 2,
};

typedef struct std::vector<std::vector<double>> Table;

Table  loadNistTable(std::string material);
double interpolateAttenuation(Table table, AttenuationType type, double photon_energy);

double calculateMassAttCoeff(double photon_energy, std::string material, AttenuationType type);
double getMaterialDensity(std::string material);
double calculateAbsorptionProb(double track_length, double attenuation_coeff, double material_density);

namespace conversions
{

double energyJtoeV(const double J);

double energyeVtoJ(const double eV);

}  // namespace conversions

namespace constants
{

const double h       = 6.6207015e-34;               // Planc constant [J * s]
const double hr      = 6.6207015e-34 / (2 * M_PI);  // reduced Planc constant [J * s]
const double c       = 299792458.0;                 // speed of light in vacuum [m/s]
const double me      = 9.10938356e-31;              // mass of an electron [kg]
const double alpha   = 1.0 / 137.04;                // fine structure constant [-]
const double r_e     = 2.8179e-15;                  // classical electron radius [m]
const double si_atom = 0.222e-9;                    // diameter of an silicone atom [m]
const double N_a     = 6.02214086e23;               // Avogadro's number [1/mol]
const double barn    = 10.0e-28;                    // m^2

}  // namespace constants

#endif
