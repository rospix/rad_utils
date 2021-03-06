#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/console.h>
#include <ros/package.h>
#include <rad_utils/physics.h>
#include <iostream>

/* loadNistTable() //{ */

Table loadNistTable(std::string material) {

  Table             table;
  std::stringstream ss;

  // open the CSV with data
  std::string package_path = ros::package::getPath("rad_utils");
  ss << package_path.c_str() << "/nist/" << material.c_str() << ".csv";
  std::ifstream nist_file;
  nist_file.open(ss.str().c_str());

  // check file existence
  if (!nist_file.is_open()) {
    std::stringstream msg;
    msg << "Table \"" << material << ".csv\" was not found in folder \"" << package_path << "/nist/\" not found!";
    throw std::runtime_error(msg.str());
  }

  // read the file
  int lines = 0;
  while (true) {
    std::string line;
    getline(nist_file, line);
    std::vector<std::string> line_elems;
    boost::split(line_elems, line, [](char c) { return c == ','; });

    // check file formatting
    if (line_elems.size() != 5) {                 // 4 numbers + 1 newline symbol
      if (line_elems.size() <= 1 && lines > 0) {  // end reached
        break;
      }
      std::stringstream msg;
      msg << "Table \"" << material << ".csv\" in folder \"" << package_path << "/nist/\" is empty or not formatted properly! Expected 4 columns, got "
          << line_elems.size() - 1;
      throw std::runtime_error(msg.str());
    }

    std::vector<std::string> line_slice(line_elems.begin(), line_elems.end() - 1);
    std::vector<double>      line_numbers;
    for (unsigned int i = 0; i < line_slice.size(); i++) {
      line_numbers.push_back(std::stod(line_slice[i]));
    }
    table.push_back(line_numbers);
    lines++;
  }
  return table;
}
//}

/* interpolateAttenuation() //{ */

double interpolateAttenuation(Table table, AttenuationType type, double photon_energy) {

  // iterate through table until we hit line with the correct energy

  int index = 1;

  for (unsigned int i = 1; i < table.size(); i++) {
    if (table[i][0] > photon_energy) {
      index = (int)i;
      break;
    }
  }

  double energy_range      = table[index][0] - table[index - 1][0];
  double attenuation_range = table[index][type] - table[index - 1][type];
  double energy_fraction   = (table[index][0] - photon_energy) / energy_range;

  return table[index][type] - (attenuation_range * energy_fraction);
}

//}

/* calculateMassAttCoeff() //{ */

double calculateMassAttCoeff(double photon_energy, std::string material, AttenuationType type) {

  try {
    Table table = loadNistTable(material);
    return interpolateAttenuation(table, type, photon_energy);
  }
  catch (...) {
    ROS_ERROR("[Radiation Data Utils]: NIST table not found for material %s!", material.c_str());
    return 0.0;
  }

}

//}

/* getMaterialDensity() //{ */

double getMaterialDensity(std::string material) {

  try {
    Table table = loadNistTable(material);
    return table[0][3];
  }
  catch (...) {
    ROS_ERROR("[Radiation Data Utils]: NIST table not found or improperly formateed for material %s!", material.c_str());
    return 0.0;
  }

}

//}

/* calculateAbsorptionProb() //{ */

double calculateAbsorptionProb(double track_length, double attenuation_coeff, double material_density) {

  return 1.0 - std::exp(-attenuation_coeff * track_length * 100 * material_density);  // multiply by 100, because other constants were measured for input in cm

}

//}

/* conversions:: //{ */

namespace conversions {

  double energyJtoeV(const double J) {

    return 6.242e18 * J;
  }

  double energyeVtoJ(const double eV) {

    return eV / 6.242e18;
  }

}

//}
