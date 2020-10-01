#include <radiation_utils/elist_parser.h>
#include <stdio.h>
#include <fstream>
#include <optional>

namespace radiation_utils
{

namespace elist_parser
{

Event::Event(const int& detector_id, const long& event_id, const double& x, const double& y, const double& energy, const double& time, const int& flags,
             const double& z_sim) {

  this->detector_id_ = detector_id;
  this->event_id_    = event_id;
  this->x_           = x;
  this->y_           = y;
  this->energy_      = energy;
  this->time_        = time;
  this->flags_       = flags;
  this->z_sim_       = z_sim;
}

std::optional<std::shared_ptr<std::vector<Event>>> parseEventList(const std::string& file_path) {

  std::ifstream file_in(file_path.c_str(), std::ifstream::in);

  if (!file_in) {

    printf("Error, could not open the file %s\n", file_path.c_str());

    return {};
  }

  std::string line;

  // the first three lines are info lines
  getline(file_in, line);
  getline(file_in, line);
  getline(file_in, line);

  // all other lines contain data
  while (getline(file_in, line)) {

    printf("line: %s\n", line.c_str());
  }

  return {};
}

}  // namespace elist_parser

}  // namespace radiation_utils
