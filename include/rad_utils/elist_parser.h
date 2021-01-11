#ifndef ELIST_PARSER_H
#define ELIST_PARSER_H

#include <vector>
#include <memory>

namespace rad_utils
{

namespace elist_parser
{

class Event {

public:
  Event(const int& detector_id, const long& event_id, const double& x, const double& y, const double& energy, const double& time, const int& flags,
        const double& z_sim);

  int    detector_id_;
  long   event_id_;
  double x_;
  double y_;
  double energy_;
  double time_;
  int    flags_;
  double z_sim_;
};

std::shared_ptr<std::vector<Event>> parseEventList(const std::string file_path);

}  // namespace elist_parser

#endif
}
