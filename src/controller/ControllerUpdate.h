/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 16.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_CONTROLLERUPDATE_H
#define SIMPLEXARCHITECTURES_CONTROLLERUPDATE_H

#include "../types.h"

namespace simplexArchitectures {

struct ControllerUpdate {
  LocPtr loc = nullptr;
  Point val{};
};

}

#endif // SIMPLEXARCHITECTURES_CONTROLLERUPDATE_H
