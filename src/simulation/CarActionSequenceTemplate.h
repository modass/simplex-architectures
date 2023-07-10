//
// Created by bmaderbacher on 10.07.23.
//

#include "../types.h"

#ifndef SIMPLEXARCHITECTURES_CARACTIONSEQUENCETEMPLATE_H
#define SIMPLEXARCHITECTURES_CARACTIONSEQUENCETEMPLATE_H

namespace simplexArchitectures {

struct CarActionSequenceTemplate {
  CarActionSequenceTemplate( size_t thetaDiscretization, size_t maxTurn )
      : mThetaDiscretization( thetaDiscretization ), mMaxTurn( maxTurn ) {}

  std::vector<hypro::Label> generateActionSequence(size_t initialHeading, size_t firstTargetHeading, size_t firstDuration, size_t secondTargetHeading, size_t secondDuration);

  size_t mThetaDiscretization;
  size_t mMaxTurn;

 private:
    size_t nextAction(size_t currentHeading, size_t targetHeading) const;



};

}  // namespace simplexArchitectures

#endif  // SIMPLEXARCHITECTURES_CARACTIONSEQUENCETEMPLATE_H
