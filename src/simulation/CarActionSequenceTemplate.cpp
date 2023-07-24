//
// Created by bmaderbacher on 10.07.23.
//

#include "CarActionSequenceTemplate.h"

namespace simplexArchitectures {
std::vector<hypro::Label> CarActionSequenceTemplate::generateActionSequence( size_t initialHeading, size_t firstTargetHeading,
                                                                             size_t firstDuration, size_t secondTargetHeading,
                                                                             size_t secondDuration ) {
  auto actionSequence = std::vector<hypro::Label>();
  actionSequence.reserve(firstDuration+secondDuration);
  auto currentHeading = initialHeading;
  for ( size_t i = 0; i < firstDuration; ++i ) {
    currentHeading = nextAction(currentHeading, firstTargetHeading );
    actionSequence.emplace_back("set_theta_"+std::to_string(currentHeading));
  }
  for ( size_t j = 0; j < secondDuration; ++j ) {
    currentHeading = nextAction(currentHeading, secondTargetHeading );
    actionSequence.emplace_back("set_theta_"+std::to_string(currentHeading));
  }

  return actionSequence;
}
size_t CarActionSequenceTemplate::nextAction(size_t currentHeading, size_t targetHeading) const {
  auto differenceLeft = targetHeading >= currentHeading ? targetHeading - currentHeading : mThetaDiscretization + targetHeading - currentHeading;
  auto differenceRight = currentHeading >= targetHeading ? currentHeading - targetHeading : mThetaDiscretization + currentHeading - targetHeading;

  size_t nextHeading;

  if (differenceLeft == 0) {
    nextHeading = targetHeading;
  } else if (differenceLeft <= differenceRight) {
    auto turn = std::min(mMaxTurn, differenceLeft);
    nextHeading = currentHeading + turn >= mThetaDiscretization ? currentHeading + turn - mThetaDiscretization : currentHeading + turn;
  } else if (differenceRight < differenceLeft) {
    auto turn = std::min(mMaxTurn, differenceRight);
    nextHeading = turn > currentHeading ? mThetaDiscretization+currentHeading-turn : currentHeading - turn;
  }

  return nextHeading;
}
}  // namespace simplexArchitectures