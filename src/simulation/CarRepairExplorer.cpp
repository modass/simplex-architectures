//
// Created by bmaderbacher on 21.07.23.
//

#include "CarRepairExplorer.h"

namespace simplexArchitectures {
bool CarRepairExplorer::findRepairSequence( LocPtr initialLocation, const Point initialState ) {

  auto initialBox = hypro::Condition<Number>{widenSample(initialState, mBloating, mBloatingDimensions)};
  auto success = semiExhaustiveSearch(initialLocation, initialBox);

  if(success) {
    auto safeStates = mSimulator.getReachStates();
    for (const auto& s : safeStates) {
      mStorage.add(s.first->getName(), s.second);
    }
    //TODO: Extend base controller
  }

  return success;
}

bool CarRepairExplorer::semiExhaustiveSearch( LocPtr initialLocation, const hypro::Condition<Number>& initialBox ) {
  size_t firstHeading;
  size_t firstDuration;
  size_t secondHeading;
  size_t secondDuration = 5;
  size_t initialHeading = getThetaBucketForLocation(initialLocation, mThetaDiscretization);

  hypro::TRIBOOL res;
  std::set<std::vector<hypro::Label>> testedSequences;

  for (firstHeading=0; firstHeading < 36; firstHeading+=3 ) {
    for (firstDuration=1; firstDuration < 12; firstDuration+=5 ) {
      for (secondHeading=0; secondHeading < 36; secondHeading+=3 ) {
        auto seq = mTemplate.generateActionSequence(initialHeading,firstHeading, firstDuration,secondHeading,secondDuration);
        if (testedSequences.count(seq)==0) {
          testedSequences.insert(seq);
          mSimulator.clear();
          res = mSimulator.simulate(seq, initialLocation, initialBox);
          if (res == hypro::TRIBOOL::TRUE && mSimulator.wasStorageReached()) {
//            spdlog::trace("firstHeading: "+std::to_string(firstHeading)+
//                           ", firstDuration: "+std::to_string(firstDuration)+
//                           ", secondHeading: "+std::to_string(secondHeading) +
//                           ", secondDuration: "+std::to_string(secondDuration) );
            return true;
          }
        }
      }
    }
  }

  return false;
}
}  // namespace simplexArchitectures