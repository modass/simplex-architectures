/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 04.07.23.
 */

#include "StateActionMap.h"

namespace simplexArchitectures {

    template<typename ValueSet, typename Action>
    std::optional<Action>
    StateActionMap<ValueSet, Action>::getAction(std::string locationName, const Point &valuation) const {
        if (mValuations.find(locationName) != mValuations.end()) {
            for (const auto &[valuations, action]: mValuations.at(locationName)) {
              auto valuation2 = valuation.projectOn({0,1});
              if (valuations.contains(valuation2)) {
                return action;
              }
            }
        }
        return std::nullopt;
    }

    template<typename ValueSet, typename Action>
    void StateActionMap<ValueSet, Action>::add(std::string locationName, const ValueSet &values, const Action &action) {
        auto values2 = values.projectOn({0,1}); //TODO spec timer should be included as well if possible
        if (mValuations.find(locationName) != mValuations.end()) {
            mValuations[locationName].emplace_back(values2, action);
        } else {
            mValuations[locationName] = {{values2, action}};
        }
    }

} // namespace
