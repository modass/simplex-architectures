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
                if (valuations.contains(valuation)) {
                    return action;
                }
            }
        }
        return std::nullopt;
    }

    template<typename ValueSet, typename Action>
    void StateActionMap<ValueSet, Action>::add(std::string locationName, const ValueSet &values, const Action &action) {
        if (mValuations.find(locationName) != mValuations.end()) {
            mValuations[locationName].emplace_back(values, action);
        } else {
            mValuations[locationName] = {{values, action}};
        }
    }

} // namespace
