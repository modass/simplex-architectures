/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 04.07.23.
 */

#include "StateActionMap.h"

namespace simplexArchitectures {

    template<typename ValueSet, typename Action>
    std::optional<Action>
    StateActionMap<ValueSet, Action>::getAction(const Location &location, const Point &valuation) const {
        if (mValuations.find(&location) != mValuations.end()) {
            for (const auto &[valuations, action]: mValuations[&location]) {
                if (valuations.contains(valuation)) {
                    return action;
                }
            }
        }
        return std::nullopt;
    }

    template<typename ValueSet, typename Action>
    void StateActionMap<ValueSet, Action>::add(const Location &location, const ValueSet &values, const Action &action) {
        mValuations[&location].push_back({values, action});
    }

} // namespace
