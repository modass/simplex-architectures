/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 04.07.23.
 */

#ifndef SIMPLEXARCHITECTURES_STATEACTIONMAP_H
#define SIMPLEXARCHITECTURES_STATEACTIONMAP_H

#include "../types.h"

namespace simplexArchitectures {

    template<typename ValueSet, typename Action>
    struct ValueActionPair {
        ValueSet values;
        Action action;
    };

    template<typename ValueSet, typename Action>
    class StateActionMap {
        using VAPair = ValueActionPair<ValueSet, Action>;
    public:

        /**
         * Lookup of the correct action for a given location and valuation. If not contained, returns an empty optional. If multiple actions are stored, returns the oldest one that has been stored.
         * @param location The location
         * @param valuation The valuation, represented as a point
         * @return An optional either containing an action in case of success or nothing in case no action has been stored for the location-valuation pair
         */
        std::optional<Action> getAction(const Location &location, const Point &valuation) const;

        /**
         * Add information to the mapping
         * @param location The location
         * @param values The set of valuation
         * @param action The targeted action for this state set
         */
        void add(const Location &location, const ValueSet &values, const Action &action);

    protected:
        std::map<Location *, std::vector<VAPair>> mValuations; ///< stores all information
    };

} // namespace

#include "StateActionMap.tpp"

#endif //SIMPLEXARCHITECTURES_STATEACTIONMAP_H
