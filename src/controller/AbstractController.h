//
// Created by bmaderbacher on 24.02.22.
//

#ifndef SIMPLEXARCHITECTURES_ABSTRACTCONTROLLER_H
#define SIMPLEXARCHITECTURES_ABSTRACTCONTROLLER_H

namespace simplexArchitectures {

template<typename State, typename Update> class AbstractController {
    public:
    virtual Update generateInput(State state) = 0;
};

}


#endif //SIMPLEXARCHITECTURES_ABSTRACTCONTROLLER_H
