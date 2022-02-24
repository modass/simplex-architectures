//
// Created by bmaderbacher on 24.02.22.
//

#ifndef SIMPLEXARCHITECTURES_CONSTANTCONTROLLER_H
#define SIMPLEXARCHITECTURES_CONSTANTCONTROLLER_H

#include "AbstractController.h"

namespace simplexArchitectures {

    template<typename State, typename Update> class ConstantController: public AbstractController<State, Update>{
    public:
        ConstantController(Update update){
            mUpdate = update;
        }
        Update generateInput(State state){
            return mUpdate;
        }
    private:
        Update mUpdate;
    };

}

#endif //SIMPLEXARCHITECTURES_CONSTANTCONTROLLER_H
