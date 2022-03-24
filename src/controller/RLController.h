//
// Created by bmaderbacher on 24.02.22.
//

#ifndef SIMPLEXARCHITECTURES_RLCONTROLLER_H
#define SIMPLEXARCHITECTURES_RLCONTROLLER_H

#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include "AbstractController.h"
#include "../types.h"

using namespace std;

typedef vector<float> VF;
typedef vector<VF> VVF;
typedef vector<VVF> VVVF;

namespace simplexArchitectures {

    class RLController: public AbstractController<Point, Point> {
    public:
        explicit RLController(string file) :
            mWeight0(input_dim, VVF(input_dim, VF(input_dim,0))),
            mBias0(input_dim, 0),
            mWeight2(output_dim, VVF(input_dim, VF(input_dim,0))),
            mBias2(output_dim, 0){
            RLController::read_network(std::move(file), mWeight0, mWeight2, mBias0, mBias2);
        }
        Point generateInput(Point state) override;
    private:
        static void print_network(const VVVF& weight0, const VVVF& weight2,
                           const VF& bias0, const VF& bias2);
        static float ReLU(float x);
        static void read_network(const string &nameOfFile, VVVF& weight0, VVVF& weight2,
                          VF& bias0, VF& bias2);
        static VF execute_network(VF input, const VVVF& weight0, const VVVF& weight2,
                           const VF& bias0, const VF& bias2);
        static float execute_controller(float height1, float height2, const VVVF& weight0, const VVVF& weight2,
                                 const VF& bias0, const VF& bias2);

        const int input_dim = 2;
        const int output_dim = 11;
        constexpr static const float max_flow = 2;
        VVVF mWeight0;
        VVVF mWeight2;
        VF mBias0;
        VF mBias2;
    };

}


#endif //SIMPLEXARCHITECTURES_RLCONTROLLER_H
