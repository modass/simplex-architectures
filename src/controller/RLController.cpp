//
// Created by bmaderbacher on 24.02.22.
//

#include "RLController.h"

#include <utility>

namespace simplexArchitectures {

    Point RLController::generateInput(Point state) {
        auto x1 = (float)state.at(0);
        auto x2 = (float)state.at(1);
        auto u = (double)execute_controller(x1, x2, mWeight0, mWeight2, mBias0, mBias2);
        return Point({u});
    }

    void RLController::print_network(const VVVF& weight0, const VVVF& weight2,
                                     const VF& bias0, const VF& bias2) {
        cout << "Weight 0 \n";

        for (int i = 0; i < weight0.size(); ++i) {
            for (int j = 0; j < weight0[0].size(); ++j) {
                cout << "[ ";
                for (int k = 0; k < weight0[0][0].size(); ++k) {
                    cout << weight0[i][j][k] << ", ";
                }
                cout << " ]\n";
            }
            cout << endl;
        }

        cout << "bias 0 \n";
        for (int i = 0; i < bias0.size(); ++i) {
            cout << bias0[i] << ", ";
        }
        cout << endl;

        cout << "Weight 2 \n";

        for (int i = 0; i < weight2.size(); ++i) {
            for (int j = 0; j < weight2[0].size(); ++j) {
                cout << "[ ";
                for (int k = 0; k < weight2[0][0].size(); ++k) {
                    cout << weight2[i][j][k] << ", ";
                }
                cout << " ]\n";
            }
            cout << endl;
        }

        cout << "bias 2 \n";
        for (int i = 0; i < bias2.size(); ++i) {
            cout << bias2[i] << ", ";
        }
        cout << endl;

    }


    float RLController::ReLU(float x) {
        return x;
        if (x > 0) return x;
        return 0;
    }


    void RLController::read_network(const string &nameOfFile, VVVF& weight0, VVVF& weight2,
                      VF& bias0, VF& bias2) {
        string line;
        ifstream nameFileout;
        nameFileout.open(nameOfFile);
        getline(nameFileout, line);
        int pos1 = line.find("tensor")+10;
        int pos2 = line.find("]");
        int pos3 = line.find(",", pos1);
        weight0[0][0][0] = stof(line.substr(pos1, pos3-pos1));
        weight0[0][0][1] = stof(line.substr(pos3+1, pos2-pos3-1));

        getline(nameFileout, line);
        pos1 = line.find("[")+1;
        pos2 = line.find(",");
        weight0[0][1][0] = stof(line.substr(pos1, pos2-pos1));
        pos3 = line.find("]");
        weight0[0][1][1] = stof(line.substr(pos2+2, pos3-pos2-2));

        getline(nameFileout, line);
        getline(nameFileout, line);
        pos1 = line.find("[[")+2;
        pos2 = line.find(",");
        weight0[1][0][0] = stof(line.substr(pos1, pos2-pos1));
        pos3 = line.find("]");
        weight0[1][0][1] = stof(line.substr(pos2+2, pos3-pos2-2));

        getline(nameFileout, line);
        pos1 = line.find("[")+1;
        pos2 = line.find(",");
        weight0[1][1][0] = stof(line.substr(pos1, pos2-pos1));
        pos3 = line.find("]");
        weight0[1][1][1] = stof(line.substr(pos2+2, pos3-pos2-2));

        pos1 = line.find("tensor")+8;
        pos2 = line.find(",", pos1);
        bias0[0] = stof(line.substr(pos1, pos2-pos1));
        pos3 = line.find("]", pos1);
        bias0[1] = stof(line.substr(pos2+1, pos3-pos2-1));
        pos1 = line.find("tensor", pos1)+10;
        pos2 = line.find(",", pos1);
        pos3 = line.find("]", pos1);
        weight2[0][0][0] = stof(line.substr(pos1, pos2-pos1));
        weight2[0][0][1] = stof(line.substr(pos2+1, pos3-pos2-1));

        getline(nameFileout, line);
        pos1 = line.find("[")+1;
        pos2 = line.find(",");
        weight2[0][1][0] = stof(line.substr(pos1, pos2-pos1));
        pos3 = line.find("]");
        weight2[0][1][1] = stof(line.substr(pos2+2, pos3-pos2-2));


        for (int i = 1; i < weight2.size(); ++i) {
            getline(nameFileout, line);
            getline(nameFileout, line);
            pos1 = line.find("[[")+2;
            pos2 = line.find(",");
            weight2[i][0][0] = stof(line.substr(pos1, pos2-pos1));
            pos3 = line.find("]");
            weight2[i][0][1] = stof(line.substr(pos2+2, pos3-pos2-2));

            getline(nameFileout, line);
            pos1 = line.find("[")+1;
            pos2 = line.find(",");
            weight2[i][1][0] = stof(line.substr(pos1, pos2-pos1));
            pos3 = line.find("]");
            weight2[i][1][1] = stof(line.substr(pos2+2, pos3-pos2-2));

        }

        pos1 = line.find("tensor", pos1)+8;
        for (int i = 0; i < weight2.size(); ++i) {
            pos2 = line.find(",", pos1);
            bias2[i] = stof(line.substr(pos1, pos2-pos1));
            pos1 = pos2+1;
        }

        // print_network(weight0, weight2, bias0, bias2);
    }


    VF RLController::execute_network(VF input, const VVVF& weight0, const VVVF& weight2,
                       const VF& bias0, const VF& bias2) {
        VF layer3(bias2.size(), 0);
        VF layer1(bias0.size(), 0);

        for (int i = 0; i < weight0.size(); ++i) {
            for (int j = 0; j < weight0[0].size(); ++j) {
                for (int k = 0; k < weight0[0][0].size(); ++k) {
                    layer1[i] += weight0[i][j][k]*input[j]*input[k];
                }
            }
            layer1[i] += bias0[i];
        }

        for (int i = 0; i < weight2.size(); ++i) {
            for (int j = 0; j < weight2[0].size(); ++j) {
                for (int k = 0; k < weight2[0][0].size(); ++k) {
                    layer3[i] += weight2[i][j][k]*layer1[j]*layer1[k];
                }
            }
            layer3[i] += bias2[i];
        }

        return layer3;
    }


    float RLController::execute_controller(float height1, float height2, const VVVF& weight0, const VVVF& weight2,
                             const VF& bias0, const VF& bias2) {

        VF input(0);
        input.push_back(1*height1);
        input.push_back(1*height2);
        VF probs = execute_network(input, weight0, weight2, bias0, bias2);
        int best_action = -1;
        float best_action_value = -10000;
        for (int i = 0; i < probs.size(); ++i) {
            if (probs[i] > best_action_value) {
                best_action = i;
                best_action_value = probs[i];
            }
        }
        float result = best_action;
        result = result*5.0/(bias2.size()-1)*1e-4;
        return result;
    }

//    void RLController::readNetwork(string file) {
//        RLController::read_network(std::move(file), mWeight0, mWeight2, mBias0, mBias2);
//    }

}