/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 20.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_TRAINER_H
#define SIMPLEXARCHITECTURES_TRAINER_H

#include "../types.h"
#include "../utility/fileExists.h"
#include <cereal/archives/binary.hpp>
#include <fstream>
#include <hypro/datastructures/Hyperoctree.h>

namespace simplexArchitectures {

class Trainer {
public:
  Trainer(std::string filename) : mFilename(filename) {
    if (fileExists(mFilename)) {
      {
        std::ifstream fs{mFilename, std::ios::binary};
        cereal::BinaryInputArchive iarchive(fs);
        iarchive(mTree);
      }
    }
  }
  ~Trainer() {
    // write tree to file upon destruction
    std::ofstream fs{mFilename, std::ios::binary};
    cereal::BinaryOutputArchive oarchive(fs);
    oarchive(mTree);
  }

protected:
  std::string mFilename = "treearchive";
  hypro::Hyperoctree<Number> mTree;
};

} // namespace simplexArchitectures

#endif // SIMPLEXARCHITECTURES_TRAINER_H
