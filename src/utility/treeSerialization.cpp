/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 03.03.22.
 */

#include "treeSerialization.h"
#include <cereal/archives/binary.hpp>
#include <cereal/types/map.hpp>
#include <fstream>
#include "fileExists.h"
#include <spdlog/spdlog.h>

namespace simplexArchitectures {

std::map<std::string, hypro::Hyperoctree<Number>> loadTrees(std::string filename) {
  std::map<std::string, hypro::Hyperoctree<Number>> res;
    if ( fileExists( filename ) ) {
      {
        std::ifstream fs{filename, std::ios::binary};
        cereal::BinaryInputArchive iarchive( fs );
        iarchive( res );
      }
  } else {
    throw std::ios_base::failure("Input file " + filename + " does not exist.");
  }
  return res;
}

void saveTrees(std::string filename, const std::map<std::string, hypro::Hyperoctree<Number>>& trees) {
  if( fileExists(filename)) {
    spdlog::warn("Output file {} exists already, will be overridden.",filename);
  }
  {
    std::ofstream fs{filename, std::ios::binary};
    cereal::BinaryOutputArchive oarchive(fs);
    oarchive(trees);
  }
}

} // namespace simplexArchitectures