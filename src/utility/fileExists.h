/*
 * Created by Stefan Schupp <stefan.schupp@tuwien.ac.at> on 20.02.22.
 */

#ifndef SIMPLEXARCHITECTURES_FILEEXISTS_H
#define SIMPLEXARCHITECTURES_FILEEXISTS_H

#include <string>
#include <sys/stat.h>

namespace simplexArchitectures {

/**
 * Taken from
 * https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exists-using-standard-c-c11-14-17-c
 * Tests whether a file exists.
 * @param name Filename
 * @return True if the file <filename> exists
 */
inline bool fileExists(const std::string &name) {
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

} // namespace simplexArchitectures

#endif // SIMPLEXARCHITECTURES_FILEEXISTS_H
