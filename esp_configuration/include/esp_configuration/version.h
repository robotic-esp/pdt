#pragma once

#include <string>

namespace esp {

namespace ompltools {

struct Version {
  static const std::string GIT_SHA1;
  static const std::string GIT_REFSPEC;
  static const std::string GIT_STATUS;
};

}  // namespace ompltools

}  // namespace esp
