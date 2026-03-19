//
// Adapted from lightning for Super-LIO offline processing
//

#include "yaml_io.h"

#include <glog/logging.h>
#include <fstream>

namespace LI2Sup {

YAML_IO::YAML_IO(const std::string &path) {
    path_ = path;
    try {
        yaml_node_ = YAML::LoadFile(path_);
        is_opened_ = true;
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to open yaml: " << path_ << ", error: " << e.what();
        is_opened_ = false;
    }
}

}  // namespace LI2Sup