//
// Adapted from lightning for Super-LIO offline processing
//

#include "file_io.h"
#include <filesystem>

namespace LI2Sup {

bool PathExists(const std::string& file_path) {
    return std::filesystem::exists(file_path);
}

bool RemoveIfExist(const std::string& path) {
    if (PathExists(path)) {
        std::filesystem::remove(path);
        return true;
    }
    return false;
}

bool IsDirectory(const std::string& path) { 
    return std::filesystem::is_directory(path); 
}

}  // namespace LI2Sup