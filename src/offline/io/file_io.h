//
// Adapted from lightning for Super-LIO offline processing
//

#ifndef SUPER_LIO_FILE_IO_H
#define SUPER_LIO_FILE_IO_H

#include <string>

namespace LI2Sup {

/// 检查某个路径是否存在
bool PathExists(const std::string& file_path);

/// 若文件存在，则删除之
bool RemoveIfExist(const std::string& path);

/// 判断某路径是否为目录
bool IsDirectory(const std::string& path);

}

#endif  // SUPER_LIO_FILE_IO_H