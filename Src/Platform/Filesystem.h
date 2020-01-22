#pragma once

namespace Filesystem
{
  int create_directory(const char* path);
  int create_directories(const char* path, const char delimiter = '/', const bool ignoreExists = true);
}
