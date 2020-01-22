//
// Created by simon on 19.11.19.
//

#pragma once

#include <sys/stat.h>

int create_directory(const char *path)
{
  __mode_t permissons = S_IRWXU | S_IRWXG | (S_IROTH | S_IXOTH);  // 775
  return mkdir(path, permissons);
}
