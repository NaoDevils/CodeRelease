//
// Created by simon on 19.11.19.
//

#pragma once

#include "direct.h"

int create_directory(const char *path)
{
  return _mkdir(path);
}
