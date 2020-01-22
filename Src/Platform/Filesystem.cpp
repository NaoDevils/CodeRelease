
#include "Filesystem.h"
#include "Tools/Debugging/Debugging.h"
#include <string.h>

#ifdef WINDOWS
#include "direct.h"
int Filesystem::create_directory(const char* path)
{
  return _mkdir(path);
}
// silence msvc warning
#define strdup _strdup
#endif

#if defined(LINUX) || defined(OSX) || defined(TARGET_ROBOT)
#include <sys/stat.h>

int Filesystem::create_directory(const char* path)
{
  __mode_t permissons = S_IRWXU | S_IRWXG | (S_IROTH | S_IXOTH);  // 775
  return mkdir(path, permissons);
}
#endif


int Filesystem::create_directories(const char *path, const char delimiter, const bool ignoreExists)
{
  char *work = strdup(path), *ptr;
  for (ptr = work; *ptr; ++ptr)
  {
    if (*ptr == delimiter)
    {
      *ptr = '\0';

      // paths starting with `delimiter` will yield an empty string on the first iteration, skip it
      if (strlen(work) > 0)
      {
        int result = create_directory(work);
        const bool ignoreError = ignoreExists && errno == EEXIST;
        if (result < 0 && !ignoreError)
        {
          OUTPUT_ERROR("Error creating directory " << work << " reason " << strerror(errno));
          free(work);
          return result;
        }
      }
      *ptr = delimiter;
    }
  }
  free(work);
  return 0;
}
