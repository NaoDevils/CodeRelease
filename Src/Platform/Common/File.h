/**
* @file Platform/Common/File.h
* Declaration of class File for Windows and Linux.
*/

#pragma once

#include <string>
#include <list>

/**
 * This class provides basic file input/output capabilities.
 */
class File
{
public:
  /**
   * Constructor.
   * @param name File name or path. If it is a relative path, it is assumed
   *             to be relative to the path for configuration files. Otherwise,
   *             the path is used directly.
   * @param mode File open mode as used by fopen defined in stdio.h.
   * @param tryAlternatives Try alternative paths.
   */
  File(const std::string& name, const char* mode, bool tryAlternatives = true);

  /**
   * Destructor.
   */
  ~File();

  /**
   * The method returns a list of full file names that should be searched
   * to find the file with a given name.
   * @param name The name of the file to search for.
   * @return The list of candidate full names. The caller has to check whether
   *         these files exist in the sequence of the list.
   */
  static std::list<std::string> getFullNames(const std::string& name);

  /**
   * The method returns a list of exiting full file names that can be read in
   * the returned order
   * @param name The name of the file to search for.
   * @return The list of exiting config files in hierarchical order.
   */
  static std::list<std::string> getFullNamesHierarchy(const std::string& name);

  /**
   * The method returns a list of directories that may contain config files.
   * @return The list of config directories.
   */
  static std::list<std::string> getConfigDirectories();

  /**
   * The function read a number of bytes from the file to a certain
   * memory location.
   * @param p The start of the memory space the data is written to.
   * @param size The number of bytes read from the file.
   */
  void read(void* p, size_t size);

  /**
   * The function read a line (up to \c size bytes long) from the file to a certain
   * memory location.
   * @param p The start of the memory space the data is written to.
   * @param size The maximal length of the line read from the file.
   * @return \c p on success, \c 0 otherwise
   */
  char* readLine(char* p, size_t size);

  /**
   * The function writes a number of bytes from a certain memory
   * location into the file.
   * @param p The start of the memory space the data is read from.
   * @param size The number of bytes written into the file.
   */
  void write(const void* p, size_t size);

  /**
   * The function implements printf for the stream represented by
   * instances of this class.
   * @param format Format string as used by printf defined in stdio.h.
   * @param ... See printf in stdio.h.
   */
  void printf(const char* format, ...);

  /**
   * The function returns whether the file represented by an
   * object of this class actually exists.
   * @return The existence of the file.
   */
  bool exists() const { return stream != 0; }

  /**
   * The function returns whether the end of the file represented
   * by an object of this class was reached.
   * @return End of file reached?
   */
  bool eof();

  /**
   * The function returns the size of the file
   * @return The size of the file in bytes
   */
  size_t getSize();

  /**
   * The function returns the full path of the file.
   * @return The full path name actually used or the file searched for
   *         if it was not found. If the file was opened, the path can
   *         still be relative to the current directory if the Framework
   *         directory was specified this way.
   */
  std::string getFullName() const { return fullName; }

  /**
   * Maps the file in memory.
   * @return Pointer to first byte.
   */
  char* getMemoryMappedFile();

  /**
   * Returns the current byte position of the file.
   * @return Byte position.
   */
  size_t getPosition();

  /**
  * The function returns the current Framework directory,
  * e.g. /home/nao or <..>/NDevils
  * @return The current BHDir
  */
  static const char* getBHDir();

#ifndef TARGET_TOOL
  /**
  * The function returns the persistent config directory,
  * e.g. /home/nao/Persistent/<robotname>
  * @return The persistent config directory
  */
  static std::string getPersistentDir();
#endif

  /**
   * Checks if the delivered path is an absolute path.
   * Empty pathes are handled as relative pathes.
   * @param path  Must be a valid c string.
   * @return true, if the delivered path is absolute.
   */
  static bool isAbsolute(const char* path);

private:
  void* stream; /**< File handle. */
  std::string fullName; /**< The full path name actually used or the file searched for if it was not found. */

  char* map = nullptr; /**< Pointer to memory-mapped file. */
#ifdef WINDOWS
  // void* types because we don't want to include Windows.h here
  void* hFile = nullptr;
  void* hFileMapping = nullptr;
#else
  long size = 0;
#endif
};
