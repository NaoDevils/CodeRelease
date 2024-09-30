/**
 * The file declares a class that represents the blackboard containing all
 * representations used in a process.
 * The file will be included by all modules and therefore avoids including
 * headers by itself.
 * @author Thomas Röfer
 */

#pragma once
#include "Platform/BHAssert.h"
#include <memory>
#include <typeinfo>
#include <string>
#include <stdexcept>

class Streamable;

class Blackboard
{
private:
  /** A single entry of the blackboard. */
  struct Entry
  {
    std::unique_ptr<Streamable> data = nullptr; /**< The representation. */
    int counter = 0; /**< How many modules requested its existance? */
  };

  struct CopyEntry
  {
    Streamable* original = nullptr;
    Streamable* copy = nullptr;
  };

  class Entries; /**< Type of the map for all entries. */
  Entries& entries; /**< All entries of the blackboard. */
  int version = 0; /**< A version that is increased with each configuration change. */

  class CopyEntries;
  CopyEntries& copyEntries;

  friend class Process;
  friend class SuperThread;
  friend class SubThread;

  /**
   * Retrieve the blackboard entry for the name of a representation.
   * @param representation The name of the representation.
   * @return The blackboard entry. If it does not exist, the it will
   * be created, but not the representation.
   */
  Entry& get(const char* representation);
  const Entry& get(const char* representation) const;

  void addCopyEntry(const char* representation);
  void copyUsedRepresentations();

  static std::string demangle(const char* name);

public:
  /**
   * The default constructor creates the blackboard and sets it as
   * the instance of this process.
   */
  Blackboard();

  /**
   * The destructor frees the blackboard and resets the instance of
   * this process.
   */
  ~Blackboard();

  /**
   * Does a certain representation exist?
   * @param representation The name of the representation.
   * @return Does it exist in this blackboard?
   */
  bool exists(const char* representation) const;

  /**
   * Allocate a new blackboard entry for a representation of a
   * certain type and name. The representation is only created
   * if this is its first allocation.
   * @tparam T The type of the representation.
   * @param representation The name of the representation.
   * @return The representation.
   */
  template <typename T> T& alloc(const char* representation)
  {
    Entry& entry = get(representation);
    if (entry.counter++ == 0)
    {
      entry.data = std::make_unique<T>();
      ++version;

      addCopyEntry(representation);
    }
    return *dynamic_cast<T*>(entry.data.get());
  }

  /**
   * Free the blackboard entry for a representation of a certain
   * name. It is only removed if it was freed as often as it was
   * allocated.
   * @param representation The name of the representation.
   */
  void free(const char* representation);

  /**
   * Access a representation of a certain name. The representation
   * must already exist.
   * @param representation The name of the representation.
   * @return The instance of the representation in the blackboard.
   */
  Streamable& operator[](const char* representation);
  const Streamable& operator[](const char* representation) const;

  /**
   * Access a representation of a certain type in current process's blackboard.
   * The representation must already exist.
   * @tparam T The type of the representation.
   * @param copy Return copied representation for USES() dependencies.
   * @return The instance of the representation in the blackboard.
  */
  template <typename T> static const T& get(bool copy = false)
  {
    const char* const name = typeid(T).name();
    const std::string demangledName = demangle(name) + (copy ? "-Copy" : "");

    const Blackboard& bb = getInstance();

    if (!bb.exists(demangledName.c_str()))
      throw std::out_of_range("Representation " + demangledName + " does not exist!");

    return static_cast<const T&>(bb[demangledName.c_str()]);
  }

  /**
   * Return the current version.
   * It can be used to determine whether the configuration of the
   * blackboard changed.
   * @return The current version. It starts with 0 and might
   *         be increased in larger steps.
   */
  int getVersion() const { return version; }

  /**
   * Access the blackboard of this process.
   * @return The instance that belongs to this process.
   */
  static Blackboard& getInstance();
};
