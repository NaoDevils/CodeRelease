/**
 * @file Tools/Enum.h
 * Defines a macro that declares an enum and provides a function to access the names of its elements.
 *
 * Enums have the following form:
 * ENUM(<enum-name>,
 *   <comma-separated-enum-list>
 * )
 *
 * Example:
 * ENUM(Fruit,
 *   apple,
 *   banana,
 *   strawberry
 * );
 *
 * @author Thomas Röfer
 * @author Alexis Tsogias
 * @author Jesse Richter-Klug
 * @author Aaron Larisch
 */

#pragma once
#include <array>
#include <cstring>

/**
 * @param enums A string that contains a comma-separated list of enum
 *              elements. It is allowed that an element is initialized
 *              with the value of its predecessor. Any other
 *              initializations are forbidden.
 *              "a, b, numOfLettersBeforeC, c = numOfLettersBeforeC, d"
 *              would be a legal parameter.
 * @param numOfEnums The number of enums in the string. Reassignments do
 *                   not count, i.e. in the example above, this
 *                   parameter had to be 4.
 * @return An array of string pointers that will be filled with
 *         the names of the enums.
 */
template <size_t numOfEnums> std::array<const char*, numOfEnums> enumInit(char* enums)
{
  const auto trim = [](char* pBegin, char* pEnd)
  {
    while (*pBegin == ' ')
      ++pBegin;
    while (pEnd > pBegin && pEnd[-1] == ' ')
      --pEnd;
    *pEnd = 0;
    return pBegin;
  };

  std::array<const char*, numOfEnums> names;
  char* pEnd = enums - 1;
  int index = 0;
  bool end;
  do
  {
    char* pBegin = pEnd + 1;
    pEnd = strchr(pBegin, ',');
    end = !pEnd;
    if (end)
      pEnd = pBegin + strlen(pBegin);
    char* name = trim(pBegin, pEnd);
    char* p = strchr(name, '=');
    if (p)
    {
      //ASSERT(index && !strcmp(trim(p + 1, pEnd), names[index - 1]));
      name = trim(name, p);
      --index;
    }
    //ASSERT(index < numOfEnums);
    names[index++] = name;
  } while (!end);
  return names;
}

/**
 * Convert arguments to a string. Also unwraps macros before.
 */
#define _ENUM_QUOTE(...) #__VA_ARGS__

/**
 * Defining an enum and a function getName(<Enum>) that can return
 * the name of each enum element. The enum will automatically
 * contain an element numOf<Enum>s that reflects the number of
 * elements defined.
 * 
 * WARNING: Assigning values to enum-elements is not allowed with these enums
 * since they are designed for streaming and getName(<Enum>) expects them to
 * have no assignments. The only exception is the assignment of a previous
 * enum-element.
 * Example:
 *
 * ENUM(Fruit,
 *   strawberry,
 *   blueberry,
 *   numOfBerries,
 *   apple = numOfBerries,
 *   banana
 * );
 */
#define ENUM(Enum, ...)                                                                           \
  enum Enum : unsigned char                                                                       \
  {                                                                                               \
    __VA_ARGS__,                                                                                  \
    numOf##Enum##s                                                                                \
  };                                                                                              \
  inline static const char* getName(Enum e)                                                       \
  {                                                                                               \
    static char enums[] = _ENUM_QUOTE(__VA_ARGS__);                                               \
    static const std::array<const char*, numOf##Enum##s> names = enumInit<numOf##Enum##s>(enums); \
    return e >= numOf##Enum##s ? 0 : names[e];                                                    \
  }

/**
 * BIGENUM(Fruit,
 *   (
 *     strawberry,
 *     blueberry
 *   ),(
 *     apple,
 *     banana
 *   )
 * );
 */
#define BIGENUM(Enum, a, b)                                                                       \
  enum Enum : unsigned char                                                                       \
  {                                                                                               \
    _BIGENUM a,                                                                                   \
    _BIGENUM b,                                                                                   \
    numOf##Enum##s                                                                                \
  };                                                                                              \
  inline static const char* getName(Enum e)                                                       \
  {                                                                                               \
    static char enums[] = _ENUM_QUOTE a "," _ENUM_QUOTE b;                                        \
    static const std::array<const char*, numOf##Enum##s> names = enumInit<numOf##Enum##s>(enums); \
    return e >= numOf##Enum##s ? 0 : names[e];                                                    \
  }

#define _BIGENUM(...) __VA_ARGS__
