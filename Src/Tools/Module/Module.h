/**
 * @file Module.h
 * Definition of the module handling schema.
 *
 * This file provides the ability to specify the requirements of and the
 * representations provided by a module. Example:
 *
 * MODULE(MyImageProcessor,
 *   REQUIRES(Image),                      // Has to be updated before
 *   REQUIRES(CameraMatrix),               // Has to be updated before
 *   USES(RobotPose),                      // Is used, but has not to be updated before
 *   PROVIDES(BallPercept),                // Class provides a method to update BallPercept.
 *   PROVIDES_WITHOUT_MODFY(GoalPercept),  // Class provides a method to update GoalsPercept. Representation cannot be MODIFYed.
 *   DEFINES_PARAMETERS(,                  // Has parameters that must have an initial value. If LOADS_PARAMETERS is used instead,
 *                                         // they are loaded from a configuration file that has the same name as the module defined, but starts with lowercase letters.
 *     (float)(42.0f) focalLen,            // Has a parameter named focalLen of type float. By default it has the value 42.0f.
 *     (int)(21) resWidth,                 // All attributes are streamed and can be accessed by requesting 'parameters:MyImageProcessor'
 *     (CameraInfo, Camera)(camera) lower  // Use an enum as parameter that was defined in another class.
 *   )
 * );
 *
 * This block defines a base class MyImageProcessorBase. The module MyImageProcessor has to be
 * derived from that class:
 *
 * class MyImageProcessor : public MyImageProcessorBase
 * {
 *   void update(BallPercept& ballPercept);
 *   void update(GoalPercept& goalPercept);
 * };
 *
 * In the implementation file, the existence of the module has to be announced:
 *
 * MAKE_MODULE(MyImageProcessor, perception)
 *
 * The second parameter defines a category that is used to group modules.
 *
 * @author Thomas Röfer
 * @author Aaron Larisch
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Blackboard.h"

namespace tf
{
  class Subflow;
}

/**
 * The class is the abstract base of all template classes that create modules.
 */
class ModuleBase
{
private:
  /**
   * Helper class to check whether a class defines a draw method itself.
   */
  template <typename T> struct HasDrawMethod
  {
    template <typename U, void (U::*)()> struct Draw
    {
    };
    template <typename U, void (U::*)() const> struct ConstDraw
    {
    };
    template <typename U> static char Test(Draw<U, &U::draw>*);
    template <typename U> static char Test(ConstDraw<U, &U::draw>*);
    template <typename U> static int Test(...);
    static const bool value = sizeof(Test<T>(0)) == sizeof(char);
  };

  /**
   * This class defines a method that is called when a class has a
   * draw() method. The latter is called.
   * @param T The type of the representation.
   * @param hasDraw States, whether T has a draw() method.
   */
  template <typename T, bool hasDraw = true> struct Draw
  {
    static void draw(T& t) { t.draw(); }
  };

  /**
   * This class defines a method that is called when a class does not
   * have a draw() method.
   * @param T The type of the representation.
   */
  template <typename T> struct Draw<T, false>
  {
    static void draw(T& t) {}
  };

public:
  ENUM(Category,
    cognitionInfrastructure,
    perception,
    modeling,
    pathPlanning,
    behaviorControl,
    motionInfrastructure,
    sensing,
    motionControl,
    dortmundWalkingEngine
  );

  static const unsigned char numOfCategories = numOfCategorys;

  class Info
  {
  public:
    ENUM(Property,
      concurrent,
      require, // s ommited because "requires" is a reserved keyword since C++20
      provide,
      use,
      preexecution
    );

    const char* representation;
    void (*update)(Streamable&);
    void (*execute)(Streamable&, tf::Subflow&);
    const std::unordered_set<Property> properties;

    Info(const char* representation, void (*update)(Streamable&), void (*execute)(Streamable&, tf::Subflow&), const std::unordered_set<Property>& properties)
        : representation(representation), update(update), execute(execute), properties(properties)
    {
    }

    bool hasProperty(Property property) const { return properties.find(property) != properties.end(); }

    using const_iterator = const Info*;
    const_iterator begin() const { return this; }
    const_iterator end() const
    {
      const Info* end = this;
      while (end->representation != nullptr)
        ++end;
      return end;
    }
  };

  /**
   * Find message id for representation name.
   * @param name The name of the representation.
   * @return The corresponding id of undefined if it does not exist.
   */
  static MessageID getMessageID(const std::string& name)
  {
    for (int i = 0; i < numOfMessageIDs; ++i)
      if (name == ::getName(static_cast<MessageID>(i)) + 2)
        return static_cast<MessageID>(i);
    return ::undefined;
  }

  /**
   * Calls a draw method if a representation has one.
   * @tparam T The type of the representation.
   * @param t The representation.
   */
  template <typename T> static void draw(T& t) { Draw<T, HasDrawMethod<T>::value>::draw(t); }

private:
  static ModuleBase* list; /**< The head of the list of all modules available. */
  ModuleBase* next; /**< The next entry in the list of all modules. */
  const char* name; /**< The name of the module that can be created by this instance. */
  Category category; /**< The category of this module. */
  const Info* infos; /**< Information about the requirements and provisions of the module. */

  const ModuleBase::Info* getInfoByRepresentation(std::string_view representation) const;
  const ModuleBase::Info* getInfoByRepresentation(std::string_view representation, ModuleBase::Info::Property property) const;
  std::unordered_set<const ModuleBase::Info*> getInfos(ModuleBase::Info::Property property) const;

  class iterator
  {
    ModuleBase* elem = nullptr;

  public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = ModuleBase;
    using difference_type = void;
    using pointer = ModuleBase*;
    using reference = ModuleBase&;

    iterator(ModuleBase* elem) : elem(elem) {}
    iterator& operator++()
    {
      elem = elem->next;
      return *this;
    }
    bool operator==(iterator other) const { return !(elem != other.elem); }
    bool operator!=(iterator other) const { return elem != other.elem; }
    ModuleBase& operator*() { return *elem; }
  };
  iterator begin() const { return list; }
  iterator end() const { return nullptr; }

protected:
  /**
   * Abstract method to create an instance of a module.
   * @return The address of the instance created.
   */
  virtual std::unique_ptr<Streamable> createNew() = 0;

public:
  /**
   * Constructor.
   * @param name The name of the module that can be created by this instance.
   * @param category The category of this module.
   */
  ModuleBase(const char* name, Category category, const Info* info) : next(list), name(name), category(category), infos(info) { list = this; }

  friend class ModuleManager; /**< The ModuleManager gathers all private data. */
};

/**
 * @class Module
 * The template class provides a method to create a certain module, and it
 * registers all requirements and representations provided by that class.
 * @param M The type of the module created.
 * @param B The base class of the module.
 */
template <class M, class B> class Module : public ModuleBase
{
private:
  /**
   * The method creates an instance of the module.
   * @return The address of the newly created instance.
   */
  std::unique_ptr<Streamable> createNew() { return std::make_unique<M>(); }

public:
  /**
   * Constructor.
   * The constructor registers all requirements and representations provided
   * of the module that can be created by this instance.
   * @attention Since the module itself should not be created to just register
   * its requirements and representations, because the constructor of the module
   * may do time-consuming operations, the base class has to be used directly
   * to perform the registration. However, the base class is abstract, so it
   * cannot be constructed. To overcome this limitation, an assignment is faked,
   * and it is used to do the registration of the information required.
   * @param name The name of the module that can be created by this instance.
   * @param category The category of this module.
   */
  Module(const char* name, Category category) : ModuleBase(name, category, B::getModuleInfo()) {}
};

/**
 * If a module has no parameters, it is derived from this class.
 */
_STREAM_STREAMABLE_I(0, NoParameters, Streamable, , ());

/**
 * Load the parameters of a module.
 * @param parameters The parameters.
 * @param moduleName The filename is determined from the name of the module if it
 *                   is not explicitly specified.
 * @param fileName The filename used of 0 if it should be created from the module's name.
 */
void loadModuleParameters(Streamable& parameters, const char* moduleName, const char* fileName);

std::string getConfigName(const char* moduleName);

// clang-format off

// Some of the following macros can also be found in AutoStreamable.h with different names.
// However, separate versions are required here, because the preprocessor only expands each
// macro once in a recursive structure.

/**
 * Determine the number of entries in a tuple.
 */
#define _MODULE_TUPLE_SIZE(...) _MODULE_TUPLE_SIZE_I((__VA_ARGS__ __VA_OPT__(,) _MODULE_TUPLE_SIZE_III))
#define _MODULE_TUPLE_SIZE_I(params) _MODULE_TUPLE_SIZE_II params
#define _MODULE_TUPLE_SIZE_II( \
                               a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, \
                               a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, \
                               a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, \
                               a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, \
                               a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, \
  a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a120, \
  a121, a122, a123, a124, a125, a126, a127, ...) a127
#define _MODULE_TUPLE_SIZE_III \
  127, 126, 125, 124, 123, 122, 121, 120,                                                             \
  119, 118, 117, 116, 115, 114, 113, 112, 111, 110, 109, 108, 107, 106, 105, 104, 103, 102, 101, 100, \
  99, 98, 97, 96, 95, 94, 93, 92, 91, 90, 89, 88, 87, 86, 85, 84, 83, 82, 81, 80, \
  79, 78, 77, 76, 75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 65, 64, 63, 62, 61, 60, \
  59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, \
  39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, \
  19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0

/**
 * Apply a macro to all elements of a tuple.
 */
#define _MODULE_ATTR_0(f, ...)
#define _MODULE_ATTR_1(f, a1) f(a1)
#define _MODULE_ATTR_2(f, a1, a2) f(a1) f(a2)
#define _MODULE_ATTR_3(f, a1, a2, a3) f(a1) f(a2) f(a3)
#define _MODULE_ATTR_4(f, a1, a2, a3, a4) f(a1) f(a2) f(a3) f(a4)
#define _MODULE_ATTR_5(f, a1, a2, a3, a4, a5) f(a1) f(a2) f(a3) f(a4) f(a5)
#define _MODULE_ATTR_6(f, a1, a2, a3, a4, a5, a6) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6)
#define _MODULE_ATTR_7(f, a1, a2, a3, a4, a5, a6, a7) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7)
#define _MODULE_ATTR_8(f, a1, a2, a3, a4, a5, a6, a7, a8) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8)
#define _MODULE_ATTR_9(f, a1, a2, a3, a4, a5, a6, a7, a8, a9) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9)
#define _MODULE_ATTR_10(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10)
#define _MODULE_ATTR_11(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11)
#define _MODULE_ATTR_12(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12)
#define _MODULE_ATTR_13(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13)
#define _MODULE_ATTR_14(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14)
#define _MODULE_ATTR_15(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15)
#define _MODULE_ATTR_16(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16)
#define _MODULE_ATTR_17(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17)
#define _MODULE_ATTR_18(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18)
#define _MODULE_ATTR_19(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19)
#define _MODULE_ATTR_20(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20)
#define _MODULE_ATTR_21(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21)
#define _MODULE_ATTR_22(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22)
#define _MODULE_ATTR_23(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23)
#define _MODULE_ATTR_24(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24)
#define _MODULE_ATTR_25(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25)
#define _MODULE_ATTR_26(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26)
#define _MODULE_ATTR_27(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27)
#define _MODULE_ATTR_28(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28)
#define _MODULE_ATTR_29(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29)
#define _MODULE_ATTR_30(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30)
#define _MODULE_ATTR_31(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31)
#define _MODULE_ATTR_32(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32)
#define _MODULE_ATTR_33(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33)
#define _MODULE_ATTR_34(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34)
#define _MODULE_ATTR_35(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35)
#define _MODULE_ATTR_36(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36)
#define _MODULE_ATTR_37(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37)
#define _MODULE_ATTR_38(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38)
#define _MODULE_ATTR_39(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39)
#define _MODULE_ATTR_40(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40)
#define _MODULE_ATTR_41(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41)
#define _MODULE_ATTR_42(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42)
#define _MODULE_ATTR_43(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43)
#define _MODULE_ATTR_44(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44)
#define _MODULE_ATTR_45(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45)
#define _MODULE_ATTR_46(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46)
#define _MODULE_ATTR_47(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47)
#define _MODULE_ATTR_48(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48)
#define _MODULE_ATTR_49(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49)
#define _MODULE_ATTR_50(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50)
#define _MODULE_ATTR_51(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51)
#define _MODULE_ATTR_52(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52)
#define _MODULE_ATTR_53(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53)
#define _MODULE_ATTR_54(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54)
#define _MODULE_ATTR_55(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55)
#define _MODULE_ATTR_56(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56)
#define _MODULE_ATTR_57(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57)
#define _MODULE_ATTR_58(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58)
#define _MODULE_ATTR_59(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59)
#define _MODULE_ATTR_60(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60)
#define _MODULE_ATTR_61(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61)
#define _MODULE_ATTR_62(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62)
#define _MODULE_ATTR_63(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63)
#define _MODULE_ATTR_64(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64)
#define _MODULE_ATTR_65(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65)
#define _MODULE_ATTR_66(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66)
#define _MODULE_ATTR_67(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67)
#define _MODULE_ATTR_68(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68)
#define _MODULE_ATTR_69(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69)
#define _MODULE_ATTR_70(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70)
#define _MODULE_ATTR_71(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71)
#define _MODULE_ATTR_72(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72)
#define _MODULE_ATTR_73(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73)
#define _MODULE_ATTR_74(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74)
#define _MODULE_ATTR_75(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75)
#define _MODULE_ATTR_76(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76)
#define _MODULE_ATTR_77(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77)
#define _MODULE_ATTR_78(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78)
#define _MODULE_ATTR_79(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79)
#define _MODULE_ATTR_80(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80)
#define _MODULE_ATTR_81(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81)
#define _MODULE_ATTR_82(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82)
#define _MODULE_ATTR_83(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83)
#define _MODULE_ATTR_84(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84)
#define _MODULE_ATTR_85(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85)
#define _MODULE_ATTR_86(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86)
#define _MODULE_ATTR_87(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87)
#define _MODULE_ATTR_88(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88)
#define _MODULE_ATTR_89(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89)
#define _MODULE_ATTR_90(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90)
#define _MODULE_ATTR_91(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91)
#define _MODULE_ATTR_92(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92)
#define _MODULE_ATTR_93(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93)
#define _MODULE_ATTR_94(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94)
#define _MODULE_ATTR_95(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95)
#define _MODULE_ATTR_96(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96)
#define _MODULE_ATTR_97(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97)
#define _MODULE_ATTR_98(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98)
#define _MODULE_ATTR_99(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99)
#define _MODULE_ATTR_100(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100)
#define _MODULE_ATTR_101(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101)
#define _MODULE_ATTR_102(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102)
#define _MODULE_ATTR_103(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103)
#define _MODULE_ATTR_104(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104)
#define _MODULE_ATTR_105(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105)
#define _MODULE_ATTR_106(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106)
#define _MODULE_ATTR_107(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107)
#define _MODULE_ATTR_108(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108)
#define _MODULE_ATTR_109(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109)
#define _MODULE_ATTR_110(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110)
#define _MODULE_ATTR_111(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111)
#define _MODULE_ATTR_112(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112)
#define _MODULE_ATTR_113(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113)
#define _MODULE_ATTR_114(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113) f(a114)
#define _MODULE_ATTR_115(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113) f(a114) f(a115)
#define _MODULE_ATTR_116(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115, a116) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113) f(a114) f(a115) f(a116)
#define _MODULE_ATTR_117(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115, a116, a117) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113) f(a114) f(a115) f(a116) f(a117)
#define _MODULE_ATTR_118(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115, a116, a117, a118) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113) f(a114) f(a115) f(a116) f(a117) f(a118)
#define _MODULE_ATTR_119(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113) f(a114) f(a115) f(a116) f(a117) f(a118) f(a119)
#define _MODULE_ATTR_120(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a120) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113) f(a114) f(a115) f(a116) f(a117) f(a118) f(a119) f(a120)
#define _MODULE_ATTR_121(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a120, a121) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113) f(a114) f(a115) f(a116) f(a117) f(a118) f(a119) f(a120) f(a121)
#define _MODULE_ATTR_122(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a120, a121, a122) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113) f(a114) f(a115) f(a116) f(a117) f(a118) f(a119) f(a120) f(a121) f(a122)
#define _MODULE_ATTR_123(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a120, a121, a122, a123) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113) f(a114) f(a115) f(a116) f(a117) f(a118) f(a119) f(a120) f(a121) f(a122) f(a123)
#define _MODULE_ATTR_124(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a120, a121, a122, a123, a124) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113) f(a114) f(a115) f(a116) f(a117) f(a118) f(a119) f(a120) f(a121) f(a122) f(a123) f(a124)
#define _MODULE_ATTR_125(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a120, a121, a122, a123, a124, a125) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113) f(a114) f(a115) f(a116) f(a117) f(a118) f(a119) f(a120) f(a121) f(a122) f(a123) f(a124) f(a125)
#define _MODULE_ATTR_126(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a120, a121, a122, a123, a124, a125, a126) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113) f(a114) f(a115) f(a116) f(a117) f(a118) f(a119) f(a120) f(a121) f(a122) f(a123) f(a124) f(a125) f(a126)
#define _MODULE_ATTR_127(f, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a120, a121, a122, a123, a124, a125, a126, a127) f(a1) f(a2) f(a3) f(a4) f(a5) f(a6) f(a7) f(a8) f(a9) f(a10) f(a11) f(a12) f(a13) f(a14) f(a15) f(a16) f(a17) f(a18) f(a19) f(a20) f(a21) f(a22) f(a23) f(a24) f(a25) f(a26) f(a27) f(a28) f(a29) f(a30) f(a31) f(a32) f(a33) f(a34) f(a35) f(a36) f(a37) f(a38) f(a39) f(a40) f(a41) f(a42) f(a43) f(a44) f(a45) f(a46) f(a47) f(a48) f(a49) f(a50) f(a51) f(a52) f(a53) f(a54) f(a55) f(a56) f(a57) f(a58) f(a59) f(a60) f(a61) f(a62) f(a63) f(a64) f(a65) f(a66) f(a67) f(a68) f(a69) f(a70) f(a71) f(a72) f(a73) f(a74) f(a75) f(a76) f(a77) f(a78) f(a79) f(a80) f(a81) f(a82) f(a83) f(a84) f(a85) f(a86) f(a87) f(a88) f(a89) f(a90) f(a91) f(a92) f(a93) f(a94) f(a95) f(a96) f(a97) f(a98) f(a99) f(a100) f(a101) f(a102) f(a103) f(a104) f(a105) f(a106) f(a107) f(a108) f(a109) f(a110) f(a111) f(a112) f(a113) f(a114) f(a115) f(a116) f(a117) f(a118) f(a119) f(a120) f(a121) f(a122) f(a123) f(a124) f(a125) f(a126) f(a127)


/** Concatenate the two parameters. */
#define _MODULE_JOIN(a, b) _MODULE_JOIN_I(a, b)
#define _MODULE_JOIN_I(a, b) _MODULE_JOIN_II(a ## b)
#define _MODULE_JOIN_II(res) res

/**
 * The following macros generate the parameter class Params from LOADS_PARAMETERS
 * and DEFINES_PARAMETERS. They filter out all other macro names.
 * @param x The type name of a representation or the set of all parameters.
 */
#define _MODULE_PARAMETERS(x) _MODULE_JOIN(_MODULE_PARAMETERS_, x)
#define _MODULE_PARAMETERS_PROVIDES(type)
#define _MODULE_PARAMETERS_PROVIDES_CONCURRENT(type)
#define _MODULE_PARAMETERS_HAS_PREEXECUTION
#define _MODULE_PARAMETERS_PROVIDES_CONCURRENT_WITHOUT_MODIFY(type)
#define _MODULE_PARAMETERS_PROVIDES_WITHOUT_MODIFY(type)
#define _MODULE_PARAMETERS_REQUIRES(type)
#define _MODULE_PARAMETERS_USES(type)
#define _MODULE_PARAMETERS__MODULE_DEFINES_PARAMETERS(header, ...) _STREAM_STREAMABLE(Params, Streamable, , header, __VA_ARGS__); typedef Params NoParameters;
#define _MODULE_PARAMETERS__MODULE_LOADS_PARAMETERS(header, ...) _STREAM_STREAMABLE(Params, Streamable, , header, __VA_ARGS__); typedef Params NoParameters;

/**
 * The following macros generate the code for loading the configuration file
 * from LOADS_PARAMETERS. They filter out all other macro names.
 * @param x The type name of a representation or the set of all parameters.
 */
#define _MODULE_LOAD(x) _MODULE_JOIN(_MODULE_LOAD_, x)
#define _MODULE_LOAD_PROVIDES(type) _MODULE_INIT_ID(type)
#define _MODULE_LOAD_PROVIDES_CONCURRENT(type) _MODULE_INIT_ID(type)
#define _MODULE_LOAD_HAS_PREEXECUTION
#define _MODULE_LOAD_PROVIDES_CONCURRENT_WITHOUT_MODIFY(type) _MODULE_INIT_ID(type)
#define _MODULE_LOAD_PROVIDES_WITHOUT_MODIFY(type) _MODULE_INIT_ID(type)
#define _MODULE_LOAD_REQUIRES(type)
#define _MODULE_LOAD_USES(type) Blackboard::getInstance().alloc<type>(#type);
#define _MODULE_LOAD__MODULE_DEFINES_PARAMETERS(...)
#define _MODULE_LOAD__MODULE_LOADS_PARAMETERS(...) loadModuleParameters(*this, moduleName, fileName);

/**
 * The following macros generate the declarations for all requirements as well as
 * the update handler for the providers.code for loading the configuration file
 * from LOADS_PARAMETERS. They filter out the *_PARAMETERS macros.
 * @param x The type name of a representation or the set of all parameters.
 */
#define _MODULE_DECLARE(x) _MODULE_JOIN(_MODULE_DECLARE_, x)
#define _MODULE_DECLARE_PROVIDES(type) _MODULE_PROVIDES(type, \
  MODIFY("representation:" #type, r); \
  ModuleBase::draw(r); )
#define _MODULE_DECLARE_PROVIDES_CONCURRENT(type) _MODULE_PROVIDES(type, \
  MODIFY("representation:" #type, r); \
  ModuleBase::draw(r); )
#define _MODULE_DECLARE_HAS_PREEXECUTION _MODULE_HAS_PREEXECUTION
#define _MODULE_DECLARE_PROVIDES_CONCURRENT_WITHOUT_MODIFY(type) _MODULE_PROVIDES(type, \
  ModuleBase::draw(r); )
#define _MODULE_DECLARE_PROVIDES_WITHOUT_MODIFY(type) _MODULE_PROVIDES(type, \
  ModuleBase::draw(r); )
#define _MODULE_DECLARE_REQUIRES(type) public: const type& the##type = Blackboard::getInstance().alloc<type>(#type);
#define _MODULE_DECLARE_USES(type) public: const type& the##type = Blackboard::getInstance().alloc<type>(#type "-Copy");
#define _MODULE_DECLARE__MODULE_DEFINES_PARAMETERS(...)
#define _MODULE_DECLARE__MODULE_LOADS_PARAMETERS(...)

/**
 * The following macros generate the code that frees all requirements
 * and providers. They filter out the *_PARAMETERS macros.
 * @param x The type name of a representation or the set of all parameters.
 */
#define _MODULE_FREE(x) _MODULE_JOIN(_MODULE_FREE_, x)
#define _MODULE_FREE_PROVIDES(type) if(_the##type) Blackboard::getInstance().free(#type);
#define _MODULE_FREE_PROVIDES_CONCURRENT(type) if(_the##type) Blackboard::getInstance().free(#type);
#define _MODULE_FREE_HAS_PREEXECUTION
#define _MODULE_FREE_PROVIDES_CONCURRENT_WITHOUT_MODIFY(type) if(_the##type) Blackboard::getInstance().free(#type);
#define _MODULE_FREE_PROVIDES_WITHOUT_MODIFY(type) if(_the##type) Blackboard::getInstance().free(#type);
#define _MODULE_FREE_REQUIRES(type) Blackboard::getInstance().free(#type);
#define _MODULE_FREE_USES(type) Blackboard::getInstance().free(#type "-Copy"); Blackboard::getInstance().free(#type);
#define _MODULE_FREE__MODULE_DEFINES_PARAMETERS(...)
#define _MODULE_FREE__MODULE_LOADS_PARAMETERS(...)

/**
 * The following macros generate the code that provides information about all requirements
 * and providers. They filter out the *_PARAMETERS and USES macros.
 * @param x The type name of a representation or the set of all parameters.
 */
#define _MODULE_INFO(x) _MODULE_JOIN(_MODULE_INFO_, x)
#define _MODULE_INFO_PROVIDES(type) ModuleBase::Info(#type, &BaseType::update##type, nullptr, {ModuleBase::Info::Property::provide}),
#define _MODULE_INFO_PROVIDES_CONCURRENT(type) ModuleBase::Info(#type, &BaseType::update##type, nullptr, {ModuleBase::Info::Property::provide, ModuleBase::Info::Property::concurrent}),
#define _MODULE_INFO_HAS_PREEXECUTION ModuleBase::Info("Pre-execution", nullptr, &BaseType::execute, {ModuleBase::Info::Property::preexecution}),
#define _MODULE_INFO_PROVIDES_CONCURRENT_WITHOUT_MODIFY(type) ModuleBase::Info(#type, &BaseType::update##type, nullptr, {ModuleBase::Info::Property::provide, ModuleBase::Info::Property::concurrent}),
#define _MODULE_INFO_PROVIDES_WITHOUT_MODIFY(type) ModuleBase::Info(#type, &BaseType::update##type, nullptr, {ModuleBase::Info::Property::provide}),
#define _MODULE_INFO_REQUIRES(type) ModuleBase::Info(#type, nullptr, nullptr, {ModuleBase::Info::Property::require}),
#define _MODULE_INFO_USES(type) ModuleBase::Info(#type, nullptr, nullptr, {ModuleBase::Info::Property::use}),
#define _MODULE_INFO__MODULE_DEFINES_PARAMETERS(...)
#define _MODULE_INFO__MODULE_LOADS_PARAMETERS(...)

/**
 * Assign message id for a representation.
 * @param type The type of the representation the id of which is assigned.
 */
#define _MODULE_INIT_ID(type) \
  _id##type = ModuleBase::getMessageID(#type);

 /**
  * The macro defines the code added for each PROVIDES.
  * It declares the abstract update method, a pointer to the representation provided,
  * and an static handler that calls the update method.
  * @param type The type of the representation provided.
  * @param mod Additional code that is added to the handler.
  */
#define _MODULE_PROVIDES(type, mod) \
  protected: virtual void update(type&) = 0; \
  \
  private: type* _the##type = &Blackboard::getInstance().alloc<type>(#type); \
  MessageID _id##type = ::undefined; \
  static void update##type(Streamable& module) \
  { \
    ((BaseType&) module).modifyParameters(); \
    type& r(*((BaseType&) module)._the##type); \
    BH_TRACE_MSG("update " #type); \
    STOPWATCH_WITH_PLOT(#type) ((BaseType&) module).update(r); \
    mod \
    if(((BaseType&) module)._id##type != ::undefined) \
      DEBUG_RESPONSE("representation:" #type) OUTPUT(((BaseType&) module)._id##type, bin, r); \
  }

/**
  * The macro defines the code added for each HAS_PREEXECUTION.
  * It declares the abstract update method, a pointer to the subflow provided,
  * and an static handler that calls the update method.
  */
#define _MODULE_HAS_PREEXECUTION \
  protected: virtual void execute(tf::Subflow&) = 0; \
  \
  static void execute(Streamable& module, tf::Subflow& subflow) \
  { \
    BH_TRACE_MSG(moduleName); \
    ((BaseType&)module).execute(subflow); \
  }

/**
 * Generate the module's base class from the MODULE description.
 * See beginning of this file.
 * @param name The name of the module.
 * @param ... The requirementes, provided representations and parameter definitions.
 */
#define MODULE(name, ...) _MODULE_I(name, _MODULE_TUPLE_SIZE(__VA_ARGS__) __VA_OPT__(,) __VA_ARGS__)
#define _MODULE_I(...) _MODULE_II(__VA_ARGS__)
#define _MODULE_II(name, n, ...)                                                   \
  namespace name##Module \
  { \
    _MODULE_ATTR_##n(_MODULE_PARAMETERS, __VA_ARGS__)                              \
    typedef NoParameters Parameters; \
  } \
  class name; \
  class name##Base : public name##Module::Parameters \
  { \
  private: \
    static constexpr const char* moduleName = #name; \
    typedef name##Base BaseType; \
    void modifyParameters() \
    { \
      if constexpr (sizeof(NoParameters) < sizeof(name##Module::Parameters)) \
        MODIFY("parameters:" #name, *this); \
    } \
                                                                                   \
  private: \
    static const ModuleBase::Info* getModuleInfo() \
    { \
      static const ModuleBase::Info infos[] = \
      { \
        _MODULE_ATTR_##n(_MODULE_INFO, __VA_ARGS__)                                \
        ModuleBase::Info(nullptr, nullptr, nullptr, {}) \
      }; \
      return infos; \
    } \
    friend class Module<name, name##Base>; \
    _MODULE_ATTR_##n(_MODULE_DECLARE, __VA_ARGS__)                                 \
    public : typedef name##Module::Parameters Parameters;                          \
    name##Base(const char* fileName = 0) \
    { \
      (void) moduleName; \
      _MODULE_ATTR_##n(_MODULE_LOAD, __VA_ARGS__)                                  \
    } \
    ~name##Base(){                                                                 \
      _MODULE_ATTR_##n(_MODULE_FREE, __VA_ARGS__)                                  \
    } \
    name##Base(const name##Base&) = delete; \
    name##Base& operator=(const name##Base&) = delete; \
  }

/**
 * These two macros encapsulate the first parameter of the parameter
 * macros in ellipses to prevent commas within them to confuse the further macro
 * expansion.
 */
#define DEFINES_PARAMETERS(header, ...) _MODULE_DEFINES_PARAMETERS((header) __VA_OPT__(,) __VA_ARGS__)
#define LOADS_PARAMETERS(header, ...) _MODULE_LOADS_PARAMETERS((header) __VA_OPT__(,) __VA_ARGS__)

/**
* The macro creates a creator for the module.
* See beginning of this file.
* It has to be part of the implementation file.
* @param module The name of the module that can be created.
* @param category The category of this module.
*/
#define MAKE_MODULE(module, category) \
  Module<module, module##Base> the##module##Module(#module, ModuleBase::category);

// clang-format on
