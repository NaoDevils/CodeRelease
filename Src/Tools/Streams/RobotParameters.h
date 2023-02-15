/**
 * @file RobotParameters.h
 * The file declares some macros that enables to automatic handling of
 * a parameter file. See Modules/TemplateModule for an example.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Streams/OutStreams.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/MessageQueue/MessageIDs.h"
#include "Platform/File.h"
#include "Tools/Debugging/Modify.h"
#include "Tools/Settings.h"
#else
#include "bhumanstub.h"
#endif
#include <string>

// clang-format off

#define ROBOT_PARAMETERS(n) \
  protected: \
class Params : public Streamable\
{ \
std::string name; \
std::string fullPath; \
void save() \
{\
  OutMapFile stream(fullPath);\
  if(stream.exists())\
  {\
    stream << *this;\
    OUTPUT_TEXT("Saved " + fullPath);\
  }\
  else\
    OUTPUT_TEXT("Failed to save " + fullPath);\
}\
void load()\
{\
  FILE *stream; \
  std::list<std::string> names = File::getFullNames(name + ".cfg"); \
  bool found = false; \
  for (auto& path : names) \
  {\
    stream = fopen(path.c_str(), "r");\
    if (stream)\
    {\
      found = true;\
      fullPath = path;\
      break;\
    }\
  }\
  if (!found) \
  {\
    fullPath = File::getBHDir() + std::string("/Config/Robots/Default/") + name + std::string(".cfg"); \
    ASSERT(false);\
  }\
  fclose(stream); \
  InMapFile file(name + std::string(".cfg"));\
  if(file.exists())\
    file >> *this; \
  else \
    OUTPUT_TEXT("Warning, could not create" + fullPath); \
}\
void init()\
{\
  FILE *stream; \
  std::list<std::string> names = File::getFullNames(name + ".cfg"); \
  bool found = false; \
  for (auto& path : names) \
  {\
    stream = fopen(path.c_str(), "r");\
    if (stream)\
    {\
      found = true;\
      fullPath = path;\
      break;\
    }\
  }\
  if (!found) \
  {\
    fullPath = File::getBHDir() + std::string("/Config/Robots/Default/") + name + std::string(".cfg"); \
    ASSERT(false);\
  }\
  fclose(stream); \
  InMapFile file(name + std::string(".cfg"));\
  if(file.exists())\
    file >> *this; \
  else \
    OUTPUT_TEXT("Warning, could not create" + fullPath); \
}\
void exit()\
{\
  File test(fullPath, "r", false);\
  if (test.exists())\
    save();\
  else\
    OUTPUT_TEXT("Warning, could not create" + fullPath);\
}\
void save(Params &params)\
{\
  OutMapFile stream(fullPath);\
  if(stream.exists())\
  {\
    stream << params;\
    OUTPUT_TEXT(std::string("Saved ") + fullPath);\
  }\
  else\
    OUTPUT_TEXT(std::string("Failed to save ") + fullPath);\
}\
public:\
void handle()\
{\
  MODIFY("module:" #n ":params", *this);\
  DEBUG_RESPONSE_ONCE("module:" #n ":load") load();\
  DEBUG_RESPONSE_ONCE("module:" #n ":save") save(*this);\
}\
Params() : name(#n) { init(); }; \
~Params() { } \
void beginstreaming(In* in,Out* out){

#define PARAM(t, p) \
stream##p(in, out);} t p; \
void stream##p(In* in,Out* out){STREAM(p);

#define PARAM_DEF(t, p, d) \
stream##p(in, out);} t p = d; \
void stream##p(In* in,Out* out){STREAM(p);

#define END_ROBOT_PARAMETERS } \
void serialize(In* in,Out* out) {\
STREAM_REGISTER_BEGIN;\
beginstreaming(in, out);\
STREAM_REGISTER_FINISH;}};\
Params params;



#define ROBOT_PARAMETER_CLASS(n,bm) \
class Params##n : public Streamable\
{ \
std::string name; \
std::string fullPath; \
void save() \
{\
  OutMapFile stream(fullPath);\
  if(stream.exists())\
  {\
    stream << *this;\
    OUTPUT_TEXT("Saved " + fullPath);\
  }\
  else\
    OUTPUT_TEXT("Failed to save " + fullPath);\
}\
void load()\
{\
  FILE *stream; \
  std::list<std::string> names = File::getFullNames(name + ".cfg"); \
  bool found = false; \
  for (auto& path : names) \
  {\
    stream = fopen(path.c_str(), "r");\
    if (stream)\
    {\
      found = true;\
      fullPath = path;\
      break;\
    }\
  }\
  if (!found) \
  {\
    fullPath = File::getBHDir() + std::string("/Config/Robots/Default/") + name + std::string(".cfg"); \
    ASSERT(false);\
  }\
  if (stream) fclose(stream); \
  InMapFile file(name + std::string(".cfg"));\
  if(file.exists())\
    file >> *this; \
  else \
    OUTPUT_TEXT("Warning, could not create" + fullPath); \
}\
void init()\
{\
  FILE *stream; \
  std::list<std::string> names = File::getFullNames(name + ".cfg"); \
  bool found = false; \
  for (auto& path : names) \
  {\
    stream = fopen(path.c_str(), "r");\
    if (stream)\
    {\
      found = true;\
      fullPath = path;\
      break;\
    }\
  }\
  if (!found) \
  {\
    fullPath = File::getBHDir() + std::string("/Config/Robots/Default/") + name + std::string(".cfg"); \
    ASSERT(false);\
  }\
  if (stream) fclose(stream); \
  InMapFile file(name + std::string(".cfg"));\
  if(file.exists())\
    file >> *this; \
  else \
    OUTPUT_TEXT("Warning, could not create" + fullPath); \
}\
void exit()\
{\
  File test(fullPath, "r", false);\
  if (test.exists())\
    save();\
  else\
    OUTPUT_TEXT("Warning, could not create" + fullPath);\
}\
void save(Params##n &params)\
{\
  OutMapFile stream(fullPath);\
  if(stream.exists())\
  {\
    stream << params;\
    OUTPUT_TEXT(std::string("Saved ") + fullPath);\
  }\
  else\
    OUTPUT_TEXT(std::string("Failed to save ") + fullPath);\
}\
public:\
void handle()\
{\
  MODIFY("module:" #bm ":" #n, *this);\
  DEBUG_RESPONSE_ONCE("module:" #bm ":" #n ":load") load();\
  DEBUG_RESPONSE_ONCE("module:" #bm ":" #n ":save") save(*this);\
}\
  Params##n() : name(#n) { init(); }; \
  ~Params##n() { } \
void beginstreaming(In* in,Out* out){

#define END_ROBOT_PARAMETER_CLASS(n)} \
void serialize(In* in,Out* out) {\
STREAM_REGISTER_BEGIN;\
beginstreaming(in, out);\
STREAM_REGISTER_FINISH;}};\
Params##n params##n;

// clang-format on
