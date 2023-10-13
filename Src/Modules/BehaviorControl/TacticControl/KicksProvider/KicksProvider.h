#pragma once

#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"
#include "Modules/MotionControl/KickEngine/KickEngineParameters.h"
#include "Representations/Modeling/Kicks.h"

#include <Platform/File.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <optional>

MODULE(KicksProvider,

  PROVIDES(Kicks),

  LOADS_PARAMETERS(,
    (float)(0.005f) dummyParameter
  )
);

class KicksProvider : public KicksProviderBase
{

public:
  inline static const std::string KICK_ENGINE_PATH = "/Config/Kicks/KickEngine";
  inline static const std::string WALK_ENGINE_PATH = "/Config/Kicks/WalkingEngine";

  KicksProvider();

  void update(Kicks& kicks) override;

  static std::vector<KickEngineParameters> loadKickEngineParameters();
  static std::vector<CustomStepsFile> loadCustomStepFiles();

  static std::vector<std::unique_ptr<Kick>> createKicks(
      const std::vector<KickEngineParameters>& kickEngineParametersVector, const std::vector<CustomStepsFile>& customStepFileVector, const std::vector<std::string>& kickNames);
  static std::vector<std::unique_ptr<Kick>> createKicks(
      const std::vector<KickEngineParameters>& kickEngineParametersVector, const std::vector<CustomStepsFile>& customStepFileVector, const bool dribble);

private:
  static std::optional<KickEngineParameters> loadKickEngineParameter(const std::filesystem::directory_entry& file);
  static std::optional<CustomStepsFile> loadCustomStepFile(const std::filesystem::directory_entry& file);

  bool loadedIntoRepresentation = false;
};
