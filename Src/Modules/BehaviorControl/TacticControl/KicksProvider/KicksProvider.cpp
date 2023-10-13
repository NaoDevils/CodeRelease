#include "KicksProvider.h"

#include "Modules/BehaviorControl/TacticControl/KicksProvider/Types/Dribble.h"
#include "Modules/BehaviorControl/TacticControl/KicksProvider/Types/KickEngineKick.h"
#include "Modules/MotionControl/DortmundWalkingEngine/StepData.h"

KicksProvider::KicksProvider() {}

void KicksProvider::update(Kicks& kicks)
{
  if (loadedIntoRepresentation)
  {
    return;
  }
  kicks.kickEngineParametersVector = loadKickEngineParameters();
  kicks.customStepFileVector = loadCustomStepFiles();
  loadedIntoRepresentation = true;
}

std::vector<std::unique_ptr<Kick>> KicksProvider::createKicks(
    const std::vector<KickEngineParameters>& kickEngineParametersVector, const std::vector<CustomStepsFile>& customStepFileVector, const std::vector<std::string>& kickNames)
{
  std::vector<std::unique_ptr<Kick>> kicksVector = {};
  for (const auto& name : kickNames)
  {
    if ("dribble" == name)
    {
      kicksVector.push_back(std::make_unique<Dribble>());
      continue;
    }
    bool found = false;
    for (const KickEngineParameters& kickEngineParameters : kickEngineParametersVector)
    {
      if (kickEngineParameters.name == name)
      {
        const KickEngineKick kick = KickEngineKick(kickEngineParameters);
        kicksVector.push_back(std::make_unique<KickEngineKick>(kick));
        found = true;
        break;
      }
    }
    if (found)
    {
      continue;
    }
    for (const CustomStepsFile& customStepsFile : customStepFileVector)
    {
      if (customStepsFile.name == name)
      {
        const WalkingEngineKick kick = WalkingEngineKick(customStepsFile);
        kicksVector.push_back(std::make_unique<WalkingEngineKick>(kick));
        found = true;
        break;
      }
    }
    if (!found)
    {
      OUTPUT_WARNING("Kick with name " + name + " required but missing in the loaded folders!");
    }
  }
  return kicksVector;
}

std::vector<std::unique_ptr<Kick>> KicksProvider::createKicks(
    const std::vector<KickEngineParameters>& kickEngineParametersVector, const std::vector<CustomStepsFile>& customStepFileVector, const bool dribble)
{
  std::vector<std::unique_ptr<Kick>> kicksVector = {};
  for (const KickEngineParameters& kickEngineParameters : kickEngineParametersVector)
  {
    const KickEngineKick kick = KickEngineKick(kickEngineParameters);
    kicksVector.push_back(std::make_unique<KickEngineKick>(kick));
  }
  for (const CustomStepsFile& customStepsFile : customStepFileVector)
  {
    const WalkingEngineKick kick = WalkingEngineKick(customStepsFile);
    kicksVector.push_back(std::make_unique<WalkingEngineKick>(kick));
  }
  if (dribble)
  {
    kicksVector.push_back(std::make_unique<Dribble>());
  }
  return kicksVector;
}

// KickEngineParameters ============================================================================================================================================================

std::vector<KickEngineParameters> KicksProvider::loadKickEngineParameters()
{
  std::vector<KickEngineParameters> params = {};
  const std::string dirname = std::string(File::getBHDir()) + KICK_ENGINE_PATH + "/";
  for (const auto& file : std::filesystem::directory_iterator(dirname))
  {
    if (auto parametersOptional = loadKickEngineParameter(file))
    {
      params.push_back(parametersOptional.value());
    }
  }
  return params;
}

std::optional<KickEngineParameters> KicksProvider::loadKickEngineParameter(const std::filesystem::directory_entry& file)
{
  const std::string fileName = file.path().filename().string();

  if (fileName == "ignore")
  {
    return {};
  }

  if (!file.is_regular_file() || fileName.substr(fileName.size() - 4) != ".kmc")
  {
    OUTPUT_TEXT("Warning: Wrong file format for " << file.path().string());
    fprintf(stderr, "Warning: Wrong file format for %s \n", file.path().string().c_str());
    return {};
  }

  InMapFile stream(file.path().string());
  ASSERT(stream.exists());

  KickEngineParameters parameters;
  stream >> parameters;

  strcpy(parameters.name, fileName.substr(0, fileName.size() - 4).c_str());
  parameters.type = KickRequest::getKickMotionFromName(parameters.name);

  if (parameters.type < KickRequest::none)
  {
    return parameters;
  }
  else
  {
    OUTPUT_TEXT("Warning: KickRequest is missing the id for " << parameters.name);
    fprintf(stderr, "Warning: KickRequest is missing the id for %s \n", parameters.name);
    return {};
  }
}

// StepFiles =====================================================================================================================================================================

std::vector<CustomStepsFile> KicksProvider::loadCustomStepFiles()
{
  std::vector<CustomStepsFile> stepFiles = {};
  const std::string dirname = std::string(File::getBHDir()) + WALK_ENGINE_PATH + "/";
  for (const auto& file : std::filesystem::directory_iterator(dirname))
  {
    if (auto stepFilesOptional = loadCustomStepFile(file))
    {
      stepFiles.push_back(stepFilesOptional.value());
    }
  }
  return stepFiles;
}

std::optional<CustomStepsFile> KicksProvider::loadCustomStepFile(const std::filesystem::directory_entry& file)
{
  const std::string fileNameWithType = file.path().filename().string();
  const std::string fileNameWithoutType = fileNameWithType.substr(0, fileNameWithType.size() - 4);

  if (fileNameWithType == "ignore")
  {
    return {};
  }

  if (!file.is_regular_file() || fileNameWithType.substr(fileNameWithType.size() - 4) != ".cfg")
  {
    OUTPUT(idText, text, std::string("Unable to load: ") << fileNameWithType);
    return {};
  }

  CustomStepsFile stepFile;
  InMapFile stream(file.path().string());
  ASSERT(stream.exists());
  stream >> stepFile;

  strcpy(stepFile.name, fileNameWithoutType.c_str());
  stepFile.stepRequest = WalkRequest::getStepRequestFromName(fileNameWithoutType.c_str());

  // Must begin with single support
  ASSERT(stepFile.steps.empty() || !(stepFile.steps.front().onFloor[0] && stepFile.steps.front().onFloor[1]));

  return stepFile;
}

MAKE_MODULE(KicksProvider, modeling)
