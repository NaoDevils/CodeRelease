-- GoalieTest
-- shots the ball from different source positions towards different target positions in the goal at different velocities
-- collects and measures relevant data
-- every combination of source, target and velocity defines one single test case.
-- These test cases are executed in cycles, one test case after another,
-- every cycle consists of three phases: 1) preparation, 2) execution, 3) capture results
-- 1) preparation: place the ball and robot, let the robot move to his goalkeeper position
-- 2) execution: shoot the ball
-- 3) capture results: calculate dive recovery time, in case the goalie made a dive motion

local xMin = 250
local xMax = 4500
local xStep = 500
local xPos = xMin

local yMin = -2750
local yMax = 3000
local yStep = 500
local yPos = yMin

local xTargetPos = 4500

local yTargetMin = -700
local yTargetMax = 700
local yTargetStep = 100
local yTargetPos = yTargetMin

local kickSpeedMin = 1.95
local kickSpeedMax = 2.35
local kickSpeedStep = 0.1
local kickSpeed = kickSpeedMin

local finished = false
local goalDetected = false
local diveDetected = false
local diveDetectionActive = false
local diveRecoveryFailureTime = 8333 --eq. 99.9s
local diveTimestamp = 0
local recoveryTimestamp = 0
local lastUprightTransition = 0

--"upright and falls again"-debounce: 400frames * 12ms -> 4.8s
local uprightDeBounce = 400 

local cycleDuration = 4000
local counter = 0
local goals = 0
local result = { details = {} }

local function MyOnEventHandler(self, event, ...)
  if (event==Events.SCRIPT_LOADED) then
    self.dt("off")
    self.ar("off")
  end
  if (event==Events.GOAL_SCORED) then
    goalDetected = true
  end
end

local function finishTest(adapter)
  finished = true;
  adapter.gc("finished")
  adapter.dt("on")
  local testCount = 0
  local goalCount = 0

  result.name = adapter.name
  result.attimestamp = adapter.createdAtTimestamp
  result.exectime = adapter.createdAtId
  result.id = adapter.name.."-"..adapter.createdAtId
  for k,test in pairs(result.details) do
    testCount = testCount + 1;
    if (test.failed) then
      goalCount = goalCount + 1      
    end
  end
  result.total = testCount
  result.goalsScored = goalCount
  result.goalsPrevented = testCount - goalCount
  if (testCount>0) then
    result.preventedGoalRatePercent = 100 * result.goalsPrevented / result.total
  else
    result.preventedGoalRatePercent = 0
  end
  adapter.saveTestResult(result.id, JSON:encode_pretty(result))

  --create bulk upload json data
  local bulkData = ""
  for k,test in pairs(result.details) do
    test.parent_id = result.id
    test.attimestamp = result.attimestamp
    test.exectime = result.exectime
    bulkData = bulkData .. "{ \"create\":{} }\n"
    bulkData = bulkData .. JSON:encode(test) .. "\n"
  end
  adapter.saveTestResult(result.id.."-details", bulkData)

  adapter.printLn("Test finished.")
  adapter.printLn("Prevented goal-rate (%): " .. result.preventedGoalRatePercent)
end

local function addResult(self,xKickPos,yKickPos,xTargetPos,yTargetPos,velocity,goalScored,dive,timeToRecover)
  local o = {}
  o.kickPosX = xKickPos
  o.kickPosY = yKickPos
  o.targetPosX = xTargetPos
  o.targetPosY = yTargetPos
  local dx = xTargetPos-xKickPos
  local dy = yTargetPos-yKickPos
  o.distance = math.sqrt(dx*dx+dy*dy)
  o.velocity = velocity
  if goalScored then
    goals = goals + 1
    o.goalsScored = 1
  else
    o.goalsScored = 0
  end
  o.dive = dive
  o.timeToRecover = math.floor(timeToRecover * 12 / 100) / 10
  o.failed = goalScored
  self.printLn("Result: goal scored: "..o.goalsScored..", dive recovery: "..o.timeToRecover.."s")
  table.insert( result.details, o)
end

local function MyOnUpdateHandler(self, timestamp)
  if (not finished) then
    local cycleTimestamp = timestamp % cycleDuration;
    --preparation phase: place the ball and robot, let the robot move to his goalkeeper position
    if (cycleTimestamp == 1) then
      local percent = ((counter-goals)/math.max(1,counter)*100.0)
      self.printLn("Start test: ".. counter+1 .. " (PreventedGoalRate:"..string.format("%.2f %%", percent).."): ball:"..xPos.."/"..yPos..", target:"..xTargetPos.."/"..yTargetPos..", velocity:"..kickSpeed)
      goalDetected = false
      diveDetected = false
      diveDetectionActive = false
      diveTimestamp = 0
      recoveryTimestamp = 0
      self.gc("set")
      self.moveBall(xPos,yPos)
    end
    if (cycleTimestamp >= 50 and cycleTimestamp <= 700) then
      --keep the robot above the ground for at least some seconds in case he needs to recover from fallen state
      self.moveRobot(4000, 0, 400, 0, 0, 180, "robot01")
    end
    if (timestamp % cycleDuration == 1000) then
      self.gc("playing")
    end
    --execution phase: shoot the ball
    if (timestamp % cycleDuration == 2000) then
      diveDetectionActive = true
      self.kickBallTarget(xTargetPos,yTargetPos,kickSpeed)
    end
    --capture result phase: calculate dive recovery time, in case the goalie made a dive motion
    if (timestamp % cycleDuration == cycleDuration-1) then
      local recoveryTime = 0
      if (diveDetected) then
        if (recoveryTimestamp == 0) then
          recoveryTimestamp = timestamp
          recoveryTime = recoveryTimestamp - diveTimestamp;
          self.printLn(string.format("Dive detected, NO recovery. Tried %.1f s",(recoveryTime * 12 / 1000)))
          recoveryTime = diveRecoveryFailureTime
        else
          recoveryTime = recoveryTimestamp - diveTimestamp;
          --self.printLn(string.format("Dive detected, recoveryTime: %.1f s",(recoveryTime * 12 / 1000)))
        end
      end
      addResult(self,xPos,yPos,xTargetPos,yTargetPos,kickSpeed,goalDetected,diveDetected,recoveryTime)
      counter = counter + 1

      --calculate next test cycle parameters
      kickSpeed = kickSpeed + kickSpeedStep
      if (kickSpeed>kickSpeedMax) then
        kickSpeed = kickSpeedMin
        yTargetPos = yTargetPos + yTargetStep
        if (yTargetPos>yTargetMax) then
          yTargetPos = yTargetMin
          yPos = yPos + yStep
          if (yPos > yMax) then
            yPos = yMin
            xPos = xPos + xStep
            if (xPos > xMax) then
              finishTest(self)
              return
            end
          end
        end
      end  
    end
    if (diveDetectionActive) then
      if (RoboCupData.robots[1].upright == false) then
        -- fallen robot detected (dive motion)
        lastUprightTransition = 0
        if (diveTimestamp==0) then
          diveDetected = true
          diveTimestamp = timestamp;
          --self.printLn("fallen:"..diveTimestamp)
        end
      else
        -- upright robot detected (recovery after dive motion)
        if (lastUprightTransition == 0) then
          lastUprightTransition = timestamp
        end
        if (diveTimestamp>0 and recoveryTimestamp==0 and lastUprightTransition > 0 and timestamp - lastUprightTransition > uprightDeBounce) then
          recoveryTimestamp = lastUprightTransition 
          --self.printLn("recovery:"..recoveryTimestamp)
        end
      end
    end
  end
end

local rc = RoboCupAdapter:new("GoalkeeperTest")
--rc.printLn("Hello from Adapter "..rc.name)
rc:setScript(rc.Handlers.OnEvent, MyOnEventHandler)
rc:registerEvent(rc.Events.SCRIPT_LOADED)
rc:registerEvent(rc.Events.BALL_MOVED)
rc:registerEvent(rc.Events.GOAL_SCORED)
rc:setScript(rc.Handlers.OnUpdate, MyOnUpdateHandler)
