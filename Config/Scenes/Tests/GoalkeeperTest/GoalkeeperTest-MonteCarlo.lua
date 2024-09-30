-- GoalieTest
-- shots the ball from different source positions towards different target positions in the goal at different velocities
-- collects and measures relevant data
-- every combination of source, target and velocity defines one single test case.
-- These test cases are executed in cycles, one test case after another,
-- every cycle consists of three phases: 1) preparation, 2) execution, 3) capture results
-- 1) preparation: place the ball and robot, let the robot move to his goalkeeper position
-- 2) execution: shoot the ball
-- 3) capture results: calculate dive recovery time, in case the goalie made a dive motion

-- the number of goal kicks executed in this test

-- iterations: every iteration takes approx. 15sec on commodity hardware
--local iterations = 1200 -- duration aprox. 5h
local iterations = 100 -- duration aprox. 25min 

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
local xKickPos, yKickPos, xTargetPos, yTargetPos, kickSpeed
local result = { details = {} }

local function MyOnEventHandler(self, event, ...)
  if (event==Events.SCRIPT_LOADED) then
    math.randomseed(os.time())
    self.dt("off")
    self.ar("off")
  end
  if (event==Events.GOAL_SCORED) then
    goalDetected = true
  end
end

local function finishTest(adapter)
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
  if counter<iterations then
    local cycleTimestamp = timestamp % cycleDuration;
    --preparation phase: place the ball and robot, let the robot move to his goalkeeper position
    if (cycleTimestamp == 1) then
      xKickPos = math.random(0,4500)
      yKickPos = math.random(-3000,3000)
      kickSpeed = math.random(18,24) / 10.0
      xTargetPos = 4500
      yTargetPos = math.random(-700,700)
      local percent = ((counter-goals)/math.max(1,counter)*100.0)
      self.printLn("Start test: ".. counter+1 .. "/" ..iterations.. " (PreventedGoalRate:" ..string.format("%.2f %%", percent).. "): ball:"..xKickPos.."/"..yKickPos..", target:"..xTargetPos.."/"..yTargetPos..", velocity:"..kickSpeed)
      goalDetected = false
      diveDetected = false
      diveDetectionActive = false
      diveTimestamp = 0
      recoveryTimestamp = 0
      self.gc("set")
      self.moveBall(xKickPos,yKickPos)
    end
    if (cycleTimestamp >= 50 and cycleTimestamp <= 700) then
      --keep the robot above the ground for at least some seconds in case he needs to recover from fallen state
      self.moveRobot(4000, 0, 400, 0, 0, 180, "robot01")
    end
    if (cycleTimestamp == 1000) then
      self.gc("playing")
    end
    --execution phase: shoot the ball
    if (cycleTimestamp == 2000) then
      diveDetectionActive = true
      goalDetected = false
      self.kickBallTarget(xTargetPos,yTargetPos,kickSpeed)
    end
    --capture result phase: calculate dive recovery time, in case the goalie made a dive motion
    if (cycleTimestamp == cycleDuration-1) then
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
      addResult(self,xKickPos,yKickPos,xTargetPos,yTargetPos,kickSpeed,goalDetected,diveDetected,recoveryTime)
      counter = counter + 1

      if (counter>=iterations) then
        finishTest(self)
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

local rc = RoboCupAdapter:new("GoalieTest-MonteCarlo")
rc:setScript(rc.Handlers.OnUpdate, MyOnUpdateHandler)
rc:setScript(rc.Handlers.OnEvent, MyOnEventHandler)
rc:registerEvent(rc.Events.SCRIPT_LOADED)
rc:registerEvent(rc.Events.GOAL_SCORED)
