local goalDetected = false
local cycleDuration = 2200
local iterations = 100
local counter = 0
local goals = 0
local xKickPos, yKickPos, yTargetPos, velocity
local result = { details = {} }

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
    if (test.goalsScored>0) then
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

local function addResult(xKickPos,yKickPos,xTargetPos,yTargetPos,velocity,goalScored)
  local o = {}
  o.kickPosX = xKickPos
  o.kickPosY = yKickPos
  o.targetPosX = xTargetPos
  o.targetPosY = yTargetPos
  local dx = xTargetPos-xKickPos
  local dy = yTargetPos-yKickPos
  o.distance = math.sqrt(dx*dx+dy*dy)
  o.velocity = velocity
  if (goalScored) then
    goals = goals + 1
    o.goalsScored = 1
  else 
    o.goalsScored = 0
  end
  o.failed = goalScored
  table.insert( result.details, o)
end

local function MyOnUpdateHandler(self, timestamp)
  if counter<iterations then
    if (timestamp % cycleDuration == 1) then
      xKickPos = math.random(0,4500)
      yKickPos = math.random(-3000,3000)
      velocity = math.random(18,24) / 10.0
      yTargetPos = math.random(-700,700)
      local percent = ((counter-goals)/math.max(1,counter)*100.0)
      self.printLn("Test "..counter.."/"..iterations.." ("..string.format("%.2f %%", percent).."): "..xKickPos.."/"..yKickPos.."->"..yTargetPos.." @"..velocity.."m/s")
      self.gc("set")
      self.moveRobot(4000, 0, 400, 0, 0, 180, "robot01")
      self.moveBall(xKickPos,yKickPos)
    end
    if (timestamp % cycleDuration == 300) then
      self.gc("playing")
    end
    if (timestamp % cycleDuration == 1000) then
      goalDetected = false
      self.kickBallTarget(4500,yTargetPos,velocity)
    end
    if (timestamp % cycleDuration == 1500) then
      self.moveBall(0,0)
      addResult(xKickPos,yKickPos,4500,yTargetPos,velocity,goalDetected)
      counter = counter + 1
      if (counter>=iterations) then
        finishTest(self)
      end
    end
  end
end
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

local rc = RoboCupAdapter:new("GoalieTest-MonteCarlo")
rc:setScript(rc.Handlers.OnUpdate, MyOnUpdateHandler)
rc:setScript(rc.Handlers.OnEvent, MyOnEventHandler)
rc:registerEvent(rc.Events.SCRIPT_LOADED)
rc:registerEvent(rc.Events.GOAL_SCORED)