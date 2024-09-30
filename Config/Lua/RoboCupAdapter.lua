
Events = {}
Events.SCRIPT_LOADED = 1
Events.GC_STATE_CHANGE = 2
Events.BALL_MOVED = 3
Events.GOAL_SCORED = 4

Handlers = {}
Handlers.OnEvent = 1
Handlers.OnUpdate = 2

Teams = {}
Teams.A = 1
Teams.B = 2

Adapters = {}

RoboCupData = {
  absoluteBallPosition = { x=0, y=0 },
  robots = {},
  frameNo = 0
}

RoboCupAdapter = { 
  scriptMap = {}, 
  eventMap = {} 
}
local latestAdapter = nil

function RoboCupAdapter:new (adaptername)
  local o = {}
  setmetatable(o, self)
  self.__index = self
  o.name = adaptername
  o.createdAt = os.time()
  o.createdAtTimestamp = os.date("%Y-%m-%dT%H:%M:%S.00Z", o.createdAt)
  o.createdAtId = os.date("%Y%m%d%H%M%S", o.createdAt)
  Adapters[adaptername] = o
  printLn("Debug: created new adapter '"..adaptername.."'")
  o.Handlers = Handlers
  o.Events = Events

  o.gc = gc
  o.dt = dt
  o.ar = ar
  o.printLn = printLn
  o.moveBall = moveBall
  o.moveRobot = moveRobot
  o.kickBallAngle = kickBallAngle
  o.kickBallTarget = kickBallTarget
  o.saveTestResult = saveTestResult
  o.walkSpeed = walkSpeed
  latestAdapter = o
  return o
end

function RoboCupAdapter:registerEvent (event)
  self.eventMap[event] = true
end

function RoboCupAdapter:unregisterEvent (event)
  self.eventMap[event] = nil
end

function RoboCupAdapter:setScript (handler, func)
  self.scriptMap[handler] = func
end

function event_onUpdate(frameNo)
  for name, adapter in pairs(Adapters) do
    if adapter.scriptMap[Handlers.OnUpdate] then
      adapter.scriptMap[Handlers.OnUpdate](adapter,frameNo)
    end
  end
end

function event_scriptLoaded()
  if (latestAdapter) then    
    printLn("Lua: script_loaded called "..latestAdapter.name)
    if latestAdapter.scriptMap[Handlers.OnEvent] then
      if latestAdapter.eventMap[Events.SCRIPT_LOADED] then
        printLn("calling onEvent")
        latestAdapter.scriptMap[Handlers.OnEvent](latestAdapter,Events.SCRIPT_LOADED)
      end
    end
  end
end

function event_ballMoved(x,y)
  --printLn("Lua: Ball has moved")
  for name, adapter in pairs(Adapters) do
    if adapter.scriptMap[Handlers.OnEvent] then
      if adapter.eventMap[Events.BALL_MOVED] then
        adapter.scriptMap[Handlers.OnEvent](adapter,Events.BALL_MOVED,x,y)
      end
    end
  end
end

function event_ballInGoal(side)
  --printLn("Lua: Ball in Goal")
  for name, adapter in pairs(Adapters) do
    if adapter.scriptMap[Handlers.OnEvent] then
      if adapter.eventMap[Events.GOAL_SCORED] then
        local team;
        if (side<0) then
          team = Teams.A;
        else
          team = Teams.B;
        end
        adapter.scriptMap[Handlers.OnEvent](adapter,Events.GOAL_SCORED,team)
      end
    end
  end
end
