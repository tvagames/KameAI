
--------------------
-- オプション
--------------------
-- 敵との位置関係
MIN_DISTANCE = 500
MAX_DISTANCE = 1300
MIN_ANGLE = 30
MAX_ANGLE = 120
SWING_ANGLE = 20

-- 高度
CRUISE_ALT = -100
MAX_ALT = 500
MIN_ALT = -500
FROM_TARGET_ALT = nil -- 敵を基準とした高度。MAXとMINの間に丸める

-- ハイドロフォイルブースターのヨー操作利用
HYDROFOIL_YAW_ANGLE = 30
HYDROFOIL_STANDBY_ANGLE = 135

-- 横向き魚雷
AIM_SIDEWAYS_SLOT = 1

-- 回避
AVOID_TERRAIN_DISTANCE_RATE = 10
AVOID_TERRAIN_ALT_UPPER = -30
AVOID_TERRAIN_ALT_LOWER = -1000
DETECT_TERRAIN_ANGLE = 40
AVOID_FRIEND_DISTANCE_RATE = 5
AVOID_FRIEND_ANGLE = 90
AVOID_ENEMY_DISTANCE = 500
AVOID_ENEMY_ANGLE = 90

-- 修理
REPAIR_DISTANCE = 50
MAX_REPAIR_DISTANCE = 15

-- 0 = around (for side weapons), 
-- 1 = figure 8 (for front weapons), 
-- 2 = reversed figure 8 (for rear weapons)
-- 3 = repair ship
-- 4 = follower
-- 5 = back to enemy
-- 6 = front to enemy
ROUTE_PATTERN = 0

-- 0 = horizon
-- 1 = upside down (天地逆転)
UPSIDE_DOWN = 0

-- 0 = horizon
-- 1 = look up enemy
PITCH_TYPE = 0

-- 0 = hovering
-- 1 = elevator
-- 2 = escalator
-- 3 = elevator follow terrain. ignore CRUISE_ALT
-- 4 = escalator follow terrain. ignore CRUISE_ALT
ALT_MANEUVER = 3
ALT_FOLLOW_TERRAIN_DISTANCE = 20 -- 地形追従・高度回避で維持する地形との距離。nilは地形追従・高度回避しない

FORWARD_USE_YAWING = false
CUTOFF_PITCH_OVER = true
CUTOFF_ROLL_OVER = true
CUTOFF_UNFORWARD = true
ALT_CONTROL_MIN_ZERO = false

MISSILE_TOP_ATTACK_ENABLED = false
MISSILE_RUN_DISTANCE = 350
MISSILE_RUN_ALTITUDE = 400
MISSILE_LAUNCH_TYPE = 0 -- 0 = VLS
MISSILE_MULTI_LOCK_COUNT = 3
MISSILE_USED_MAINFRAME = 0
MISSILE_ASSIGN_TARGET_DISTANCE = 2500

ALT_DEDIBLADE_RANGE_FROM_COM = 15

--------------------
-- 構造的制約メモ
--------------------
-- デディブレの向き（Gキ－押下時）と使用（driveが正の時）
-- 下：右ヨー
-- 上：左ヨー
-- 前：上昇（ピッチ・高度・ロール）
-- 後：下降（ピッチ・高度・ロール）
-- 右：前進
-- 左：後進

--------------------
-- PID
--------------------
PidSetting = {}
PidSetting.new = function(p, i, d) 
    local self = {}
    self.kP = p
    self.Ti = i
    self.Td = d
    return self
  end

PidTurning = {}
PidTurning.CreateConfig = function(p, i, d)
    local self = {}
    self.logs = {}
    self.current = PidSetting.new(p, i, d, s)
    self.total = 0
    return self
  end

ppid = PidTurning.CreateConfig(0.05, 400, 1) -- 1200 * 25msecs = 30secs
apid = PidTurning.CreateConfig(0.05, 1200, 1)
rpid = PidTurning.CreateConfig(0.1, 400, 3)
fpid = PidTurning.CreateConfig(1.0, 1200, 1)
ypid = PidTurning.CreateConfig(0.1, 1200, 3)
  

Pid = {}
function Pid.GetGain(self, I, name, obj, target, actual)
    value = target - actual

    -- P動作
    p = value * obj.current.kP

    -- I動作
    obj.total = obj.total + value
    table.insert(obj.logs, 1, value)
    c = table.maxn(obj.logs)

    while c > obj.current.Ti do
        obj.total = obj.total - obj.logs[c]
        table.remove(obj.logs, c)
        c = table.maxn(obj.logs)
    end  
    i = (obj.total / obj.current.Ti) * obj.current.kP

    -- D動作
    d = obj.current.Td * obj.current.kP

    return p + i + d
end

--------------------
-- オイラー角に変換
--------------------
function ToEulerAngle(deg)
  return deg / Mathf.PI * 180
end

--------------------
-- 0～360を-180～180に変換
--------------------
function ZeroOrigin(a)
  if a == 0 then
    return 0
  end
  return ((a + 180) % 360) - 180
end

--------------------
-- 二つのベクトルのなす角
--------------------
function Angle(a, b)
  return math.deg(math.acos(Vector3.Dot(a, b) / (Vector3.Magnitude(a) * Vector3.Magnitude(b))))
end

--------------------
-- 範囲内にあるかどうか
--------------------
function InRange(value, min, max)
  return min <= value and value <= max
end

lastForwardDrive = 0
YAW_LEFT = 0
YAW_RIGHT = 1
lasyYaw = 0

function GetDistance(a, b)
  return (a - b).magnitude
end

function IsTransceiverAssigned(transceiverItem, targets)
  for index = 1, table.maxn(targets), 1 do
    
    if transceiverItem.TargetId == targets[index].Id and targets[index].Info.AimPointPosition ~= nil then
      targets[index].AssignedCount = targets[index].AssignedCount + 1
      transceiverItem.TargetInfo = targets[index].Info
      return transceiverItem
    end
  end
  return nil
end


--------------------
-- 
--------------------
transceiverMap = {}
function SetMissleTargets(I)
  -- ターゲットリストの作成
  local targets = {}
  for mainframeIndex = 0, I:GetNumberOfMainframes() - 1, 1 do
    for targetIndex = 0, I:GetNumberOfTargets(mainframeIndex) - 1, 1 do
      local targetInfo = I:GetTargetInfo(mainframeIndex, targetIndex)
      local d = GetDistance(targetInfo.Position, me.selfPos)
      if d < MISSILE_ASSIGN_TARGET_DISTANCE then
        table.insert(targets, {Id = targetInfo.Id, Info = targetInfo, AssignedCount = 0})
        if MISSILE_MULTI_LOCK_COUNT ~= 0 and table.maxn(targets) == MISSILE_MULTI_LOCK_COUNT then
          break
        end
      end
    end
  end

  if table.maxn(targets) == 0 then
    return
  end
    

  local newMap = {}
  local unassinedMap = {}
  -- 割り当て済みの判定
  for luaTransceiverIndex = 0, I:GetLuaTransceiverCount() - 1, 1 do
    local isAssined = false
    local transcieverInfo = I:GetLuaTransceiverInfo(luaTransceiverIndex)
    local i = 1
    while (table.maxn(transceiverMap) >= i) do
      if transceiverMap[i].LocalPosition == transcieverInfo.LocalPosition then
        local assigned = IsTransceiverAssigned(transceiverMap[i], targets)
        if assigned ~= nil then
          transceiverMap[i] = assigned
          transceiverMap[i].Index = luaTransceiverIndex
          table.insert(newMap, transceiverMap[i])
          table.remove(transceiverMap, i)
          isAssined = true
          break
        end
      end
      i = i + 1
    end 

    if isAssined == false then
      table.insert(unassinedMap, {LocalPosition = transcieverInfo.LocalPosition, TargetId = nil, Index = luaTransceiverIndex})
    end
  end

  -- 未割当をアサイン
  for index = 1, table.maxn(unassinedMap), 1 do
    local minCount = nil
    local minCountIndex = nil
    for targetIndex = 1, table.maxn(targets), 1 do
      if minCount == nil or targets[targetIndex].AssignedCount < minCount then
        minCount = targets[targetIndex].AssignedCount
        minCountIndex = targetIndex
      end
    end

    table.insert(newMap, {Index = unassinedMap[index].Index,
                          LocalPosition = unassinedMap[index].LocalPosition, 
                          TargetId = targets[minCountIndex].Id, 
                          TargetInfo = targets[minCountIndex].Info})

    targets[minCountIndex].AssignedCount = targets[minCountIndex].AssignedCount + 1
  end

  transceiverMap = newMap
  
end

--------------------
-- 
--------------------
function ControlMissiles(I)
  
  SetMissleTargets(I)

  
  for index = 1, table.maxn(transceiverMap), 1 do
    local luaTransceiverIndex = transceiverMap[index].Index
    local transcieverInfo = transceiverMap[index].Info
    local target = transceiverMap[index].TargetInfo
    local aim = target.AimPointPosition
    if aim == nil then
      aim = target.Position
    end
  -- end
  -- for luaTransceiverIndex = 0, I:GetLuaTransceiverCount() - 1, 1 do
  --   local transcieverInfo = I:GetLuaTransceiverInfo(luaTransceiverIndex)
    for missileIndex = 0, I:GetLuaControlledMissileCount(luaTransceiverIndex) - 1, 1 do
      local missileInfo = I:GetLuaControlledMissileInfo(luaTransceiverIndex,missileIndex)
      if missileInfo.Valid then
        local missilePosition = missileInfo.Position
        local d = GetDistance(aim, missilePosition)
        local d2 = GetDistance(aim, Vector3(missilePosition.x, aim.y, missilePosition.z))
        
        if d < MISSILE_RUN_DISTANCE and target.Velocity.magnitude > 20 then
          local hitTime = d / (missileInfo.Velocity - target.Velocity).magnitude
          aim = target.AimPointPosition + target.Velocity * hitTime
        end
  
        local a = (aim - missilePosition)
        local b = missilePosition + (a.normalized * (a.magnitude / 4))
  
        if target.Velocity.magnitude > 20 then
          -- direct
          if d < 1 then
            I:DetonateLuaControlledMissile(luaTransceiverIndex, missileIndex)
          else
            I:SetLuaControlledMissileAimPoint(luaTransceiverIndex, missileIndex, aim.x, aim.y, aim.z)
          end
        elseif d2 > MISSILE_RUN_DISTANCE and MISSILE_TOP_ATTACK_ENABLED then
          if MISSILE_LAUNCH_TYPE then
            -- top attack
            if missilePosition.y < 200 then
              I:SetLuaControlledMissileAimPoint(luaTransceiverIndex, missileIndex, missilePosition.x, MISSILE_RUN_ALTITUDE, missilePosition.z)
            else
              I:SetLuaControlledMissileAimPoint(luaTransceiverIndex, missileIndex, b.x, MISSILE_RUN_ALTITUDE, b.z)
            end
          else
            -- bottom attack
            I:SetLuaControlledMissileAimPoint(luaTransceiverIndex, missileIndex, b.x, Mathf.Max(100, aim.y - MISSILE_RUN_ALTITUDE), b.z)
          end
        elseif d < 1 then
          I:DetonateLuaControlledMissile(luaTransceiverIndex, missileIndex)
        else
          I:SetLuaControlledMissileAimPoint(luaTransceiverIndex, missileIndex, aim.x, aim.y, aim.z)
        end
        end
    end
  end
end

--------------------
-- 
--------------------
function RequestControlYaw(I, index, info, steer)
  if info.LocalPositionRelativeToCom.z > 0 then
    -- front
    I:SetSpinnerInstaSpin(index, steer)
  elseif info.LocalPositionRelativeToCom.z < 0 then
    -- rear
    I:SetSpinnerInstaSpin(index, -steer)
  end
end

--------------------
-- 
--------------------
function RequestControl(I, me, request, repairTarget)
  
  local fd = request.forwardDrive
  local steer = request.steer

  local forwardDrive = Pid:GetGain(I, "FORWARD", fpid, fd, lastForwardDrive)
  if fd == 0 then
    forwardDrive = 0
  end
  lastForwardDrive = forwardDrive
  I:Log("fd="..fd..", forwardDrive="..forwardDrive)
  I:Log("req:a=".. request.again..", r=".. request.rgain.. ", p=".. request.pgain..", y="..request.ygain.. ", d"..request.forwardDrive)
  
  local r = 1
  if me.roll < 1 or me.roll > -1 then
    r = 1
  elseif me.roll > 45 or me.roll < -45 then
    r = 0
  else
    r = 45 / me.roll
  end
  local hydrofoilBooster = 45 * request.forwardDrive * r * me.isForwarding
  I:Log("hydrofoilBooster"..hydrofoilBooster)

  if me.pitch > 0 then
    I:RequestControl(2, 5, -request.pgain)
  else
    I:RequestControl(2, 4, request.pgain)
  end

  for index = 0, I:Component_GetCount(8) - 1, 1 do
    local info = I:Component_GetBlockInfo(8, index)
    
    if 1 == info.LocalForwards.y then
      -- AI hook

    elseif 1 == info.LocalForwards.z and 0 == info.LocalForwards.x  then
      local pr = 0
      local pp = 0
      local pa = request.again
  
      if info.LocalPositionRelativeToCom.z > 0 then
        pp = -request.pgain
      elseif info.LocalPositionRelativeToCom.z < 0 then
        pp = request.pgain
      end

      if info.LocalPositionRelativeToCom.x > 0 then
        pr = request.rgain
      elseif info.LocalPositionRelativeToCom.x < 0 then
        pr = -request.rgain
      elseif info.LocalPositionRelativeToCom.x == 0 then
--          pp = 0
      end

      local ha = (pp + pr + pa)
      I:Component_SetFloatLogic(8, index, ha * me.isForwarding)
    else
      I:Component_SetFloatLogic(8, index, hydrofoilBooster)
    end
  end
  
  local yawAngle = (-request.steer) * HYDROFOIL_YAW_ANGLE * (10 - Mathf.Min(Mathf.Abs(me.roll), 10)) / 10 * me.isForwarding

  for index = 0, I:GetSpinnerCount() - 1, 1 do
    local info = I:GetSpinnerInfo(index)
    if I:IsSpinnerDedicatedHelispinner(index) then
      -- デディブレの向き（Gキ－押下時）と使用（driveが正の時）
      -- 下：右ヨー
      -- 上：左ヨー
      -- 前：上昇（ピッチ・高度・ロール）
      -- 後：下降（ピッチ・高度・ロール）
      -- 右：前進
      -- 左：後進
      if info.LocalForwards.x == 0 and info.LocalForwards.z == 0 then
        -- 下：右ヨー
        -- 上：左ヨー
        RequestControlYaw(I, index, info, request.steer * info.LocalForwards.y * -1)
      elseif info.LocalForwards.x == 0 and info.LocalForwards.y == 0 then
        -- 前：上昇（ピッチ・高度・ロール）
        -- 後：下降（ピッチ・高度・ロール）
        local pr = 0
        local pp = 0
        local pa = request.again
    
        if info.LocalPositionRelativeToCom.z > ALT_DEDIBLADE_RANGE_FROM_COM then
          pp = -request.pgain * info.LocalForwards.z
          pa = 0
        elseif info.LocalPositionRelativeToCom.z < -ALT_DEDIBLADE_RANGE_FROM_COM then
          pp = request.pgain * info.LocalForwards.z
          pa = 0
        elseif ALT_CONTROL_MIN_ZERO and pa < 0 then
          pa = 0
        end
  
        if info.LocalPositionRelativeToCom.x > 0 then
          pr = request.rgain * info.LocalForwards.z
        elseif info.LocalPositionRelativeToCom.x < 0 then
          pr = -request.rgain * info.LocalForwards.z
        elseif info.LocalPositionRelativeToCom.x == 0 then
--          pp = 0
        end

        if UPSIDE_DOWN == 1 then
          --pa = -pa
          pp = -pp
        end

        local ha = (pp + pr + pa)

        I:SetSpinnerInstaSpin(index, ha)
      elseif info.LocalForwards.z == 0 and info.LocalForwards.y == 0 then
        -- 右：前進
        -- 左：後進
        if request.cutoff ~= true then
          local f = fd
          if FORWARD_USE_YAWING then
            if info.LocalPositionRelativeToCom.x > 0 and request.steer == 1 then
              f = 0
            elseif info.LocalPositionRelativeToCom.x < 0  and request.steer == -1 then
              f = 0
            end
          end
          I:SetSpinnerInstaSpin(index, info.LocalForwards.x * f)
          I:Log("f "..f)
        end
      else
        I:SetSpinnerInstaSpin(index, 0)
      end
    else      
      -- spin block
      if info.LocalForwards.x == 0 and info.LocalForwards.y == 0 and info.LocalForwards.z == 1 then
        -- hydrofoil booster base
        if info.LocalPositionRelativeToCom.z > 0 then
          I:SetSpinnerRotationAngle(index, HYDROFOIL_STANDBY_ANGLE - yawAngle)
        else
          I:SetSpinnerRotationAngle(index, HYDROFOIL_STANDBY_ANGLE + yawAngle)
        end
      else
        --        I:Log("spinner "..info.LocalForwards:ToString() .. ", pos".. info.LocalPosition:ToString())
--        I:SetSpinnerRotationAngle(index, 0)
      end
    end
  end

  I:RequestControl(0,8,fd)
  I:RequestControl(2,8,fd)
  
  if request.isHeliYaw then
    -- yaw control
    if request.steer == 1 then
      I:RequestControl(0, YAW_LEFT, 0)
      I:RequestControl(2, YAW_LEFT, 0)
      I:RequestControl(0, YAW_RIGHT, request.ygain)
      I:RequestControl(2, YAW_RIGHT, request.ygain)
    elseif steer == -1 then
      I:RequestControl(0, YAW_RIGHT, 0)
      I:RequestControl(2, YAW_RIGHT, 0)
      I:RequestControl(0, YAW_LEFT, request.ygain)
      I:RequestControl(2, YAW_LEFT, request.ygain)
    elseif steer == 0 then
      ygain = 0
      I:RequestControl(0, YAW_RIGHT, 0)
      I:RequestControl(2, YAW_RIGHT, 0)
      I:RequestControl(0, YAW_LEFT, ygain)
      I:RequestControl(2, YAW_LEFT, ygain)
      lasyYaw = ygain
    end
  end

end
  

turnSwitch = ""
lastSide = ""
swingSwitch = ""

--------------------
-- 
--------------------
function KeepMaxAngle(angle, sideFactor)
  if angle > MAX_ANGLE then
    swingSwitch = "inside"
    return 1 * sideFactor
  elseif angle < MAX_ANGLE - SWING_ANGLE then
    swingSwitch = "border"
    return -1 * sideFactor
  elseif swingSwitch == "inside" then
    return 1 * sideFactor
  elseif swingSwitch == "border" then
    return -1 * sideFactor
  else
    return 0
  end
end

--------------------
-- 
--------------------
function KeepMinAngle(angle, sideFactor)
  if angle < MIN_ANGLE then
    swingSwitch = "inside"
    return -1 * sideFactor
  elseif angle > MIN_ANGLE + SWING_ANGLE then
    swingSwitch = "border"
    return 1 * sideFactor
  elseif swingSwitch == "inside" then
    return -1 * sideFactor
  elseif swingSwitch == "border" then
    return 1 * sideFactor
  else
    return 0
  end
end

--------------------
-- 
--------------------
function GetTargetPositionInfo(I)
  if I:GetNumberOfMainframes() == 0 then
    return nil
  end

  if I:GetNumberOfTargets(0) == 0 then
    return nil
  end

  return I:GetTargetPositionInfo(0, 0)
end

--------------------
-- 
--------------------
function GetTargetInfo(I)
  if I:GetNumberOfMainframes() == 0 then
    return nil
  end

  if I:GetNumberOfTargets(0) == 0 then
    return nil
  end

  return I:GetTargetInfo(0, 0)
end

--------------------
-- 
--------------------
function ResetSwitch()
  turnSwitch = ""
  lastSide = ""
end

--------------------
-- 
--------------------
function DetectTerrainAngle(I, direction, detectAngle, angleResolution)
  local distanceResolution = 20
  local score = 0
  local avoidTerrainDistance = AVOID_TERRAIN_DISTANCE_RATE * me.mag
  for angle = angleResolution, detectAngle, angleResolution do
    local q = Quaternion.Euler(0, angle * direction, 0) * (me.fvec * me.isForwarding)
    local scoreAtAngle = 0
    for distance = distanceResolution, avoidTerrainDistance, distanceResolution do
      local point = me.selfPos + (q * distance)
      local terrain = I:GetTerrainAltitudeForPosition(point)
  
      if InRange(terrain, AVOID_TERRAIN_ALT_LOWER, AVOID_TERRAIN_ALT_UPPER) == false then
        scoreAtAngle = scoreAtAngle + (avoidTerrainDistance - distance)
        break
      end
    end
    score = score + scoreAtAngle
    if scoreAtAngle == 0 then
      return score
    end
  end
  return score
end
  
--------------------
-- 
--------------------
function DetectTerrainRange(I, detectAngle, angleResolution)
  local leftScore = DetectTerrainAngle(I, -1, detectAngle, angleResolution)
  local rightScore = DetectTerrainAngle(I, 1, detectAngle, angleResolution)
  local ret = {}
  
  if leftScore > rightScore then
    ret.steer = 1
  elseif leftScore < rightScore then
    ret.steer = -1
  else
    ret.steer = 0
  end
  return ret
end

--------------------
-- 
--------------------
function AvoidTarget(I, me, targetPosition, avoidAngle, avoidDistance)
  local v = (targetPosition - me.selfPos)
  local targetAngle = Angle(me.fvec, v)
  if me.isForwarding == -1 then
    targetAngle = 180 - targetAngle
  end
  local distance = v.magnitude
  local ret = {}
  ret.steer = 0
  if targetAngle < avoidAngle and distance < avoidDistance then
    local side = Vector3.Dot(v.normalized, me.rvec)
    local distanceRate = distance / avoidDistance
    ResetSwitch()
    if side > 0 then
      ret.distanceRate = distanceRate
      ret.distance = distance
      ret.steer = -1
      return ret
    else
      ret.distanceRate = distanceRate
      ret.distance = distance
      ret.steer = 1
      return ret
    end
  end
  return ret
end

--------------------
-- 
--------------------
function AvoidCollision(I, me, followTarget, request)
  local distanceResolution = 20
  local ret = {}

  -- 味方回避
  local friendJudge = nil
  local fridneDistance = nil
  local avoidFriendDistance = AVOID_FRIEND_DISTANCE_RATE * me.mag
  for index = 0, I:GetFriendlyCount() - 1, 1 do
    local info = I:GetFriendlyInfo(index)
    local targetPosition = me.selfPos
    local vectorFromTarget = (targetPosition - info.CenterOfMass)
    local targetDistance = vectorFromTarget.magnitude
    local avoidJudge = nil

    if followTarget ~= nil and followTarget.Info ~= nil and followTarget.Info.Id == info.Id then
       -- repair
       local front = Vector3.Dot(vectorFromTarget.normalized, info.ForwardVector)

       if front < 0 then
        -- 自分が味方の後ろにいる、かつ、近い
        ret.targetDistance = targetDistance
        ret.fromBehind = true
        ret.friend = true
        avoidJudge = AvoidTarget(I, me, info.ReferencePosition, AVOID_FRIEND_ANGLE, MAX_REPAIR_DISTANCE)
        I:Log("detect friend0 ".. avoidJudge.steer)
      else
        -- 自分が味方の前にいる、または、後ろにいるけど遠い
        avoidJudge = AvoidTarget(I, me, info.ReferencePosition, AVOID_FRIEND_ANGLE, avoidFriendDistance)
        ret.friend = true
        I:Log("detect friend1 ".. avoidJudge.steer)
      end
    else
      avoidJudge = AvoidTarget(I, me, info.ReferencePosition, AVOID_FRIEND_ANGLE, avoidFriendDistance)
      ret.friend = true
      I:Log("detect friend2")
    end

    if avoidJudge ~= nil then
      -- 一番近いやつを優先
      if fridneDistance == nil or fridneDistance > targetDistance then
        fridneDistance = targetDistance
        friendJudge = avoidJudge
      end
    end
  end
  if friendJudge ~= nil then
    return friendJudge
  end

  -- 正面地形回避
  local avoidTerrainDistance = AVOID_TERRAIN_DISTANCE_RATE * me.mag

  for distance = distanceResolution, avoidTerrainDistance, distanceResolution do
    local terrain = I:GetTerrainAltitudeForPosition(me.selfPos + (me.fvec * distance * request.needForward))
    if InRange(terrain, AVOID_TERRAIN_ALT_LOWER, AVOID_TERRAIN_ALT_UPPER) == false then
      ResetSwitch()
      ret = DetectTerrainRange(I, 180, 10)
      ret.distance = distance
      ret.distanceRate = (distance / avoidTerrainDistance)
      I:Log("detect terrain".. terrain..", "..AVOID_TERRAIN_ALT_LOWER .."-"..AVOID_TERRAIN_ALT_UPPER)
      return ret
    end
  end

  -- 敵との衝突回避
  for mainframeIndex = 0, I:GetNumberOfMainframes() - 1, 1 do
    for targetIndex = 0, I:GetNumberOfTargets(mainframeIndex), 1 do
      local avoidJudge = AvoidTarget(I, me, I:GetTargetInfo(mainframeIndex, targetIndex).Position, AVOID_ENEMY_ANGLE, AVOID_ENEMY_DISTANCE)
      if avoidJudge ~= nil then
        return avoidJudge
      end
    end
  end

  -- 正面じゃないけど前方の地形回避
  ret = DetectTerrainRange(I, 90, 10)
  ret.distanceRate = 1
  return ret
end

--------------------
-- 操作指示の取得
--------------------
function GetControlRequest(I, me, tpi, forwardDrive, followTarget)
  local ret = {}
  --local steer = 0 -- 0 = steady, 1 = right, -1 = left

  if me.fmag < 3 then
    ret.steer = 0
  end

  local needRoll = 0
  local needPitch = 0
  local needAlt = CRUISE_ALT

  ret.forwardDrive = forwardDrive
  ret.steer = 0
  I:Log("needRoll"..needRoll..", forwardDrive"..forwardDrive)

  --ret.hydrofoilBooster = 0
  --local p = me.pitch
  --local a = Angle(me.vvec, me.fvec)

  if ALT_FOLLOW_TERRAIN_DISTANCE ~= nil then
    -- 地形追従
    local minAngle = 0
    local maxAngle = 0
    local minAlt = nil
    local maxAlt = nil
    local forwardingRate = me.isForwarding
    if forwardingRate == 0 then
      forwardingRate = 1
    end

    for distance = 0, AVOID_TERRAIN_DISTANCE_RATE * me.mag, 5 do
      local point = me.com + (me.fvec * forwardingRate * distance)
      local terrain = I:GetTerrainAltitudeForPosition(point)
      local terrainPos = Vector3(point.x, terrain, point.z)
      local vectorToTerrain = terrainPos - me.selfPos
      local angleToTerrain = Angle(vectorToTerrain, me.fvec * forwardingRate)
      local altDirection = 0

      if minAngle > angleToTerrain then
        minAngle = angleToTerrain
      end
      if minAlt == nil or minAlt < terrain + ALT_FOLLOW_TERRAIN_DISTANCE then
        minAlt = terrain + ALT_FOLLOW_TERRAIN_DISTANCE
      end
    end

    I:Log("minAlt"..minAlt)

    if ALT_MANEUVER == 1 then
      -- elevator
      if AVOID_TERRAIN_ALT_UPPER < minAlt then
        needAlt = AVOID_TERRAIN_ALT_UPPER
      elseif me.com.y < CRUISE_ALT then
        needAlt = CRUISE_ALT
      elseif AVOID_TERRAIN_ALT_LOWER > minAlt then
        needAlt = AVOID_TERRAIN_ALT_LOWER
      end

    elseif ALT_MANEUVER == 2 then
      -- escalator
      if maxAngle > 0 then
        needPitch = maxAngle
      elseif me.selfPos.y < CRUISE_ALT then
        needAlt = CRUISE_ALT
      end
    elseif ALT_MANEUVER == 3 then
      -- elevator follow terrain
      if AVOID_TERRAIN_ALT_UPPER < minAlt then
        needAlt = AVOID_TERRAIN_ALT_UPPER
      elseif AVOID_TERRAIN_ALT_LOWER > minAlt then
        needAlt = AVOID_TERRAIN_ALT_LOWER
      else
        needAlt = minAlt
      end
    elseif ALT_MANEUVER == 4 then
      -- escalator follow terrain
      if maxAngle > 0 then
        needPitch = maxAngle
      elseif minAngle < 0 then
        needPitch = minAngle
      end
    end

  end

  if followTarget ~= nil and followTarget.Info ~= nil then
    -- 修理対象の真後ろ50mを目指す
    local vectorToTarget = (me.selfPos - followTarget.Info.ReferencePosition)
    local vectorFromTarget = (followTarget.Info.ReferencePosition - me.selfPos)
    local nosePos = me.selfPos + (me.fvec * I:GetConstructMaxDimensions().z)
    local destination = followTarget.Info.ReferencePosition - (followTarget.Info.ForwardVector * (REPAIR_DISTANCE + followTarget.Info.NegativeSize.z))
    local vectorToDest = (destination - nosePos)
    local side = Vector3.Dot(vectorToDest.normalized, me.rvec)
    local angle = Angle(vectorToDest, me.fvec)
    local front = Vector3.Dot(vectorToDest.normalized, me.fvec)
    local fromFront = Vector3.Dot(vectorFromTarget.normalized, followTarget.Info.Velocity.normalized)
    local s = ((180 - angle) ^ 3) / (180 ^ 3)
    if vectorToDest.magnitude > 300 then
      f = s
    else
      f = vectorToDest.magnitude / 300 * s
    end
    if fromFront > 0 and vectorToDest.magnitude > 500 then
      I:Log("repair 1")
      -- 目標の前方にいる（遠め）
      if side > 0 and angle > 5 then
        ret.steer = -1
      elseif side < 0 and angle > 5 then
        ret.steer = 1
      end
    elseif vectorToDest.magnitude < 100 then
      I:Log("repair 2")
      if side > 0 and angle > 5 then
        ret.steer = -1
      elseif side < 0 and angle > 5 then
        ret.steer = 1
      end
    elseif front > 0 or (front < 0 and vectorToDest.magnitude > 300) then
      I:Log("repair 3")
      if side > 0 and angle > 5 then
        ret.steer = 1
      elseif side < 0 and angle > 5 then
        ret.steer = -1
      end
    elseif front < 0 and vectorToDest.magnitude < 300 then
      I:Log("repair 4")
      if side > 0 and angle > 5 then
        ret.steer = 1
      elseif side < 0 and angle > 5 then
        ret.steer = -1
      end
      f = -f
    elseif front < 0 and vectorToDest.magnitude > MAX_REPAIR_DISTANCE then
      I:Log("repair 5")
      if side > 0 then
        ret.steer = -1
      elseif side < 0 then
        ret.steer = 1
      end
      f = forwardDrive
    end
    forwardDrive = f
    ret.forwardDrive = f

    ret.ygain = Pid:GetGain(I, "YAW", ypid, ret.steer, lasyYaw)
    lasyYaw = ret.ygain
    I:Log("MOVE TO REPAIR!")
  end



  if tpi ~= nil then
    local targetPosition = tpi.Position
    local vectorToTarget = (targetPosition - me.selfPos)
    local side = Vector3.Dot(vectorToTarget.normalized, me.rvec)
    local angle = Angle(vectorToTarget, me.fvec)
    local targetDistance = vectorToTarget.magnitude

    if FROM_TARGET_ALT ~= nil then
      needAlt = targetPosition.y + FROM_TARGET_ALT
      if MAX_ALT < needAlt then
        needAlt = MAX_ALT
      end
      if MIN_ALT > needAlt then
        needAlt = MIN_ALT
      end
    end

    if PITCH_TYPE == 1 then
      -- local p1 = Vector3.Project(vectorToTarget, me.rvec)
      -- local p2 = Vector3.Project(vectorToTarget, me.uvec)
      -- local p3 = Vector3.Project(vectorToTarget, me.fvec)
      -- local targetLocalPosition = Vector3(p1.x, p2.y, p3.z)
      -- local localPitcnAngle = Angle(targetLocalPosition, Vector3(targetLocalPosition.x, 0, targetLocalPosition.z))

      local targetHorizonVec = Vector3(targetPosition.x, me.selfPos.y, targetPosition.z)
      needPitch = Angle(targetHorizonVec, targetPosition)
      I:LogToHud("needPitch".. needPitch)
    end

    if lastSide == "" and side < 0 then
      lastSide = "left"
    elseif lastSide == "" and side >= 0 then
      lastSide = "right"
    end

    if side < 0 then
      currentSide = "left"
    else
      currentSide = "right"
    end


    if ROUTE_PATTERN == 0 then
      -- around (for side weapons)
      if side > 0 then
        sideFactor = 1
      else
        sideFactor = -1
      end

      -- target in right
      if targetDistance > MAX_DISTANCE then
        turnSwitch = "enter"
        -- too far
        steer = KeepMinAngle(angle, sideFactor)
      elseif targetDistance < MIN_DISTANCE then
        turnSwitch = "leave"
        -- too near
        steer = KeepMaxAngle(angle, sideFactor)
      elseif turnSwitch == "" then
        turnSwitch = "leave"
      end
      --I:LogToHud('turnSwitch='..turnSwitch)

    elseif ROUTE_PATTERN == 1 then
      -- figure 8 (for front weapons)
      sideFactor = 1

      if lastSide == "left" then
        sideFactor = -1
      elseif lastSide == "right" then
        sideFactor = 1
      end

      if targetDistance < MIN_DISTANCE and turnSwitch ~= "turn" then
        turnSwitch = "leave"
      elseif targetDistance > MAX_DISTANCE and lastSide == currentSide then
        turnSwitch = "turn"
      elseif lastSide ~= currentSide and angle > MIN_ANGLE then
        turnSwitch = "enter"
        lastSide = currentSide
      elseif turnSwitch == "" then
        turnSwitch = "leave"
      end

      if lastSide == "right" then
        turnFactor = 1
      elseif lastSide == "left" then
        turnFactor = -1
      end

      --I:LogToHud(turnSwitch.. "lastSide="..lastSide.. ", angle=".. angle.. ",distance=".. targetDistance)
    elseif ROUTE_PATTERN == 2 then
      -- reversed figure 8 (for rear weapons)
      sideFactor = 1
      
      if lastSide == "left" then
        sideFactor = -1
      elseif lastSide == "right" then
        sideFactor = 1
      end

      if turnSwitch == "" then
        turnSwitch = "leave"
      elseif turnSwitch == "leave" and targetDistance >= MAX_DISTANCE then
        turnSwitch = "enter"
      elseif turnSwitch == "enter" and targetDistance < MIN_DISTANCE then
        turnSwitch = "turn"
      elseif turnSwitch == "turn" and lastSide ~= currentSide and angle < MAX_ANGLE then
        turnSwitch = "switch"
        lastSide = currentSide
      elseif turnSwitch == "switch" and targetDistance < MAX_DISTANCE then
        turnSwitch = "leave"
      elseif turnSwitch == "switch" and targetDistance >= MAX_DISTANCE then
        turnSwitch = "enter"
      end

      if lastSide == "right" then
        turnFactor = -1
      elseif lastSide == "left" then
        turnFactor = 1
      end
    
    elseif ROUTE_PATTERN == 5 then
      -- back to enemy

      if side > 0 then
        sideFactor = 1
      else
        sideFactor = -1
      end

      if targetDistance >= MAX_DISTANCE then
        turnSwitch = "back"
      elseif targetDistance <= MIN_DISTANCE then
        turnSwitch = "leave"
      else
        turnSwitch = "leave"
--        ret.forwardDrive = ret.forwardDrive * 0.1
      end

    elseif ROUTE_PATTERN == 6 then
      -- front to enemy
      if side > 0 then
        sideFactor = 1
      else
        sideFactor = -1
      end

      if targetDistance >= MAX_DISTANCE then
        turnSwitch = "enter"
      elseif targetDistance <= MIN_DISTANCE then
        turnSwitch = "backMin"
      elseif turnSwitch == "" then
        turnSwitch = "backMin"
      end
    end

    if turnSwitch == "leave" then
      ret.steer = KeepMaxAngle(angle, sideFactor)
    elseif turnSwitch == "turn" or turnSwitch == "switch" then
      if lastSide == "right" then
        ret.steer = turnFactor
      elseif lastSide == "left" then
        ret.steer = turnFactor
      end
    elseif turnSwitch == "enter" then
      ret.steer = KeepMinAngle(angle, sideFactor)
    elseif turnSwitch == "back" then
      ret.steer = KeepMaxAngle(angle, sideFactor)
      ret.forwardDrive = ret.forwardDrive * -1
    elseif turnSwitch == "backMin" then
      ret.steer = KeepMinAngle(angle, sideFactor)
      ret.forwardDrive = ret.forwardDrive * -1
    end
    I:Log("steer"..ret.steer.."drive"..ret.forwardDrive)
  end
    

  -- AI hook. AI優先
  if I.AIMode == "patrol" then
    for index = 0, I:Component_GetCount(8) - 1, 1 do
      local info = I:Component_GetBlockInfo(8, index)
      if 1 == info.LocalForwards.y then
        if info.LocalPosition.x > 0 then
          -- right. yaw
          local yaw = I:Component_GetFloatLogic(8, index)
          if yaw > 0 then
            ret.steer = 1
          elseif yaw < 0 then
            ret.steer = -1
          end
          I:Log("yaw "..yaw)
        else
          -- left. forward
          local forward = I:Component_GetFloatLogic(8, index)
          ret.forwardDrive = forward
          I:Log("forward "..forward)
        end
      end
    end
  end  

  ret.cutoff = false
  ret.isHeliYaw = true
  if CUTOFF_PITCH_OVER and me.pitch > 5 + needPitch or me.pitch < -5 + needPitch then
    I:Log("cut off. p="..me.pitch..",needPitch"..needPitch)
    ret.cutoff = true
    ret.isHeliYaw = false
  end

  if CUTOFF_ROLL_OVER and me.roll < -10 + needRoll or me.roll > 10 + needRoll then
    I:Log("cut off. r="..me.roll..",needRoll="..needRoll)
    ret.cutoff = true
    ret.isHeliYaw = false
  end

  if me.selfPos.y > MAX_ALT + 5 or me.selfPos.y < MIN_ALT - 5 then
    I:Log("cut off. a="..me.selfPos.y.. ",MIN_ALT"..MIN_ALT..",MAX_ALT"..MAX_ALT)
    ret.cutoff = true
    ret.isHeliYaw = false
--    ret.hydrofoilBooster = 0
  end

  if CUTOFF_UNFORWARD and me.isForwarding == 0 then
    ret.cutoff = true
    ret.isHeliYaw = false
  end

  if ret.forwardDrive > 0 then
    ret.needForward = 1
  elseif ret.forwardDrive < 0 then
    ret.needForward = -1
  else
    ret.needForward = 0
  end

  local avoid = AvoidCollision(I, me, followTarget, ret)

  -- 回避最優先
  if avoid.steer > 0 then
    ret.steer = avoid.steer
    ret.forwardDrive = avoid.distanceRate
  elseif avoid.steer < 0 then
    ret.steer = avoid.steer
    ret.forwardDrive = avoid.distanceRate
  else
--    ret.steer = 0
  end

  ret.pgain = Pid:GetGain(I, "PITCH", ppid, needPitch, me.pitch)
  ret.rgain = Pid:GetGain(I, "ROLL", rpid, needRoll, me.roll)
  ret.again = Pid:GetGain(I, "ALT", apid, needAlt, me.selfPos.y)
  ret.ygain = Pid:GetGain(I, "YAW", ypid, ret.steer, lasyYaw)
  lasyYaw = ret.ygain
  I:Log("needAlt"..needAlt.. ",needPitch="..needPitch)
  I:Log("req:turnSwitch=".. turnSwitch..",a=".. ret.again..", r=".. ret.rgain.. ", pgain=".. ret.pgain)
  return ret
end

--------------------
-- 横向き魚雷
--------------------
function AimSideways(I, targetPosition, me)

  local vectorToTarget = (targetPosition - me.selfPos)
  local side = Vector3.Dot(vectorToTarget.normalized, me.rvec)
  local aim = nil

  if side > 0 then
    aim = me.rvec
  else
    aim = me.rvec * -1
  end  

  local isFire = false
  if vectorToTarget.magnitude < 3000 and targetPosition.y < 10 then
    isFire = true
  end

  for weaponIndex = 0, I:GetWeaponCount() - 1, 1 do
    local info = I:GetWeaponInfo(weaponIndex)
    if info.WeaponSlot == AIM_SIDEWAYS_SLOT and info.WeaponType == 4 then -- 4=turret
      local aimedCount = I:AimWeaponInDirection(weaponIndex,aim.x,aim.y,aim.z,AIM_SIDEWAYS_SLOT)

      local aimedAngle = Angle(info.CurrentDirection, aim)
      if aimedAngle < 1 and isFire then
        I:FireWeapon(weaponIndex, info.WeaponSlot)
      end
    end
  end
end

--------------------
-- 
--------------------
lastHelth = ""
function ViewHelth(I)
  local i=1
  while (I.Fleet ~= nil and I.Fleet.Members ~= nil and I.Fleet.Members[i].CenterOfMass ~= me.com) do
   i=i+1
  end
  
  local myname = I:GetFriendlyInfoById(I.Fleet.Members[i].Id).BlueprintName
  local myhp = math.floor(I:GetHealthFraction()*50)*2
  local myAmmo = math.floor(I:GetAmmoFraction()*10)*10
  local myFuel = math.floor(I:GetFuelFraction()*10)*10
  local s = ""
  if I.Fleet.Name == myname then
    s = myname .." HP:"..myhp.."%" .." Ammo:"..myAmmo.."%" .." Fuel:"..myFuel.."%"
  else
    s = I.Fleet.Name .. " " .. myname .." HP:"..myhp.."%" .." Ammo:"..myAmmo.."%" .." Fuel:"..myFuel.."%"
  end
  if lastHelth ~= s then
    I:LogToHud(s)
    lastHelth = s
  end
end


--------------------
-- 
--------------------
function GetVehicleStatus(I)
  me = {}

  if I.Fleet.Members ~= nil and table.maxn(I.Fleet.Members) > 1 then
    local i=1
    while (I.Fleet.Members[i].CenterOfMass ~= me.com) do
     i=i+1
    end
  end

  me.info = I.Fleet.Members[i]
  me.selfPos = I:GetConstructPosition()
  me.com = I:GetConstructCenterOfMass()
  me.fvec = I:GetConstructForwardVector()
  me.rvec = I:GetConstructRightVector()
  me.uvec = I:GetConstructUpVector()
  me.vvec = I:GetVelocityVectorNormalized()
  me.fmag = I:GetForwardsVelocityMagnitude()
  me.pitch = ZeroOrigin(I:GetConstructPitch())
  me.mag = I:GetVelocityMagnitude()  

  if UPSIDE_DOWN == 1 then
    me.roll = I:GetConstructRoll() - 180
  else
    me.roll = ZeroOrigin(I:GetConstructRoll())
  end

  local a = Angle(me.vvec, me.fvec)
  if a < 45 then -- ?
    me.isForwarding = 1
  elseif a > 135 then
    me.isForwarding = -1
  else
    me.isForwarding = 0
  end
  return me
end


friendLastPosition = nil
friendId = nil
--------------------
-- 
--------------------
function GetRepairTarget(I)
  local hp = 2
  local ret = {}
  for index = 0, I:GetFriendlyCount() - 1, 1 do
    local info = I:GetFriendlyInfo(index)
    if hp > info.HealthFraction then
      hp = info.HealthFraction
      ret.Info = info
      
      if friendId ~= info.Id then
        if friendLastPosition ~= nil then
          ret.Speed = (info.ReferencePosition - friendLastPosition).magnitude * 40
        else
          ret.Speed = 0
        end
        friendLastPosition = info.ReferencePosition
        friendId = info.Id
      end
    end
  end
  return ret
end

--------------------
-- 
--------------------
function GetPervMember(I)
  local id = 0
  local ret = {}
  local memberCount = table.maxn(I.Fleet.Members)
  local flagShip = nil

  if I.IsFlagship == true then
    return nil
  end

  for index = 0, memberCount, 1 do
    local info = I.Fleet.Members[index]
    if info.Id ~= me.info.Id then
      if id == 0 or (info.Id < id and id < me.info.Id) then
        id = info.Id
        ret.Info = info
        if friendId ~= info.Id then
          if friendLastPosition ~= nil then
            ret.Speed = (info.ReferencePosition - friendLastPosition).magnitude * 40
          else
            ret.Speed = 0
          end
          friendLastPosition = info.ReferencePosition
          friendId = info.Id
        end
        end
    end
  end
  return ret
end


--------------------
-- 
--------------------
function Update(I)
  I:ClearLogs()
  local me = GetVehicleStatus(I)
  local tpi = GetTargetPositionInfo(I)
  local ti = GetTargetInfo(I)
  local request = {forwardDrive = 0, steer = 0, pgain = 0, rgain = 0, again = 0}
  
  local followTarget = nil
  local forwardDrive = 1
  if ROUTE_PATTERN == 3 then
    followTarget = GetRepairTarget(I)
  elseif ROUTE_PATTERN == 4 then
    followTarget = GetPervMember(I)
  elseif tpi == nil and I.AIMode == "on" then
    forwardDrive = 0
  end

  request = GetControlRequest(I, me, tpi, forwardDrive, followTarget)

  if I.AIMode == "off" then
    request.steer = 0
    request.forwardDrive = 0
    RequestControl(I, me, request, followTarget)
  elseif I.AIMode == "on" then
    request.forwardDrive = 0
    RequestControl(I, me, request, followTarget)
  elseif I.AIMode == "combat" then
    I:TellAiThatWeAreTakingControl()
    if tpi == nil and followTarget == nil then
      request.forwardDrive = 0
    end
    RequestControl(I, me, request, followTarget)
  else
    RequestControl(I, me, request, followTarget)
  end
  
  if tpi ~= nil and AIM_SIDEWAYS_SLOT ~= nil then
    AimSideways(I, tpi.Position, me)
  end

  if ti ~= nil then
    ControlMissiles(I)
  end

  ViewHelth(I)
end

