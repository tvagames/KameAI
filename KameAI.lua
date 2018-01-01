--------------------
-- オプション
--------------------
-- 敵との位置関係
MIN_DISTANCE = 1500
MAX_DISTANCE = 1550
MIN_ANGLE = 60
MAX_ANGLE = 120
SWING_ANGLE = 30

-- 高度
CRUISE_ALT = 400
MAX_ALT = 500
MIN_ALT = 300

-- ハイドロフォイルブースターのヨー操作利用
HYDROFOIL_YAW_ANGLE = 0
HYDROFOIL_STANDBY_ANGLE = 135

-- 横向き魚雷
AIM_SIDEWAYS_SLOT = 1

-- 回避
AVOID_TERRAIN_DISTANCE = 1000
AVOID_TERRAIN_ALT = 370
DETECT_TERRAIN_ANGLE = 40
AVOID_FRIEND_DISTANCE = 100
AVOID_FRIEND_ANGLE = 90
AVOID_ENEMY_DISTANCE = 200
AVOID_ENEMY_ANGLE = 90

-- 修理
REPAIR_DISTANCE = 50
MAX_REPAIR_DISTANCE = 100

-- 0 = around (for side weapons), 
-- 1 = figure 8 (for front weapons), 
-- 2 = reversed figure 8 (for rear weapons)
-- 3 = repair ship
-- 4 = follower
-- 5 = back to enemy
-- 6 = front to enemy
ROUTE_PATTERN = 1

-- 0 = horizon
-- 1 = upside down (天地逆転)
UPSIDE_DOWN = 0

-- 0 = horizon
-- 1 = look up enemy
PITCH_TYPE = 0

-- 0 = hovering
-- 1 = elevator
-- 2 = escalator
ALT_MANEUVER = 0

FORWARD_USE_YAWING = false
CUTOFF_PITCH_OVER = true
CUTOFF_ROLL_OVER = true
ALT_CONTROL_MIN_ZERO = true

MISSLE_TOP_ATTACK_ENABLED = true
MISSLE_RUN_DISTANCE = 500
MISSLE_RUN_ALTITUDE = 500

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

ppid = PidTurning.CreateConfig(0.1, 400, 1) -- 1200 * 25msecs = 30secs
apid = PidTurning.CreateConfig(0.05, 1200, 1)
rpid = PidTurning.CreateConfig(0.1, 400, 3)
fpid = PidTurning.CreateConfig(0.1, 1200, 10)
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



--------------------
-- 
--------------------
function ControlMissiles(I, target)
  local aim = target.AimPointPosition
  for luaTransceiverIndex = 0, I:GetLuaTransceiverCount() - 1, 1 do
    local transcieverInfo = I:GetLuaTransceiverInfo(luaTransceiverIndex)
    for missileIndex = 0, I:GetLuaControlledMissileCount(luaTransceiverIndex) - 1, 1 do
      local missileInfo = I:GetLuaControlledMissileInfo(luaTransceiverIndex,missileIndex)
      local missilePosition = missileInfo.Position
      local d = GetDistance(aim, missilePosition)
      local d2 = GetDistance(aim, Vector3(missilePosition.x, aim.y, missilePosition.z))
      if d < 100 then
        local hitTime = d / (missileInfo.Velocity - target.Velocity).magnitude
        aim = target.AimPointPosition + target.Velocity * hitTime
      end

      local a = (aim - missilePosition)
      local b = missilePosition + (a.normalized * (a.magnitude / 3))

      if target.Velocity.magnitude > 20 then
        -- direct
        if d < 5 then
          I:DetonateLuaControlledMissile(luaTransceiverIndex, missileIndex)
        else
          I:SetLuaControlledMissileAimPoint(luaTransceiverIndex, missileIndex, aim.x, aim.y, aim.z)
        end
      elseif d2 > MISSLE_RUN_DISTANCE and MISSLE_TOP_ATTACK_ENABLED then
        if aim.y < missilePosition.y then
          -- top attack
          I:SetLuaControlledMissileAimPoint(luaTransceiverIndex, missileIndex, b.x, aim.y + MISSLE_RUN_ALTITUDE, b.z)
        else
          -- bottom attack
          I:SetLuaControlledMissileAimPoint(luaTransceiverIndex, missileIndex, b.x, Mathf.Max(100, aim.y - MISSLE_RUN_ALTITUDE), b.z)
        end
      elseif d < 0 then
        I:DetonateLuaControlledMissile(luaTransceiverIndex, missileIndex)
      else
        I:SetLuaControlledMissileAimPoint(luaTransceiverIndex, missileIndex, aim.x, aim.y, aim.z)
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
    
        if info.LocalPositionRelativeToCom.z > 5 then
          pp = -request.pgain * info.LocalForwards.z
          pa = 0
        elseif info.LocalPositionRelativeToCom.z < -5 then
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
  for angle = angleResolution, detectAngle, angleResolution do
    local q = Quaternion.Euler(0, angle * direction, 0) * (me.fvec * me.isForwarding)
    local scoreAtAngle = 0
    for distance = distanceResolution, AVOID_TERRAIN_DISTANCE, distanceResolution do
      local point = me.selfPos + (q * distance)
      local terrain = I:GetTerrainAltitudeForPosition(point)
  
      if AVOID_TERRAIN_ALT < terrain then
        scoreAtAngle = scoreAtAngle + (AVOID_TERRAIN_DISTANCE - distance)
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
        avoidJudge = AvoidTarget(I, me, info.ReferencePosition, AVOID_FRIEND_ANGLE, AVOID_FRIEND_DISTANCE)
        ret.friend = true
        I:Log("detect friend1 ".. avoidJudge.steer)
      end
    else
      avoidJudge = AvoidTarget(I, me, info.ReferencePosition, AVOID_FRIEND_ANGLE, AVOID_FRIEND_DISTANCE)
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
  for distance = distanceResolution, AVOID_TERRAIN_DISTANCE, distanceResolution do
    local terrain = I:GetTerrainAltitudeForPosition(me.selfPos + (me.fvec * distance * request.needForward))
    if AVOID_TERRAIN_ALT < terrain then
      ResetSwitch()
      ret = DetectTerrainRange(I, 180, 10)
      ret.distance = distance
      ret.distanceRate = (distance / AVOID_TERRAIN_DISTANCE)
      I:Log("detect terrain")
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
  -- if UPSIDE_DOWN == 1 then
  --   needRoll = 180
  -- end

  local needPitch = 0


  ret.forwardDrive = forwardDrive
  ret.steer = 0
  I:Log("needRoll"..needRoll..", forwardDrive"..forwardDrive)

  --ret.hydrofoilBooster = 0
  --local p = me.pitch
  --local a = Angle(me.vvec, me.fvec)

  if followTarget ~= nil and followTarget.Info ~= nil and avoid.friend ~= true then
    -- 修理対象の真後ろ50mを目指す
    local vectorToTarget = (me.selfPos - followTarget.Info.ReferencePosition)
    local nosePos = me.selfPos + (me.fvec * I:GetConstructMaxDimensions().z)
    local destination = followTarget.Info.ReferencePosition - (followTarget.Info.ForwardVector * (REPAIR_DISTANCE + followTarget.Info.NegativeSize.z))
    local vectorToDest = (destination - nosePos)
    local side = Vector3.Dot(vectorToDest.normalized, me.rvec)
    local angle = Angle(vectorToDest, me.fvec)
    local front = Vector3.Dot(vectorToDest.normalized, me.fvec)
    local s = ((180 - angle) ^ 3) / (180 ^ 3)
    if vectorToDest.magnitude > 300 then
      f = s
    else
      f = vectorToDest.magnitude / 300 * s
    end
    if front > 0 or (front < 0 and vectorToDest.magnitude > 300) then
      if side > 0 and angle > 5 then
        ret.steer = 1
      elseif side < 0 and angle > 5 then
        ret.steer = -1
      end
    elseif front < 0 and vectorToDest.magnitude < 300 then
      if side > 0 and angle > 5 then
        ret.steer = 1
      elseif side < 0 and angle > 5 then
        ret.steer = -1
      end
      me.isForwarding = -1
    elseif front < 0 and vectorToDest.magnitude > MAX_REPAIR_DISTANCE then
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
    I:Log("MOVE TO REPAIR! req:a=".. ret.again..", r=".. ret.rgain.. ", pgain=".. ret.pgain.. ", forwardDrive=".. forwardDrive)
  end



  if tpi ~= nil then
    local targetPosition = tpi.Position
    local vectorToTarget = (targetPosition - me.selfPos)
    local side = Vector3.Dot(vectorToTarget.normalized, me.rvec)
    local angle = Angle(vectorToTarget, me.fvec)
    local targetDistance = vectorToTarget.magnitude

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

  if ret.forwardDrive > 0 then
    ret.needForward = 1
  elseif ret.forwardDrive < 0 then
    ret.needForward = -1
  else
    ret.needForward = 0
  end

  -- 回避最優先
  I:Log("forwardDrive"..forwardDrive)
  local avoid = AvoidCollision(I, me, followTarget, ret)
  I:Log("AvoidCollision"..avoid.steer)
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
  ret.again = Pid:GetGain(I, "ALT", apid, CRUISE_ALT, me.selfPos.y)
  ret.ygain = Pid:GetGain(I, "YAW", ypid, ret.steer, lasyYaw)
  lasyYaw = ret.ygain

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
function ViewHelth(I)
  local i=1
  while (I.Fleet.Members[i].CenterOfMass ~= me.com) do
   i=i+1
  end
  
  local myname = I:GetFriendlyInfoById(I.Fleet.Members[i].Id).BlueprintName
  local myhp = math.floor(I:GetHealthFraction()*100)
  local myAmmo = math.floor(I:GetAmmoFraction()*100)
  local myFuel = math.floor(I:GetFuelFraction()*100)
  
  if I.Fleet.Name == myname then
    I:LogToHud(myname .." HP:"..myhp.."%" .." Ammo:"..myAmmo.."%" .." Fuel:"..myFuel.."%")
  else
    I:LogToHud(I.Fleet.Name .. " " .. myname .." HP:"..myhp.."%" .." Ammo:"..myAmmo.."%" .." Fuel:"..myFuel.."%")
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
    ControlMissiles(I, ti)
  end

  ViewHelth(I)
end
