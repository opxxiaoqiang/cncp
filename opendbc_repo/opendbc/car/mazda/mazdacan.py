from opendbc.car.mazda.values import Buttons, MazdaFlags
from opendbc.car.common.conversions import Conversions as CV


def create_steering_control(packer, CP, frame, apply_steer, lkas):

  tmp = apply_steer + 2048

  lo = tmp & 0xFF
  hi = tmp >> 8

  # copy values from camera
  b1 = int(lkas["BIT_1"])
  er1 = int(lkas["ERR_BIT_1"])
  lnv = 0
  ldw = 0
  er2 = int(lkas["ERR_BIT_2"])

  # Some older models do have these, newer models don't.
  # Either way, they all work just fine if set to zero.
  steering_angle = 0
  b2 = 0

  tmp = steering_angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | (( amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2

  ctr = frame % 16
  # bytes:     [    1  ] [ 2 ] [             3               ]  [           4         ]
  csum = 249 - ctr - hi - lo - (lnv << 3) - er1 - (ldw << 7) - ( er2 << 4) - (b1 << 5)

  # bytes      [ 5 ] [ 6 ] [    7   ]
  csum = csum - ahi - amd - alo - b2

  if ahi == 1:
    csum = csum + 15

  if csum < 0:
    if csum < -256:
      csum = csum + 512
    else:
      csum = csum + 256

  csum = csum % 256

  values = {}
  if CP.flags & MazdaFlags.GEN1:
    values = {
      "LKAS_REQUEST": apply_steer,
      "CTR": ctr,
      "ERR_BIT_1": er1,
      "LINE_NOT_VISIBLE" : lnv,
      "LDW": ldw,
      "BIT_1": b1,
      "ERR_BIT_2": er2,
      "STEERING_ANGLE": steering_angle,
      "ANGLE_ENABLED": b2,
      "CHKSUM": csum
    }

  return packer.make_can_msg("CAM_LKAS", 0, values)


def create_alert_command(packer, cam_msg: dict, ldw: bool, steer_required: bool):
  values = {s: cam_msg[s] for s in [
    "LINE_VISIBLE",
    "LINE_NOT_VISIBLE",
    "LANE_LINES",
    "BIT1",
    "BIT2",
    "BIT3",
    "NO_ERR_BIT",
    "S1",
    "S1_HBEAM",
  ]}
  values.update({
    # TODO: what's the difference between all these? do we need to send all?
    "HANDS_WARN_3_BITS": 0b111 if steer_required else 0,
    "HANDS_ON_STEER_WARN": steer_required,
    "HANDS_ON_STEER_WARN_2": steer_required,

    # TODO: right lane works, left doesn't
    # TODO: need to do something about L/R
    "LDW_WARN_LL": 0,
    "LDW_WARN_RL": 0,
  })
  return packer.make_can_msg("CAM_LANEINFO", 0, values)


def create_button_cmd(packer, CP, counter, button):

  can = int(button == Buttons.CANCEL)
  res = int(button == Buttons.RESUME)
  inc = int(button == Buttons.SET_PLUS)
  dec = int(button == Buttons.SET_MINUS)

  if CP.flags & MazdaFlags.GEN1:
    values = {
      "CAN_OFF": can,
      "CAN_OFF_INV": (can + 1) % 2,

      "SET_P": inc,
      "SET_P_INV": (inc + 1) % 2,

      "RES": res,
      "RES_INV": (res + 1) % 2,

      "SET_M": dec,
      "SET_M_INV": (dec + 1) % 2,

      "DISTANCE_LESS": 0,
      "DISTANCE_LESS_INV": 1,

      "DISTANCE_MORE": 0,
      "DISTANCE_MORE_INV": 1,

      "MODE_X": 0,
      "MODE_X_INV": 1,

      "MODE_Y": 0,
      "MODE_Y_INV": 1,

      "BIT1": 1,
      "BIT2": 1,
      "BIT3": 1,
      "CTR": (counter + 1) % 16,
    }

    return packer.make_can_msg("CRZ_BTNS", 0, values)

def create_mazda_acc_spam_command(packer, controller, CS, slcSet, Vego, is_metric, accel):
  """
  通过模拟巡航控制按钮按压来控制车速。
  这个函数基于目标速度和当前设定速度的差异来决定是生成增加速度按钮还是减少速度按钮。

  Args:
    packer: CAN消息打包器
    controller: 车辆控制器实例
    CS: 车辆状态对象
    slcSet: 目标速度(m/s)
    Vego: 当前车速(m/s)
    is_metric: 是否使用公制单位
    accel: 当前加速度，用于特定场景的计算

  Returns:
    包含CAN消息的列表，用于模拟按钮按压
  """
  cruiseBtn = Buttons.NONE

  # 检查巡航控制是否启用，如果未启用则不进行操作
  if not CS.out.cruiseState.enabled:
    return []

  MS_CONVERT = CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH

  # 获取当前巡航设定速度和目标速度（转换单位）
  speedSetPoint = int(round(CS.out.cruiseState.speed * MS_CONVERT))
  slcSet = int(round(slcSet * MS_CONVERT))

  # 针对减速场景的特殊处理：如果目标速度远低于当前速度，调整目标速度以增加减速度
  if slcSet + 5 < Vego * MS_CONVERT:
    slcSet = slcSet - 10  # 降低10以增加减速度，直到差距小于5

  # 根据单位系统对速度值进行取整
  if is_metric:  # 公制单位，按5 km/h取整
    slcSet = int(round(slcSet/5.0)*5.0)
    speedSetPoint = int(round(speedSetPoint/5.0)*5.0)
  else:  # 英制单位，按1 mph取整
    slcSet = int(round(slcSet))
    speedSetPoint = int(round(speedSetPoint))

  # 设置速度上下限 (根据马自达车辆规格)
  min_speed = 30 if is_metric else 20  # 最低30公里/小时或20英里/小时
  max_speed = 160 if is_metric else 100  # 最高160公里/小时或100英里/小时

  if slcSet < min_speed:
    slcSet = min_speed
  elif slcSet > max_speed:
    slcSet = max_speed

  # 根据目标速度和当前设定速度决定按钮动作
  # 只有当当前设定速度在有效范围内才进行调整
  if slcSet < speedSetPoint and speedSetPoint >= min_speed:
    cruiseBtn = Buttons.SET_MINUS  # 需要减速
  elif slcSet > speedSetPoint and speedSetPoint < max_speed:
    cruiseBtn = Buttons.SET_PLUS   # 需要加速
  else:
    cruiseBtn = Buttons.NONE       # 速度匹配，无需调整

  # 如果需要按下按钮，生成相应的CAN消息
  if cruiseBtn != Buttons.NONE:
    return [create_button_cmd(packer, controller.CP, controller.frame // 10, cruiseBtn)]
  else:
    return []
