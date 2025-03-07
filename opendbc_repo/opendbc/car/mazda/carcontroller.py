from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_driver_steer_torque_limits, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import CarControllerParams, Buttons
from opendbc.car.common.conversions import Conversions as CV
from openpilot.common.params import Params

VisualAlert = structs.CarControl.HUDControl.VisualAlert
ButtonType = structs.CarState.ButtonEvent.Type


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_steer_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0

    self.activateCruise = 0
    self.speed_from_pcm = 1
    self.is_metric = False  # 默认为英制，会在update中更新

  def update(self, CC, CS, now_nanos):

    if self.frame % 50 == 0:
      params = Params()
      self.speed_from_pcm = params.get_int("SpeedFromPCM")
      # 尝试从CS获取is_metric属性或从CS.out推断
      if hasattr(CS, 'is_metric'):
        self.is_metric = CS.is_metric
      else:
        # 根据车速估计单位系统
        # 一般情况下，如果车速数值较大，可能使用公制单位
        self.is_metric = True

      # 从CS获取activateCruise状态（如果可用）
      if hasattr(CS, 'activateCruise'):
        self.activateCruise = CS.activateCruise
      elif hasattr(CS.out, 'activateCruise'):
        self.activateCruise = 1 if CS.out.activateCruise else 0

    can_sends = []

    apply_steer = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_steer = int(round(CC.actuators.steer * CarControllerParams.STEER_MAX))
      apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last,
                                                     CS.out.steeringTorque, CarControllerParams)

    if CC.cruiseControl.cancel:
      # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
      # a race condition with the stock system, where the second cancel from openpilot
      # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
      # read 3 messages and most likely sync state before we attempt cancel.
      self.brake_counter = self.brake_counter + 1
      if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
        # Cancel Stock ACC if it's enabled while OP is disengaged
        # Send at a rate of 10hz until we sync with stock ACC state
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.brake_counter = 0
      if CC.cruiseControl.resume and self.frame % 5 == 0:
        # Mazda Stop and Go requires a RES button (or gas) press if the car stops more than 3 seconds
        # Send Resume button when planner wants car to move
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))
      # 速度控制逻辑：根据speed_from_pcm选择不同的控制方式
      elif CC.enabled and not CS.out.gasPressed:  # 只有在启用巡航且未踩油门时才进行速度控制
        if self.speed_from_pcm != 1:
          # 使用CSLC风格的速度控制
          if self.frame % 10 == 0:  # 每10帧执行一次
            # 获取巡航和距离按钮状态
            cruise_buttons = Buttons.NONE
            distance_button = 0

            # 尝试从CS获取cruise_buttons
            if hasattr(CS, 'cruise_buttons'):
              cruise_buttons = CS.cruise_buttons
            else:
              # 尝试从buttonEvents推断按钮状态
              for btn in CS.out.buttonEvents:
                if btn.type in [ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.resumeCruise, ButtonType.cancel]:
                  if btn.type == ButtonType.accelCruise:
                    cruise_buttons = Buttons.SET_PLUS
                  elif btn.type == ButtonType.decelCruise:
                    cruise_buttons = Buttons.SET_MINUS
                  elif btn.type == ButtonType.resumeCruise:
                    cruise_buttons = Buttons.RESUME
                  elif btn.type == ButtonType.cancel:
                    cruise_buttons = Buttons.CANCEL
                  break

            # 尝试从CS获取distance_button
            if hasattr(CS, 'distance_button'):
              distance_button = CS.distance_button

            # 只有在未检测到巡航按钮按下、未按距离按钮时，才进行速度控制
            if cruise_buttons == Buttons.NONE and not distance_button:
              hud_v_cruise = CC.hudControl.setSpeed
              # 设置最大速度限制
              if hud_v_cruise > 35:  # 最大35m/s约等于126km/h或78mph
                hud_v_cruise = 35

              # 获取加速度值
              accel = 0
              if hasattr(CC.actuators, 'accel'):
                accel = CC.actuators.accel

              # 使用新添加的函数控制车速
              speed_cmds = mazdacan.create_mazda_acc_spam_command(self.packer, self, CS, hud_v_cruise, CS.out.vEgo, self.is_metric, accel)
              if speed_cmds:  # 确保返回的命令列表不为空
                can_sends.extend(speed_cmds)
        else:
          # 使用原始的按钮控制方法
          if self.frame % 20 == 0:  # 每20帧执行一次
            spam_button = self.make_spam_button(CC, CS)
            if spam_button > 0:
              self.brake_counter = 0
              can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, self.frame // 10, spam_button))

    self.apply_steer_last = apply_steer

    # send HUD alerts
    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      # TODO: find a way to silence audible warnings so we can add more hud alerts
      steer_required = steer_required and CS.lkas_allowed_speed
      can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_steer, CS.cam_lkas))

    new_actuators = CC.actuators.as_builder()
    new_actuators.steer = apply_steer / CarControllerParams.STEER_MAX
    new_actuators.steerOutputCan = apply_steer

    self.frame += 1
    return new_actuators, can_sends

  def make_spam_button(self, CC, CS):
    """
    根据目标速度和当前车速决定按下哪个巡航按钮。
    与CSLC不同，此方法更简单，主要用于激活巡航和基本的调速。

    Args:
      CC: 车辆控制对象
      CS: 车辆状态对象

    Returns:
      按钮代码(Buttons枚举值)
    """
    # 获取UI设置的目标速度
    hud_control = CC.hudControl

    # 将m/s转换为km/h或mph
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH)
    # 四舍五入到整数
    target = int(set_speed_in_units+0.5)
    # 取整到最接近的5单位(仅公制)
    if self.is_metric:
      target = int(round(target / 5.0) * 5.0)

    # 获取当前巡航设定速度
    current = int(CS.out.cruiseState.speed * (CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH) + 0.5)
    if self.is_metric:
      current = int(round(current / 5.0) * 5.0)

    # 当前车速(公里/小时)
    v_ego_kph = CS.out.vEgo * CV.MS_TO_KPH

    # 检查是否可以激活巡航(没有踩刹车或油门)
    cant_activate = CS.out.brakePressed or CS.out.gasPressed

    # 处理不同场景
    if CC.enabled:
      # 如果巡航未激活但应该激活
      if not CS.out.cruiseState.enabled:
        if (hud_control.leadVisible or v_ego_kph > 10.0) and self.activateCruise == 0 and not cant_activate:
          self.activateCruise = 1
          print("RESUME - 激活巡航控制")
          return Buttons.RESUME
      # 如果巡航已激活且需要恢复
      elif CC.cruiseControl.resume:
        return Buttons.RESUME
      # 如果需要减速且不是使用PCM控制速度
      elif target < current and current >= (30 if self.is_metric else 20) and self.speed_from_pcm != 1:
        print(f"SET_MINUS target={target}, current={current}")
        return Buttons.SET_MINUS
      # 如果需要加速且不是使用PCM控制速度
      elif target > current and current < (160 if self.is_metric else 100) and self.speed_from_pcm != 1:
        print(f"SET_PLUS target={target}, current={current}")
        return Buttons.SET_PLUS
    # 如果需要激活巡航
    elif CS.out.activateCruise:
      if (hud_control.leadVisible or v_ego_kph > 10.0) and self.activateCruise == 0 and not cant_activate:
        self.activateCruise = 1
        print("RESUME - 激活巡航控制")
        return Buttons.RESUME

    # 默认情况下不发送按钮
    return 0
