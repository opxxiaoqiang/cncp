#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda.values import CAR, LKAS_LIMITS
from opendbc.car.interfaces import CarInterfaceBase



class CarInterface(CarInterfaceBase):

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda)]
    ret.radarUnavailable = True

    ret.dashcamOnly = candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021)

    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate not in (CAR.MAZDA_CX5_2022,):
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    ret.centerToFront = ret.wheelbase * 0.41

    # 检查SpeedFromPCM参数是否已设置
    try:
      from openpilot.common.params import Params
      params = Params()
      speed_from_pcm = params.get_int("SpeedFromPCM")

      print(f"MAZDA_INTERFACE: SpeedFromPCM = {speed_from_pcm}")

      # 如果SpeedFromPCM不等于1，启用纵向控制参数
      if speed_from_pcm != 1:
        print("MAZDA_INTERFACE: Enabling longitudinal control parameters for CSLC mode")
        ret.openpilotLongitudinalControl = True

        # 配置纵向控制参数
        # 这些参数针对马自达车型进行了调整，以实现平稳的加减速
        ret.longitudinalTuning.deadzoneBP = [0.]
        ret.longitudinalTuning.deadzoneV = [0.9]  # 允许2mph的速度误差
        ret.stoppingDecelRate = 4.5  # 10mph/s的减速度
        ret.longitudinalActuatorDelayLowerBound = 1.
        ret.longitudinalActuatorDelayUpperBound = 2.

        # 配置PID控制参数
        ret.longitudinalTuning.kpBP = [8.94, 7.2, 28.]  # 8.94 m/s == 20 mph
        ret.longitudinalTuning.kpV = [0., 4., 2.]  # 低速时设为0，因为无法在该速度以下驾驶
        ret.longitudinalTuning.kiBP = [0.]
        ret.longitudinalTuning.kiV = [0.1]

        print("MAZDA_INTERFACE: Longitudinal control parameters set successfully")
      else:
        print("MAZDA_INTERFACE: Using standard control mode (SpeedFromPCM=1)")
    except Exception as e:
      # 如果出现任何错误，记录错误并保持默认设置
      print(f"MAZDA_INTERFACE: Error setting longitudinal control params: {e}")
      pass  # 保持默认设置

    return ret
