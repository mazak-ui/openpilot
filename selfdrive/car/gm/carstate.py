from cereal import car
from common.numpy_fast import mean
from selfdrive.config import Conversions as CV
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.gm.values import DBC, CAR, AccState, CanBus, \
                                    CruiseButtons, STEER_THRESHOLD

from common.params import Params

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["ECMPRDNL"]["PRNDL"]
    self.adaptive_Cruise = False
    self.enable_lkas = True

    # scc smoother
    self.acc_mode = False
    self.cruise_gap = 1
    self.brake_pressed = False
    self.gas_pressed = False
    self.standstill = False
    self.cruiseState_enabled = False
    self.cruiseState_speed = 0

    self.use_cluster_speed = Params().get_bool('UseClusterSpeed')
    self.long_control_enabled = Params().get_bool('LongControlEnabled')



  def update(self, pt_cp):
    ret = car.CarState.new_message()
    ret.adaptiveCruise = self.adaptive_Cruise
    ret.lkasEnable = self.enable_lkas
    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = pt_cp.vl["ASCMSteeringButton"]["ACCButtons"]
    ret.wheelSpeeds.fl = pt_cp.vl["EBCMWheelSpdFront"]["FLWheelSpd"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = pt_cp.vl["EBCMWheelSpdFront"]["FRWheelSpd"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = pt_cp.vl["EBCMWheelSpdRear"]["RLWheelSpd"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = pt_cp.vl["EBCMWheelSpdRear"]["RRWheelSpd"] * CV.KPH_TO_MS
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.1
    self.vehicleSpeed = pt_cp.vl["ECMVehicleSpeed"]["VehicleSpeed"] * CV.MPH_TO_KPH
    ret.vehicleSpeed = self.vehicleSpeed
    ret.vEgo = pt_cp.vl["ECMVehicleSpeed"]["VehicleSpeed"] * CV.MPH_TO_MS

    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]["PRNDL"], None))
    ret.brake = pt_cp.vl["EBCMBrakePedalPosition"]["BrakePedalPosition"] / 0xd0
    # Brake pedal's potentiometer returns near-zero reading even when pedal is not pressed.
    if ret.brake < 10/0xd0:
      ret.brake = 0.

    ret.gas = pt_cp.vl["AcceleratorPedal"]["AcceleratorPedal"] / 254.
    ret.gasPressed = ret.gas > 1e-5

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelAngle"]
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]["SteeringWheelRate"]
    ret.steeringTorque = pt_cp.vl["PSCMStatus"]["LKADriverAppldTrq"]
    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]["LKATorqueDelivered"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # 0 inactive, 1 active, 2 temporarily limited, 3 failed
    self.lkas_status = pt_cp.vl["PSCMStatus"]["LKATorqueDeliveredStatus"]
    ret.steerWarning = self.lkas_status == 2
    ret.steerError = self.lkas_status == 3

    # 1 - open, 0 - closed
    ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]["FrontLeftDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["FrontRightDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["RearLeftDoor"] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]["RearRightDoor"] == 1)

    # 1 - latched
    ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBeltStatus"]["LeftSeatBelt"] == 0
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]["TurnSignals"] == 2

    self.park_brake = pt_cp.vl["EPBStatus"]['EPBClosed']
    self.main_on = bool(pt_cp.vl["ECMEngineStatus"]['CruiseMainOn'])
    ret.espDisabled = pt_cp.vl["ESPStatus"]['TractionControlOn'] != 1
    self.pcm_acc_status = pt_cp.vl["AcceleratorPedal2"]['CruiseState']
    ret.cruiseState.available = self.pcm_acc_status != 0
    ret.cruiseState.standstill = False

    ret.brakePressed = ret.brake > 1e-5
    ret.regenPressed = False
    if self.car_fingerprint == CAR.BOLT:
      ret.regenPressed = bool(pt_cp.vl["EBCMRegenPaddle"]['RegenPaddle'])
    brake_light_enable = False
    if self.car_fingerprint == CAR.BOLT:
      if ret.aEgo < -1.3:
        brake_light_enable = True
    ret.brakeLights = ret.brakePressed or ret.regenPressed or brake_light_enable

    ret.cruiseState.enabled = self.main_on or ret.adaptiveCruise

    # scc smoother
    # driver_override = cp.vl["TCS13"]["DriverOverride"]
    # self.acc_mode = cp_scc.vl["SCC12"]['ACCMode'] != 0
    # self.cruise_gap = cp_scc.vl["SCC11"]['TauGapSet'] if not self.no_radar else 1
    self.gas_pressed = ret.gasPressed #or driver_override == 1
    self.brake_pressed = ret.brakePressed #or driver_override == 2
    self.standstill = ret.standstill or ret.cruiseState.standstill
    self.cruiseState_enabled = ret.cruiseState.enabled
    self.cruiseState_speed = ret.cruiseState.speed
    ret.cruiseGap = self.cruise_gap
    return ret

  @staticmethod
  def get_can_parser(CP):
    # this function generates lists for signal, messages and initial values
    signals = [
      # sig_name, sig_address, default
      ("BrakePedalPosition", "EBCMBrakePedalPosition", 0),
      ("FrontLeftDoor", "BCMDoorBeltStatus", 0),
      ("FrontRightDoor", "BCMDoorBeltStatus", 0),
      ("RearLeftDoor", "BCMDoorBeltStatus", 0),
      ("RearRightDoor", "BCMDoorBeltStatus", 0),
      ("LeftSeatBelt", "BCMDoorBeltStatus", 0),
      ("RightSeatBelt", "BCMDoorBeltStatus", 0),
      ("TurnSignals", "BCMTurnSignals", 0),
      ("AcceleratorPedal", "AcceleratorPedal", 0),
      ("CruiseState", "AcceleratorPedal2", 0),
      ("ACCButtons", "ASCMSteeringButton", CruiseButtons.UNPRESS),
      ("SteeringWheelAngle", "PSCMSteeringAngle", 0),
      ("SteeringWheelRate", "PSCMSteeringAngle", 0),
      ("FLWheelSpd", "EBCMWheelSpdFront", 0),
      ("FRWheelSpd", "EBCMWheelSpdFront", 0),
      ("RLWheelSpd", "EBCMWheelSpdRear", 0),
      ("RRWheelSpd", "EBCMWheelSpdRear", 0),
      ("VehicleSpeed", "ECMVehicleSpeed", 0),
      ("PRNDL", "ECMPRDNL", 0),
      ("LKADriverAppldTrq", "PSCMStatus", 0),
      ("LKATorqueDelivered", "PSCMStatus", 0),
      ("LKATorqueDeliveredStatus", "PSCMStatus", 0),
      ("TractionControlOn", "ESPStatus", 0),
      ("EPBClosed", "EPBStatus", 0),
      ("CruiseMainOn", "ECMEngineStatus", 0),
      ("ACCCmdActive", "ASCMActiveCruiseControlStatus", 0),
      ("LKATotalTorqueDelivered", "PSCMStatus", 0),
    ]


    if CP.carFingerprint == CAR.BOLT:
      signals += [
        ("RegenPaddle", "EBCMRegenPaddle", 0),
      ]


    if CP.enableGasInterceptor:
      signals += [
        ("INTERCEPTOR_GAS", "GAS_SENSOR", 0),
        ("INTERCEPTOR_GAS2", "GAS_SENSOR", 0)
      ]


    return CANParser(DBC[CP.carFingerprint]['pt'], signals, [], CanBus.POWERTRAIN, enforce_checks=False)
