from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from selfdrive.config import Conversions as CV
from cereal import car
from cereal import log
from selfdrive.kegman_conf import kegman_conf


class LatControlPID():
  def __init__(self, CP, CI):
    self.kegman = kegman_conf(CP)
    self.CI = CI
    self.deadzone = float(self.kegman.conf['deadzone'])
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                            (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                            (CP.lateralTuning.pid.kdBP, CP.lateralTuning.pid.kdV),
                            k_f=CP.lateralTuning.pid.kf, pos_limit=1.0, neg_limit=-1.0,
                            sat_limit=CP.steerLimitTimer, derivative_period=0.1)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()
    self.angle_steers_des = 0.
    self.mpc_frame = 0

  def reset(self):
    self.pid.reset()
    
  def live_tune(self, CP):
    if self.get_steer_feedforward == self.CI.get_steer_feedforward_default:
      self.mpc_frame += 1
      if self.mpc_frame % 300 == 0:
        # live tuning through /data/openpilot/tune.py overrides interface.py settings
        self.kegman = kegman_conf()
        if self.kegman.conf['tuneGernby'] == "1":
          self.steerKpV = [float(self.kegman.conf['Kp'])]
          self.steerKiV = [float(self.kegman.conf['Ki'])]
          self.steerKdV = [float(self.kegman.conf['Kd'])]
            # custom feedforward values are not allowed to be tuned.
          self.steerKf = float(self.kegman.conf['Kf'])
          self.pid = PIDController((CP.lateralTuning.pid.kpBP, self.steerKpV),
                                  (CP.lateralTuning.pid.kiBP, self.steerKiV),
                                  (CP.lateralTuning.pid.kdBP, self.steerKdV),
                                  k_f=self.steerKf, pos_limit=self.pid.pos_limit, neg_limit=self.pid.neg_limit,
                                  sat_limit=CP.steerLimitTimer, derivative_period=0.1)
          self.deadzone = float(self.kegman.conf['deadzone'])
        
        self.mpc_frame = 0    


  def update(self, active, CS, CP, lat_plan):
    self.live_tune(CP)
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)

    if CS.vEgo < 0.3 or not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      self.angle_steers_des = lat_plan.steeringAngleDeg # get from MPC/LateralPlanner

      steers_max = get_steer_max(CP, CS.vEgo)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      
      steer_feedforward = self.get_steer_feedforward(self.angle_steers_des, CS.vEgo)
      
      # torque for steer rate. ~0 angle, steer rate ~= steer command.
      steer_rate_actual = CS.steeringRateDeg
      steer_rate_desired = lat_plan.lat_plan.steeringAngleDeg
      speed_mph =  CS.vEgo * CV.MS_TO_MPH
      steer_rate_max = 0.0389837 * speed_mph**2 - 5.34858 * speed_mph + 223.831

      steer_feedforward += ((steer_rate_desired - steer_rate_actual) / steer_rate_max)
      
      deadzone = self.deadzone    
        
      check_saturation = (CS.vEgo > 10) and not CS.steeringRateLimited and not CS.steeringPressed
      output_steer = self.pid.update(self.angle_steers_des, CS.steeringAngleDeg, check_saturation=check_saturation, override=CS.steeringPressed,
                                     feedforward=steer_feedforward, speed=CS.vEgo, deadzone=deadzone)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = bool(self.pid.saturated)

    return output_steer, float(self.angle_steers_des), pid_log
