from cereal import car
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfa_mfa, create_scc12, create_mdps12
from selfdrive.car.hyundai.values import Buttons, SteerLimitParams, CAR
from opendbc.can.packer import CANPacker

import common.log as trace1

VisualAlert = car.CarControl.HUDControl.VisualAlert


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0    
    self.car_fingerprint = CP.carFingerprint
    self.packer = CANPacker(dbc_name)
    self.steer_rate_limited = False
    self.resume_cnt = 0
    self.last_resume_frame = 0
    self.last_lead_distance = 0

    self.turning_signal_timer = 0


  def process_hud_alert(self, enabled, visual_alert, left_lane, right_lane,  button_on):
    #sys_warning = (visual_alert == VisualAlert.steerRequired)
    sys_warning = 0
    if visual_alert == VisualAlert.steerRequired:
      sys_warning = 3

    # initialize to no line visible
    sys_state = 1
    if not button_on:
      lane_visible = 0
    if left_lane and right_lane:  #HUD alert only display when LKAS status is active
      if enabled:
        sys_state = 3
      else:
        sys_state = 4
    elif left_lane:
      sys_state = 5
    elif right_lane:
      sys_state = 6

    return sys_warning, sys_state   

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert,
             left_lane, right_lane, left_lane_depart, right_lane_depart):
    # Steering Torque
    new_steer = actuators.steer * SteerLimitParams.STEER_MAX
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, SteerLimitParams)
    self.steer_rate_limited = new_steer != apply_steer

    # disable if steer angle reach 90 deg, otherwise mdps fault in some models
    lkas_active = enabled and abs(CS.out.steeringAngle) < 90. and CS.out.lkas_button_on

    # fix for Genesis hard fault at low speed
    if CS.out.vEgo < 16.7 and self.car_fingerprint == CAR.HYUNDAI_GENESIS:
      lkas_active = 0

    # Disable steering while turning blinker on and speed below 60 kph
    if CS.out.leftBlinker or CS.out.rightBlinker:
      if self.car_fingerprint not in [CAR.KIA_OPTIMA, CAR.KIA_OPTIMA_H]:
        self.turning_signal_timer = 100  # Disable for 1.0 Seconds after blinker turned off
      elif CS.left_blinker_flash or CS.right_blinker_flash: # Optima has blinker flash signal only
        self.turning_signal_timer = 100
    if self.turning_signal_timer and CS.out.vEgo < 16.7:
      lkas_active = 0

    if self.turning_signal_timer:
      self.turning_signal_timer -= 1
      
    if not lkas_active:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    sys_warning, sys_state = self.process_hud_alert( lkas_active, visual_alert,  left_lane, right_lane, CS.out.lkas_button_on)



    can_sends = []
    can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled, 
                                   left_lane, right_lane))

    if pcm_cancel_cmd:
      can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.CANCEL))

    elif CS.out.cruiseState.standstill:
      # run only first time when the car stopped
      if self.last_lead_distance == 0:
        # get the lead distance from the Radar
        self.last_lead_distance = CS.lead_distance
        self.resume_cnt = 0
      # when lead car starts moving, create 6 RES msgs
      elif CS.lead_distance != self.last_lead_distance and (frame - self.last_resume_frame) > 5:
        can_sends.append(create_clu11(self.packer, frame, CS.clu11, Buttons.RES_ACCEL))
        self.resume_cnt += 1
        # interval after 6 msgs
        if self.resume_cnt > 5:
          self.last_resume_frame = frame
          self.clu11_cnt = 0
    # reset lead distnce after the car starts moving
    elif self.last_lead_distance != 0:
      self.last_lead_distance = 0  


    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.car_fingerprint in [CAR.SONATA, CAR.PALISADE]:
      can_sends.append(create_lfa_mfa(self.packer, frame, enabled))

    str_log1 = 'torg:{:5.0f} SA={:.1f} brak={} lkas={}'.format( apply_steer, CS.out.steeringAngle, CS.out.brakePressed, CS.lkas_button_on )  #.brakeLights )
    ##str_log2 = 'steer={:5.0f} sw{:.0f}'.format( CS.steer_torque_driver, CS.clu_CruiseSwState  )
    trace1.printf( '{} '.format( str_log1 ) )

    return can_sends
