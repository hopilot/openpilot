import math

import numpy as np
from common.realtime import sec_since_boot, DT_MDL
from common.numpy_fast import interp
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.lateral_mpc_lib.lat_mpc import LateralMpc
from selfdrive.controls.lib.drive_helpers import CONTROL_N, LAT_MPC_N, MIN_SPEED
from selfdrive.controls.lib.lane_planner import LanePlanner
from selfdrive.controls.lib.desire_helper import DesireHelper
import cereal.messaging as messaging
from cereal import log

from common.conversions import Conversions as CV
from common.params import Params
from decimal import Decimal

TRAJECTORY_SIZE = 33

PATH_COST = 1.0
LATERAL_MOTION_COST = 0.11
LATERAL_ACCEL_COST = 0.0
LATERAL_JERK_COST = 0.04
# Extreme steering rate is unpleasant, even
# when it does not cause bad jerk.
# TODO this cost should be lowered when low
# speed lateral control is stable on all cars
STEERING_RATE_COST = 700.0

LaneChangeState = log.LateralPlan.LaneChangeState

class LateralPlanner:
  def __init__(self, CP, use_lanelines=True, wide_camera=False):
    self.use_lanelines = use_lanelines
    self.LP = LanePlanner(wide_camera)
    self.DH = DesireHelper(CP)

    # Vehicle model parameters used to calculate lateral movement of car
    self.factor1 = CP.wheelbase - CP.centerToFront
    self.factor2 = (CP.centerToFront * CP.mass) / (CP.wheelbase * CP.tireStiffnessRear)
    self.last_cloudlog_t = 0
    self.solution_invalid_cnt = 0

    self.path_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.velocity_xyz = np.zeros((TRAJECTORY_SIZE, 3))
    self.path_xyz_stds = np.ones((TRAJECTORY_SIZE, 3))
    self.plan_yaw = np.zeros((TRAJECTORY_SIZE,))
    self.plan_yaw_rate = np.zeros((TRAJECTORY_SIZE,))
    self.t_idxs = np.arange(TRAJECTORY_SIZE)
    self.y_pts = np.zeros(TRAJECTORY_SIZE)

    self.lat_mpc = LateralMpc()
    self.reset_mpc(np.zeros(4))

    self.laneless_mode = int(Params().get("LanelessMode", encoding="utf8"))
    self.laneless_mode_status = False
    self.laneless_mode_status_buffer = False

    self.standstill_elapsed_time = 0.0
    self.v_cruise_kph = 0
    self.stand_still = False
    
    self.second = 0.0
    self.model_speed = 255.0

  def curve_speed(self, sm, v_ego):
    md = sm['modelV2']
    curvature = sm['controlsState'].curvature
    if md is not None and len(md.position.x) == TRAJECTORY_SIZE and len(md.position.y) == TRAJECTORY_SIZE:
      x = md.position.x
      y = md.position.y
      dy = np.gradient(y, x)
      d2y = np.gradient(dy, x)
      curv = d2y / (1 + dy ** 2) ** 1.5
      start = int(interp(v_ego, [10., 27.], [10, TRAJECTORY_SIZE-10])) # neokii's factor
      if abs(curvature) > 0.0008: # opkr
        curv = curv[5:TRAJECTORY_SIZE-10]
      else:
        curv = curv[start:min(start+10, TRAJECTORY_SIZE)]
      a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
      v_curvature = np.sqrt(a_y_max / np.clip(np.abs(curv), 1e-4, None))
      model_speed = np.mean(v_curvature) * 0.9
      curve_speed = float(max(model_speed, 30 * CV.KPH_TO_MS))
      if np.isnan(curve_speed):
          curve_speed = 255
    else:
      curve_speed = 255
    return min(255, curve_speed * CV.MS_TO_KPH)

  def reset_mpc(self, x0=np.zeros(4)):
    self.x0 = x0
    self.lat_mpc.reset(x0=self.x0)

  def update(self, sm, CP):
    self.second += DT_MDL
    if self.second > 1.0:
      self.use_lanelines = not Params().get_bool("EndToEndToggle")
      self.laneless_mode = int(Params().get("LanelessMode", encoding="utf8"))
      self.second = 0.0

    self.v_cruise_kph = sm['controlsState'].vCruise
    self.stand_still = sm['carState'].standStill
    self.v_ego = max(MIN_SPEED, sm['carState'].vEgo)
    v_ego = max(MIN_SPEED, sm['carState'].vEgo)
    if sm.frame % 5 == 0:
      self.model_speed = self.curve_speed(sm, v_ego)
    measured_curvature = sm['controlsState'].curvature

    # Parse model predictions
    md = sm['modelV2']
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.orientation.x) == TRAJECTORY_SIZE:
      self.path_xyz = np.column_stack([md.position.x, md.position.y, md.position.z])
      self.t_idxs = np.array(md.position.t)
      self.plan_yaw = np.array(md.orientation.z)
      self.plan_yaw_rate = np.array(md.orientationRate.z)
      self.velocity_xyz = np.column_stack([md.velocity.x, md.velocity.y, md.velocity.z])
      car_speed = np.linalg.norm(self.velocity_xyz, axis=1)
      self.v_plan = np.clip(car_speed, MIN_SPEED, np.inf)
      self.v_ego = self.v_plan[0]
    if len(md.position.xStd) == TRAJECTORY_SIZE:
      self.path_xyz_stds = np.column_stack([md.position.xStd, md.position.yStd, md.position.zStd])

    # Lane change logic
    lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
    self.DH.update(CP, sm['carState'], sm['controlsState'], lane_change_prob, md)

    # Turn off lanes during lane change
    if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
      self.LP.lll_prob *= self.DH.lane_change_ll_prob
      self.LP.rll_prob *= self.DH.lane_change_ll_prob

    # Calculate final driving path and set MPC costs
    if self.use_lanelines:
      self.LP.parse_model(md, sm, self.v_ego)
      self.path_xyz = self.LP.get_d_path(self.v_ego, self.t_idxs, self.path_xyz)
      self.laneless_mode_status = False
    elif self.laneless_mode == 0:
      self.LP.parse_model(md, sm, self.v_ego)
      self.path_xyz = self.LP.get_d_path(self.v_ego, self.t_idxs, self.path_xyz)
      self.laneless_mode_status = False
    elif self.laneless_mode == 1:
      self.laneless_mode_status = True
      pass #d_path_xyz = self.path_xyz
    elif self.laneless_mode == 2 and ((self.LP.lll_prob + self.LP.rll_prob)/2 < 0.3) and self.DH.lane_change_state == LaneChangeState.off:
      self.laneless_mode_status = True
      self.laneless_mode_status_buffer = True
      pass #d_path_xyz = self.path_xyz
    elif self.laneless_mode == 2 and ((self.LP.lll_prob + self.LP.rll_prob)/2 > 0.5) and \
      self.laneless_mode_status_buffer and self.DH.lane_change_state == LaneChangeState.off:
      self.LP.parse_model(md, sm, self.v_ego)
      self.path_xyz = self.LP.get_d_path(self.v_ego, self.t_idxs, self.path_xyz)
      self.laneless_mode_status = False
      self.laneless_mode_status_buffer = False
    elif self.laneless_mode == 2 and self.laneless_mode_status_buffer == True and self.DH.lane_change_state == LaneChangeState.off:
      self.laneless_mode_status = True
      pass #d_path_xyz = self.path_xyz
    else:
      self.LP.parse_model(md, sm, self.v_ego)
      self.path_xyz = self.LP.get_d_path(self.v_ego, self.t_idxs, self.path_xyz)
      self.laneless_mode_status = False
      self.laneless_mode_status_buffer = False

    self.path_xyz[:, 1] += self.pathOffset
    self.lat_mpc.set_weights(PATH_COST, LATERAL_MOTION_COST,
                             LATERAL_ACCEL_COST, LATERAL_JERK_COST,
                             interp(self.v_ego, [2., 10.], [STEERING_RATE_COST, STEERING_RATE_COST/2.]))
    y_pts = self.path_xyz[:LAT_MPC_N+1, 1]
    heading_pts = self.plan_yaw[:LAT_MPC_N+1]
    yaw_rate_pts = self.plan_yaw_rate[:LAT_MPC_N+1]
    self.y_pts = y_pts

    assert len(y_pts) == LAT_MPC_N + 1
    assert len(heading_pts) == LAT_MPC_N + 1
    assert len(yaw_rate_pts) == LAT_MPC_N + 1
    lateral_factor = np.clip(self.factor1 - (self.factor2 * self.v_plan**2), 0.0, np.inf)
    p = np.column_stack([self.v_plan, lateral_factor])
    self.lat_mpc.run(self.x0,
                     p,
                     y_pts,
                     heading_pts,
                     yaw_rate_pts)
    # init state for next iteration
    # mpc.u_sol is the desired second derivative of psi given x0 curv state.
    # with x0[3] = measured_yaw_rate, this would be the actual desired yaw rate.
    # instead, interpolate x_sol so that x0[3] is the desired yaw rate for lat_control.
    self.x0[3] = interp(DT_MDL, self.t_idxs[:LAT_MPC_N + 1], self.lat_mpc.x_sol[:, 3])

    #  Check for infeasible MPC solution
    mpc_nans = np.isnan(self.lat_mpc.x_sol[:, 3]).any()
    t = sec_since_boot()
    if mpc_nans or self.lat_mpc.solution_status != 0:
      self.reset_mpc()
      self.x0[3] = measured_curvature * self.v_ego
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning("Lateral mpc - nan: True")

    if self.lat_mpc.cost > 20000. or mpc_nans:
      self.solution_invalid_cnt += 1
    else:
      self.solution_invalid_cnt = 0

  def publish(self, sm, pm):
    plan_solution_valid = self.solution_invalid_cnt < 2
    plan_send = messaging.new_message('lateralPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])

    lateralPlan = plan_send.lateralPlan
    lateralPlan.modelMonoTime = sm.logMonoTime['modelV2']
    lateralPlan.laneWidth = float(self.LP.lane_width)
    lateralPlan.dPathPoints = self.y_pts.tolist()
    lateralPlan.psis = self.lat_mpc.x_sol[0:CONTROL_N, 2].tolist()
    lateralPlan.curvatures = (self.lat_mpc.x_sol[0:CONTROL_N, 3]/self.v_ego).tolist()
    lateralPlan.curvatureRates = [float(x/self.v_ego) for x in self.lat_mpc.u_sol[0:CONTROL_N - 1]] + [0.0]
    lateralPlan.lProb = float(self.LP.lll_prob)
    lateralPlan.rProb = float(self.LP.rll_prob)
    lateralPlan.dProb = float(self.LP.d_prob)

    lateralPlan.mpcSolutionValid = bool(plan_solution_valid)
    lateralPlan.solverExecutionTime = self.lat_mpc.solve_time

    lateralPlan.desire = self.DH.desire
    lateralPlan.useLaneLines = self.use_lanelines
    lateralPlan.laneChangeState = self.DH.lane_change_state
    lateralPlan.laneChangeDirection = self.DH.lane_change_direction

    lateralPlan.modelSpeed = float(self.model_speed)
    lateralPlan.outputScale = float(self.DH.output_scale)
    lateralPlan.vCruiseSet = float(self.v_cruise_kph)
    lateralPlan.vCurvature = float(sm['controlsState'].curvature)
    lateralPlan.lanelessMode = bool(self.laneless_mode_status)
    lateralPlan.totalCameraOffset = float(self.LP.total_camera_offset)

    if self.stand_still:
      self.standstill_elapsed_time += DT_MDL
    else:
      self.standstill_elapsed_time = 0.0
    lateralPlan.standstillElapsedTime = int(self.standstill_elapsed_time)

    pm.send('lateralPlan', plan_send)
