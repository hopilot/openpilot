#!/usr/bin/env python3
import numpy as np
from cereal import car
from selfdrive.modeld.constants import T_IDXS
from common.params import Params
from common.realtime import Priority, config_realtime_process
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.longitudinal_planner import Planner
from selfdrive.controls.lib.lateral_planner import LateralPlanner
from selfdrive.hardware import TICI
import cereal.messaging as messaging


def cumtrapz(x, t):
  return np.concatenate([[0], np.cumsum(((x[0:-1] + x[1:])/2) * np.diff(t))])

def publish_ui_plan(sm, pm, lateral_planner, longitudinal_planner):
  plan_odo = cumtrapz(longitudinal_planner.v_desired_trajectory_full, T_IDXS)
  model_odo = cumtrapz(lateral_planner.v_plan, T_IDXS)

  ui_send = messaging.new_message('uiPlan')
  ui_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'modelV2'])
  uiPlan = ui_send.uiPlan
  uiPlan.position.x = np.interp(plan_odo, model_odo, lateral_planner.lat_mpc.x_sol[:,0]).tolist()
  uiPlan.position.y = np.interp(plan_odo, model_odo, lateral_planner.lat_mpc.x_sol[:,1]).tolist()
  uiPlan.position.z = np.interp(plan_odo, model_odo, lateral_planner.path_xyz[:,2]).tolist()
  uiPlan.accel = longitudinal_planner.a_desired_trajectory_full.tolist()
  pm.send('uiPlan', ui_send)

def plannerd_thread(sm=None, pm=None):
  config_realtime_process(5 if TICI else 2, Priority.CTRL_LOW)

  cloudlog.info("plannerd is waiting for CarParams")
  params = Params()
  CP = car.CarParams.from_bytes(params.get("CarParams", block=True))
  cloudlog.info("plannerd got CarParams: %s", CP.carName)

  use_lanelines = not params.get_bool('EndToEndToggle')
  wide_camera = params.get_bool('EnableWideCamera') if TICI else False

  cloudlog.event("e2e mode", on=use_lanelines)

  longitudinal_planner = Planner(CP)
  lateral_planner = LateralPlanner(CP, use_lanelines=use_lanelines, wide_camera=wide_camera)

  if sm is None:
    sm = messaging.SubMaster(['carState', 'controlsState', 'radarState', 'modelV2'],
                             poll=['radarState', 'modelV2'], ignore_avg_freq=['radarState'])

  if pm is None:
    pm = messaging.PubMaster(['longitudinalPlan', 'lateralPlan', 'uiPlan'])

  while True:
    sm.update()

    if sm.updated['modelV2']:
      lateral_planner.update(sm, CP)
      lateral_planner.publish(sm, pm)
      longitudinal_planner.update(sm, CP)
      longitudinal_planner.publish(sm, pm)

      publish_ui_plan(sm, pm, lateral_planner, longitudinal_planner)

def main(sm=None, pm=None):
  plannerd_thread(sm, pm)


if __name__ == "__main__":
  main()
