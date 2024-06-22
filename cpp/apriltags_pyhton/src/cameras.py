from typing import Dict

class _camera(Dict):
  id: str
  apriltags: bool = False
  matrix: "list[list, list, list]"
  distCoeffs: "list[list]"


WEIRD_USB_CAMERA: _camera = {
  "matrix": [
    [604.72866309, 0, 671.93738572],
    [0, 601.63145784, 529.56708924],
    [0, 0, 1]
  ],
  "distCoeffs": [[
    -1.10624671e-01, -2.10266584e-01, 2.39710184e-05, -3.82402191e-04, 2.50396497e-01
  ]]
}

MAIN_45FOV: _camera = {
  "id": 0,
  "apriltags": True,
  # "camToRobotPos": [800,250,300, 0], # anchored at bottom right of robot
  "camToRobotPos": [0,0,0, 0],
  # "matrix":
  #   [[710.8459662, 0, 584.09769116],
  #   [0.,710.64515618, 485.94212983],
  #   [0., 0., 1., ]],
  # "distCoeffs":
  # [[-0.3689181,0.12470983,-0.0062236,0.00298559,-0.01839474]]
  "matrix":
    [[673.49634849, 0, 616.93113106],
    [0, 670.71012973, 536.45109056],
    [0, 0, 1]],
  "distCoeffs": 
    [[-0.18422303, 0.04338743, -0.0010019, 0.00080675, -0.00543398]]
}