#!/usr/bin/env python3

# import ntcore
# from networktables import NetworkTables
# import networktables as ntab
from argparse import ArgumentParser
import cv2 as cv
import numpy as np
import importlib
import os
# DEBUG = True
from estimator import Estimator
from estimator import EstimationResult
# import cameras
import json
import world

import const as k
import helpers

# def updateNetworkTag(tagTable: ntab.NetworkTable, results: "list[EstimationResult]"):
#   for res in results: 
#     table = tagTable.getSubTable(str(res.id))
#     rot = table.getSubTable("Rot")
#     trans = table.getSubTable("Trans")

#     rot.putNumber("X", res.rvecs[0])
#     rot.putNumber("Y", res.rvecs[1])
#     rot.putNumber("Z", res.rvecs[2])

#     trans.putNumber("X", res.tvecs[0])
#     trans.putNumber("Y", res.tvecs[1])
#     trans.putNumber("Z", res.tvecs[2])

# def updateNetworkRobotPos(poseTable: ntab.NetworkTable, pos: world.RobotPos):
#   print(pos.x, pos.y, pos.theta)
#   poseTable.putNumber("X", pos.x)
#   poseTable.putNumber("Y", pos.y)
#   poseTable.putNumber("R", pos.theta)

parser = ArgumentParser(
  description = "AprilTag pose estimation solution meant to run on an NVIDIA Jetson Nano.",
  add_help = True
)

parser.add_argument("-s", "--server")
# parser.add_argument("-c", "--cameras", action="store", default="./cameras.py")
parser.add_argument("-d", "--debug", action="store_false")
parser.add_argument("--nt3", action = "store_true")
args = parser.parse_args()

import cameras
# os.environ["DEBUG"] = str(args.debug)
# DEBUG = os.environ["DEBUG"]

# NetworkTables.initialize(server = args.server)
# mainTable = NetworkTables.getTable("SmartDashboard/VisionServer")
# tagTable = mainTable.getSubTable("Tags")
# poseTable = mainTable.getSubTable("Pose")

poseEstimator = Estimator(cameras.MAIN_45FOV)

# for tag in tagTable.getSubTables():
#   updateNetworkTag(tagTable, tag, [0, 0, 0], [0, 0, 0])

robot = world.RobotPositionTracker()

roboSize = (100, 70) #np.array([110,70])
frontCord = np.abs(100 - np.array([150, 100]))
pt1Cord = np.abs(100 - np.array([150, 100]))
# pt2Norm = 100 + roboSize/2
def drawRobot(rot):
  if not k.DEBUG: return
  rot = -rot
  theta = np.deg2rad(rot)
  screen = np.zeros([200,200,3], dtype=np.uint8)

  xp1, yp1 = helpers.rotatePoint(*pt1Cord, theta)

  # r2 = np.sqrt(np.sum(np.power(pt2Norm, 2)))
  # xp2 = r2*np.cos(np.arctan(pt2Norm[1]/pt2Norm[0]) + theta)
  # yp2 = r2*np.sin(np.arctan(pt2Norm[1]/pt2Norm[0]) + theta)
  
  # draw robot 
  rect = cv.RotatedRect((100,100), roboSize, rot)
  screen = cv.drawContours(screen, [np.array(rect.points(), dtype=int)], -1, (100, 0, 60), 3) # robot

  # robot front
  frect = cv.RotatedRect((100 + helpers.rotatePoint(*frontCord, theta)).astype(np.int8), (5,30), rot)
  screen = cv.drawContours(screen, [np.array(frect.points(), dtype=int)], -1, (0, 0, 155), -1) 

  screen = cv.circle(screen, (int(100 + xp1), int(100 + yp1)), 2, (0, 255, 0), -1) #camera
  screen = cv.putText(screen, "{}deg".format(str(round(-rot, 2))), (100, 190), cv.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2)
  cv.imshow("robot char", screen)
  

while True:
  # image = cv.rotate(image, cv.ROTATE_180)
  # image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

  tagPoses = poseEstimator.estimate()
  for tag in tagPoses:
    print(tag.tvecs)
    print(tag.rvecs)
  robot.update(tagPoses)
  robot_pos = robot.getPos()
  print(np.sqrt(pow(robot_pos.x,2)+pow(robot_pos.y,2)))
  # updateNetworkRobotPos(tagTable, robot_pos)
  
  # drawRobot(robot_pos.theta)

  if cv.waitKey(30) == 1: break
