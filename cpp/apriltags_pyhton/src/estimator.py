import cv2 as cv
import numpy as np
import helpers
import const as k
import os
# DEBUG = True #os.environ["DEBUG"]
# import multiprocessing

lineThingPoints = np.float32([
  (0, 0, 0), # origin
  (-2, 0, 0), # x 
  (0, -2, 0), # y 
  (0, 0, 2) # z
])
point = lambda t: (int(t[0][0]), int(t[0][1]))


class EstimationResult:
  def __init__(self, camera, tag_id, rvecs, tvecs) -> None:
    self.camera = camera
    self.id = tag_id
    self.rvecs = rvecs 

    tvecs = tvecs * (k.APRILTAG_BLOCK_SIZE_mm / 10) # block size to cm
    self.tvecs = tvecs

class Estimator:
  def __init__(self, camera):
    self.camera = camera
    self.cap = cv.VideoCapture(self.camera["id"])
    self.matrix = np.matrix(camera["matrix"])

    self.distCoeffs = np.matrix(camera["distCoeffs"])

    self.objectPoints = np.matrix([
      [-4, 4, 0], # every 1 distance is 33 mm
      [4, 4, 0],
      [4, -4, 0],
      [-4, -4, 0]
    ], dtype = "double")

    self.dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36h11)
    # self.detector = cv.aruco.ArucoDetector(
    #   dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36h11),
    #   detectorParams = cv.aruco.DetectorParameters()
    # )

  def estimate(self) -> "list[EstimationResult]":
    out = []

    ret, image = self.cap.read()
    if not ret: return out
    
    image_old = image.copy()
    image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    # corners, ids, rejected = self.detector.detectMarkers(image)
    corners, ids, rejected = cv.aruco.detectMarkers(image, self.dictionary)	
    for i in range(len(corners)):
      corner = corners[i]
      id = ids[i][0] # index zero is needed cuz it returns a numpy array

      ret, rvec, tvec = cv.solvePnP(self.objectPoints, corner, self.matrix, self.distCoeffs)
      rmat = cv.Rodrigues(rvec)[0] # index 0 cuz returns array of array
      pmat = np.ravel(-np.matrix(rmat).T * np.matrix(tvec))
      # invedRvec, _ = cv.Rodrigues(np.linalg.inv(np.matrix(rmat)))
      # newTvec = -tvec * invedRvec

      # tagDeg = 120
      # tagRvec, _ = cv.Rodrigues(np.array([0, 0, np.deg2rad(tagDeg)]))
      # tagTvec = np.array([150, 50, 250]) # y ,z ,x  


      # fRvec = tagRvec + invedRvec
      # fTvec = tagTvec + newTvec

      # finalLoc = np.ravel(np.matrix(cv.Rodrigues(fRvec)[0]).T * fTvec)

      # print(finalLoc)

      theta = np.rad2deg(helpers.rodRotMatToEuler(cv.Rodrigues(rvec)[0]))
      out.append(EstimationResult(self.camera, id, theta, pmat))
      # out.append(EstimationResult(self.camera, id, theta, fTvec))

      
      if not k.DEBUG: continue
      computedImagePoints, j = cv.projectPoints(self.objectPoints, rvec, tvec, self.matrix, self.distCoeffs)
      for [[x, y]] in computedImagePoints: cv.circle(image_old, (int(x), int(y)), 5, (0, 255, 0), -1)

      computedLineThingPoints, j = cv.projectPoints(lineThingPoints, rvec, tvec, self.matrix, self.distCoeffs)
      cv.line(image_old, point(computedLineThingPoints[0]), point(computedLineThingPoints[1]), (255, 0, 0), 3)
      cv.line(image_old, point(computedLineThingPoints[0]), point(computedLineThingPoints[2]), (0, 255, 0), 3)
      cv.line(image_old, point(computedLineThingPoints[0]), point(computedLineThingPoints[3]), (0, 0, 255), 3)

      cv.putText(image_old, "Rot: " + str(np.around(rvec * (180 / np.pi), 3)), (10, image.shape[0] - 40), cv.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2)
      cv.putText(image_old, "Trans: "+ str(np.around(tvec, 3)), (10, image.shape[0] - 10), cv.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2)

    if k.DEBUG: 
      image_old = helpers.image_resize(image_old, 1080, 920)
      cv.imshow("img", image_old)
    return out

class EstimatorAsync:
  def __init__(self, camera):
    ## DEBUG MUST BE FORCED FALSE, cv2.imshow is not designed for multi threading
    global DEBUG
    DEBUG = False

    # self.proc = multiprocessing.Process()
