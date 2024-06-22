from estimator import EstimationResult
import numpy as np
import time 
import helpers
import cv2 as cv
import os
import const as k
# DEBUG = True #os.environ["DEBUG"]

#https://stackoverflow.com/a/45399188 #1.2 works pretty well to kick out more than ~5 cm of difference
def reject_outliers_2(data, m=1.2):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d / (mdev if mdev else 1.)
    return data[s < m]

ns_to_sec = lambda x: x * (1/(1000*1000*1000))
cm_to_m = lambda x: x * (1/100)
## Blue alliance bottom left corner with the Source Station is 0,0
## alliance walls are X and the other 2 sides areY
## the right direction is 0 deg or 0 rad --> 
##   the robot should start facing 0 on blue and start facing -180 on red
#### EVERYTHING IS IN CM AND DEGREES

class RobotPos:
    def __init__(self, x, y, theta) -> None:
        self.x = x
        self.y = y
        self.theta = theta
        
def camRelativeToAbsoulote(cx, cy, tx, ty, tRot) -> "np.ndarray[np.double, np.double]":
    cx = np.double(cx)
    cy = np.double(cy)
    tx = np.double(tx)
    ty = np.double(ty)
    
    theta = np.deg2rad(np.array(helpers.normAngle(180-tRot), dtype=np.double)) # force datatype and radians

    cords = helpers.rotatePoint(cx, cy, theta)
    return np.array([tx - cords[0], ty - cords[1]])
    

APRILTAG_MAX_ID_EXIST = 16
SINGLE_SPEED_LIMIT_MPS = 10

# source to main alliance wall angle is 120 deg
WORLD_SIZE = (1654, 821)
WORLD_TAG_LOCATIONS = { # x,y,z,yaw
    "2": (0, 0, 0, 0), # testing tag, doesn't actually exist on the field
    "1": (1507.9472, 24.5872, 135.5852, 120), 
    "0": (1618.5134, 88.2904, 135.5852, 120),
    "3": (1657.9342, 498.2718, 145.1102, 180),
    "4": (1657.9342, 218.42, 145.1102, 180),
    "5": (1470.0758, 820.42, 135.5852, 270),
    "6": (184.15, 820.42, 135.5852, 270),
    "7": (0, 554.7868, 145.1102, 0), 
    "8": (0, 498.2818, 145.1102, 0), 
    "9": (35.6108, 88.3666, 135.5852, 60),
    "10":(146.1516, 24.5872, 135.5852, 60),
    "11":(1190.4726, 371.3226, 132.08, 300),
    "12":(1190.4726, 449.834, 132.08, 60),
    "13":(1122.0196, 410.5148, 132.08, 180),
    "14":(532.0792, 410.5148, 132.08, 0),
    "15":(464.1342, 449.834, 132.08, 120),
    "16":(464.1342, 371.3226, 132.08, 240),
}


#x, y, z of robot dimensions
ROBOT_SIZE_cm = (800, 500, 600)

def drawMiniMap(cords):
  if not k.DEBUG: return
  screen = np.zeros([500, 500, 3], dtype=np.uint8)

  # tag
  screen = cv.circle(screen, (150, 250), 5, (0,255,0), -1)
  screen = cv.circle(screen, (150, 250), 90, (100, 0, 100), 2)
  screen = cv.putText(screen, "tag", (150+10, 250+10), cv.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2)
  screen = cv.putText(screen, "+X", (480, 20), cv.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2)
  screen = cv.putText(screen, "+Y", (0, 490), cv.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2)
  for c in cords:
    screen = cv.circle(screen, c.astype(int), 3, (0,0,255), -1)

  cv.imshow("robot mini map", screen)
  

class RobotPositionTracker:
    def __init__(self) -> None:
        self.previousLocations = []# buffer
        self._prevLocBufLimit = 20

        self.visionBestLocation = RobotPos(0, 0, 0)
        self.lastTrustWorthyUpdateFrameTimeNs = 0

    def update(self, results: "list[EstimationResult]"):
        if len(results) < 1: return

        all_cords_possible = []
        all_yaw = []
        # all_z = []
        # convert camera relative cords into world cords
        for i in results:
            if i.id > APRILTAG_MAX_ID_EXIST: 
                del results[i]
                print("Found a tag with ID: {}, this tag doesn't exist".format(i.id))
                continue
            tag_x = WORLD_TAG_LOCATIONS[str(i.id)][0]
            tag_y = WORLD_TAG_LOCATIONS[str(i.id)][1]
            tag_yaw = WORLD_TAG_LOCATIONS[str(i.id)][3]

            cameraTrans = i.camera["camToRobotPos"]

            # camera axis mapping
            # yaw = -i.rvecs[1] + 180 - tag_yaw
            yaw = -i.rvecs[1]
            yaw += cameraTrans[3] + tag_yaw
            yaw = 180 - yaw
            yaw = helpers.normAngle(yaw)
            all_yaw.append(yaw)

            cam_x = i.tvecs[2] + cameraTrans[0] # camera parallel/z is x
            cam_y = i.tvecs[0] + cameraTrans[1] # camera throught/x is y
            # print("pos", cam_x, cam_y)
            cord = camRelativeToAbsoulote(cam_x, cam_y, tag_x, tag_y, tag_yaw)
            # cord = np.array([i.tvecs[2], i.tvecs[0]])
            print(cord)

            all_cords_possible.append(cord)

        # normalize data and kick out outliars due to detection incorrections
        yaw = np.average(reject_outliers_2(np.array(all_yaw)))

        # conclude all the cords into 2, hopefully 1 set of cords
        best_cords = helpers.groupCords(all_cords_possible)

        drawMiniMap(best_cords)

        # curTimeNs = time.time_ns()
        # if (cm_to_m(abs(x - self.visionBestLocation.x))/ns_to_sec(curTimeNs - self.lastTrustWorthyUpdateFrameTimeNs) > SINGLE_SPEED_LIMIT_MPS or
        #   cm_to_m(abs(y - self.visionBestLocation.y))/ns_to_sec(curTimeNs - self.lastTrustWorthyUpdateFrameTimeNs)):
        #     print("Exceed speed limit, loc update at time: {} will not be considered".format(curTimeNs))
        #     return
        
        # stores new locations if physics is obeyed
        self.previousLocations.append(self.visionBestLocation)
        if len(self.previousLocations) > self._prevLocBufLimit: del self.previousLocations[0]
        print(best_cords)
        final_loc = best_cords[0]
        if len(best_cords) > 1: # use th elocation closes to the predicted one
            last = self.visionBestLocation
            prev_dist = np.sqrt((final_loc[0] - last.x)**2 + (final_loc[1] - last.y)**2) # finalloc = 0th cord in best cords
            idx = 0
            for i in range(1, len(best_cords)):
                c = best_cords[i]
                dist = np.sqrt((c[0] - last.x)**2 + (c[1] - last.y)**2)
                if dist < prev_dist:
                    prev_dist = dist
                    idx = i
            final_loc = best_cords[idx]
            
        self.visionBestLocation = RobotPos(final_loc[0], final_loc[1], yaw) 
        # self.lastTrustWorthyUpdateFrameTimeNs = curTimeNs

    def getPos(self):
        return self.visionBestLocation

