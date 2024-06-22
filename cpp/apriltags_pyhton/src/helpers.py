import cv2
import math
import numpy as np
import os
import const as k
# DEBUG = True #os.environ["DEBUG"]

# https://stackoverflow.com/a/44659589 with GPU  cuda resize
def image_resize(image, width = None, height = None, inter = cv2.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # if both the width and height are None, then return the
    # original image
    if width is None and height is None:
        return image

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the image
    cuda_img_buf = cv2.cuda.GpuMat()
    cuda_img_buf.upload(image)
    cuda_img_buf = cv2.cuda.resize(cuda_img_buf, dim, interpolation = inter)
    resized = cuda_img_buf.download()

    # return the resized image
    return resized

# rotate point 
def rotatePoint(x, y, theta):
    cords = np.array([x,y])
    r1 = np.sqrt(np.sum(np.power(cords, 2)))
    xp1 = r1*np.cos(np.arctan(cords[1]/cords[0]) + theta)
    yp1 = r1*np.sin(np.arctan(cords[1]/cords[0]) + theta)

    return np.array([xp1, yp1])

# implenmentaion of Rodugriz Rotation matrix to Euler angles based on
# https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2012/07/euler-angles1.pdf
# or /docs/euler-angles
# (x,y,z)
def rodRotMatToEuler(m):
    x = np.arctan2(m[1][2], m[2][2], dtype=np.double)
    c_2 = np.sqrt(m[0][0]**2 + m[0][1]**2, dtype=np.double)
    y = np.arctan2(-m[0][2], c_2)
    s_1 = np.sin(x, dtype=np.double)
    c_1 = np.cos(x, dtype=np.double)
    z = np.arctan2(s_1*m[2][0] - c_1 * m[1][0], c_1*m[1][1] - s_1*m[2][1], dtype=np.double)
    return (x, y, z)

# normalize angle between 0 and 360
def normAngle(angle):
    return angle % 360

# group cordinates based on distance, computionaly expensive
# limit radius = 10 CM
def groupCords(cords: "list[np.ndarray[np.double, np.double]]", limitRadiCM = 10):

    groups = [] # array of shape (1,2<groupCord, numCordInGroup>)

    groups.append([cords[0], 1]) # initialize with first cordinate 
    for i in range(1, len(cords)):
        current = cords[i]
        # calculate current cord to all known groups
        distances = []
        for j in range(len(groups)):
            g = groups[j][0]
            dist = np.sqrt((g[0] - current[0])**2 + (g[1] - current[1])**2)
            distances.append(dist)
        

        # iterate throught the distances and assign cordinate to group or make a new group
        groupToJoin = None
        is_ignored = False
        for di in range(len(distances)):
            d = distances[di]
            if d > limitRadiCM: continue
            if groupToJoin != None:  # ambigious case where the point is between 2 groups, skip
                is_ignored = True
                break
            groupToJoin = di # the distance array should have the same size as groups

        if is_ignored: continue
        if groupToJoin == None:
            groups.append([current, 1])    
            continue
        g = groups[groupToJoin]
        g[0] = (g[0] + current)/2 # average the previus ave cord with the new cord
        g[1] += 1 # increase counter

    best_res = [] # a list of best cordinates # from the groups
    last_best_score = 0
    for g in groups:
        if g[1] > last_best_score:
            best_res = [g[0]]
            last_best_score = g[1]
        elif g[1] == last_best_score:
            best_res.append(g[0])
        # otherwise skip
    
    return best_res

            


    # all_distances = []
    # for i in range(len(cords)):
    #     distances = []

    #     current = cords[i]

    #     for j in range(i, len(cords)):
    #         c = cords[j]
    #         dist = np.sqrt((c[0] - current[0])**2 + (c[1] - current[1])**2)
    #         distances.append(dist)
        
    #     all_distances.append(distances)
        
    
