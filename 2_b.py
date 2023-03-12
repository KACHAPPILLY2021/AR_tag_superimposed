from email.policy import default
import cv2

from cv2 import threshold
from cv2 import NORM_MINMAX
from cv2 import CV_8U
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy import misc
from scipy.fft import *
import scipy
import math

# function for getting corners of tag after Shi-Tomasi detection
def required_corners(corner , length) :
    # finding mean of x and y coordinates
    x_mean , y_mean = np.mean(corner , axis = 0)
    dis = []
   # calculating distances of each point from mean coordinates
    for i in range(length) :
        dist = math.sqrt((corner[i][0]-x_mean)**2+(corner[i][1]-y_mean)**2)
        dis.append(dist)
    # deleting 4 corners of A4 paper
    for i in range(4) :
        max_i = dis.index(max(dis))
        corner = np.delete(corner , [max_i],0 )
        dis.pop(max_i)
    # getting 4 corners of tag
    x_min , y_min= np.argmin(corner , axis = 0) 
    x_max , y_max = np.argmax(corner , axis = 0)
 
    A = corner[x_min]
    C = corner[x_max]
    U = corner[y_min]
    V = corner[y_max]
    corner_new = np.delete(corner , [x_min  ],0 )
    x_min1 , y_min1= np.argmin(corner_new , axis = 0)
    B = corner_new[x_min1]
    corner_new1 = np.delete(corner , [y_min ],0 )
    x_min2 , y_min2= np.argmin(corner_new1 , axis = 0)
    
    W = corner_new1[y_min2]
    # THese 'if' statements will only implement when AR tag edges are parallel with edges of image frame 
    tag_corner = []
    if A[0] == B[0]:
        if A[1] < B[1] :
            tag_corner.append(np.array([ A[0] , A[1] ]))
            tag_corner.append(np.array([ C[0] , A[1]]))
            tag_corner.append(np.array([ C[0] , B[1]]))
            tag_corner.append(np.array([ A[0] , B[1]]))

            return tag_corner

        else :
            tag_corner.append(np.array([ A[0] , B[1] ]))
            tag_corner.append(np.array([ C[0] , B[1]]))
            tag_corner.append(np.array([ C[0] , A[1]]))
            tag_corner.append(np.array([ A[0] , A[1]]))

            return tag_corner

    if U[1] == W[1] :
        if U[0] < W[0] :
            tag_corner.append(np.array([ U[0] , U[1]]))
            tag_corner.append(np.array([ W[0] , U[1]]))
            tag_corner.append(np.array([ W[0] , V[1]]))
            tag_corner.append(np.array([ U[0] , V[1]]))
            return tag_corner

        else :
            tag_corner.append(np.array([ W[0] , U[1]]))
            tag_corner.append(np.array([ U[0] , U[1]]))
            tag_corner.append(np.array([ U[0] , V[1]]))
            tag_corner.append(np.array([ W[0] , V[1]]))
            return tag_corner

    tag_corner.append(corner[x_min])
    tag_corner.append(corner[y_min])
    tag_corner.append(corner[x_max])
    tag_corner.append(corner[y_max])

    return tag_corner

# Homography matrix between corners of unit square and ar corners
def a_matrix_homo( x1 , y1  , c_f) :

    A = np.array([ [ x1 , x1 , 1 , 0 , 0 , 0 , -c_f[0][0]*x1 , -c_f[0][0]*x1 , -c_f[0][0]] ,
                    [ 0 , 0 , 0 , x1 , x1 , 1 , -c_f[0][1]*x1 , -c_f[0][1]*x1 , -c_f[0][1] ] ,

                    [ y1 , x1 , 1 , 0 , 0 , 0 , -c_f[1][0]*y1 , -c_f[1][0]*x1 , -c_f[1][0]] ,
                    [ 0 , 0 , 0 , y1 , x1 , 1 , -c_f[1][1]*y1 , -c_f[1][1]*x1 , -c_f[1][1] ] ,

                    [ y1 , y1 , 1 , 0 , 0 , 0 , -c_f[2][0]*y1 , -c_f[2][0]*y1 , -c_f[2][0]] ,
                    [ 0 , 0 , 0 , y1 , y1 , 1 , -c_f[2][1]*y1 , -c_f[2][1]*y1 , -c_f[2][1] ] ,


                    [ x1 , y1 , 1 , 0 , 0 , 0 , -c_f[3][0]*x1 , -c_f[3][0]*y1 , -c_f[3][0]] ,
                    [ 0 , 0 , 0 ,x1 , y1 , 1 , -c_f[3][1]*x1 , -c_f[3][1]*y1 , -c_f[3][1] ]
    ])

    U , S , V = np.linalg.svd(A)
    v = V[-1,:]
    h = V[-1,:]/V[-1,-1]
    h = np.array(h)
    return np.reshape(h ,(3 ,3 ))

# Determination of projection matrix
def Projection_matrix(H , K) :
    h1 = H[:,0]
    h2 = H[:,1]
    A = np.matmul(np.linalg.inv(K) , h1)
    B = np.matmul(np.linalg.inv(K) , h2)
    a = np.linalg.norm(A)
    b = np.linalg.norm(B)

    Lambbda = np.float32(2/(a+b))

    B_t = np.matmul(np.linalg.inv(K) , H)
    det_B_t = np.linalg.det(B_t)

    if det_B_t < 0 :
        B = -B_t

    B = B_t

    r1 = Lambbda*B[:,0]
    r2 = Lambbda*B[:,1]

    r3 = np.cross(r1 , r2 , axis =0)

    t = Lambbda*B[:,2]

    BB = np.column_stack((r1 , r2 , r3 , t))

    return np.matmul(K , BB)

# Using projection matrix to determine coordinates of upper square faace of cube in image frame
def Mission_corner(P) :
    p1 = np.array([[0] , [0] , [-1] , [1]])
    p2 = np.array([[1] , [0] , [-1] , [1]])
    p3 = np.array([[1] , [1] , [-1] , [1]])
    p4 = np.array([[0] , [1] , [-1] , [1]])

    c1 = np.matmul(P , p1)
    c1 = c1/c1[-1]


    c2 = np.matmul(P , p2)
    c2 = c2/c2[-1]

    c3 = np.matmul(P , p3)
    c3 = c3/c3[-1]

    c4 = np.matmul(P , p4)
    c4 = c4/c4[-1]

    return np.float32([np.array(c1[:-1]) , np.array(c2[:-1]) , np.array(c3[:-1]) , np.array(c4[:-1]) ])

# Plotting lines for the determined points
def cube(point , ar_tag , final) :
    point = point.astype(int)
    for i in range(4) :
        p1 = (point[i][0][0] , point[i][1][0] )
        p2 = (ar_tag[i][0] , ar_tag[i][1])
        final = cv2.line(final , p2 , p1 , (255,255,0) , thickness = 3 , lineType=4 )

    for i in range(3) :
        p1 = (point[i][0][0] , point[i][1][0])
        p2 = (point[i+1][0][0] , point[i+1][1][0])
        final = cv2.line(final , p1 , p2 , (255 , 255 ,0) , thickness = 3 , lineType=4)

    p1 = (point[0][0][0] , point[0][1][0])
    p2 = (point[3][0][0] , point[3][1][0])
    final = cv2.line(final , p1 , p2 , (255 , 255 ,0) , thickness = 3 , lineType=4)

    return final


# Create a VideoCapture object and] read from input file
cap = cv2.VideoCapture('video/1tagvideo.mp4')

# Define the codec and create VideoWriter object

cube_3d = cv2.VideoWriter('cube_3d.avi',cv2.VideoWriter_fourcc(*'XVID'), 27.0, (960,540))

# Check if camera opened successfully
if (cap.isOpened()== False):
    print("Error opening video file")

# Intrinsic matrix K 
K = np.array([[673.05 , 0 , 466.081] , [0 , 677.966 , 327.449] , [0 , 0 , 1]]) 
# Read until video is completed
while(cap.isOpened()):
    try:	
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret == True:
            dimensions = ( int(frame.shape[1]*0.5) , int(frame.shape[0]*0.5))
            FR = cv2.resize(frame, dimensions , interpolation = cv2.INTER_AREA)

    # convert to grayscale
            gray = cv2.cvtColor(FR,cv2.COLOR_BGR2GRAY)
    # apply gaussian blur        
            gaus_gray = cv2.GaussianBlur(gray,(55,55),cv2.BORDER_DEFAULT)
    # converting to binary image
            threshold, thresh = cv2.threshold(gaus_gray, 190, 235, cv2.THRESH_BINARY)
    # again applying gaussian blur
            gaus_gray_1 = cv2.GaussianBlur(thresh,(151,151),cv2.BORDER_DEFAULT)
    # Good features to track for corners
            corners_img = cv2.goodFeaturesToTrack(gaus_gray_1, 16 , 0.12 , 25)
            corners_img = np.int0(corners_img)
            size, xx,yy =np.shape(corners_img)
    # resize the array for corner manipulation
            reshape_array = np.reshape(corners_img , (size , 2))
    # list of corners after throwing away unwanted ones
            ar_corner = required_corners(reshape_array , size)

            for corners in ar_corner:        
                x,y = corners.ravel()
                #Circling the corners 
                cv2.circle(FR,(x,y),3,[255,0,0],3)


    # homograohy matrix determined
            Homo = a_matrix_homo(0 , 1 ,ar_corner )

            P = Projection_matrix(Homo , K)
    # Using projection matrix to determine coordinates of upper square faace of cube in image frame
            corner = Mission_corner(P)
    # Plotting lines for the determined points
            cube_done = cube(corner , ar_corner , FR)

            cv2.imshow("cube" , cube_done)

            cube_3d.write(cube_done)

            # Press Q on keyboard to exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        # Break the loop
        else:
            break
    except:
            ret, frame = cap.read()



# When everything done, release
# the video capture object
cube_3d.release()
cap.release()

# Closes all the frames
cv2.destroyAllWindows()