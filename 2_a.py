#try_2
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

# Function for determining homography matrix
def a_matrix_homo( x1 , y1 , w_f , c_f) :

    A = np.array([ [ x1 , y1 , 1 , 0 , 0 , 0 , -c_f[0][0]*x1 , -c_f[0][0]*y1 , -c_f[0][0]] ,
                    [ 0 , 0 , 0 , x1 , y1 , 1 , -c_f[0][1]*x1 , -c_f[0][1]*y1 , -c_f[0][1] ] ,

                    [ w_f[1] , y1 , 1 , 0 , 0 , 0 , -c_f[1][0]*w_f[1] , -c_f[1][0]*y1 , -c_f[1][0]] ,
                    [ 0 , 0 , 0 , w_f[1] , y1 , 1 , -c_f[1][1]*w_f[1] , -c_f[1][1]*y1 , -c_f[1][1] ] ,

                    [ w_f[1] , w_f[0] , 1 , 0 , 0 , 0 , -c_f[2][0]*w_f[1] , -c_f[2][0]*w_f[0] , -c_f[2][0]] ,
                    [ 0 , 0 , 0 , w_f[1] , w_f[0] , 1 , -c_f[2][1]*w_f[1] , -c_f[2][1]*w_f[0] , -c_f[2][1] ] ,


                    [ x1 , w_f[0] , 1 , 0 , 0 , 0 , -c_f[3][0]*x1 , -c_f[3][0]*w_f[0] , -c_f[3][0]] ,
                    [ 0 , 0 , 0 ,x1 , w_f[0] , 1 , -c_f[3][1]*x1 , -c_f[3][1]*w_f[0] , -c_f[3][1] ]
                ])

    U , S , V = np.linalg.svd(A)
    h = V[-1,:]/V[-1,-1]
    h = np.array(h)
    return np.reshape(h ,(3 ,3 ))

# Homography between blank and gray frame image
def homo_estimate(H , gray_frame , blank_image) :
    for i in range(blank_image.shape[0]) :
        for j in range(blank_image.shape[1]) :
            B = np.array([ [j] , [i] , [1]])
            A = np.matmul(H,B)
            A = A/A[-1]
            A = A.astype(int)
            x = A[0].item()
            y = A[1].item()
            blank_image[i][j][:] = gray_frame[y][x]
    return blank_image

# Homography between testudo and gray frame image
def testud0_homo_estimate(H , template , gray_frame ) :
    for i in range(template.shape[0]) :
        for j in range(template.shape[1]) :
            B = np.array([ [j] , [i] , [1]])
            A = np.matmul(H,B)
            A = A/A[-1]
            A = A.astype(int)
            x = A[0].item()
            y = A[1].item()
            gray_frame[y][x][:] = template[i][j][:] 
    return gray_frame

# function to cyclically change ar corners
def rotate_tag_check(TAG , ar_corner) :

    threshold1, thresh1 = cv2.threshold(TAG, 190, 255, cv2.THRESH_BINARY)
    slices = np.array_split(thresh1 , 8 , axis =0)

    packet = [np.array_split(part , 8 , axis =1) for part in slices]

    for i in range(len(packet)) :
        for j in range(len(packet[i])) :
            median = np.median(packet[i][j] , axis = 1)
            median = np.median(median)
            packet[i][j][:] = median

    if np.median(packet[5][5]) == 255 :
        return ar_corner
    if np.median(packet[5][2]) ==255: 
        return [ar_corner[1] , ar_corner[2] , ar_corner[3] , ar_corner[0]]

    if np.median(packet[2][2]) == 255: 
        return [ar_corner[2] , ar_corner[3] ,ar_corner[0] , ar_corner[1]]

    if np.median(packet[2][5])== 255:
        return [ar_corner[3] , ar_corner[0] ,ar_corner[1] , ar_corner[2]]


# Define the codec and create VideoWriter object

testudo = cv2.VideoWriter('testudo.avi',cv2.VideoWriter_fourcc(*'XVID'), 27.0, (960,540))

# Create a VideoCapture object and read from input file
cap = cv2.VideoCapture('video/1tagvideo.mp4')
# Reading testudo image
testudo_pic = cv2.imread('testudo.png' , cv2.IMREAD_COLOR)
testudo_shape = testudo_pic.shape

# Check if camera opened successfully
if (cap.isOpened()== False):
    print("Error opening video file")

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

    # Blank image created
            blank_image = np.zeros((200,200,3), np.uint8)

            blank_shape = blank_image.shape
    # homograohy matrix determined
            Homo = a_matrix_homo(0 , 0, blank_shape ,ar_corner )
    # homograpghy between balnk and image frame
            homo_image = homo_estimate(Homo , gray , blank_image)
    # checking orientation and determining new AR corners
            new_ar_corner = rotate_tag_check(homo_image , ar_corner)
    # homography matrix between testudo and new set of ar corners
            H_matrix_testudo = a_matrix_homo(0 , 0 , testudo_shape , new_ar_corner)
    # homograpghy between testudo and image frame
            homo_testudo = testud0_homo_estimate(H_matrix_testudo , testudo_pic , FR)

            testudo.write(homo_testudo)

            cv2.imshow("Testudo" , homo_testudo)
    
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        # Break the loop
        else:
            break
    except:
            ret, frame = cap.read()



# When everything done, release
# the video capture object
testudo.release()
cap.release()
print("Done")

# Closes all the frames
cv2.destroyAllWindows()