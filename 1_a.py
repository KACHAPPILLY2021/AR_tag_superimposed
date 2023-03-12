# importing libraries
import cv2
from cv2 import threshold
import numpy as np
import matplotlib.pyplot as plt
import scipy
from scipy.fft import *

# Create a VideoCapture object and read from input file
cap = cv2.VideoCapture('video/1tagvideo.mp4')

# Check if camera opened successfully
if (cap.isOpened()== False):
    print("Error opening video file")

ret, frame = cap.read()
dimensions = ( int(frame.shape[1]*0.5) , int(frame.shape[0]*0.5))
FR = cv2.resize(frame, dimensions , interpolation = cv2.INTER_AREA)
gray = cv2.cvtColor(FR,cv2.COLOR_BGR2GRAY)
fourier = fft2(gray)

f_shift = scipy.fft.fftshift(fourier)
magnitude_spectrum = 20*np.log(np.abs(f_shift))

rows, cols = gray.shape
crow,ccol = int(rows/2) , int(cols/2)
f_shift[crow-50:crow+50, ccol-50:ccol+50] = 0
f_ishift = scipy.fft.ifftshift(f_shift)
img_back = ifft2(f_ishift)
img_back = np.abs(img_back)

magnitude_spectrum_i = 20*np.log(np.abs(f_shift)+1)

plt.subplot(141),plt.imshow(FR,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])

plt.subplot(144),plt.imshow(img_back,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

plt.subplot(142),plt.imshow(magnitude_spectrum,cmap = 'gray')
plt.title('Fourier'), plt.xticks([]), plt.yticks([])

plt.subplot(143),plt.imshow(magnitude_spectrum_i,cmap = 'gray')
plt.title('After HPF'), plt.xticks([]), plt.yticks([])
plt.show()


# Read until video is completed
while(cap.isOpened()):

    if ret == True:
 
        cv2.imshow('Frame', FR)
   
    # Press Q on keyboard to  exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
   
  # Break the loop
    else: 
        break
   
# When everything done, release 
# the video capture object
cap.release()
   
# Closes all the frames
cv2.destroyAllWindows()


