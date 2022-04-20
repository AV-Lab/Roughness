#!/usr/bin/env python3
from cmath import isnan
import math


import pyzed.sl as sl

from segmentation_system.segmentation import  Segmentation
from segmentation_system.utils.utils import *
from calibration import ZedCalib
from sklearn.linear_model import RANSACRegressor


from sklearn.metrics import mean_squared_error

#ROS imports
import rospy
from std_msgs.msg import Float64


# TODO: Look here Eyad ^_^
'''
    This function just to visualize the output ... please add the following parts to getAvg() function.
    infrontMeters, shrink parameters are newly added along side depth. just copy and paste the function definition if poossbile.

'''

def scoreWindowDrawer(score_image, original_image, depth, window_size  = 4, infrontMeters = 2,shrink= 0.2):

    max_row_index = meterToRows(depth, window_size,infrontMeters) # TODO: Copy this function to getAvg() This function retrieve number of needed rows based on given infront meters.
    image_copy = np.copy(original_image)
    mask = np.zeros(original_image.shape)

    crop_boxes = shrink * original_image.shape[1] # TODO: Copy this line to getAvg() This line to get number of columns will be shrinked based on given shrink factor .. 0.2 means shrink 20% of columns from both sides.
    # score window
    border_color = (0, 0, 0)
    high_box_color = (255, 0, 0)
    mid_box_color = (0, 0, 255)
    low_box_color = (0, 255 ,0)

    for row_index in range(max_row_index):
        start_row = image_copy.shape[0] - (window_size * row_index)
        end_row = start_row - window_size

        for cell in range(0, original_image.shape[1]-window_size, window_size):
            # TODO: Copy this if statement to getAvg().
            if cell + window_size < crop_boxes or cell + window_size > original_image.shape[1] - crop_boxes:
                continue
            # TODO: That is it.
            cropped_box = score_image[end_row:start_row, cell:cell+window_size].flatten()

            #if np.sum(np.isnan(cropped_box)) > window_size//2:
                #continue
            # Skip any box contains nan value
            cropped_box[np.isnan(cropped_box)] = 0
            # Compute average
            avg_score = np.average(np.abs(cropped_box))
            # Draw in the original image

            if avg_score <= 5:
                print('low', avg_score)
                mask = cv2.rectangle(mask, (cell, start_row), (cell + window_size, end_row), color=low_box_color,
                                     thickness=-1)
                mask = cv2.rectangle(mask, (cell, start_row), (cell+window_size, end_row), color=border_color, thickness=1)

            elif avg_score <= 7:
                print('med', avg_score)
                mask = cv2.rectangle(mask, (cell, start_row), (cell + window_size, end_row), color=mid_box_color,
                                     thickness=-1)
                mask = cv2.rectangle(mask, (cell, start_row), (cell+window_size, end_row), color=border_color, thickness=1)

            else:

                print('high', avg_score)
                mask = cv2.rectangle(mask, (cell, start_row), (cell + window_size, end_row), color=high_box_color,
                                     thickness=-1)
                mask = cv2.rectangle(mask, (cell, start_row), (cell+window_size, end_row), color=border_color, thickness=1)


    # original makeup
    return cv2.addWeighted(original_image, 0.8, mask, 0.2, 0, dtype=cv2.CV_32F).astype('uint8')


class PolynomialRegression(object):
    def __init__(self, degree=2, coeffs=None):
        self.degree = degree
        self.coeffs = coeffs

    def fit(self, X, y):
        # print('defree',self.degree)
        self.coeffs = np.polyfit(X.ravel(), y, self.degree)

    def get_params(self, deep=False):
        return {'coeffs': self.coeffs}

    def set_params(self, coeffs=None, random_state=None):
        self.coeffs = coeffs

    def predict(self, X):
        # print('coff',self.coeffs)
        poly_eqn = np.poly1d(self.coeffs)
        y_hat = poly_eqn(X.ravel())
        return y_hat

    def score(self, X, y):
        return mean_squared_error(y, self.predict(X))


def setupParameters():

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD2K
    init_params.coordinate_units = sl.UNIT.CENTIMETER
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.sdk_verbose = True

    # TODO: To be deleted
    init_params.set_from_svo_file('/home/eyad/husky_ws/src/recording/d_repaired.svo')

    return init_params


def disc_avg(value,upper_value=7, med_value=5, lower_value=3):
    if value>7:
        return 0.3
    if 7>value>5:
        return 0.5
    if 5>value>3:
        return 0.7
    if value<3:
        return 1

def getAvg(score_image, original_image, window_size, max_row_index = 4):
    image_copy = np.copy(original_image)
    mask = np.zeros(original_image.shape)

    # score window
    border_color = (0, 0, 0)
    high_box_color = (255, 0, 0)
    mid_box_color = (0, 0, 255)
    low_box_color = (0, 255 ,0)

    # TODO: To shrink infront grid.
    start_crop_col  = 1100 - 100
    end_crop_col    = 1100 + 100
    x = []
    for row_index in range(max_row_index):
        start_row = image_copy.shape[0] - (window_size * row_index)
        end_row = start_row - window_size

        for cell in range(0, original_image.shape[1]-window_size, window_size):
            if cell < start_crop_col or cell > end_crop_col:
                continue
            cropped_box = score_image[end_row:start_row, cell:cell+window_size].flatten()

            #if np.sum(np.isnan(cropped_box)) > window_size//2:
                #continue
            # Skip any box contains nan value
            cropped_box[np.isnan(cropped_box)] = 0
            # Compute average
            x.append(np.average(np.abs(cropped_box)))
            # Draw in the original image
    x = np.array(x)

    return np.average(x[~np.isnan(x)])

    # original makeup
    return cv2.addWeighted(original_image, 0.8, mask, 0.2, 0, dtype=cv2.CV_32F).astype('uint8')

def meterToRows(depth, window_size, meters):
    # Locate middle point
    mid_x = depth.shape[1]//2
    rows = 0
    for row in range(depth.shape[0] - window_size, 0, -window_size):
        rows +=1
        
        if depth[row, mid_x] >= meters * 100:
            break

    
    
    return rows


# Camera Initialization
zed = sl.Camera()

# Setup Parameters
init_params = setupParameters()

# Open Camera
error = zed.open(init_params)
if error != sl.ERROR_CODE.SUCCESS:
    exit(1)

# Variables
image_zed = sl.Mat()
depth = sl.Mat()

runtime_parameters = sl.RuntimeParameters()
runtime_parameters.sensing_mode = sl.SENSING_MODE.FILL

# Segmentation
segmentation_sys = Segmentation(
    road_combination= True,
    sidewalk= True,
    sink= True,
    crosswalk= True,
    traffic= True,
    curb= True,
    laneline= True
)

point_cloud = sl.Mat()
zed_calib = ZedCalib()
# TODO: To be deleted
zed.set_svo_position(8100)
# 7092 end
# 7700 start
# TODO: Uncomment
# talker interface to start publishing
#pub = rospy.Publisher('roughness_estimate', Float64, queue_size=10)
#rospy.init_node('roughness_estimate')
#rate = rospy.Rate(10) # 10hz

#4322
while True:
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        # Retrieve Datashow_3dscatter
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        # Preparation
        image_k = cv2.cvtColor(image_zed.get_data(), cv2.COLOR_BGR2RGB)[:,:,:3]
        image_seg, img_seg_nodrive, road, sidewalk = segmentation_sys.run(image_k)
        image = addMask2Img(mask=image_seg,img = image_k)
        cutter = np.zeros(depth.get_data().shape)
        cutter[image_k.shape[0] - (81 * 4):, :(image_k.shape[1] // 81) * 81 + 1] = -1
        # To 3D-Space
        road_segmented_pixels, depth_seg = zed_calib.test(road, depth.get_data(), cutter)

        # TODO: To be deleted
        print(road_segmented_pixels.shape)
        print(road_segmented_pixels[2])
        print('=======')
        print(road_segmented_pixels[0])
        plt.subplot(1, 3, 1)
        plt.imshow(image.astype('uint8'))
        plt.subplot(1, 3, 2)
        plt.imshow(depth.get_data(), cmap='gray')
        plt.subplot(1, 3, 3)

        plt.scatter(road_segmented_pixels[2], road_segmented_pixels[1])

        y = road_segmented_pixels[1]
        dep = road_segmented_pixels[2]
        ransac = RANSACRegressor(PolynomialRegression(degree=2),
                                 #residual_threshold= 2 * np.std(dep),
                                 random_state=0)
        ransac.fit(np.expand_dims(y, axis=1), dep)
        inlier_mask = ransac.predict(y)

        # TODO: To be deleted
        plt.plot(inlier_mask,y, c='r')
        plt.show()

        score_map = np.zeros(depth.get_data().shape) + np.nan
        score_map[road_segmented_pixels[1].astype('int32'), road_segmented_pixels[0].astype('int32')] = np.abs(inlier_mask - dep)
        avg = getAvg(original_image=image_k, window_size= 81, score_image=score_map)
        
        # TODO: Uncomment
        #print(avg) 
        #print(disc_avg(avg))
        #rospy.loginfo(avg)
        #pub.publish(avg)

        # To visualize (TO be deleted after updating getAvg())

        image_out = scoreWindowDrawer(
            original_image=image_k,
            window_size= 81, # Grid box size.
            score_image=score_map,
            depth = depth.get_data(),
            infrontMeters= 2, # Meters to consider infront of the car.
            shrink= 0.2) # Shrink factors to eliminate columns at both sides.

        # rate.sleep()
        # # TODO: To be deleted
        plt.imshow(image_out)
        # # TODO: To be deleted
        plt.show()
         
        

cv2.destroyAllWindows()
zed.close()
