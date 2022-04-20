#!/usr/bin/env python3
from cmath import isnan
import math
import pyzed.sl as sl
from segmentation_system.segmentation import  Segmentation
from segmentation_system.utils.utils import *
from calibration import ZedCalib
from sklearn.linear_model import RANSACRegressor
from sklearn.metrics import mean_squared_error


####ROS imports
import rospy
from std_msgs.msg import Float64

pub = rospy.Publisher('roughness_estimate', Float64, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10hz

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

    # init_params.set_from_svo_file('recording/d_repaired.svo')

    return init_params

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
zed.set_svo_position(7700)
# 7092 end
# 7700 start

#4322
while True:
    if zed.grab() == sl.ERROR_CODE.SUCCESS:
        
        # Retrieve Datashow_3dscatter
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        # Preparation••••••••••••••
        image_k = cv2.cvtColor(image_zed.get_data(), cv2.COLOR_BGR2RGB)[:,:,:3]

        image_seg, img_seg_nodrive, road, sidewalk = segmentation_sys.run(image_k)
        image = addMask2Img(mask=image_seg,img = image_k)

        cutter = np.zeros(depth.get_data().shape)
        cutter[image_k.shape[0] - (81 * 4):, :(image_k.shape[1] // 81) * 81 + 1] = -1
        # To 3D-Space
        road_segmented_pixels, depth_seg = zed_calib.test(road, depth.get_data(), cutter)

        y = road_segmented_pixels[1]
        dep = road_segmented_pixels[2]
        ransac = RANSACRegressor(PolynomialRegression(degree=2),
                                 #residual_threshold= 2 * np.std(dep),
                                 random_state=0)
        ransac.fit(np.expand_dims(y, axis=1), dep)
        inlier_mask = ransac.predict(y)
     

        score_map = np.zeros(depth.get_data().shape) + np.nan
        # print(road_segmented_pixels)
        score_map[road_segmented_pixels[1].astype('int32'), road_segmented_pixels[0].astype('int32')] = np.abs(inlier_mask - dep)
      

        avg = getAvg(original_image=image_k, window_size= 81, score_image=score_map)
        
  
        # print(avg) 
        roughness_estimate=1/avg
        print(1/avg)
        published_msg = roughness_estimate

        # hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(roughness_estimate)
        pub.publish(roughness_estimate)
        rate.sleep()
        # # TODO: To be deleted
        plt.imshow(image_k)
        # # # TODO: To be deleted
        plt.show()
         
        

cv2.destroyAllWindows()
zed.close()
