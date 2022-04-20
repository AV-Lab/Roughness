import os

# Path
MODEL_PATH = os.path.dirname(os.getcwd()) + "/src/segmentation_system/saved_model/retrained_model"
SAVE_PATH  = "save_video"

# Identify lower resolution


# Model Settings
IMAGE_SIZE = 320
COLOR_CHANNEL = 3
N_CLASSES = 11
# Classes
SIDEWALK_SCORE_NUM = 1
LANELINE_SCORE_NUM = 17
CURB_SCORE_NUM = 3
TRAFFIC_SCORE_NUM = 5
ROAD_SCORE_NUM = 4
ROAD_SUDDEN_SCORE_NUM = 6
ROAD_BICYCLE_SCORE_NUM = 7
ROAD_PARKING_SCORE_NUM = 8
CROSSWALK_SCORE_NUM = 9
SINK_SCORE_NUM = 10


# ERROR ALERT MSG
MODEL_LOADING_ASSERT = "Cannot load saved_model"

# Colors
# Sidewalk
SIDEWALK_COLOR        = [244, 35, 232]
# Laneline
LANELINE_COLOR        = [255,0,0]
# Kerb
CURB_COLOR     = [0,0,255]
# Traffic
TRAFFIC_LIGHT_COLOR    = [250, 170, 30]
# Road
ROAD_COLOR       = [0,255,0]
ROAD_SUDDEN_ISSUE_COLOR = [110, 110, 110]
PARKING_COLOR          = [250, 170, 160]
ROAD_BICYCLE_COLOR     = [128, 64, 255]
CROSSWALK_COLOR        = [128,0,128]
# Objects
STREET_SINK_COLOR      = [0,128,128]
