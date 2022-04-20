
from segmentation_system.models import *
from segmentation_system.utils.parameters import *
from albumentations.pytorch import ToTensorV2

import numpy as np
import torch
import albumentations as Aug
import cv2

class Segmentation:


    def __init__(self, road= False, road_sudden= False, road_bicycle= False,
                 road_parking= False, crosswalk= False, sidewalk = False,
                 curb= False, laneline= False,sink = False,
                 traffic = False, road_combination = True):

        # Params to segment
        self.road           = road
        self.sidewalk       = sidewalk
        self.traffic        = traffic
        self.road_sudden    = road_sudden
        self.road_bicycle   = road_bicycle
        self.road_parking   = road_parking
        self.crosswalk      = crosswalk
        self.curb           = curb
        self.laneline       = laneline
        self.sink           = sink
        self.road_combination = road_combination

        # Model Initialization
        self.initialization()

    def initialization(self):
        # Define saved_model
        self.model = UNET(in_channels= COLOR_CHANNEL, out_channels= N_CLASSES)
        #Load state
        state = torch.load(MODEL_PATH)

        if state is None:
            print(MODEL_LOADING_ASSERT)
            return

        self.model.load_state_dict(state['state_dict'])
        self.model = self.model.cuda()
        return self.model

    def run(self, frame):
        height, width = frame.shape[:2]
        # augmentation (transform)
        aug_frame = self.transform_augmentation(frame)
        # Predict segmentation
        scored_frame = self.predict(frame= aug_frame)
        # Segment per params
        segmented_frame, segmented_frame_withoutDrivableArea, road, sidewalk, laneline, obst = self.frame_segmentation(scored_frame= scored_frame)
        # d-augmentation (transform)
        original_segmented_frame = self.dtransform_augmentation(segmented_frame, height, width)
        original_segmented_WithoutDrive_frame = self.dtransform_augmentation(segmented_frame, height, width)
        original_road_frame      = self.dtransform_augmentation(road, height, width)
        original_sidewalk_frame  = self.dtransform_augmentation(sidewalk, height, width)
        original_laneline_frame  = self.dtransform_augmentation(laneline, height, width)
        original_obst_frame  = self.dtransform_augmentation(obst, height, width)

        return original_segmented_frame,original_segmented_WithoutDrive_frame, original_road_frame, original_sidewalk_frame

    def transform_augmentation(self, frame):
        transform = Aug.Compose([
            Aug.Resize(height=IMAGE_SIZE, width=IMAGE_SIZE),
            Aug.Normalize(
                mean=[0.0, 0.0, 0.0],
                std=[1.0, 1.0, 1.0],
                max_pixel_value=255.0
            ),
            ToTensorV2()
        ])
        out = transform(image=frame)
        return out['image']


    def dtransform_augmentation(self, frame, height, width):

        dtransform = Aug.Compose([Aug.Resize(height=height, width=width)])
        out = dtransform(image=frame)
        return out['image']

    def predict(self, frame):

        with torch.no_grad():
            # Transform
            pre_image = frame.unsqueeze(0).cuda()
            # Segmentation
            out = self.model(pre_image)
            # Scoring
            out = torch.softmax(out, dim=1)
            scoring_img = torch.argmax(out, dim=1).squeeze(0).detach().cpu().numpy()

        return scoring_img

    def frame_segmentation(self, scored_frame):
        # Dimension
        scored_frame_cp = np.copy(scored_frame)
        segmented_map   = np.zeros((IMAGE_SIZE, IMAGE_SIZE, COLOR_CHANNEL), dtype= np.float32)
        segmented_map_withoutDrivableArea   = np.zeros((IMAGE_SIZE, IMAGE_SIZE, COLOR_CHANNEL), dtype= np.float32)

        road_frame      = np.zeros(((IMAGE_SIZE, IMAGE_SIZE)))
        sidewalk_frame  = np.zeros(((IMAGE_SIZE, IMAGE_SIZE)))
        laneline_frame  = np.zeros(((IMAGE_SIZE, IMAGE_SIZE)))
        obst_frame      = np.zeros(((IMAGE_SIZE, IMAGE_SIZE)))
        # Segmentation
        if self.road_combination:
            print('test')
            segmented_map[scored_frame_cp == ROAD_SCORE_NUM]            = ROAD_COLOR
            segmented_map[scored_frame_cp == ROAD_BICYCLE_SCORE_NUM]    = 0
            segmented_map[scored_frame_cp == ROAD_SUDDEN_SCORE_NUM]     = 0
            segmented_map[scored_frame_cp == ROAD_PARKING_SCORE_NUM]    = 0
        else:
            print('test2')
            segmented_map[scored_frame_cp == ROAD_SCORE_NUM]            = ROAD_COLOR
            segmented_map[scored_frame_cp == ROAD_BICYCLE_SCORE_NUM]    = ROAD_BICYCLE_COLOR
            segmented_map[scored_frame_cp == ROAD_SUDDEN_SCORE_NUM]     = ROAD_SUDDEN_ISSUE_COLOR
            segmented_map[scored_frame_cp == ROAD_PARKING_SCORE_NUM]    = PARKING_COLOR

        if self.laneline:
            segmented_map[scored_frame_cp == LANELINE_SCORE_NUM]        = LANELINE_COLOR
            segmented_map_withoutDrivableArea[scored_frame_cp == LANELINE_SCORE_NUM]        = LANELINE_COLOR

        if self.curb:
            segmented_map[scored_frame_cp == CURB_SCORE_NUM]            = CURB_COLOR
            segmented_map_withoutDrivableArea[scored_frame_cp == CURB_SCORE_NUM]            = CURB_COLOR

        if self.sidewalk:
            segmented_map[scored_frame_cp == SIDEWALK_SCORE_NUM]        = SIDEWALK_COLOR
            segmented_map_withoutDrivableArea[scored_frame_cp == SIDEWALK_SCORE_NUM]        = SIDEWALK_COLOR

        if self.traffic:
            segmented_map[scored_frame_cp == TRAFFIC_SCORE_NUM]         = TRAFFIC_LIGHT_COLOR
            segmented_map_withoutDrivableArea[scored_frame_cp == TRAFFIC_SCORE_NUM]         = TRAFFIC_LIGHT_COLOR

        if self.sink:
            segmented_map[scored_frame_cp == SINK_SCORE_NUM]            = STREET_SINK_COLOR
            segmented_map_withoutDrivableArea[scored_frame_cp == SINK_SCORE_NUM]            = STREET_SINK_COLOR

        if self.crosswalk:
            segmented_map[scored_frame_cp == CROSSWALK_SCORE_NUM]       = CROSSWALK_COLOR
            segmented_map_withoutDrivableArea[scored_frame_cp == CROSSWALK_SCORE_NUM]       = CROSSWALK_COLOR

        road_frame[scored_frame_cp == ROAD_SCORE_NUM]         = 1
        road_frame[scored_frame_cp == CROSSWALK_SCORE_NUM]         = 1
        road_frame[scored_frame_cp == LANELINE_SCORE_NUM]         = 1
        sidewalk_frame[scored_frame_cp == SIDEWALK_SCORE_NUM] = 1
        laneline_frame[scored_frame_cp == LANELINE_SCORE_NUM] = 1
        obst_frame[(scored_frame_cp == 0) | ((scored_frame_cp != CURB_SCORE_NUM) &
                                             (scored_frame_cp != 6) & (scored_frame_cp != 11) & (
                                                         scored_frame_cp != 17) &
                                             (scored_frame_cp != 21) & (scored_frame_cp != 30) & (
                                                         scored_frame_cp != 33) &
                                             (scored_frame_cp != 35) & (scored_frame_cp != 36) & (
                                                         scored_frame_cp != 46) &
                                             (scored_frame_cp != 60) & (scored_frame_cp != 61))] = 1
        '''
        obst_frame[(scored_frame_cp == 0) | ((scored_frame_cp != 4) &
                                             (scored_frame_cp != 6) & (scored_frame_cp != 11) & (scored_frame_cp != 17) &
                                             (scored_frame_cp != 21) & (scored_frame_cp != 30) & (scored_frame_cp != 33) &
                                             (scored_frame_cp != 35) & (scored_frame_cp != 36) & (scored_frame_cp != 46) &
                                             (scored_frame_cp != 60)& (scored_frame_cp != 61))] = 1
        '''

        return segmented_map,  segmented_map_withoutDrivableArea, road_frame, sidewalk_frame, laneline_frame, obst_frame