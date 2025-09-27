import torch
import torch.nn as nn
import numpy as np
import torch.nn.functional as F
from global_define import *
from PiCO import *
from Path_model import *

class DPM_Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.cont_net = PiCO(Environment_Feature_Net)
        self.path_net = Path_Net()

    def forward(self, feature_map_batch, past_path_batch, mask_road_batch, GT_Y, b_test=False):
        if b_test is False:
            feature_env, big_features, big_GT_Y, output = self.cont_net(feature_map_batch, mask_road_batch, GT_Y)
            predict_waypoint = self.path_net(past_path_batch, feature_env)
            return [[big_features, big_GT_Y, output], predict_waypoint]
        else:
            feature_env, output, feature = self.cont_net(feature_map_batch, mask_road_batch, GT_Y, b_test=True)
            predict_waypoint = self.path_net(past_path_batch, feature_env)
            return output, predict_waypoint, feature
