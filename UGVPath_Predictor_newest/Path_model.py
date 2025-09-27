import torch
import torch.nn as nn
import numpy as np
import torch.nn.functional as F
from global_define import *
from Terrain_Feature_Net import *
from Mask_Road_Net import *
from CBAM_net import *
from WayEncoder import *
from Transformer import *

Medium_Feature_Dim = 256   # ALS: 484, KITTI: 256

class Path_Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.environment_transformer = Transformer(in_chans=Medium_Feature_Dim, embed_dim=96, output_dim=64, num_patches=WayPoint_NUM)
        self.past_path_transformer = Transformer(in_chans=2, embed_dim=48, output_dim=16, num_patches=WayPoint_NUM)
        self.waypoint_MLP = WayPoint_MLP()

    def forward(self, past_path_batch, environment_feature):
        # [B, 20, 64]
        environment_waypoint_feature = self.environment_transformer(environment_feature)
        # [B, 20, 16]
        path_waypoint_feature = self.past_path_transformer(past_path_batch)
        # [B, 20, 80]
        waypoint_feature = torch.cat([environment_waypoint_feature, path_waypoint_feature], dim=2)
        predict_waypoint = self.waypoint_MLP(waypoint_feature)
        return predict_waypoint


class Environment_Feature_Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.terrin_model = Terrain_Feature_Net()
        self.mask_model = Mask_Road_Net()
        self.attention = CBAM(80)
        self.way_encoder = WayEncoder(80, WayPoint_NUM)
        self.project = nn.Sequential(
            nn.Linear(Medium_Feature_Dim*20, 1024),
            nn.ReLU(inplace=True),
            nn.Linear(1024, 256),
            nn.ReLU(inplace=True),
            nn.Linear(256, Feature_Dim)
        )
        self.head = nn.Sequential(
            nn.Linear(Feature_Dim, 16),
            nn.ReLU(inplace=True),
            nn.Linear(16, Feature_Dim)
        )
        self.classifier = nn.Linear(Feature_Dim, Class_Num)

    def forward(self, feature_map_batch, mask_road_batch):
        # [B, 64, 44, 44] / [B, 64, 32, 32]
        terrain_feature = self.terrin_model(feature_map_batch)
        # [B, 16, 44, 44] / [B, 16, 32, 32]
        road_feature = self.mask_model(mask_road_batch)
        # [B, 80, 44, 44] / [B, 80, 32, 32]
        fuse_features = torch.cat([terrain_feature, road_feature], dim=1)
        fuse_features = self.attention(fuse_features)
        # [B, 20, 484]  /  [B, 20, 256]
        waypoint_feature = self.way_encoder(fuse_features)
        B = waypoint_feature.shape[0]
        flat_feature = waypoint_feature.view(B, -1)
        # [B, 64]
        final_feature = self.project(flat_feature)
        # [B, C]
        output_class = self.classifier(final_feature)
        output_feature = F.normalize(self.head(final_feature), dim=1)
        return waypoint_feature, output_feature, output_class

    def initialize_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.BatchNorm2d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)









