import torch
import torch.nn as nn
import numpy as np
import torch.nn.functional as F
from global_define import *
from scipy.spatial.distance import pdist
from bisect import bisect_left


class WayEncoder(nn.Module):
    def __init__(self, in_dim, way_point_num):
        super().__init__()
        self.conv1 = nn.Conv2d(in_dim, in_dim, kernel_size=3, stride=1, padding=1, bias=False)
        self.bn = nn.BatchNorm2d(in_dim)
        self.conv2 = nn.Conv2d(in_dim, in_dim, kernel_size=3, stride=1, padding=1, bias=False)
        self.conv3 = nn.Conv2d(in_dim, in_dim, kernel_size=3, stride=1, padding=1, bias=False)
        self.conv4 = nn.Conv2d(in_dim, in_dim, kernel_size=3, stride=1, padding=1, bias=False)
        self.conv5 = nn.Conv2d(in_dim, way_point_num, kernel_size=3, stride=2, padding=1, bias=True)

    def forward(self, feature_map):
        x = self.bn(self.conv1(feature_map))
        x = self.bn(self.conv2(x))
        x = self.bn(self.conv3(x))
        x = self.bn(self.conv4(x))
        waypoints_feature = self.conv5(x)
        b, c, h, w = waypoints_feature.shape
        waypoints_feature = waypoints_feature.view(b, c, -1)
        return waypoints_feature


class WayPoint_MLP(nn.Module):
    def __init__(self, n_inputs=80):
        super().__init__()
        # input to first hidden layer, 80->512
        self.hidden1 = nn.Linear(n_inputs, 32)
        # nn.init.kaiming_uniform_(self.hidden1.weight, nonlinearity='relu')
        self.act1 = nn.ReLU()
        # second hidden layer
        self.hidden2 = nn.Linear(32, 16)
        self.act2 = nn.ReLU()
        # third hidden layer and output
        self.hidden3 = nn.Linear(16, 8)
        self.output = nn.Linear(8, 2)

    def forward(self, x):
        x = self.hidden1(x)
        x = self.act1(x)
        # second hidden layer
        x = self.hidden2(x)
        x = self.act2(x)
        # third hidden layer and output
        x = self.hidden3(x)
        output_waypoint = self.output(x)
        return output_waypoint
