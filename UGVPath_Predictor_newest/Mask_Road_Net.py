import torch
import torch.nn as nn
import numpy as np
import torch.nn.functional as F



class Mask_Road_Net(nn.Module):
    def __init__(self):
        super().__init__()
        # first layer
        # [B, 3, 119, 119] -> [B, 8, 109, 109]  /  [B, 3, 95, 95] -> [B, 8, 85, 85]
        self.conv1 = nn.Conv2d(3, 8, kernel_size=11, stride=1)
        self.bn1 = nn.BatchNorm2d(8)
        # [B, 8, 109, 109] -> [B, 8, 54, 54]  /  [B, 8, 85, 85] -> [B, 8, 42, 42]
        self.maxpool1 = nn.MaxPool2d(kernel_size=3, stride=2)
        # second layer
        # [B, 8, 54, 54] -> [B, 16, 48, 48]  /  [B, 8, 42, 42] -> [B, 16, 36, 36]
        self.conv2 = nn.Conv2d(8, 16, kernel_size=7, stride=1)
        self.bn2 = nn.BatchNorm2d(16)
        # third layer
        # [B, 16, 48, 48] -> [B, 16, 44, 44]  /  [B, 16, 36, 36] -> [B, 16, 32, 32]
        self.conv3 = nn.Conv2d(16, 16, kernel_size=5, stride=1)
        self.bn3 = nn.BatchNorm2d(16)
        self.relu = nn.ReLU(inplace=True)


    def forward(self, x):
        x = self.maxpool1(self.relu(self.bn1(self.conv1(x))))
        x = self.relu(self.bn2(self.conv2(x)))
        road_feature = self.relu(self.bn3(self.conv3(x)))
        return road_feature
