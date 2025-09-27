import torch
import torch.nn as nn
import numpy as np
import torch.nn.functional as F



class Terrain_Feature_Net(nn.Module):
    # n_inputs: [B, 5, 400, 400]
    def __init__(self):
        super().__init__()
        # first layer
        # [B, 5, 400, 400] -> [B, 16, 396, 396]  /  [B, 5, 300, 300] -> [B, 16, 296, 296]
        self.conv1 = nn.Conv2d(5, 16, kernel_size=11, stride=1, padding=3)
        self.bn1 = nn.BatchNorm2d(16)
        # [B, 16, 396, 396] -> [B, 16, 197, 197]  /  [B, 16, 296, 296] -> [B, 16, 147, 147]
        self.maxpool1 = nn.MaxPool2d(kernel_size=3, stride=2)
        # second layer
        # [B, 16, 197, 197] -> [B, 32, 193, 193]  /  [B, 16, 147, 147] -> [B, 32, 143, 143]
        self.conv2 = nn.Conv2d(16, 32, kernel_size=9, stride=1, padding=2)
        self.bn2 = nn.BatchNorm2d(32)
        # [B, 32, 193, 193] -> [B, 32, 96, 96]  /  [B, 32, 143, 143] -> [B, 32, 71, 71]
        self.maxpool2 = nn.MaxPool2d(kernel_size=3, stride=2)
        # third layer
        # [B, 32, 96, 96] -> [B, 64, 92, 92]  /  [B, 32, 71, 71] -> [B, 64, 67, 67]
        self.conv3 = nn.Conv2d(32, 64, kernel_size=7, stride=1, padding=1)
        self.bn3 = nn.BatchNorm2d(64)
        # fourth layer
        # [B, 64, 92, 92] -> [B, 128, 90, 90]  /  [B, 64, 67, 67] -> [B, 64, 65, 65]
        self.conv4 = nn.Conv2d(64, 128, kernel_size=5, stride=1, padding=1)
        self.bn4 = nn.BatchNorm2d(128)
        # fifth layer
        # [B, 128, 90, 90] -> [B, 64, 90, 90]  /  [B, 64, 65, 65] -> [B, 64, 65, 65]
        self.conv5 = nn.Conv2d(128, 64, kernel_size=3, stride=1, padding=1)
        self.bn5 = nn.BatchNorm2d(64)
        # [B, 64, 90, 90] -> [B, 64, 44, 44]  /  [B, 64, 65, 65] -> [B, 64, 32, 32]
        self.maxpool5 = nn.MaxPool2d(kernel_size=3, stride=2)
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x):
        x = self.maxpool1(self.relu(self.bn1(self.conv1(x))))
        x = self.maxpool2(self.relu(self.bn2(self.conv2(x))))
        x = self.relu(self.bn3(self.conv3(x)))
        x = self.relu(self.bn4(self.conv4(x)))
        terrain_feature = self.maxpool5(self.relu(self.bn5(self.conv5(x))))
        return terrain_feature
