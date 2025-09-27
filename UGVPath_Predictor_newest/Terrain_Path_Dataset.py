import torch.utils.data
import torch
import numpy as np
import os
import random
from scipy.spatial.distance import pdist
from bisect import bisect_left
from torchvision import transforms

from global_define import *

class Terrain_Path_DataSet(torch.utils.data.Dataset):
    def __init__(self, train=True, data_path=None, bin_edges=None, bin_weight=None):
        self.terrain_feature_path = data_path + "/Terrain_Feature_Bin"
        self.label_path = data_path + "/Label_bin"
        self.past_path_path = data_path + "/Past_path_bin"
        self.mask_road_path = data_path + "/Satellite_Mask_Road"
        self._train = train
        self.bin_edges = bin_edges
        self.bin_weight = bin_weight
        terrain_feature_files = os.listdir(self.terrain_feature_path)
        label_files = os.listdir(self.label_path)
        past_path_files = os.listdir(self.past_path_path)
        mask_road_files = os.listdir(self.mask_road_path)
        terrain_feature_files.sort()
        label_files.sort()
        past_path_files.sort()
        mask_road_files.sort()
        self.terrain_feature_files = terrain_feature_files
        self.label_files = label_files
        self.past_path_files = past_path_files
        self.mask_road_files = mask_road_files

    def __getitem__(self, index):
        terrain_feature_file_name = self.terrain_feature_path + "/" + self.terrain_feature_files[index]
        label_file_name = self.label_path + "/" + self.label_files[index]
        past_path_file_name = self.past_path_path + "/" + self.past_path_files[index]
        mask_road_file_name = self.mask_road_path + "/" + self.mask_road_files[index]
        # read and norm terrain_feature
        terrain_feature_origin = np.fromfile(terrain_feature_file_name).reshape(-1, 7)
        terrain_feature_origin = np.transpose(terrain_feature_origin)
        feature_map = terrain_feature_origin[2:7, :].reshape(5, rows_, cols_)

        feature_map_norm = self.normalize_feature_map(feature_map)
        # read and norm past_path
        past_path = np.fromfile(past_path_file_name).reshape(-1, 2)
        # read and norm mask_road
        mask_road = cv2.imread(mask_road_file_name, 1)
        mask_road = cv2.resize(mask_road, (Mask_size, Mask_size))  # ALS: 119, KITTI: 95
        mask_road = mask_road.transpose(2, 0, 1)
        mask_road_norm = np.float32(mask_road) / 255.0
        # read label
        label = np.fromfile(label_file_name).reshape(-1, 2)
        label = label[0: WayPoint_NUM]
        past_path = past_path[0: WayPoint_NUM]
        # Draw_Sigma_Map(feature_map_norm[1, :], map_size=rows_, b_norm=True)
        # Draw_DeltaZ_Map(feature_map_norm[2, :], map_size=rows_, b_norm=True)
        # Draw_Elevation_Map(feature_map_norm[3, :], map_size=rows_, b_norm=True)
        # Draw_Label(past_path, map_size=rows_, input_win_name="Past_path")
        # Draw_Label(label, map_size=rows_, input_win_name="GT_label")
        # Draw_Mask_Road(mask_road_norm, map_size=rows_, b_test=False)

        if self._train:
            p = random.random()
            if p < 0.5:
                feature_map_output = feature_map_norm
                past_path_output = past_path
                mask_road_output = mask_road_norm
                label_output = label
            else:
                feature_map_output, past_path_output, mask_road_output, label_output = self.random_flip_data_x(feature_map_norm, past_path, mask_road_norm, label)

            # Draw_Sigma_Map(feature_map_output[1, :], map_size=rows_, b_norm=True, input_win_name="after_flip_sigma")
            # Draw_DeltaZ_Map(feature_map_output[2, :], map_size=rows_, b_norm=True, input_win_name="after_flip_delta_z")
            # Draw_Elevation_Map(feature_map_output[3, :], map_size=rows_, b_norm=True, input_win_name="after_flip_elevation")
            # Draw_Label(label_output, map_size=rows_, input_win_name="after_flip_GT_label")
            # Draw_Label(past_path_output, map_size=rows_, input_win_name="after_flip_Past_path")
            # Draw_Mask_Road(mask_road_output, map_size=rows_, b_test=False, input_win_name="after_flip_mask_road")

            # compute curve for each point (besides start and end point)
            curve_sum = 0.0
            sum_num = 0
            sum_delta_x = 0.0
            for j in range(1, label_output.shape[0] - 1):
                x = (label_output[j, 0] - label_output[j - 1, 0], label_output[j, 1] - label_output[j - 1, 1])
                y = (label_output[j + 1, 0] - label_output[j, 0], label_output[j + 1, 1] - label_output[j, 1])
                d = 1 - pdist([x, y], 'cosine')
                sin = np.sqrt(1 - d ** 2)
                dis = np.sqrt((label_output[j - 1, 0] - label_output[j + 1, 0]) ** 2 + (label_output[j - 1, 1] - label_output[j + 1, 1]) ** 2)
                k = 2 * sin / dis
                delta_x = (x[0] + y[0]) / 2
                sum_delta_x = sum_delta_x + delta_x
                curve_sum = curve_sum + k
                sum_num = sum_num + 1
            average_curve = curve_sum / sum_num
            average_delta_x = sum_delta_x / sum_num
            average_curve = average_curve / (average_delta_x / np.abs(average_delta_x))
            if average_curve >= max(self.bin_edges):
                bin_idx = len(self.bin_weight) - 1
            elif average_curve <= min(self.bin_edges):
                bin_idx = 0
            else:
                bin_idx = bisect_left(self.bin_edges, average_curve) - 1
            loss_weight = self.bin_weight[bin_idx]
            return feature_map_output, past_path_output, mask_road_output, label_output, loss_weight, bin_idx
        else:
            curve_sum = 0.0
            sum_num = 0
            sum_delta_x = 0.0
            for j in range(1, label.shape[0] - 1):
                x = (label[j, 0] - label[j - 1, 0], label[j, 1] - label[j - 1, 1])
                y = (label[j + 1, 0] - label[j, 0], label[j + 1, 1] - label[j, 1])
                d = 1 - pdist([x, y], 'cosine')
                sin = np.sqrt(1 - d ** 2)
                dis = np.sqrt((label[j - 1, 0] - label[j + 1, 0]) ** 2 + (label[j - 1, 1] - label[j + 1, 1]) ** 2)
                k = 2 * sin / dis
                delta_x = (x[0] + y[0]) / 2
                sum_delta_x = sum_delta_x + delta_x
                curve_sum = curve_sum + k
                sum_num = sum_num + 1
            average_curve = curve_sum / sum_num
            average_delta_x = sum_delta_x / sum_num
            average_curve = average_curve / (average_delta_x / np.abs(average_delta_x))
            if average_curve >= max(self.bin_edges):
                bin_idx = len(self.bin_weight) - 1
            elif average_curve <= min(self.bin_edges):
                bin_idx = 0
            else:
                bin_idx = bisect_left(self.bin_edges, average_curve) - 1
            loss_weight = self.bin_weight[bin_idx]
            return feature_map_norm, past_path, mask_road_norm, label, loss_weight, bin_idx

    def __len__(self):
        return len(self.terrain_feature_files)

    def normalize_feature_map(self, input_feature_map):
        # norm mu
        mu = input_feature_map[0, :]
        valid_mu = mu[np.where(mu != INVALID_Z)]
        max_mu = max(valid_mu)
        min_mu = min(valid_mu)
        mu[np.where(mu != INVALID_Z)] = (mu[np.where(mu != INVALID_Z)] - min_mu) / (max_mu - min_mu)
        mu[np.where(mu == INVALID_Z)] = -1
        input_feature_map[0, :] = mu
        # norm sigma
        sigma = input_feature_map[1, :]
        valid_sigma = sigma[np.where(sigma != INVALID_Z)]
        max_sigma = 0.02
        sigma[np.where(sigma != INVALID_Z) and np.where(sigma > max_sigma)] = max_sigma
        min_sigma = min(valid_sigma)
        sigma[np.where(sigma != INVALID_Z)] = (sigma[np.where(sigma != INVALID_Z)] - min_sigma) / (
                max_sigma - min_sigma)
        sigma[np.where(sigma == INVALID_Z)] = -1
        input_feature_map[1, :] = sigma
        # norm delta_z
        delta_z = input_feature_map[2, :]
        valid_delta_z = delta_z[np.where(delta_z != INVALID_Z)]
        max_delta_z = 1
        delta_z[np.where(delta_z != INVALID_Z) and np.where(delta_z > max_delta_z)] = max_delta_z
        min_delta_z = min(valid_delta_z)
        delta_z[np.where(delta_z != INVALID_Z)] = (delta_z[np.where(delta_z != INVALID_Z)] - min_delta_z) / (
                max_delta_z - min_delta_z)
        delta_z[np.where(delta_z == INVALID_Z)] = -1
        input_feature_map[2, :] = delta_z
        # norm estimate_height
        estimated_height = input_feature_map[3, :]
        valid_height = estimated_height[np.where(estimated_height != INVALID_Z)]
        max_height = max(valid_height)
        min_height = min(valid_height)
        estimated_height[np.where(estimated_height != INVALID_Z)] = \
            (estimated_height[np.where(estimated_height != INVALID_Z)] - min_height) / (max_height - min_height)
        estimated_height[np.where(estimated_height == INVALID_Z)] = -1
        input_feature_map[3, :] = estimated_height
        # norm normal_angle
        normal_angle = input_feature_map[4, :]
        valid_normal_angle = normal_angle[np.where(normal_angle != INVALID_Z)]
        max_normal_angle = max(valid_normal_angle)
        min_normal_angle = min(valid_normal_angle)
        normal_angle[np.where(normal_angle != INVALID_Z)] = \
            (normal_angle[np.where(normal_angle != INVALID_Z)] - min_normal_angle) / (
                    max_normal_angle - min_normal_angle)
        normal_angle[np.where(normal_angle == INVALID_Z)] = -1
        input_feature_map[4, :] = normal_angle
        return input_feature_map

    def random_flip_data_y(self, feature_map, past_path, mask_road, label):
        mu = feature_map[0, :]
        sigma = feature_map[1, :]
        delta_z = feature_map[2, :]
        estimated_height = feature_map[3, :]
        normal_angle = feature_map[4, :]
        mask_road_r = mask_road[0, :]
        mask_road_g = mask_road[1, :]
        mask_road_b = mask_road[2, :]
        # flip feature map
        flip_mu = np.flip(mu, axis=0)
        flip_sigma = np.flip(sigma, axis=0)
        flip_delta_z = np.flip(delta_z, axis=0)
        flip_estimated_height = np.flip(estimated_height, axis=0)
        flip_normal_angle = np.flip(normal_angle, axis=0)
        flip_mu = flip_mu.copy()
        flip_sigma = flip_sigma.copy()
        flip_delta_z = flip_delta_z.copy()
        flip_estimated_height = flip_estimated_height.copy()
        flip_normal_angle = flip_normal_angle.copy()
        feature_map[0, :] = flip_mu
        feature_map[1, :] = flip_sigma
        feature_map[2, :] = flip_delta_z
        feature_map[3, :] = flip_estimated_height
        feature_map[4, :] = flip_normal_angle
        # flip mask_road
        flip_r = np.flip(mask_road_r, axis=0)
        flip_g = np.flip(mask_road_g, axis=0)
        flip_b = np.flip(mask_road_b, axis=0)
        flip_r = flip_r.copy()
        flip_g = flip_g.copy()
        flip_b = flip_b.copy()
        mask_road[0, :] = flip_r
        mask_road[1, :] = flip_g
        mask_road[2, :] = flip_b
        # flip past_path
        past_path_y = past_path[:, 1]
        past_path_y_flip = -past_path_y
        past_path[:, 1] = past_path_y_flip
        # flip label
        label_y = label[:, 1]
        label_y_flip = -label_y
        label[:, 1] = label_y_flip
        return feature_map, past_path, mask_road, label

    def random_flip_data_x(self, feature_map, past_path, mask_road, label):
        mu = feature_map[0, :]
        sigma = feature_map[1, :]
        delta_z = feature_map[2, :]
        estimated_height = feature_map[3, :]
        normal_angle = feature_map[4, :]
        mask_road_r = mask_road[0, :]
        mask_road_g = mask_road[1, :]
        mask_road_b = mask_road[2, :]
        # flip feature map
        flip_mu = np.flip(mu, axis=1)
        flip_sigma = np.flip(sigma, axis=1)
        flip_delta_z = np.flip(delta_z, axis=1)
        flip_estimated_height = np.flip(estimated_height, axis=1)
        flip_normal_angle = np.flip(normal_angle, axis=1)
        flip_mu = flip_mu.copy()
        flip_sigma = flip_sigma.copy()
        flip_delta_z = flip_delta_z.copy()
        flip_estimated_height = flip_estimated_height.copy()
        flip_normal_angle = flip_normal_angle.copy()
        feature_map[0, :] = flip_mu
        feature_map[1, :] = flip_sigma
        feature_map[2, :] = flip_delta_z
        feature_map[3, :] = flip_estimated_height
        feature_map[4, :] = flip_normal_angle
        # flip mask_road
        flip_r = np.flip(mask_road_r, axis=1)
        flip_g = np.flip(mask_road_g, axis=1)
        flip_b = np.flip(mask_road_b, axis=1)
        flip_r = flip_r.copy()
        flip_g = flip_g.copy()
        flip_b = flip_b.copy()
        mask_road[0, :] = flip_r
        mask_road[1, :] = flip_g
        mask_road[2, :] = flip_b
        # flip past_path
        past_path_x = past_path[:, 0]
        past_path_x_flip = -past_path_x
        past_path[:, 0] = past_path_x_flip
        # flip label
        label_x = label[:, 0]
        label_x_flip = -label_x
        label[:, 0] = label_x_flip
        return feature_map, past_path, mask_road, label
