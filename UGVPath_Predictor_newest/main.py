import os
import numpy as ny
import torch
from scipy.spatial.distance import pdist
from tqdm import tqdm
from Decision_Planning_Model import *
import matplotlib.pyplot as plt
from global_define import *
from Terrain_Path_Dataset import *
from SY_MetaGF.MetaGFgrad_ECCV_multitask import *
from loss import *
from sklearn.manifold import TSNE


Initial_lr = 0.001
b_load_checkpoint = False

color=['cyan','green','blue','olive','black','yellow','red']

# load FeatureMap
@torch.no_grad()
def Load_FeatureMap(file):
    f = open(file, 'r')
    data = f.readlines()
    feature_map_list = []
    for data_each_line in data:
        split_data = data_each_line.split(' ')
        feature_list = []
        for it in split_data:
            feature_list.append(float(it))
        feature = np.array(feature_list)
        feature_map_list.append(feature)
    feature_map = np.array(feature_map_list)
    print(feature_map.shape)
    return feature_map


# load Label
@torch.no_grad()
def Load_Label(file):
    f = open(file, 'r')
    data = f.readlines()
    label_list = []
    for data_each_line in data:
        split_data = data_each_line.split(' ')
        single_label_list = []
        for it in split_data:
            single_label_list.append(float(it))
        label = np.array(single_label_list)
        label_list.append(label)
    final_label = np.array(label_list)
    return final_label

def Curve_sample_enhance(root_path):
    featuremap_path = root_path + "/Terrain_Feature_Bin"
    label_path = root_path + "/Label_bin"
    past_path_path = root_path + "/Past_path"
    mask_road_path = root_path + "/Satellite_Mask_Road"
    feature_files = os.listdir(featuremap_path)
    feature_files.sort()
    label_files = os.listdir(label_path)
    label_files.sort()
    past_path_files = os.listdir(past_path_path)
    past_path_files.sort()
    mask_road_files = os.listdir(mask_road_path)
    mask_road_files.sort()
    delta_idx = 3241
    for i in range(970, 1618):
        featuremap_file_name = featuremap_path + "/" + feature_files[i]
        label_file_name = label_path + "/" + label_files[i]
        past_path_file_name = past_path_path + "/" + past_path_files[i]
        mask_road_file_name = mask_road_path + "/" + mask_road_files[i]
        featuremap_copy_name = featuremap_path + "/Terrain_map_" + str(i+delta_idx).zfill(6) + ".bin"
        label_copy_name = label_path + "/path_label_" + str(i+delta_idx).zfill(6) + ".bin"
        past_path_copy_name = past_path_path + "/past_path_" + str(i+delta_idx).zfill(6) + ".png"
        mask_road_copy_name = mask_road_path + "/satellite_road_" + str(i+delta_idx).zfill(6) + ".png"
        feature_map_origin = np.fromfile(featuremap_file_name)
        feature_map_origin.tofile(featuremap_copy_name)
        label = np.fromfile(label_file_name)
        label.tofile(label_copy_name)
        past_path = cv2.imread(past_path_file_name, 1)
        cv2.imwrite(past_path_copy_name, past_path)
        mask_road = cv2.imread(mask_road_file_name, 1)
        cv2.imwrite(mask_road_copy_name, mask_road)
        print(featuremap_file_name)
        print(featuremap_copy_name)


def Extract_Curve_sample(root_path):
    featuremap_path = root_path + "/Terrain_Feature_Bin"
    label_path = root_path + "/Label_bin"
    past_path_path = root_path + "/Past_path"
    mask_road_path = root_path + "/Satellite_Mask_Road"
    featuremap_curve_path = root_path + "/Terrain_Feature_Bin_Curve"
    label_curve_path = root_path + "/Label_bin_Curve"
    past_path_curve_path = root_path + "/Past_path_Curve"
    mask_road_curve_path = root_path + "/Satellite_Mask_Road_Curve"
    feature_files = os.listdir(featuremap_path)
    feature_files.sort()
    label_files = os.listdir(label_path)
    label_files.sort()
    past_path_files = os.listdir(past_path_path)
    past_path_files.sort()
    mask_road_files = os.listdir(mask_road_path)
    mask_road_files.sort()
    new_ID = 0
    for i in range(0, 1619):
        featuremap_file_name = featuremap_path + "/" + feature_files[i]
        label_file_name = label_path + "/" + label_files[i]
        past_path_file_name = past_path_path + "/" + past_path_files[i]
        mask_road_file_name = mask_road_path + "/" + mask_road_files[i]
        label = np.fromfile(label_file_name).reshape(-1, 2)
        curve_sum = 0.0
        sum_num = 0
        for j in range(1, label.shape[0] - 1):
            x = (label[j, 0] - label[j - 1, 0], label[j, 1] - label[j - 1, 1])
            y = (label[j + 1, 0] - label[j, 0], label[j + 1, 1] - label[j, 1])
            d = 1 - pdist([x, y], 'cosine')
            sin = np.sqrt(1 - d ** 2)
            dis = np.sqrt((label[j - 1, 0] - label[j + 1, 0]) ** 2 + (label[j - 1, 1] - label[j + 1, 1]) ** 2)
            k = 2 * sin / dis
            curve_sum = curve_sum + k
            sum_num = sum_num + 1
        average_curve = curve_sum / sum_num * 100.0
        if average_curve > 0.5:
            featuremap_curve_name = featuremap_curve_path + "/Terrain_map_" + str(new_ID).zfill(6) + ".bin"
            label_curve_name = label_curve_path + "/path_label_" + str(new_ID).zfill(6) + ".bin"
            past_path_curve_name = past_path_curve_path + "/past_path_" + str(new_ID).zfill(6) + ".png"
            mask_road_curve_name = mask_road_curve_path + "/satellite_road_" + str(new_ID).zfill(6) + ".png"
            feature_map_origin = np.fromfile(featuremap_file_name)
            feature_map_origin.tofile(featuremap_curve_name)
            label_origin = np.fromfile(label_file_name)
            label_origin.tofile(label_curve_name)
            past_path = cv2.imread(past_path_file_name, 1)
            cv2.imwrite(past_path_curve_name, past_path)
            mask_road = cv2.imread(mask_road_file_name, 1)
            cv2.imwrite(mask_road_curve_name, mask_road)
            new_ID = new_ID + 1
            print(featuremap_curve_name)
        print(featuremap_file_name)


# Transform txt files to bin files
@torch.no_grad()
def Txt_2_Bin(root_path):
    # feature map
    featuremap_path = root_path + "/Terrain_Feature"
    featuremap_bin_path = root_path + "/Terrain_Feature_Bin"
    featuremap_files = os.listdir(featuremap_path)
    featuremap_files.sort()
    for featuremap_file in featuremap_files:
        print(featuremap_file)
        feature_map = Load_FeatureMap(featuremap_path + "/" + featuremap_file)
        feature_map.tofile(featuremap_bin_path + "/" + featuremap_file[0:18] + ".bin")
    print("End feature_map parse!")
    # label
    label_path = root_path + "/Label"
    label_bin_path = root_path + "/Label_bin"
    label_files = os.listdir(label_path)
    label_files.sort()
    for label_file in label_files:
        print(label_file)
        label = Load_Label(label_path + "/" + label_file)
        if label.shape[0] != 30:
            print("Error label.shape!!!!!!!")
        label.tofile(label_bin_path + "/" + label_file[0:17] + ".bin")
    print("End label parse!")
    # past_path
    past_path_path = root_path + "/Past_path"
    past_path_bin_path = root_path + "/Past_path_bin"
    past_path_files = os.listdir(past_path_path)
    past_path_files.sort()
    for past_path_file in past_path_files:
        print(past_path_file)
        past_path = Load_Label(past_path_path + "/" + past_path_file)
        if past_path.shape[0] != 30:
            print("Error past_path.shape!!!!!!!")
        past_path.tofile(past_path_bin_path + "/" + past_path_file[0:16] + ".bin")
    print("End past_path parse!")


@torch.no_grad()
def Obtain_Terrian_BackGround(root_path):
    terrian_img = []
    terrian_feature_path = root_path + "/Terrain_Feature_Bin"
    files = os.listdir(terrian_feature_path)
    files.sort()
    for i in range(len(files)):
        print("Load {}/{}".format(i, len(files)))
        featuremap_file_name = terrian_feature_path + "/" + files[i]
        feature_map_origin = np.fromfile(featuremap_file_name).reshape(-1, 10)
        feature_map_origin = np.transpose(feature_map_origin)
        feature_map = feature_map_origin[2:7, :].reshape(5, rows_, cols_)
        normal_angle = feature_map[4, :]
        terrian_i = Draw_NormalMap(normal_angle, map_size=rows_, b_norm=False)
        terrian_img.append(terrian_i)
    return terrian_img


@torch.no_grad()
def Vis_Sample(root_path):
    terrian_feature_path = root_path + "/Terrain_Feature_Bin"
    past_path_path = root_path + "/Past_path_bin"
    mask_road_path = root_path + "/Satellite_Mask_Road"
    label_path = root_path + "/Label_bin"
    files = os.listdir(terrian_feature_path)
    files.sort()
    past_path_files = os.listdir(past_path_path)
    past_path_files.sort()
    mask_road_files = os.listdir(mask_road_path)
    mask_road_files.sort()
    label_files = os.listdir(label_path)
    label_files.sort()
    # for i in range(1619, 2227):
    for i in range(len(files)):
        featuremap_file_name = terrian_feature_path + "/" + files[i]
        past_path_file_name = past_path_path + "/" + past_path_files[i]
        mask_road_file_name = mask_road_path + "/" + mask_road_files[i]
        label_file_name = label_path + "/" + label_files[i]
        label = np.fromfile(label_file_name).reshape(-1, 2)
        print(past_path_file_name, np.fromfile(past_path_file_name).shape)
        past_path = np.fromfile(past_path_file_name).reshape(-1, 2)
        mask_road = cv2.imread(mask_road_file_name, 1)
        feature_map_origin = np.fromfile(featuremap_file_name).reshape(-1, 7)
        feature_map_origin = np.transpose(feature_map_origin)
        feature_map = feature_map_origin[2:7, :].reshape(5, rows_, cols_)
        # norm mu
        mu = feature_map[0, :]
        valid_mu = mu[np.where(mu != INVALID_Z)]
        max_mu = max(valid_mu)
        min_mu = min(valid_mu)
        mu[np.where(mu != INVALID_Z)] = (mu[np.where(mu != INVALID_Z)] - min_mu)/(max_mu - min_mu)
        mu[np.where(mu == INVALID_Z)] = -1
        # norm sigma
        sigma = feature_map[1, :]
        valid_sigma = sigma[np.where(sigma != INVALID_Z)]
        max_sigma = 0.02
        sigma[np.where(sigma != INVALID_Z) and np.where(sigma > max_sigma)] = max_sigma
        min_sigma = min(valid_sigma)
        sigma[np.where(sigma != INVALID_Z)] = (sigma[np.where(sigma != INVALID_Z)] - min_sigma) / (max_sigma - min_sigma)
        sigma[np.where(sigma == INVALID_Z)] = -1

        # norm delta_z
        delta_z = feature_map[2, :]
        valid_delta_z = delta_z[np.where(delta_z != INVALID_Z)]
        max_delta_z = 1
        delta_z[np.where(delta_z != INVALID_Z) and np.where(delta_z > max_delta_z)] = max_delta_z
        min_delta_z = min(valid_delta_z)
        delta_z[np.where(delta_z != INVALID_Z)] = (delta_z[np.where(delta_z != INVALID_Z)] - min_delta_z) / (max_delta_z - min_delta_z)
        delta_z[np.where(delta_z == INVALID_Z)] = -1
        # norm estimate_height
        estimated_height = feature_map[3, :]
        valid_height = estimated_height[np.where(estimated_height != INVALID_Z)]
        max_height = max(valid_height)
        min_height = min(valid_height)
        estimated_height[np.where(estimated_height != INVALID_Z)] = \
            (estimated_height[np.where(estimated_height != INVALID_Z)] - min_height) / (max_height - min_height)
        estimated_height[np.where(estimated_height == INVALID_Z)] = -1
        # norm normal_angle
        normal_angle = feature_map[4, :]
        valid_normal_angle = normal_angle[np.where(normal_angle != INVALID_Z)]
        max_normal_angle = max(valid_normal_angle)
        min_normal_angle = min(valid_normal_angle)
        normal_angle[np.where(normal_angle != INVALID_Z)] = \
            (normal_angle[np.where(normal_angle != INVALID_Z)] - min_normal_angle) / (max_normal_angle - min_normal_angle)
        normal_angle[np.where(normal_angle == INVALID_Z)] = -1

        # Draw_Elevation_Map(mu, map_size=rows_, b_norm=True)
        cv2.namedWindow("mask_road", cv2.WINDOW_NORMAL)
        cv2.imshow("mask_road", mask_road)
        cv2.waitKey(2)
        Draw_Label(label, map_size=rows_, input_win_name="GT_label")
        Draw_Label(past_path, map_size=rows_, input_win_name="Past_path")
        Draw_Sigma_Map(sigma, map_size=rows_, b_norm=True)
        Draw_DeltaZ_Map(delta_z, map_size=rows_, b_norm=True)
        Draw_Elevation_Map(estimated_height, map_size=rows_, b_norm=True)
        # Draw_NormalMap(normal_angle, map_size=rows_, b_norm=True, max_normal_angle=max_normal_angle, min_normal_angle=min_normal_angle)

@torch.no_grad()
def Statistic_Curve_Distribution(root_path):
    label_path = root_path + "/Label_bin"
    label_files = os.listdir(label_path)
    label_files.sort()
    curve_value = np.array([])
    print("***** begin curve distribution statistic *****")
    for i in range(len(label_files)):
        label_file_name = label_path + "/" + label_files[i]
        label = np.fromfile(label_file_name).reshape(-1, 2)
        # compute curve for each point (besides start and end point)
        curve_sum = 0.0
        sum_num = 0
        sum_delta_x = 0.0
        for j in range(1, WayPoint_NUM-1):
            x = (label[j, 0] - label[j-1, 0], label[j, 1]-label[j-1, 1])
            y = (label[j+1, 0] - label[j, 0], label[j+1, 1] - label[j, 1])
            d = 1 - pdist([x, y], 'cosine')
            sin = np.sqrt(1-d**2)
            dis = np.sqrt((label[j-1, 0] - label[j+1, 0])**2 + (label[j-1, 1] - label[j+1, 1])**2)
            k = 2*sin/dis
            delta_x = (x[0] + y[0]) / 2
            sum_delta_x = sum_delta_x + delta_x
            curve_sum = curve_sum + k
            sum_num = sum_num + 1
        average_curve = curve_sum / sum_num
        average_delta_x = sum_delta_x / sum_num
        average_curve = average_curve / (average_delta_x / np.abs(average_delta_x))
        curve_value = np.append(curve_value, average_curve)
    hist, bin_edges = np.histogram(curve_value, bins=Class_Num)  # bin_num
    bin_weight = 1 / (hist / sum(hist))
    bin_weight = (bin_weight / sum(bin_weight))
    bin_weight_min = 1/min(bin_weight)
    bin_weight = bin_weight * bin_weight_min
    print("bin_edges: {}, bin_weight: {}".format(bin_edges, bin_weight))
    # plt.hist(curve_value, bins=5)
    # plt.title("histogram")
    # plt.show()
    return bin_edges, bin_weight


def compute_curve(path_points):
    path_points = path_points.cpu().detach()
    curve_value = np.array([])
    for j in range(1, path_points.shape[0] - 1):
        x = (path_points[j, 0] - path_points[j - 1, 0], path_points[j, 1] - path_points[j - 1, 1])
        y = (path_points[j + 1, 0] - path_points[j, 0], path_points[j + 1, 1] - path_points[j, 1])
        d = 1 - pdist([x, y], 'cosine')
        sin = np.sqrt(1 - d ** 2)
        dis = np.sqrt((path_points[j - 1, 0] - path_points[j + 1, 0]) ** 2 + (path_points[j - 1, 1] - path_points[j + 1, 1]) ** 2)
        k = 2 * sin / dis
        curve_value = np.append(curve_value, k)
    return curve_value

def deepcopy_parameter(model, oldmodel):
    with torch.no_grad():
        for n, p in model.named_parameters():
            p.data=deepcopy(oldmodel.state_dict()[n])

### ALS:
## cluster 3:  ADE = 0.50394207239151
## cluster 4:  ADE = 0.4553220570087433,   ADE = 0.4536835551261902  ADE = 0.4716310501098633
## cluster 5:  ADE = 0.5828450918197632
## cluster 6:  ADE = 0.8174521923065186
## cluster 7:  ADE = 0.9720660448074341
## cluster 8:  ADE = 1.3320051431655884
## cluster 9:  ADE = 0.8731395602226257
## cluster 10: ADE = 0.723854124546051

## WithMeta:    ADE = 0.5118693113327026, ACC = 93.90971275612281
## WithoutMeta: ADE = 1.4579205513000488, ACC = 79.97736340940195

### KITTI
## cluster 5_512: ADE = 0.6953355073928833, ACC = 84.19543298182518
## cluster 5_1024: ADE = 0.39083898067474365, ACC = 88.15956796341925
## cluster 5_2048: ADE = 0.6292952299118042, ACC = 88.21399650414834
## cluster 5_4096: ADE = 0.5547776818275452, ACC = 80.75183771555962

def main():
    # plt.figure(1)
    # plt.clf()
    # plt.plot(np.array(range(0, len(Loss_1))), np.stack(Loss_1), '--', color=color[0], label="exit" + str(0))
    # plt.plot(np.array(range(0, len(Loss_2))), np.stack(Loss_2), '--', color=color[1], label="exit" + str(1))
    # plt.legend()
    # plt.pause(10)
    random.seed(36)
    np.random.seed(36)
    torch.manual_seed(36)
    torch.cuda.manual_seed(36)
    torch.cuda.manual_seed_all(36)
    torch.backends.cudnn.deterministic = True
    # print(seed)

    train_data_path = "/data/projects/Satellite_Terrain_data/KITTI/train"
    test_data_path = "/data/projects/Satellite_Terrain_data/KITTI/test"
    # Extract_Curve_sample(train_data_path)
    # Curve_sample_enhance(train_data_path)
    # Txt_2_Bin(test_data_path)
    # Vis_Sample(train_data_path)
    bin_edges, bin_weight = Statistic_Curve_Distribution(train_data_path)
    # bin_edges = [0.07868236, 0.2232419, 0.36780145, 0.51236099, 0.65692053, 0.80148008, 0.94603962, 1.09059917, 1.23515871, 1.37971825, 1.5242778]
    # bin_edges = np.array(bin_edges)
    # bin_weight = [0.01861612, 0.02862667, 0.03205381, 0.09894872, 0.10837241, 0.11670875, 0.15695315, 0.18206565, 0.07915898, 0.17849574]
    # bin_weight = np.array(bin_weight)
    # terrian_image = Obtain_Terrian_BackGround(train_data_path)
    terrian_image = None
    torch.cuda.set_device(1)
    batch_size = 32
    train_dataset = Terrain_Path_DataSet(train=True, data_path=train_data_path, bin_edges=bin_edges, bin_weight=bin_weight)
    train_dataLoader = torch.utils.data.DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=1, drop_last=True)
    valid_dataset = Terrain_Path_DataSet(train=False, data_path=test_data_path, bin_edges=bin_edges, bin_weight=bin_weight)
    valid_dataLoader = torch.utils.data.DataLoader(valid_dataset, batch_size=1, shuffle=False, num_workers=1)

    path_model = DPM_Net()
    # path_model = PiCO(UGVPath_Net)
    path_model.cuda()
    # Defining the fusion model
    weight_model = Meta_fusion_weights_list(path_model, tasknum=TASKNUM)
    weight_model = torch.nn.DataParallel(weight_model).cuda()

    optimization_paramslist = []
    for i in range(0, TASKNUM):
        optimization_paramslist.append({"params": weight_model.module.weightlist[i].parameters(), "initial_lr": 0.1, "lr": 0.1})

    FusionOptimizer = torch.optim.Adam(optimization_paramslist, lr=1e-4)  # the weight decay matters
    taskmodel_optimizer = optim.Adam(path_model.parameters(), lr=Initial_lr)
    scheduler1 = optim.lr_scheduler.StepLR(taskmodel_optimizer, step_size=10, gamma=0.5)
    optimizer = MetaGrad(optimizer=taskmodel_optimizer, temperature=1, tasknum=TASKNUM, meta_Optimizer=FusionOptimizer, inneriteration=1, device='cuda:1')
    lambda_weight = np.ones([TASKNUM, 200])

    losslist = []
    for i in range(0, TASKNUM):
        losslist.append([])

    start_epoch = 0
    best_ADE = 99999
    if b_load_checkpoint:
        print("=> loading checkpoint_best_1024_seed！")
        checkpoint = torch.load('checkpoint_best_1024_seed.pth.tar', map_location='cuda:1')
        start_epoch = checkpoint['epoch']
        best_ADE = checkpoint['best_ADE']
        path_model.load_state_dict(checkpoint['state_dict'])
        taskmodel_optimizer.load_state_dict(checkpoint['optimizer'])
        print("=> loaded checkpoint '{}' (epoch {})".format('checkpoint_best_1024_seed.pth.tar', checkpoint['epoch']))
        ADE, ACC = test_model(valid_dataLoader, path_model, start_epoch, terrian_image)
        print("epoch {}: ADE = {}, ACC = {}".format(start_epoch, ADE, ACC))
    loss_smooth_L1 = torch.nn.SmoothL1Loss(reduction='none')
    loss_cont = SupConLoss()
    loss_fn = torch.nn.CrossEntropyLoss()
    ## loss_smooth_L1 = torch.nn.SmoothL1Loss()
    ## loss_weight = torch.tensor([1., 10.]).float().cuda()
    ## lossBCE = BCELoss_class_weighted(weight=loss_weight)

    for epoch in range(start_epoch, 200):
        print("epoch: {}".format(epoch))
        b_is_best = False
        # losses = train_model(train_dataLoader, path_model, epoch, taskmodel_optimizer, loss_smooth_L1, loss_cont, loss_fn)
        # adjust_learning_rate(taskmodel_optimizer, epoch)
        weightlist = []
        for i in range(0, TASKNUM):
            weightlist.append([])
        oldmodel = deepcopy(path_model)
        for taskid in range(0, TASKNUM):
            print("Task {}".format(taskid))
            losses = train_model(train_dataLoader, path_model, optimizer, weight_model, epoch, taskid, lambda_weight, taskmodel_optimizer, loss_smooth_L1, loss_cont, loss_fn)
            adaptmodel = deepcopy(path_model)
            weightlist[taskid] = (deepcopy(adaptmodel.state_dict()))
            del adaptmodel
            deepcopy_parameter(path_model, oldmodel)
        oldweightmodel = deepcopy(weight_model)
        print(">>>>>meta updating")
        adaptionmodel = deepcopy(path_model)
        tmp = optimizer.pc_backward(weightlist, adaptionmodel, weight_model, train_dataLoader, epoch, loss_smooth_L1, loss_cont, loss_fn)
        innner_loop_state = deepcopy(tmp)
        del tmp
        with torch.no_grad():
            for n, p in weight_model.named_parameters():
                p.data = deepcopy((1 - EMAoldmomentum) * p.data + EMAoldmomentum * oldweightmodel.state_dict()[n])
        path_model.load_state_dict(innner_loop_state)
        del adaptionmodel
        print(">>>>>meta updating")
        scheduler1.step()

        for i in range(0, TASKNUM):
            losslist[i].append(losses[i].avg)

        plt.figure(2)
        plt.clf()
        result_path = "/home/lthpc/projects/x_hertz/UGVPath_Predictor_new/Results"
        with open(os.path.join(result_path, 'results_KITTI_class_5_1024_seed.log'), 'a+') as f:
            f.write('Epoch {}: Loss_1 {}, Loss_2 {}\n'.format(epoch, losslist[0][len(losslist[0]) - 1],
                                                          losslist[1][len(losslist[1]) - 1]))
        for i in range(0, TASKNUM):
            plt.plot(np.array(range(0, len(losslist[i]))), np.stack(losslist[i]), '--', color=color[i], label="exit" + str(i))
        plt.legend()
        plt.pause(1)

        if (epoch % 5) == 0:
            ADE, ACC = test_model(valid_dataLoader, path_model, epoch, terrian_image)
            print("epoch {}: ADE = {}, ACC = {}".format(epoch, ADE, ACC))
            with open(os.path.join(result_path, 'results_KITTI_class_5_1024_seed.log'), 'a+') as f:
                f.write('Epoch {}: ADE {}, ACC {}\n'.format(epoch, ADE, ACC))
            if ADE < best_ADE:
                best_ADE = ADE
                b_is_best = True
            if b_is_best:
                torch.save({'epoch': epoch + 1, 'best_ADE': best_ADE, 'ACC': ACC, 'state_dict': path_model.state_dict(),
                            'optimizer': taskmodel_optimizer.state_dict()}, "checkpoint_best_1024_seed.pth.tar")


# def train_model(train_dataloader, multi_task_model, epoch, taskOptimizer, loss_smooth_L1_fun, loss_cont_fun, loss_fn_fun):
def train_model(train_dataloader, multi_task_model, optimizer, weightmodel, epoch, task, lambda_weight, taskOptimizer, loss_smooth_L1_fun, loss_cont_fun, loss_fn_fun):
        # loss_log = 0.0
        # loss_con_log = 0.0
        # loss_L1_log = 0.0
        # loss_fn_log = 0.0
        # log_num = 0
    # total_sum_grad = 0
    # total_frame = 0
    p_bar = tqdm(range(len(train_dataloader)))
    losses = []
    for i in range(0, TASKNUM):
        losses.append(AverageMeter())

    multi_task_model.train()

    for index, (feature_map_batch, past_path_batch, mask_road_batch, label_batch, loss_weight_batch, class_batch) in enumerate(train_dataloader):
        feature_map_batch, past_path_batch, mask_road_batch, label_batch, loss_weight_batch, class_batch = feature_map_batch.float().cuda(), \
                    past_path_batch.to(torch.float32).cuda(), mask_road_batch.cuda(), label_batch.to(torch.float32).cuda(), loss_weight_batch.cuda(), class_batch.cuda()
        output_decision, predict_waypoint = multi_task_model(feature_map_batch, past_path_batch, mask_road_batch, class_batch)
        big_features = output_decision[0]
        big_GT_Y = output_decision[1]
        output = output_decision[2]
        batch_size = predict_waypoint.shape[0]
        mask = torch.eq(big_GT_Y[:batch_size], big_GT_Y.T).float().cuda()
        loss_con = loss_cont_fun(features=big_features, mask=mask, batch_size=batch_size)
        loss_whole = loss_smooth_L1_fun(predict_waypoint, label_batch)
        loss_fn = loss_fn_fun(output, class_batch)
        # loss_weight_batch = torch.unsqueeze(loss_weight_batch, 1)
        # loss_weight_batch = torch.unsqueeze(loss_weight_batch, 1)
        # weighted_loss = torch.mul(loss_whole, loss_weight_batch)
        loss_task_1 = (loss_weight * loss_con + loss_fn)
        loss_task_2 = Loss_Weight * torch.mean(loss_whole)
        train_loss = [loss_task_1, loss_task_2]
        # loss = loss_task_2
        # loss = loss_task_1 + loss_task_2
        loss = 0
        for i in range(TASKNUM):
            if i == task:
                loss = loss + train_loss[i] * lambda_weight[i, epoch]
            else:
                loss = loss + train_loss[i] * lambda_weight[i, epoch] * AUXILIARY
        taskOptimizer.zero_grad()
        loss.backward()
        taskOptimizer.step()

        ## print grad value
        # sum_grad = 0
        # grad_terrin_model = sum(p.grad.data.norm() for p in multi_task_model.cont_net.encoder_q.terrin_model.parameters())
        # grad_mask_model = sum(p.grad.data.norm() for p in multi_task_model.cont_net.encoder_q.mask_model.parameters())
        # grad_attention = sum(p.grad.data.norm() for p in multi_task_model.cont_net.encoder_q.attention.parameters())
        # grad_way_encoder = sum(p.grad.data.norm() for p in multi_task_model.cont_net.encoder_q.way_encoder.parameters())
        # sum_grad = sum_grad+grad_terrin_model+ grad_mask_model + grad_attention + grad_way_encoder
        # total_sum_grad = total_sum_grad + sum_grad
        # total_frame = total_frame + 1
        # # task1: 20.1125, 15
        # # task2: 0.1655  0.1
        # print('--> average grad:', total_sum_grad/total_frame, total_frame)
        with torch.no_grad():
            for i in range(0, TASKNUM):
                losses[i].update(train_loss[i].item(), batch_size)
            # loss_log = loss_log + loss
            # loss_con_log = loss_con_log + loss_con
            # loss_L1_log = loss_L1_log + loss_L1
            # loss_fn_log = loss_fn_log + loss_fn
            # log_num = log_num + 1
        p_bar.set_description("epoch {}： {} / {}".format(epoch, index, len(train_dataloader)))
        p_bar.update()
    p_bar.close()
    return losses
    # print("epoch {}: loss = {}, loss_L1 = {}, loss_con = {}".format(epoch, loss_log / log_num, loss_L1_log / log_num, loss_con_log / log_num))


def test_model(test_loader, model, epoch, terrian_image):
    with torch.no_grad():
        model.eval()
        ADE_log = 0.0
        ACC_log = 0.0
        log_num = 0
        # b_first_append = True
        # fig1 = plt.figure(num=0)
        # color_list = ['darkorange', 'lawngreen', 'red', 'blue', 'yellow']
        # color_list = ['lawngreen', 'fuchsia', 'yellow', 'red', 'blue', 'indigo', 'olive', 'navy', 'silver', 'black', 'coral', 'tan']
        p_bar = tqdm(range(len(test_loader)))
        correct_label = 0
        sum_label = 0
        for index, (feature_map_batch, past_path_batch, mask_road_batch, label_batch, loss_weight_batch, class_batch) in enumerate(test_loader):
            feature_map_batch, past_path_batch, mask_road_batch, label_batch, loss_weight_batch, class_batch = \
                feature_map_batch.float().cuda(), past_path_batch.to(torch.float32).cuda(), mask_road_batch.cuda(), label_batch.to(torch.float32).cuda(), loss_weight_batch.cuda(), class_batch.cuda()
            output_class, predict_path_batch, feature = model(feature_map_batch, past_path_batch, mask_road_batch, class_batch, b_test=True)
            batch_size = predict_path_batch.shape[0]
            _, pred_prot = output_class.topk(1, 1, True, True)
            pred_prot = pred_prot.t()
            sum_label = sum_label + 1
            if pred_prot == class_batch:
                correct_label = correct_label + 1
            current_sum_ADE = 0
            for i in range(batch_size):
                delta_x = predict_path_batch[i, :, 0] - label_batch[i, :, 0]
                delta_y = predict_path_batch[i, :, 1] - label_batch[i, :, 1]
                dis = torch.sqrt(delta_x**2 + delta_y**2)
                average_ADE = sum(dis)/len(dis)
                current_sum_ADE = current_sum_ADE + average_ADE
                if terrian_image is None:
                    Draw_Predict_Path(predict_path_batch, label_batch, map_size=rows_)
            batch_average_ADE = current_sum_ADE / batch_size
            accuracy = correct_label / sum_label * 100.0

            # if b_first_append:
            #     ft = feature.cpu().detach().numpy()
            #     la = class_batch.cpu().detach().numpy()
            #     b_first_append = False
            # else:
            #     ft = np.append(ft, feature.cpu().detach().numpy(), axis=0)
            #     la = np.append(la, class_batch.cpu().detach().numpy())
            ADE_log = ADE_log + batch_average_ADE
            ACC_log = ACC_log + accuracy
            log_num = log_num + 1
            p_bar.set_description("epoch {}： {} / {}, ADE = {}， ACC = {}".format(epoch, index, len(test_loader), batch_average_ADE, accuracy))
            p_bar.update()
        ADE_output = ADE_log / log_num
        ACC_output = ACC_log / log_num
        # fig1.clf()
        # g = fig1.add_subplot(111)
        # tsne = TSNE(n_components=2, perplexity=20.0, init='pca', metric='cosine', random_state=0).fit_transform(ft)
        # x_min, x_max = tsne.min(0), tsne.max(0)
        # x_norm = (tsne - x_min) / (x_max - x_min)
        # g.scatter(x_norm[:, 0], x_norm[:, 1], s=0.1, c=np.array(color_list)[la[:]])
        # plt.pause(1)
        # plt.show()
        return ADE_output, ACC_output


def adjust_learning_rate(optimizer, epoch):
    # 初始学习率lr
    lr = Initial_lr
    # 学习率衰减，最小学习率eta_min = lr*学习率的衰减率^3
    eta_min = lr * (0.1 ** 3)
    # 利用余弦退火调整学习率，公式见上
    lr = eta_min + (lr - eta_min) * (1 + math.cos(math.pi * epoch / 200)) / 2
    # 在优化器参数列表中对学习率进行更新
    for param_group in optimizer.param_groups:
        param_group['lr'] = lr



if __name__ == '__main__':
    main()

# precision: 0.9436169343394465, recall: 0.9220063303382948
