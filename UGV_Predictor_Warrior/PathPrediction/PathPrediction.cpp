#include "PathPrediction.h"
const int width = 300;
const int height = 300;
const int Mask_SIZE = 95;
const float RESOLUTION = 0.2;
const float INVALID_Z = -123456.0; // 无效的Z值

// 假设我们有一个简单的CSV读取函数

std::vector<std::vector<float>> read_csv(const std::string &filename)
{

    std::vector<std::vector<float>> data;

    std::ifstream file(filename);

    std::string line;

    while (std::getline(file, line))
    {

        std::istringstream ss(line);

        std::string cell;

        std::vector<float> row;

        while (std::getline(ss, cell, ','))
        {

            row.push_back(std::stof(cell));
        }

        data.push_back(row);
    }

    return data;
}

cv::Mat tensorToMat(torch::Tensor tensor)
{
    // 将 Tensor 转换为 CPU 上的数据
    tensor = tensor.to(torch::kCPU);

    // 获取 Tensor 的数据指针和尺寸
    auto data_ptr = tensor.accessor<float, 2>();
    int rows = tensor.size(0);
    int cols = tensor.size(1);

    // 创建相应尺寸的 Mat
    cv::Mat mat(rows, cols, CV_32F);

    // 将 Tensor 数据复制到 Mat
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            mat.at<float>(i, j) = data_ptr[i][j];
        }
    }

    return mat;
}

void Draw_Elevation_Map(cv::Mat elevation, bool b_norm, std::string input_win_name)
{
    int map_size = elevation.rows;

    // 计算有效索引和有效高度
    std::vector<float> valid_elevation;
    std::vector<int> valid_idx;
    std::vector<int> invalid_idx;
    for (int i = 0; i < map_size * map_size; ++i)
    {
        float value = elevation.at<float>(i);
        // if (value < -10)
        // printf("value: %f\n", value);
        if ((!b_norm && value != INVALID_Z) || (b_norm && value != -1))
        {
            valid_elevation.push_back(value);
            valid_idx.push_back(i);
        }
        else
            invalid_idx.push_back(i);
    }

    int valid_num = valid_elevation.size();
    int invalid_num = invalid_idx.size();
    if (valid_num == 0)
    {
        std::cerr << "No valid elevation data found!" << std::endl;
        return;
    }

    // 计算最小和最大高度
    float min_elevation = *std::min_element(valid_elevation.begin(), valid_elevation.end());
    float max_elevation = *std::max_element(valid_elevation.begin(), valid_elevation.end());

    // 计算彩色索引
    std::vector<int> coloridx_int(valid_num);
    for (int i = 0; i < valid_num; ++i)
    {
        float coloridx = (valid_elevation[i] - min_elevation) / (max_elevation - min_elevation) * 639;
        coloridx_int[i] = static_cast<int>(coloridx);
        if (coloridx_int[i] < 0)
        {
            printf("valid_elevation: %lf, min_elevation: %lf, max_elevation: %lf\n", valid_elevation[i], min_elevation, max_elevation);
        }
    }

    // 创建图像
    cv::Mat img(map_size, map_size, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < valid_num; ++i)
    {
        int ID = valid_idx[i];
        int c = ID % map_size;
        int r = ID / map_size;
        img.at<cv::Vec3b>(r, c) = cv::Vec3b(jet_color_map[coloridx_int[i]][2],
                                            jet_color_map[coloridx_int[i]][1],
                                            jet_color_map[coloridx_int[i]][0]);
    }

    for (int i = 0; i < invalid_num; i++)
    {
        int ID = invalid_idx[i];
        int c = ID % map_size;
        int r = ID / map_size;
        img.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 0);
    }

    // 显示图像
    std::string win_name = input_win_name.empty() ? (b_norm ? "after norm elevation" : "before norm elevation") : input_win_name;
    cv::namedWindow(win_name, cv::WINDOW_NORMAL);
    cv::imshow(win_name, img);
    cv::waitKey(2);
}

PathPrediction::PathPrediction(ros::NodeHandle p_node_)
{
    p_node = p_node_;
    Initialize();
    ros_sub_terrain = p_node.subscribe<world_state::TerrainMapComplex>("/world_state/TerrainMapComplex", 1, &PathPrediction::PathHandler, this);
    ros_pub_path = p_node.advertise<sensor_msgs::PointCloud>("Path_predict", 10);
    std::string satellite_path = "./Satellite_Map";
    satellite_patch = new Satellite_Patch(satellite_path);
}
void PathPrediction::Initialize()
{
    msg = new world_state::TerrainMapComplex();

    // 为每个字段赋初值

    msg->local_time = 0.0; // 初始化 local_time
    msg->UTC_time = 0.0;   // 初始化 UTC_time
    msg->message_num = 0;  // 初始化 message_num

    // 初始化 Pose6D 类型的 globalPoseStamped
    msg->globalPoseStamped.x = 0.0;
    msg->globalPoseStamped.y = 0.0;
    msg->globalPoseStamped.z = 0.0;
    msg->globalPoseStamped.azimuth = 0.0;
    msg->globalPoseStamped.pitch = 0.0;
    msg->globalPoseStamped.roll = 0.0;

    // 初始化 geometry_msgs/Pose2D 类型的 localPoseStamped
    msg->localPoseStamped.x = 0.0;
    msg->localPoseStamped.y = 0.0;
    msg->localPoseStamped.theta = 0.0;

    // 初始化 elevation_mu, elevation_sigma, elevation_maxmin, elevation_predict 和 elevation_slope
    for (int i = 0; i < 250000; i++)
    {
        msg->elevation_mu[i] = 0.0;
        msg->elevation_sigma[i] = 0.0;
        msg->elevation_maxmin[i] = 0.0;
        msg->elevation_predict[i] = 0.0;
        msg->elevation_slope[i] = 0.0;
    }

    // 初始化 geometry_msgs/Point32[20] 类型的 path_past
    for (int i = 0; i < 20; i++)
    {
        geometry_msgs::Point32 point;
        point.x = 0.0;
        point.y = 0.0;
        point.z = 0.0;
        msg->path_past[i] = point;
    }

    // 初始化 lidar_h_offset、residual_x 和 residual_y
    msg->lidar_h_offset = 0.0;
    msg->residual_x = 0.0;
    msg->residual_y = 0.0;
}
torch::Tensor extractVec(const std::vector<float> &ori)
{
    std::vector<float> tar(ori.begin(), ori.begin() + width * height);
    torch::Tensor tensor = torch::from_blob(tar.data(), {width, height}, torch::kFloat);
    tensor = tensor.unsqueeze(0).unsqueeze(0);
    return tensor;
}

void PathPrediction::Run()
{
    ros::Rate rate(20);
    torch::Device device(torch::kCUDA);
    torch::jit::script::Module module = torch::jit::load("pathmodel_hexi.pt");

    // // 打印模型参数
    // std::cout << "Model Parameters:" << std::endl;
    // std::ofstream outFile("model_parameters.txt"); // 打开文件
    // auto net_name = module.named_parameters();
    // for (auto n = net_name.begin(); n != net_name.end(); n++)
    // {
    //     // std::string name = param.name;
    //     // std::cout << "Parameter Name: " << name << std::endl;
    //     // std::cout << "Parameter Value: " << param.value()<< std::endl;  // 打印参数值的大小
    //     // outFile << "Parameter Name: " << name << std::endl;
    //     std::cout << "1111111 " << std::endl; // 将参数值写入文件

    //     // 将参数名写入文件
    //     std::cout << "Parameter Value: " << (*n).key() << std::endl; // 将参数值写入文件
    // }
    // outFile.close(); // 关闭文件

    module.to(device);
    std::chrono::steady_clock::time_point t0, t1, t2, t3, t4, t5;
    std::chrono::duration<double> duration;

    while (ros::ok())
    {
        ros::spinOnce();
        // 将5个维度的特征提取300*300区域
        t0 = std::chrono::steady_clock::now();
        torch::Tensor tensor_elevation_mu = ExtractSubtensor(msg->elevation_mu, 300, 300);
        torch::Tensor tensor_elevation_sigma = ExtractSubtensor(msg->elevation_sigma, 300, 300);
        torch::Tensor tensor_elevation_maxmin = ExtractSubtensor(msg->elevation_maxmin, 300, 300);
        torch::Tensor tensor_elevation_predict = ExtractSubtensor(msg->elevation_predict, 300, 300);
        torch::Tensor tensor_elevation_slope = ExtractSubtensor(msg->elevation_slope, 300, 300);
        c10::IntArrayRef tsize = tensor_elevation_maxmin.sizes();

        // Draw_Elevation_Map(tensorToMat(tensor_elevation_predict), false, "maxmin");
        // Draw_Elevation_Map(tensorToMat(tensor_elevation_maxmin), false, "elevation_maxmin");
        // Draw_Elevation_Map(tensorToMat(tensor_elevation_predict), false, "elevation_predict");

        // 将5个[300,300]的feature进行变换得到一个[1,5,300,300]的feature
        torch::Tensor feature_tensors = torch::stack({tensor_elevation_mu, tensor_elevation_sigma, tensor_elevation_maxmin, tensor_elevation_predict, tensor_elevation_slope}, 0);
        torch::Tensor feature_map_batch_before = feature_tensors.unsqueeze(0);

        torch::Tensor feature_map_batch = NormalizeFeatureMap(feature_map_batch_before);

        // Draw_Elevation_Map(tensorToMat(feature_map_batch[0][3]), true, "maxmin_norm");
        t1 = std::chrono::steady_clock::now();
        duration = t1 - t0;
        std::cout << "111 Run " << duration.count() << " seconds." << std::endl;
        // Draw_Elevation_Map(tensorToMat(ExtractSubtensor(msg->elevation_mu, 300, 300)), false, "elevation");

        // 创建past_path_batch的tensor，大小为[1， 20, 2]
        torch::Tensor past_path_batch = torch::zeros({1, 20, 2}, torch::kFloat32);
        for (size_t i = 0; i < msg->path_past.size(); ++i)
        {
            past_path_batch[0][i][0] = msg->path_past[i].x;
            past_path_batch[0][i][1] = msg->path_past[i].y;
        }

        // 创建mask_road_path
        double2D gauss;
        gauss.x = msg->globalPoseStamped.x;
        gauss.y = msg->globalPoseStamped.y;

        double3D current_pose;
        current_pose.x = msg->globalPoseStamped.azimuth;
        // current_pose.x = msg->globalPoseStamped.azimuth - PI / 2;

        current_pose.y = msg->globalPoseStamped.pitch;
        current_pose.z = msg->globalPoseStamped.roll;
        Eigen::Matrix3d R_w_l = YPR2RotationMatrix(current_pose);

        double2D SLAM_blh = xy2blh(gauss);
        cv::Mat satellite_mask_patch;

        satellite_patch->Run3(SLAM_blh, R_w_l, satellite_mask_patch);

        // 调整Mask大小
        cv::Mat resized_mask_road;
        cv::resize(satellite_mask_patch, resized_mask_road, cv::Size(Mask_SIZE, Mask_SIZE));

        // VisMaskRoad(resized_mask_road);

        torch::Tensor mask_road_path = MatToTensor(resized_mask_road);

        std::cout << "mask road get and resize!!!" << std::endl;

        // 创建class_batch
        torch::Tensor class_batch = torch::zeros({3});

        // 推理得到输出
        t2 = std::chrono::steady_clock::now();
        duration = t2 - t1;
        std::cout << "222 Run " << duration.count() << " seconds." << std::endl;

        auto output = module.forward({feature_map_batch.to(device), past_path_batch.to(device), mask_road_path.to(device), class_batch.to(device)});
        t3 = std::chrono::steady_clock::now();
        duration = t3 - t2;
        std::cout << "333 Run " << duration.count() << " seconds." << std::endl;
        torch::Tensor predict_waypoint = output.toTuple()->elements()[0].toTensor();
        // 遍历 predict_waypoint 并填充 waypoints 消息
        for (int i = 0; i < predict_waypoint.size(1); ++i)
        {
            geometry_msgs::Point32 point;
            point.x = predict_waypoint[0][i][0].item().toFloat();
            point.y = predict_waypoint[0][i][1].item().toFloat();
            point.z = 0.0;
            // std::cout << "x:" << point.x << "y:" << point.y << std::endl;
            path_predict.points.push_back(point);
        }
        // 一次性发布所有点
        ros_pub_path.publish(path_predict);
        // test 给定了预测轨迹点
        // VisPredictPath(tensorToMat(feature_map_batch[0][0]), true, "predict_waypoint", predict_waypoint);
        VisPredictPath(tensorToMat(feature_map_batch[0][0]), true, "predict_waypoint", predict_waypoint);

        t4 = std::chrono::steady_clock::now();
        duration = t4 - t0;
        std::cout << "total Run " << duration.count() << " seconds." << std::endl;
        rate.sleep();
    }
}

void PathPrediction::PathHandler(const world_state::TerrainMapComplex::ConstPtr &terrainmsg)
{
    *msg = *terrainmsg;
}

torch::Tensor PathPrediction::MatToTensor(const cv::Mat &mat)
{
    // 检查mat的通道数是否为3
    TORCH_CHECK(mat.channels() == 3, "Mat must have 3 channels (RGB).");

    // 创建与mat大小相同的tensor
    torch::Tensor tensor = torch::from_blob(mat.data, {1, mat.rows, mat.cols, 3}, torch::kByte);
    // std::cout << "tensor size: " << tensor.sizes() << std::endl;

    tensor = tensor.permute({0, 3, 1, 2});
    // // 归一化到[0, 1]范围（如果需要）
    tensor = tensor.toType(torch::kFloat32);
    tensor = tensor / 255.0;

    return tensor;
}

// 归一化函数
torch::Tensor PathPrediction::NormalizeFeatureMap(torch::Tensor input_feature_map)
{

    // 归一化 mu
    torch::Tensor mu = input_feature_map[0][0];
    torch::Tensor valid_mu = mu.masked_select(mu.ne(INVALID_Z));
    float max_mu = torch::max(valid_mu).item<float>();
    float min_mu = torch::min(valid_mu).item<float>();
    mu = torch::where(mu != INVALID_Z, (mu - min_mu) / (max_mu - min_mu), -1);

    // 归一化 sigma
    torch::Tensor sigma = input_feature_map[0][1];
    torch::Tensor valid_sigma = sigma.masked_select(sigma.ne(INVALID_Z));
    float max_sigma = 0.02;
    float min_sigma = torch::min(valid_sigma).item<float>();
    sigma = torch::where(sigma != INVALID_Z, (sigma - min_sigma) / (max_sigma - min_sigma), -1);

    // 归一化 delta_z
    torch::Tensor delta_z = input_feature_map[0][2];
    torch::Tensor valid_delta_z = delta_z.masked_select(delta_z.ne(INVALID_Z));
    float max_delta_z = 1.0;
    float min_delta_z = torch::min(valid_delta_z).item<float>();
    delta_z = torch::where(delta_z != INVALID_Z, (delta_z - min_delta_z) / (max_delta_z - min_delta_z), -1);

    // 归一化 estimated_height
    torch::Tensor estimated_height = input_feature_map[0][3];
    torch::Tensor valid_height = estimated_height.masked_select(estimated_height.ne(INVALID_Z));
    float max_height = torch::max(valid_height).item<float>();
    float min_height = torch::min(valid_height).item<float>();
    estimated_height = torch::where(estimated_height != INVALID_Z, (estimated_height - min_height) / (max_height - min_height), -1);

    // 归一化 normal_angle
    torch::Tensor normal_angle = input_feature_map[0][4];
    torch::Tensor valid_normal_angle = normal_angle.masked_select(normal_angle.ne(INVALID_Z));
    float max_normal_angle = torch::max(valid_normal_angle).item<float>();
    float min_normal_angle = torch::min(valid_normal_angle).item<float>();
    normal_angle = torch::where(normal_angle != INVALID_Z, (normal_angle - min_normal_angle) / (max_normal_angle - min_normal_angle), -1);

    torch::Tensor combined_tensor = torch::stack({mu, sigma, delta_z, estimated_height, normal_angle}, 0);
    torch::Tensor output_feature_map = combined_tensor.unsqueeze(0);

    // 返回归一化后的 feature map
    return output_feature_map;
}

void PathPrediction::VisMaskRoad(cv::Mat mask_road)
{

    std::string win_name = "mask_road";
    cv::namedWindow(win_name, cv::WINDOW_NORMAL);
    cv::imshow(win_name, mask_road);
    cv::waitKey(2);
}

void PathPrediction::VisPredictPath(cv::Mat elevation, bool b_norm, std::string input_win_name, torch::Tensor predict_path)
{
    int map_size = elevation.rows;
    // 计算有效索引和有效高度
    std::vector<float> valid_elevation;
    std::vector<int> valid_idx;
    for (int i = 0; i < map_size * map_size; ++i)
    {
        float value = elevation.at<float>(i);
        if ((!b_norm && value != INVALID_Z) || (b_norm && value != -1))
        {
            valid_elevation.push_back(value);
            valid_idx.push_back(i);
        }
    }

    int valid_num = valid_elevation.size();
    if (valid_num == 0)
    {
        std::cerr << "No valid elevation data found!" << std::endl;
        return;
    }

    // 计算最小和最大高度
    float min_elevation = *std::min_element(valid_elevation.begin(), valid_elevation.end());
    float max_elevation = *std::max_element(valid_elevation.begin(), valid_elevation.end());

    // 计算彩色索引
    std::vector<int> coloridx_int(valid_num);
    for (int i = 0; i < valid_num; ++i)
    {
        float coloridx = (valid_elevation[i] - min_elevation) / (max_elevation - min_elevation) * 639;
        coloridx_int[i] = static_cast<int>(coloridx);
    }

    // 创建图像
    cv::Mat img(map_size, map_size, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < valid_num; ++i)
    {
        int ID = valid_idx[i];
        int c = ID % map_size;
        int r = ID / map_size;
        img.at<cv::Vec3b>(r, c) = cv::Vec3b(jet_color_map[coloridx_int[i]][2],
                                            jet_color_map[coloridx_int[i]][1],
                                            jet_color_map[coloridx_int[i]][0]);
    }
    // 绘制预测路径
    predict_path = predict_path.squeeze(0).to(torch::kCPU);
    int point_num = predict_path.size(0);
    for (int i = 0; i < point_num; ++i)
    {
        float predict_path_x = predict_path[i][0].item<float>();
        float predict_path_y = predict_path[i][1].item<float>();
        // // test
        // predict_path_x = 0.;
        // predict_path_y = i / 1.;
        int c = int(img.cols / 2 + predict_path_x / RESOLUTION);
        int r = int(img.rows / 2 - 1 - predict_path_y / RESOLUTION);
        // 绘制预测路径点
        if (c >= 0 && r >= 0 && c < img.cols && r < img.rows)
        {
            cv::circle(img, cv::Point(c, r), 1, cv::Scalar(255, 255, 255), -1);
        }
    }

    // 显示图像
    std::string win_name;

    win_name = input_win_name;

    cv::namedWindow(win_name, cv::WINDOW_NORMAL);
    cv::imshow(win_name, img);
    cv::waitKey(2);
}

// torch::Tensor PathPrediction::ExtractSubtensor(const boost::array<float, 250000> &array, int width, int height)
// {
//     // 计算起始索引
//     int start_row = (500 - height) / 2;
//     int start_col = (500 - width) / 2;

//     // 构造一个空的张量
//     torch::Tensor tensor = torch::zeros({height, width}, torch::kFloat32);

//     // 将数据复制到张量中
//     for (int i = 0; i < height; i++)
//     {
//         for (int j = 0; j < width; j++)
//         {
//             int index = (start_row + i) * 500 + start_col + j; // 注意这里使用了500
//             tensor.index_put_({i, j}, array[index]);
//         }
//     }

//     return tensor;
// }

torch::Tensor PathPrediction::ExtractSubtensor(const boost::array<float, 250000> &array, int width, int height)
{
    // 计算起始索引
    int start_row = (500 - height) / 2;
    int start_col = (500 - width) / 2;

    // 构造一个空的张量
    torch::Tensor tensor = torch::empty({height, width}, torch::kFloat32);

    // 获取张量数据指针
    float *data_ptr = tensor.data_ptr<float>();

    // 将数据复制到张量中
    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            int index = (start_row + i) * 500 + start_col + j; // 注意这里使用了500
            *(data_ptr + i * width + j) = array[index];
        }
    }

    return tensor;
}
