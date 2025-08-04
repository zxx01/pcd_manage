#include <chrono>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "PointCloudRegistration/pointcloud_registration.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// 获取点云包围框
std::pair<Eigen::Vector4f, Eigen::Vector4f> getCloudBoundingBox(const PointCloudT::Ptr& cloud)
{
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    Eigen::Vector4f min_bounds(min_pt.x, min_pt.y, min_pt.z, 1);
    Eigen::Vector4f max_bounds(max_pt.x, max_pt.y, max_pt.z, 1);

    return std::make_pair(min_bounds, max_bounds);
}

// 根据包围框自动生成重叠的切分区域
std::pair<std::pair<Eigen::Vector4f, Eigen::Vector4f>, std::pair<Eigen::Vector4f, Eigen::Vector4f>>
generateOverlapRegions(const PointCloudT::Ptr& cloud, float overlap_ratio = 0.3f)
{
    auto [min_bounds, max_bounds] = getCloudBoundingBox(cloud);

    float x_range = max_bounds[0] - min_bounds[0];
    float y_range = max_bounds[1] - min_bounds[1];
    float z_range = max_bounds[2] - min_bounds[2];

    // 计算重叠长度
    float x_overlap = x_range * overlap_ratio;
    float y_overlap = y_range * overlap_ratio;

    // 区域1：前半部分（在X和Y方向上）
    Eigen::Vector4f min_pt1 = min_bounds;
    Eigen::Vector4f max_pt1(min_bounds[0] + x_range * 0.7f, min_bounds[1] + y_range * 0.7f,
                            max_bounds[2], 1);

    // 区域2：后半部分（与区域1有重叠）
    Eigen::Vector4f min_pt2(min_bounds[0] + x_range * 0.3f, min_bounds[1] + y_range * 0.3f,
                            min_bounds[2], 1);
    Eigen::Vector4f max_pt2 = max_bounds;

    return std::make_pair(std::make_pair(min_pt1, max_pt1), std::make_pair(min_pt2, max_pt2));
}

// 从点云中切取子区域（返回共享指针）
PointCloudT::Ptr cropCloud(const PointCloudT::Ptr& cloud, const Eigen::Vector4f& min_pt,
                           const Eigen::Vector4f& max_pt)
{
    pcl::CropBox<PointT> crop;
    crop.setInputCloud(cloud);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    PointCloudT::Ptr output(new PointCloudT);
    crop.filter(*output);
    return output;
}

/**
 * @brief 生成用于测试的扰动变换矩阵
 *
 * 创建一个包含平移和旋转的4x4变换矩阵，用于测试点云配准算法的鲁棒性。
 * 该矩阵可以作为初始猜测或用于扰动源点云的位姿。
 *
 * @param trans_x X轴方向的平移量（米），默认0.3米
 * @param trans_y Y轴方向的平移量（米），默认0.2米
 * @param trans_z Z轴方向的平移量（米），默认0.0米
 * @param rot_z 绕Z轴的旋转角度（弧度），默认0.05弧度（约2.9度）
 * @return Eigen::Matrix4f 4x4齐次变换矩阵
 *
 * @note 变换矩阵格式：
 *       [R11 R12 R13 tx]
 *       [R21 R22 R23 ty]
 *       [R31 R32 R33 tz]
 *       [ 0   0   0   1]
 *       其中R为旋转矩阵，(tx,ty,tz)为平移向量
 */
Eigen::Matrix4f generatePerturbationMatrix(float trans_x = 0.3f, float trans_y = 0.2f,
                                           float trans_z = 0.0f, float rot_z = 0.05f)
{
    Eigen::Matrix4f perturbation = Eigen::Matrix4f::Identity();

    // 设置平移分量
    perturbation(0, 3) = trans_x;
    perturbation(1, 3) = trans_y;
    perturbation(2, 3) = trans_z;

    // 设置旋转分量（绕Z轴旋转）
    perturbation.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(rot_z, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    return perturbation;
}

/**
 * @brief 使用封装的PointCloudRegistration类测试点云配准算法
 *
 * 执行点云配准测试，包括参数设置、配准执行、结果评估和可视化。
 * 使用封装好的类接口，简化配准流程并提供统一的测试框架。
 *
 * @param source 源点云，需要被配准的点云
 * @param target 目标点云，作为配准基准的点云
 * @param method 配准算法类型（ICP/GICP/NDT）
 * @param method_name 算法名称字符串，用于输出显示
 * @param initial_guess 初始变换猜测矩阵，默认为单位矩阵
 *
 * @note 函数会输出以下信息：
 *       - 配准耗时（毫秒）
 *       - 适应度得分（越小越好）
 *       - 最终变换矩阵
 *       - 收敛状态
 *       - 可视化结果
 *
 * @note 默认参数设置：
 *       - 最大迭代次数：50
 *       - 最大对应点距离：0.5米
 *       - 变换收敛阈值：1e-6
 *       - 体素降采样大小：0.1米
 */
void testRegistrationAdvanced(const PointCloudT::Ptr& source, const PointCloudT::Ptr& target,
                              PointCloudRegistration::RegistrationMethod method,
                              const std::string& method_name,
                              const Eigen::Matrix4f& initial_guess = Eigen::Matrix4f::Identity())
{
    std::cout << "\n=== Testing " << method_name << " (Advanced) ===\n";

    auto start_time = std::chrono::high_resolution_clock::now();

    // 创建并配置配准对象
    PointCloudRegistration registration;
    registration.setInputSource(source);
    registration.setInputTarget(target);
    registration.setRegistrationMethod(method);

    // 设置配准参数
    registration.setMaxIterations(50);               // 最大迭代次数
    registration.setMaxCorrespondenceDistance(0.5f); // 最大对应点距离（米）
    registration.setTransformationEpsilon(1e-6f);    // 变换收敛阈值
    registration.setVoxelSize(0.1f);                 // 体素降采样大小（米）
    registration.setInitialGuess(initial_guess);     // 初始变换猜测

    // 执行配准
    registration.align();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // 输出配准结果
    std::cout << "Time cost: " << duration.count() << " ms\n";
    std::cout << "Fitness score: " << registration.getFitnessScore() << "\n";
    std::cout << "Transformation matrix:\n" << registration.getFinalTransformation() << "\n";
    std::cout << "Converged: " << (registration.hasConverged() ? "Yes" : "No") << "\n";

    // 可视化配准结果
    registration.visualize(method_name + " Registration Result");
}

/**
 * @brief 测试扰动初始猜测的点云配准算法
 *
 * 通过为配准算法提供一个扰动的初始变换猜测来测试算法的鲁棒性和收敛能力。
 * 这种方式模拟了在有初始位姿估计（但不完全准确）情况下的配准性能。
 *
 * @param source 源点云，需要被配准的点云
 * @param target 目标点云，作为配准基准的点云
 * @param method 配准算法类型（ICP/GICP/NDT）
 * @param method_name 算法名称字符串，用于输出显示
 * @param trans_x X轴方向的平移扰动量（米），默认0.3米
 * @param trans_y Y轴方向的平移扰动量（米），默认0.2米
 * @param trans_z Z轴方向的平移扰动量（米），默认0.0米
 * @param rot_z 绕Z轴的旋转扰动角度（弧度），默认0.05弧度（约2.9度）
 *
 * @note 该函数测试的是算法在有初始猜测时的性能：
 *       - 模拟SLAM中运动模型预测的场景
 *       - 测试算法从近似初值收敛到精确解的能力
 *       - 评估初始猜测对配准速度和精度的影响
 *
 * @note 与扰动源点云的区别：
 *       - 这里只是给算法提供一个初始猜测
 *       - 源点云和目标点云的相对位置实际上是对齐的
 *       - 最终变换矩阵应该接近单位矩阵
 */
void testWithPerturbedInitialGuessAdvanced(const PointCloudT::Ptr& source,
                                           const PointCloudT::Ptr& target,
                                           PointCloudRegistration::RegistrationMethod method,
                                           const std::string& method_name, float trans_x = 0.3f,
                                           float trans_y = 0.2f, float trans_z = 0.0f,
                                           float rot_z = 0.05f)
{
    std::cout << "\n=== " << method_name << " with Perturbed Initial Guess (Advanced) ===\n";
    std::cout << "Perturbation: tx=" << trans_x << ", ty=" << trans_y << ", tz=" << trans_z
              << ", rz=" << rot_z << " rad\n";

    // 生成扰动的初始变换猜测
    Eigen::Matrix4f init_guess = generatePerturbationMatrix(trans_x, trans_y, trans_z, rot_z);

    // 使用扰动的初始猜测进行配准测试
    testRegistrationAdvanced(source, target, method, method_name, init_guess);
}

/**
 * @brief 测试扰动源点云的点云配准算法
 *
 * 通过对源点云应用扰动变换来测试配准算法在完全未知初始位姿情况下的性能。
 * 这种方式模拟了真实的配准任务，算法需要从零开始找到正确的变换关系。
 *
 * @param source 源点云，需要被配准的点云
 * @param target 目标点云，作为配准基准的点云
 * @param method 配准算法类型（ICP/GICP/NDT）
 * @param method_name 算法名称字符串，用于输出显示
 * @param trans_x X轴方向的平移扰动量（米），默认0.3米
 * @param trans_y Y轴方向的平移扰动量（米），默认0.2米
 * @param trans_z Z轴方向的平移扰动量（米），默认0.0米
 * @param rot_z 绕Z轴的旋转扰动角度（弧度），默认0.05弧度（约2.9度）
 *
 * @note 该函数测试的是算法在无先验信息时的性能：
 *       - 模拟真实的点云配准任务场景
 *       - 测试算法的全局收敛能力和鲁棒性
 *       - 评估算法处理未知相对位姿的能力
 *
 * @note 与扰动初始猜测的区别：
 *       - 这里实际改变了源点云的空间位置
 *       - 算法必须从单位矩阵开始寻找变换
 *       - 最终变换矩阵应该是扰动变换的逆矩阵
 *
 * @note 输出信息说明：
 *       - Applied perturbation matrix: 应用到源点云的扰动变换
 *       - Expected transformation: 期望的配准结果（扰动的逆变换）
 *       - 可用于验证配准算法的准确性
 */
void testWithPerturbedSourceCloudAdvanced(const PointCloudT::Ptr& source,
                                          const PointCloudT::Ptr& target,
                                          PointCloudRegistration::RegistrationMethod method,
                                          const std::string& method_name, float trans_x = 0.3f,
                                          float trans_y = 0.2f, float trans_z = 0.0f,
                                          float rot_z = 0.05f)
{
    std::cout << "\n=== " << method_name << " with Perturbed Source Cloud (Advanced) ===\n";
    std::cout << "Perturbation: tx=" << trans_x << ", ty=" << trans_y << ", tz=" << trans_z
              << ", rz=" << rot_z << " rad\n";

    // 生成扰动变换矩阵并应用到源点云
    Eigen::Matrix4f perturbation = generatePerturbationMatrix(trans_x, trans_y, trans_z, rot_z);
    PointCloudT::Ptr perturbed_source(new PointCloudT);
    pcl::transformPointCloud(*source, *perturbed_source, perturbation);

    // 输出扰动信息，用于结果验证
    std::cout << "Applied perturbation matrix:\n" << perturbation << "\n";
    std::cout << "Expected transformation (inverse of perturbation):\n"
              << perturbation.inverse() << "\n";

    // 使用单位矩阵作为初始猜测进行配准测试
    testRegistrationAdvanced(perturbed_source, target, method, method_name,
                             Eigen::Matrix4f::Identity());
}

/**
 * @brief 对比测试多种点云配准算法的性能
 *
 * 使用封装的PointCloudRegistration类对比测试ICP、GICP和NDT三种配准算法。
 * 针对每种算法，分别测试扰动初始猜测和扰动源点云两种测试方式，
 * 全面评估算法在不同条件下的性能表现。
 *
 * @param source 源点云，需要被配准的点云
 * @param target 目标点云，作为配准基准的点云
 *
 * @note 测试内容包括：
 *       - ICP算法：扰动初始猜测 + 扰动源点云
 *       - GICP算法：扰动初始猜测 + 扰动源点云
 *       - NDT算法：扰动初始猜测 + 扰动源点云
 *
 * @note 固定扰动参数：
 *       - X轴平移：0.5米
 *       - Y轴平移：0.3米
 *       - Z轴平移：0.1米
 *       - Z轴旋转：0.1弧度（约5.7度）
 *
 * @note 输出格式：
 *       - 按算法类型分组显示结果
 *       - 每个算法显示两种测试方式的对比
 *       - 便于分析不同算法和测试方式的性能差异
 *
 * @see testWithPerturbedInitialGuessAdvanced() 扰动初始猜测测试
 * @see testWithPerturbedSourceCloudAdvanced() 扰动源点云测试
 */
void compareRegistrationMethodsAdvanced(const PointCloudT::Ptr& source,
                                        const PointCloudT::Ptr& target)
{
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "COMPARISON: USING POINTCLOUDREGISTRATION CLASS\n";
    std::cout << std::string(60, '=') << "\n";

    // 定义要测试的配准算法
    std::vector<std::pair<PointCloudRegistration::RegistrationMethod, std::string>> methods = {
        {PointCloudRegistration::RegistrationMethod::ICP, "ICP"},
        {PointCloudRegistration::RegistrationMethod::GICP, "GICP"},
        {PointCloudRegistration::RegistrationMethod::NDT, "NDT"}};

    // 对每种算法进行两种方式的测试
    for (const auto& [method, name] : methods)
    {
        std::cout << "\n" << std::string(40, '-') << "\n";
        std::cout << "Testing " << name << "\n";
        std::cout << std::string(40, '-') << "\n";

        // 方式1：测试扰动初始猜测的配准性能
        testWithPerturbedInitialGuessAdvanced(source, target, method, name, 0.5f, 0.3f, 0.1f, 0.1f);

        // 方式2：测试扰动源点云的配准性能
        testWithPerturbedSourceCloudAdvanced(source, target, method, name, 0.5f, 0.3f, 0.1f, 0.1f);
    }
}

int main()
{
    constexpr float overlap_ratio = 0.3f; // 30%重叠率

    // 1. 加载原始点云
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(
            "/home/zxx/Workspace/pcd_manager_ws/src/pcd_manage/pcd_raw/juxian_cave_5cm.pcd",
            *cloud) == -1)
    {
        std::cerr << "Error loading point cloud file!\n";
        return -1;
    }
    std::cout << "Loaded point cloud with " << cloud->size() << " points.\n";

    // 2. 自动根据包围框生成重叠切分区域
    auto regions = generateOverlapRegions(cloud, overlap_ratio);
    auto [region1, region2] = regions;
    auto [min_pt1, max_pt1] = region1;
    auto [min_pt2, max_pt2] = region2;

    std::cout << "Region 1: (" << min_pt1[0] << "," << min_pt1[1] << "," << min_pt1[2] << ") to ("
              << max_pt1[0] << "," << max_pt1[1] << "," << max_pt1[2] << ")\n";
    std::cout << "Region 2: (" << min_pt2[0] << "," << min_pt2[1] << "," << min_pt2[2] << ") to ("
              << max_pt2[0] << "," << max_pt2[1] << "," << max_pt2[2] << ")\n";

    PointCloudT::Ptr cloud1 = cropCloud(cloud, min_pt1, max_pt1);
    PointCloudT::Ptr cloud2 = cropCloud(cloud, min_pt2, max_pt2);

    std::cout << "Cloud1 size: " << cloud1->size() << " points\n";
    std::cout << "Cloud2 size: " << cloud2->size() << " points\n";

    // 3. 保存切分后的点云（可选）
    // pcl::io::savePCDFile("cloud1.pcd", *cloud1);
    // pcl::io::savePCDFile("cloud2.pcd", *cloud2);

    // 4. 对比测试两种扰动方式
    compareRegistrationMethodsAdvanced(cloud1, cloud2);

    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << "INDIVIDUAL TESTS\n";
    std::cout << std::string(60, '=') << "\n";

    return 0;
}