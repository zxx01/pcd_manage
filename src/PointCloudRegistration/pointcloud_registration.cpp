/*
 * @Author: Xiaoxun Zhang
 * @Date: 2025-08-04 20:09:51
 * @LastEditTime: 2025-08-04 21:53:49
 * @Description:
 */

#include "PointCloudRegistration/pointcloud_registration.h"
#include <pcl/visualization/pcl_visualizer.h>

PointCloudRegistration::PointCloudRegistration()
    : method_(RegistrationMethod::ICP), max_iter_(50), max_correspondence_distance_(0.1f),
      transformation_epsilon_(1e-6f), euclidean_fitness_epsilon_(1e-6f), voxel_size_(0.01f),
      initial_guess_(Eigen::Matrix4f::Identity()),
      fitness_score_(std::numeric_limits<float>::max()), converged_(false)
{
}

void PointCloudRegistration::setInputSource(const PointCloudT::Ptr& source) { source_ = source; }

void PointCloudRegistration::setInputTarget(const PointCloudT::Ptr& target) { target_ = target; }

/**
 * @brief 设置点云配准算法类型
 *
 * 选择用于点云配准的算法类型，不同算法适用于不同的应用场景。
 * ICP适用于一般配准，GICP适用于需要考虑点云法向量的场景，NDT适用于大场景配准。
 *
 * @param method 配准算法类型
 *               ICP: 迭代最近点算法，适用于大多数配准任务
 *               GICP: 广义迭代最近点算法，考虑点的法向量信息，精度更高
 *               NDT: 正态分布变换算法，适用于大场景和噪声较多的环境
 */
void PointCloudRegistration::setRegistrationMethod(RegistrationMethod method) { method_ = method; }
/**
 * @brief 设置配准算法的最大迭代次数
 *
 * 控制算法停止的迭代次数上限，防止算法无限迭代，控制计算时间。
 * 迭代次数越多，精度可能越高，但计算时间增加。
 *
 * @param max_iter 最大迭代次数
 *                 建议值：30-100（室内场景），100-200（复杂场景），20-30（实时应用）
 */
void PointCloudRegistration::setMaxIterations(int max_iter) { max_iter_ = max_iter; }

/**
 * @brief 设置点对匹配的最大允许距离
 *
 * 超过此距离的点对不参与配准计算，用于过滤异常匹配。
 * 阈值太大会引入错误匹配，太小会丢失有效匹配。
 *
 * @param distance 最大对应点距离（米）
 *                 建议值：高精度点云0.1-0.3米，一般点云0.3-0.8米，稀疏点云0.5-1.5米
 */
void PointCloudRegistration::setMaxCorrespondenceDistance(float distance)
{
    max_correspondence_distance_ = distance;
}

/**
 * @brief 设置变换矩阵的收敛阈值
 *
 * 当两次迭代间变换矩阵变化小于此值时认为已收敛，算法停止。
 * 值越小精度越高，但可能增加迭代次数。
 *
 * @param epsilon 变换收敛阈值
 *                建议值：1e-8（高精度），1e-6（一般应用），1e-4（实时应用）
 */
void PointCloudRegistration::setTransformationEpsilon(float epsilon)
{
    transformation_epsilon_ = epsilon;
}

/**
 * @brief 设置欧几里得适应度的收敛阈值
 *
 * 当两次迭代间适应度变化小于此值时认为已收敛。
 * 主要用于ICP算法，控制基于点云匹配质量的收敛条件。
 *
 * @param epsilon 欧几里得适应度收敛阈值
 *                建议值：1e-6（一般情况），1e-8（高精度要求）
 */
void PointCloudRegistration::setEuclideanFitnessEpsilon(float epsilon)
{
    euclidean_fitness_epsilon_ = epsilon;
}

/**
 * @brief 设置体素降采样的网格大小
 *
 * 用于减少点云密度，提高计算效率，过滤噪声。
 * 体素越小保留细节越多但计算量大，越大则相反。
 *
 * @param size 体素网格边长（米）
 *             建议值：高精度场景0.01-0.03米，一般场景0.03-0.08米，大范围场景0.05-0.15米
 */
void PointCloudRegistration::setVoxelSize(float size) { voxel_size_ = size; }

/**
 * @brief 设置配准算法的初始变换猜测
 *
 * 提供配准的初始估计，帮助算法快速收敛到全局最优解。
 * 好的初始猜测可以显著提高配准的成功率和速度。
 *
 * @param initial_guess 4x4初始变换矩阵
 *                      建议值：Identity（无先验信息）或从其他传感器/算法获得的粗略估计
 */
void PointCloudRegistration::setInitialGuess(const Eigen::Matrix4f& initial_guess)
{
    initial_guess_ = initial_guess;
}

void PointCloudRegistration::downsample(PointCloudT::Ptr& cloud)
{
    if (voxel_size_ <= 0.0f)
        return;
    PointCloudT::Ptr downsampled(new PointCloudT);
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_filter.filter(*downsampled);
    cloud = downsampled;
}

void PointCloudRegistration::computeNormals(PointCloudT::Ptr& cloud,
                                            pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    ne.setKSearch(10);
    ne.compute(*normals);
}

void PointCloudRegistration::align()
{
    if (!source_ || !target_)
    {
        std::cerr << "Error: Input clouds are not set!" << std::endl;
        return;
    }

    // 降采样
    PointCloudT::Ptr source_downsampled(new PointCloudT(*source_));
    PointCloudT::Ptr target_downsampled(new PointCloudT(*target_));
    downsample(source_downsampled);
    downsample(target_downsampled);

    // 执行配准
    switch (method_)
    {
    case RegistrationMethod::ICP:
    {
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(source_downsampled);
        icp.setInputTarget(target_downsampled);
        icp.setMaximumIterations(max_iter_);
        icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
        icp.setTransformationEpsilon(transformation_epsilon_);
        icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
        aligned_.reset(new PointCloudT);
        icp.align(*aligned_, initial_guess_);
        final_transformation_ = icp.getFinalTransformation();
        fitness_score_ = icp.getFitnessScore();
        converged_ = icp.hasConverged();
        break;
    }
    case RegistrationMethod::GICP:
    {
        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
        gicp.setInputSource(source_downsampled);
        gicp.setInputTarget(target_downsampled);
        gicp.setMaximumIterations(max_iter_);
        gicp.setMaxCorrespondenceDistance(max_correspondence_distance_);
        gicp.setTransformationEpsilon(transformation_epsilon_);
        aligned_.reset(new PointCloudT);
        gicp.align(*aligned_, initial_guess_);
        final_transformation_ = gicp.getFinalTransformation();
        fitness_score_ = gicp.getFitnessScore();
        converged_ = gicp.hasConverged();
        break;
    }
    case RegistrationMethod::NDT:
    {
        pcl::NormalDistributionsTransform<PointT, PointT> ndt;
        ndt.setInputSource(source_downsampled);
        ndt.setInputTarget(target_downsampled);
        ndt.setMaximumIterations(max_iter_);
        ndt.setResolution(1.0f); // NDT体素分辨率
        ndt.setTransformationEpsilon(transformation_epsilon_);
        aligned_.reset(new PointCloudT);
        ndt.align(*aligned_, initial_guess_);
        final_transformation_ = ndt.getFinalTransformation();
        fitness_score_ = ndt.getFitnessScore();
        converged_ = ndt.hasConverged();
        break;
    }
    default:
        throw std::runtime_error("Unknown registration method!");
    }
}

PointCloudT::Ptr PointCloudRegistration::getAlignedCloud() { return aligned_; }

Eigen::Matrix4f PointCloudRegistration::getFinalTransformation() { return final_transformation_; }

float PointCloudRegistration::getFitnessScore() { return fitness_score_; }

bool PointCloudRegistration::hasConverged() { return converged_; }

void PointCloudRegistration::visualize(const std::string& title)
{
    if (!aligned_ || !target_)
    {
        std::cerr << "Error: No aligned cloud or target cloud to visualize!" << std::endl;
        return;
    }

    pcl::visualization::PCLVisualizer viewer(title);
    viewer.addPointCloud<PointT>(target_, "target", 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0,
                                            "target");
    viewer.addPointCloud<PointT>(aligned_, "aligned", 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0,
                                            "aligned");
    viewer.spin();
}