/*
 * @Author: Xiaoxun Zhang
 * @Date: 2025-08-04 19:58:56
 * @LastEditTime: 2025-08-04 22:10:06
 * @Description:
 */
#ifndef POINT_CLOUD_REGISTRATION_H
#define POINT_CLOUD_REGISTRATION_H

#include <memory>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PointCloudRegistration
{
  public:
    // 配准算法类型
    enum class RegistrationMethod
    {
        ICP,
        GICP,
        NDT
    };

    // 构造函数
    PointCloudRegistration();

    // 设置输入点云
    void setInputSource(const PointCloudT::Ptr& source);
    void setInputTarget(const PointCloudT::Ptr& target);

    // 设置配准算法参数
    void setRegistrationMethod(RegistrationMethod method);
    void setMaxIterations(int max_iter);
    void setMaxCorrespondenceDistance(float distance);
    void setTransformationEpsilon(float epsilon);
    void setEuclideanFitnessEpsilon(float epsilon);
    void setVoxelSize(float size);
    void setInitialGuess(const Eigen::Matrix4f& initial_guess);

    // 执行配准
    void align();

    // 获取结果
    PointCloudT::Ptr getAlignedCloud();
    Eigen::Matrix4f getFinalTransformation();
    float getFitnessScore();
    bool hasConverged();

    // 可视化结果
    void visualize(const std::string& title = "Registration Result");

  private:
    // 内部方法
    void downsample(PointCloudT::Ptr& cloud);
    void computeNormals(PointCloudT::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals);

    // 点云数据
    PointCloudT::Ptr source_;
    PointCloudT::Ptr target_;
    PointCloudT::Ptr aligned_;

    // 配准参数
    RegistrationMethod method_;
    int max_iter_;
    float max_correspondence_distance_;
    float transformation_epsilon_;
    float euclidean_fitness_epsilon_;
    float voxel_size_;
    Eigen::Matrix4f initial_guess_;

    // 配准结果
    Eigen::Matrix4f final_transformation_;
    float fitness_score_;
    bool converged_;
};

#endif // POINT_CLOUD_REGISTRATION_H