<!--
 * @Author: Xiaoxun Zhang
 * @Date: 2025-08-04 19:43:59
 * @LastEditTime: 2025-08-05 22:01:58
 * @Description: 
-->

# pcd_manage
Operations on pcd files: merge, visualize, etc.

## 点云配准部分
综合对比测试对比ICP、GICP、NDT三种算法每种算法都测试两种扰动方式。

* `testWithPerturbedInitialGuessAdvanced` : 给算法提供错误的初始变换猜测, 测试算法从近似初值收敛的能力。

* `testWithPerturbedSourceCloudAdvanced`: 
改变源点云的空间位置, 测试算法在无先验信息下的收敛能力。

### 使用方法——配准参数配置
```cpp
// 创建并配置配准对象
PointCloudRegistration registration;
registration.setInputSource(source); // 源点云(被加上旋转偏差)
registration.setInputTarget(target); // 目标点云
registration.setRegistrationMethod(method);

// 设置配准参数
registration.setMaxIterations(50);               // 最大迭代次数
registration.setMaxCorrespondenceDistance(0.5f); // 最大对应点距离（米）:初始误差不要大于这个值
registration.setTransformationEpsilon(1e-6f);    // 变换收敛阈值
registration.setVoxelSize(0.1f);                 // 体素降采样大小（米）
registration.setInitialGuess(initial_guess);     // 初值

```

### 测试部分函数可施加的扰动
```cpp
 * @param trans_x X轴方向的平移扰动量（米），默认0.3米
 * @param trans_y Y轴方向的平移扰动量（米），默认0.2米
 * @param trans_z Z轴方向的平移扰动量（米），默认0.0米
 * @param rot_z 绕Z轴的旋转扰动角度（弧度），默认0.05弧度（约2.9度）
```