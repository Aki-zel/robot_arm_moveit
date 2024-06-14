import open3d as o3d
import numpy as np
import os


class PreProcess:
    def __init__(self, config_path=os.path.join(os.getcwd(), 'preprocess_arg.txt')):
        self.config_path = config_path
        # print(config_path)
        self.load_config()

    def load_config(self):
        # 从文件加载配置
        try:
            with open(self.config_path, 'r') as f:
                lines = f.readlines()
                for line in lines:
                    key, value = line.strip().split('=')
                    setattr(self, key, float(value)
                            if '.' in value else int(value))
        except FileNotFoundError:
            print("打开文件失败，使用默认参数。")
            self.set_default_parameters()

    def set_default_parameters(self):
        # 设置默认参数
        self.pass_through_x_f = 0
        self.pass_through_y_f = 0
        self.pass_through_z_f = 0
        self.subsample_f = 1
        self.platform_remove_f = 1
        self.outlier_remove_f = 1
        # 直通滤波参数
        self.z_min = -0.8
        self.z_max = -0.4
        self.x_left = -0.3
        self.x_right = 0.3
        self.y_up = 0.3
        self.y_down = -0.2
        # 下采样参数
        self.subsampling_voxel_size = 0.003
        # 平面剔除参数
        self.remain_rate = 0.6
        self.remain_points = 1000
        # 去除离群点参数
        self.k_points = 350
        self.thresh = 0.05

    def pass_through(self, cloud_in, field_name):
        # 直通滤波
        # 将点云数据转换为ndarray
        points = np.array(cloud_in.points)

        if field_name == 'z':
            index = np.where((points[:, 2] >= self.z_min) & (
                points[:, 2] <= self.z_max))[0]
        elif field_name == 'x':
            index = np.where((points[:, 0] >= self.x_left) & (
                points[:, 0] <= self.x_right))[0]
        elif field_name == 'y':
            index = np.where((points[:, 1] >= self.y_down) & (
                points[:, 1] <= self.y_up))[0]
        else:
            raise ValueError("Invalid field name")

        # 获得筛选后的点云数据
        cloud_out = cloud_in.select_by_index(index)
        print(
            f"========直通滤波({field_name})前点数：{len(cloud_in.points)}, 直通滤波后点数：{len(cloud_out.points)}========")
        return cloud_out

    def roi_filter(self, cloud_in, x_min, y_min, x_max, y_max):
        # 将点云数据转换为ndarray
        points = np.array(cloud_in.points)

        # 根据ROI进行过滤
        index = np.where(
            (points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
            (points[:, 1] >= y_min) & (points[:, 1] <= y_max)
        )[0]

        cloud_out = cloud_in.select_by_index(index)
        print(
            f"========ROI过滤前点数:{len(cloud_in.points)}, ROI过滤后点数:{len(cloud_out.points)}========")
        return cloud_out

    def subsample(self, cloud_in):
        # 下采样
        cloud_out = cloud_in.voxel_down_sample(self.subsampling_voxel_size)
        print(
            f"========下采样前点数: {len(cloud_in.points)}, 下采样后点数: {len(cloud_out.points)} ========")
        return cloud_out

    def platform_remove(self, cloud_in):
        # 平面剔除
        cloud_in_num = len(cloud_in.points)
        cloud_out = o3d.geometry.PointCloud(cloud_in)

        remain_rate = self.remain_rate
        remain_points = self.remain_points
        while len(cloud_out.points) > remain_rate * cloud_in_num and len(cloud_out.points) > remain_points:

            # 使用平面模型进行分割
            plane_model, inliers = cloud_out.segment_plane(distance_threshold=0.005,
                                                           ransac_n=3,
                                                           num_iterations=1000)
            # 如果没有找到足够的平面内点，停止
            if len(inliers) < remain_points:
                break
            # 移除平面点
            cloud_out = cloud_out.select_by_index(inliers, invert=True)
        # 移除NaN点
        cloud_out.remove_non_finite_points(
            remove_nan=True, remove_infinite=True)
        print(
            f"========平面剔除前点数：{len(cloud_in.points)}, 平面剔除后点数：{len(cloud_out.points)} ========")
        return cloud_out

    def outlier_remove(self, cloud_in):
        # 离群点剔除
        k_points = self.k_points  # 近邻点数
        thresh = self.thresh  # 阈值
        # 创建点云输出
        cloud_out = o3d.geometry.PointCloud(cloud_in)
        # 查找离群点
        cl, ind = cloud_out.remove_radius_outlier(
            nb_points=k_points, radius=thresh)
        # 移除离群点
        cloud_out = cloud_out.select_by_index(ind)
        print(
            f"========离群剔除前点数：{len(cloud_in.points)}, 离群剔除后点数：{len(cloud_out.points)}========")
        return cloud_out

    def surface_rebuild(self, cloud_in):
        # 表面重建
        cloud_in.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
        # 尝试使用Poisson表面重建算法
        with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                cloud_in, depth=8, width=0, scale=1.1, linear_fit=False)
        # 删除低密度的三角面
        densities = np.asarray(densities)
        vertices_to_remove = densities < np.quantile(densities, 0.01)
        mesh.remove_vertices_by_mask(vertices_to_remove)
        # 可视化生成的三角网格
        o3d.visualization.draw_geometries([mesh])
        return mesh

    def preprocess(self, cloud_in):
        print("================= 开始点云预处理 =================")
        if self.pass_through_x_f:
            cloud_in = self.pass_through(cloud_in, 'x')
        if self.pass_through_y_f:
            cloud_in = self.pass_through(cloud_in, 'y')
        if self.pass_through_z_f:
            cloud_in = self.pass_through(cloud_in, 'z')
        if self.subsample_f:
            cloud_in = self.subsample(cloud_in)
        if self.platform_remove_f:
            cloud_in = self.platform_remove(cloud_in)
        if self.outlier_remove_f:
            cloud_in = self.outlier_remove(cloud_in)
        return cloud_in


if __name__ == '__main__':
    cloud_in = o3d.io.read_point_cloud("../open3d_data/scene.pcd")
    preprocess = PreProcess()
    preprocess.preprocess(cloud_in)  # 预处理流程

    # 测试代码
    # print("Min Z:", np.min(np.asarray(cloud_in.points)[:, 2]))
    # print("Max Z:", np.max(np.asarray(cloud_in.points)[:, 2]))
    # cloud_out = preprocess.pass_through(cloud_in, 'z') # 直通滤波

    # cloud_out = preprocess.subsample(cloud_in) # 体素下采样
    # cloud_out1 = preprocess.platform_remove(cloud_in) # 平面剔除
    # cloud_out2 = preprocess.outlier_remove(cloud_out1) # 离群点剔除

    # 将点云添加到列表中
    # geometries = [cloud_out]
    # # 显示点云
    # o3d.visualization.draw_geometries(geometries)
