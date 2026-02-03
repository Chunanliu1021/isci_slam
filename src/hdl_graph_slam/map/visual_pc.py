import open3d as o3d
import numpy as np

# 讀取點雲
point_cloud = o3d.io.read_point_cloud("map2F_0717_velodyne_longer2.pcd")

# 獲取點的座標
points = np.asarray(point_cloud.points)
# print("點雲中的座標：")
# print(points)

# 可視化三軸
axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2, origin=[0, 0, 0])

# 定義長方體過濾條件
x_min, x_max = 0.3, 1.8
y_min, y_max = -0.25, 0.15
z_min, z_max = 0.04, 0.3

# 過濾出在長方體內的點
filtered_points = points[
    (points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
    (points[:, 1] >= y_min) & (points[:, 1] <= y_max) &
    (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
]

# 創建新的點雲對象
filtered_point_cloud = o3d.geometry.PointCloud()
filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)

# 可視化原始點雲和過濾後的點雲
o3d.visualization.draw_geometries([point_cloud, axis_pcd])
# o3d.visualization.draw_geometries([filtered_point_cloud, axis_pcd])
