import open3d as o3d

pcd = o3d.io.read_point_cloud("/home/caliiu/ws_hdl_slam/src/hdl_graph_slam/map/map2f_1108_1.pcd")
# pcd = o3d.io.read_point_cloud("/home/leo/plan_ws/src/hdl_graph_slam/map/example.pcd")
o3d.visualization.draw_geometries([pcd])