import open3d as o3d

# Load a .pcd file
pcd = o3d.io.read_point_cloud("pcd_1 for each object/initial_cup_1(1).pcd")

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])