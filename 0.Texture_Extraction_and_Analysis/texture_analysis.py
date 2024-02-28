import open3d as o3d
import os
import numpy as np
from scipy.spatial import KDTree

"""
need to modify the value of the following parameters for different .pcd files
    leaf_size,
    kdtree_radius,
    threshold_normal_y,
    threshold_color_var
"""
base_dir = "../3.Filtered_pcd"
file_path = os.path.join(base_dir, "vc_1.pcd")  # path of storaging the point cloud
initial_pcd: o3d.geometry.PointCloud = o3d.io.read_point_cloud(file_path)  # read the .pcd file


# solve the eigenvalues and eigenvectors based on PCA
def PCA(data):
    mean_data = np.mean(data, axis=0)  # normalization
    normal_data = data - mean_data
    H = np.dot(normal_data.T, normal_data)  # solve the covariance matrix
    eigen_vectors, eigen_values, _ = np.linalg.svd(H)  # SVD is utilized to obtain the eigenvalues and eigenvectors of H
    sort = eigen_values.argsort()[::-1]
    eigen_values = eigen_values[sort]
    eigen_vectors = eigen_vectors[:, sort]

    return eigen_values, eigen_vectors  # return the eigenvalues and eigenvectors


initial_points = np.asarray(initial_pcd.points)  # Obtain the 3D coordinates of the point cloud
initial_colors = np.asarray(initial_pcd.colors)  # Get the RGB value of the point cloud (R=G=B, grey-scale map)

leaf_size = 32  # minimum number of KD-tree searches
kdtree_radius = 0.05  # radius of neighbourhood
kdtree = KDTree(initial_points, leafsize=leaf_size)  # initialize the KDTree
neighbor_idx_list = kdtree.query_ball_point(initial_points,
                                            kdtree_radius)  # obtain the index of the neighboring points of each point
point_normals = []  # a list of normal vectors
texture_feature_points = []  # a list of feature points (points that can represent the texture)
non_texture_feature_points = []  # a list of non-feature points
texture_feature_colors = []  # a list of RGB values for feature points
non_texture_feature_colors = []  # a list of RGB values for non-feature points
threshold_normal_y = 0.2  # a threshold for the Y component of the unit normal vector
threshold_color_var = 0.002  # a threshold for the variance of the grey value

for i in range(len(neighbor_idx_list)):
    neighbor_idx = neighbor_idx_list[i]  # obtain the index of the neighbor points of the ith point
    neighbor_points = initial_points[neighbor_idx]  # obtain the 3D coordinates of the neighboring points
    neighbor_colors = initial_colors[neighbor_idx]  # obtain the color values of the neighboring points
    eigen_values, eigen_vectors = PCA(neighbor_points)  # Principal Component Analysis (PCA) is used to obtain the eigenvalues and eigenvectors of the neighboring points
    normal = eigen_vectors[:, 2]  # the smallest eigenvalue corresponds to the normal vector
    if abs(normal[1]) / np.sqrt(normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2) >= threshold_normal_y and np.std(neighbor_colors[:, 0]) ** 2 >= threshold_color_var:
        # the determination conditions of texture feature points are as follows:
        # the Y component of unit normal vector is greater than or equal to threshold_normal_y,
        # and the color variance of neighboring points is greater than or equal to threshold_color_var
        texture_feature_points.append(initial_points[i])
        texture_feature_colors.append(initial_colors[i])
    else:
        non_texture_feature_points.append(initial_points[i])
        non_texture_feature_colors.append(initial_colors[i])
    point_normals.append(normal)

point_normals = np.array(point_normals, dtype=np.float64)
initial_pcd.normals = o3d.utility.Vector3dVector(point_normals)
texture_feature_points = np.array(texture_feature_points, dtype=np.float64)
texture_feature_colors = np.array(texture_feature_colors, dtype=np.float64)
texture_pcd = o3d.geometry.PointCloud()  # point clouds that contains texture features
texture_pcd.points = o3d.utility.Vector3dVector(texture_feature_points)
non_texture_pcd = o3d.geometry.PointCloud()  # point clouds that do not contain texture features
non_texture_pcd.points = o3d.utility.Vector3dVector(non_texture_feature_points)
non_texture_pcd.colors = o3d.utility.Vector3dVector(non_texture_feature_colors)
height_min = np.min(texture_feature_colors[:, 1])  # the minimum value of the texture point cloud color
height_max = np.max(texture_feature_colors[:, 1])  # the maximum value of the texture point cloud color
delta_c = np.abs(height_max - height_min) / 255
colors = np.zeros((texture_feature_colors.shape[0], 3))
for i in range(texture_feature_colors.shape[0]):
    color_n = (texture_feature_colors[i, 1] - height_min) / delta_c
    if color_n <= 255:
        colors[i, :] = [0, color_n / 255, 1]
    else:
        colors[i, :] = [color_n / 255, 0, 1]

texture_pcd.colors = o3d.utility.Vector3dVector(colors)
non_texture_pcd.paint_uniform_color([0, 1, 0])
# visualization
o3d.visualization.draw_geometries([texture_pcd, non_texture_pcd], point_show_normal=False)
