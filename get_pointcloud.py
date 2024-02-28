import numpy as np
import open3d as o3d
import pyrealsense2 as rs

# camera configuration
points = rs.points()
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(
    rs.stream.depth, 640, 480, rs.format.z16, 30
)
config.enable_stream(
    rs.stream.color, 640, 480, rs.format.bgr8, 30
)
profile = pipeline.start(config)
# align
align_to = rs.stream.color
align = rs.align(align_to)
try:
    for frame_id in range(20):
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()


    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    profile = aligned_frames.get_profile()
    intrinsics = profile.as_video_stream_profile().get_intrinsics()
    # pinhole camera intrinsic
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    # convert the depth_frame and color_frame into depth_image and color_image respectively
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    # generate the depth map and RGB image
    img_depth = o3d.geometry.Image(depth_image)
    img_color = o3d.geometry.Image(color_image)
    # generate RGB-D map from RGB image and depth map
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth)
    # create point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
    # Conversion of camera coordinate system to world coordinate system
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # point cloud visualization
    o3d.visualization.draw_geometries([pcd])
    o3d.io.write_point_cloud('pcd/cup.pcd', pcd)

finally:
    pipeline.stop()
    print('done')
