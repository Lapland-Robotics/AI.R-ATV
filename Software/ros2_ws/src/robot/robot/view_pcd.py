import open3d as o3d
import numpy as np

def visualize_point_cloud(pcd_path, voxel_size=0.05, point_size=5.0, radius=0.1, max_nn=30):
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        print("No points found in the point cloud.")
        return

    print(f"Loaded point cloud with {np.asarray(pcd.points).shape[0]} points.")

    # Optionally downsample the point cloud if voxel_size is provided
    if voxel_size is not None:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        print(f"Downsampled point cloud to {np.asarray(pcd.points).shape[0]} points using voxel_size {voxel_size}")

    # Estimate and normalize normals to improve shading and surface detail
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
    pcd.normalize_normals()

    # Create a coordinate frame for spatial reference
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

    # Create a visualizer window
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Point Cloud Visualizer", width=800, height=600)
    
    # Add geometries to the scene
    vis.add_geometry(pcd)
    vis.add_geometry(coordinate_frame)

    # Adjust rendering options
    render_opt = vis.get_render_option()
    render_opt.background_color = np.asarray([0.1, 0.1, 0.1])  # Dark background for high contrast
    render_opt.point_size = point_size

    # Run the visualizer
    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    # Replace with the path to your point cloud file
    point_cloud_file = "/home/robotics/ATV/Dataset/008957-23-08-15_03-02-2025/lidar.pcd"
    visualize_point_cloud(point_cloud_file, voxel_size=0.05, point_size=5.0)
