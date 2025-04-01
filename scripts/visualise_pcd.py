import open3d as o3d
import numpy as np

cloud = o3d.io.read_point_cloud('/scratchdata/organised_pcd.pcd')
print(np.asarray(cloud.points))
print(np.asarray(cloud.points).max(axis=0))
print(np.asarray(cloud.points).shape)
o3d.visualization.draw_geometries([cloud])