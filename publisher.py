import roslib
import os
import pdb
import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32MultiArray
# Testing below
# import time
# import open3d as o3d

LABELS_REMAP = np.array([
    0, # Free
    1, # Building
    2, # Barrier
    3, # Other
    4, # Pedestrian
    5, # Pole or Traffic Light/Sign
    7, # Roadline -> Road
    7, # Road
    8, # Sidewalk
    9, # Vegetation
    10, # Vehicles
    2, # Wall -> Barrier
    5, # Traffic Sign -> Pole
    6, # Sky -> Other
    11, # Ground
    6, # Bridge -> Other
    6, # Railtrack -> Other
    2, # GuardRail -> Barrier
    5, # Traffic Light -> Pole
    6, # Static -> Other
    6, # Dynamic -> Other
    6, # Water -> Other
    11, # Terrain -> Ground
])  

LABEL_COLORS = np.array([
    (0, 0, 0), # None
    (70, 70, 70),    # Building
    (100, 40, 40),   # Fences
    (55, 90, 80),    # Other
    (255, 255, 0),   # Pedestrian
    (153, 153, 153), # Pole
    (157, 234, 50),  # RoadLines
    (0, 0, 255),  # Road
    (255, 255, 255),  # Sidewalk
    (0, 155, 0),  # Vegetation
    (255, 0, 0),     # Vehicle
    (102, 102, 156), # Wall
    (220, 220, 0),   # TrafficSign
    (70, 130, 180),  # Sky
    (0, 0, 0),     # Ground
    (150, 100, 100), # Bridge
    (230, 150, 140), # RailTrack
    (180, 165, 180), # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160), # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),   # Water
    (145, 170, 100), # Terrain
]) / 255.0 # normalize each channel [0-1] since is what Open3D uses

"""
Publish Format
0:  Frame index
1:  Number of points
2:  Number of classes per point
3a-3b: Flattened xyz coordinates for all points
3b-3c: Flattened softmax probabilities for all points
"""
def talker(coords_fp, preds_fp, poses_fp, pkg_name, start_idx=450):
    pub = rospy.Publisher('floats', Float32MultiArray,queue_size=1)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(0.3) # This shoud be set so that it releases at a rate slower than the insertion of points, but making it as fast as possible will change the bottleneck of the program, tune it as mcuh as you want.  
    
    # pub_MotionSC = rospy.Publisher('MotionSC_mapper', MarkerArray, queue_size=10)

    prediction_files    = os.listdir(preds_fp)
    coord_files         = os.listdir(coords_fp)
    prediction_files    = sorted(prediction_files)
    coord_files         = sorted(coord_files)

    with open(poses_fp, 'rb') as f:
        poses = np.loadtxt(f, dtype=np.float64) 

    for idx in range(start_idx, len(coord_files)):
        # MotionSC_markers.markers.clear()
        pred_file   = os.path.join(preds_fp, prediction_files[idx])
        coord_file  = os.path.join(coords_fp, coord_files[idx])

        # Open predictions and cooresponding xyz coordinates
        predsf = None
        coordsf= None
        try:
            print("opening pred file ", pred_file)
            predsf  = open(pred_file, 'rb')
            print("opening coord file ", coord_file)
            coordsf = open(coord_file, 'rb')
        except Exception as e:
            print("Error encountered while opening file ", e)

        preds = np.array(np.load(predsf), dtype=np.float64)
        coords= np.array(np.load(coordsf), dtype=np.float64)

        data_info   = [idx, preds.shape[0]*preds.shape[1]*preds.shape[2], preds.shape[3] ]
        pose_mat    = poses[idx].reshape(3, 4) # rot | trans

        new_coords = np.zeros(coords.shape)
        new_coords[:, 0] = coords[:, 1]
        new_coords[:, 1] = coords[:, 0]
        new_coords[:, 2] = coords[:, 2]
        coords = (pose_mat[0:3, 0:3]@new_coords.T).T + pose_mat[:3, 3]

        if pkg_name == "nn_semantic_bki":
            flat_preds = preds.reshape(-1).tolist()
            flat_coords= coords.reshape(-1).tolist()

            final_data = data_info + flat_coords + flat_preds
            message = Float32MultiArray(data=final_data)
        elif pkg_name == "semantic_bki":
            max_preds = np.argmax(preds, axis=3)
            flat_preds = max_preds.reshape(-1).tolist()
            flat_coords = coords.reshape(-1).tolist()

            final_data = data_info + flat_coords + flat_preds
            message = Float32MultiArray(data=final_data)

        pub.publish(message)
        r.sleep()

if __name__ == '__main__':
    # Change BASE_DIR to parent folder for Coords and Preds directories
    BASE_DIR = os.path.join('data', 'carla_townheavy')
    coords_fp   = os.path.join(BASE_DIR, 'coords')
    preds_fp    = os.path.join(BASE_DIR, 'preds')
    poses_fp    = os.path.join(BASE_DIR, 'poses.txt')

    # main(coords_fp, preds_fp, poses_fp)
    talker(coords_fp, preds_fp, poses_fp, pkg_name="nn_semantic_bki", start_idx=0)

    # with open('../catkin_ws/src/BKINeuralNet/data/carla_townheavy/labels/000000.label', 'rb') as f:
    #     label = np.fromfile(f, dtype=np.uint32)
    #     pdb.set_trace()
    # pdb.set_trace()
    # print(poses) 


# Visualizer for debugging
# def main(coords_fp, preds_fp, poses_fp):
#     vis = o3d.visualization.Visualizer()
#     try: 
#         sensor = 0
#         vis.create_window(
#             window_name='Segmented Scene',
#             width=960,
#             height=540,
#             left=480,
#             top=270)
#         vis.get_render_option().background_color = [0.0, 0.0, 0.0]
#         vis.get_render_option().point_size = 3

#         # Load frames
#         frame = 450
#         point_list = talker(coords_fp, preds_fp, poses_fp, frame)
#         geometry = o3d.geometry.PointCloud(point_list)
#         vis.add_geometry(geometry)
#         while True:
#             print("frame:", frame)

#             new_list = talker(coords_fp, preds_fp, poses_fp, frame)
#             point_list = point_list + new_list
#             # point_list = gen_points(load_dir, frame)
            
#             geometry.points = point_list.points
#             geometry.colors = point_list.colors

#             vis.update_geometry(geometry)
            
#             for i in range(10):
#                 vis.poll_events()
#                 vis.update_renderer()
#                 time.sleep(0.005)

#             frame += 1
    
#     finally:
#         vis.destroy_window()