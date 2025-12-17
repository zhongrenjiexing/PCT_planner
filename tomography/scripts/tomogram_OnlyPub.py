# conda env: unitreerl

import os
import sys
import pickle
import numpy as np

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

sys.path.append('../')
from config import POINT_FIELDS_XYZI, GRID_POINTS_XYZI
from config import Config

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


class TomogramPublisher(object):
    def __init__(self, pickle_file):
        # 加载 pickle 文件
        pickle_path = rsg_root + pickle_file
        rospy.loginfo("Loading tomogram from: %s", pickle_path)
        
        with open(pickle_path, 'rb') as handle:
            data_dict = pickle.load(handle)
        
        # 提取数据
        tomogram_data = data_dict['data'].astype(np.float32)  # 转换为 float32
        self.resolution = data_dict['resolution']
        self.center = data_dict['center']
        self.slice_h0 = data_dict['slice_h0']
        self.slice_dh = data_dict['slice_dh']
        
        # 从数据中提取各个层
        # data 的形状应该是 (5, n_slice, dim_x, dim_y)
        # 顺序: layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c
        layers_t = tomogram_data[0]
        trav_grad_x = tomogram_data[1]
        trav_grad_y = tomogram_data[2]
        layers_g = tomogram_data[3]
        layers_c = tomogram_data[4]
        
        self.layers_t = layers_t
        self.layers_g = layers_g
        self.layers_c = layers_c
        
        # 从 layers_g 的形状确定维度
        self.n_slice = layers_g.shape[0]
        self.map_dim_x = layers_g.shape[1]
        self.map_dim_y = layers_g.shape[2]
        
        rospy.loginfo("Loaded tomogram:")
        rospy.loginfo("  Resolution: %.3f", self.resolution)
        rospy.loginfo("  Center: [%.2f, %.2f]", self.center[0], self.center[1])
        rospy.loginfo("  Slice h0: %.2f", self.slice_h0)
        rospy.loginfo("  Slice dh: %.2f", self.slice_dh)
        rospy.loginfo("  Num slices: %d", self.n_slice)
        rospy.loginfo("  Map dim_x: %d", self.map_dim_x)
        rospy.loginfo("  Map dim_y: %d", self.map_dim_y)
        
        # 生成可视化原型点
        self.VISPROTO_I, self.VISPROTO_P = \
            GRID_POINTS_XYZI(self.resolution, self.map_dim_x, self.map_dim_y)
        
        # 初始化 ROS 发布器
        self.initROS()
        
        # 发布数据
        self.publishLayers(self.layer_G_pub_list, layers_g, layers_t)
        self.publishLayers(self.layer_C_pub_list, layers_c, None)
        self.publishTomogram(layers_g, layers_t)
        
        rospy.loginfo("Tomogram published to ROS topics")

    def initROS(self):
        cfg = Config()
        self.map_frame = cfg.ros.map_frame

        layer_G_topic = cfg.ros.layer_G_topic
        layer_C_topic = cfg.ros.layer_C_topic
        
        self.layer_G_pub_list = []
        self.layer_C_pub_list = []
        for i in range(self.n_slice):
            layer_G_pub = rospy.Publisher(layer_G_topic + str(i), PointCloud2, latch=True, queue_size=1)
            self.layer_G_pub_list.append(layer_G_pub)
            layer_C_pub = rospy.Publisher(layer_C_topic + str(i), PointCloud2, latch=True, queue_size=1)
            self.layer_C_pub_list.append(layer_C_pub)

        tomogram_topic = cfg.ros.tomogram_topic
        self.tomogram_pub = rospy.Publisher(tomogram_topic, PointCloud2, latch=True, queue_size=1)

    def publishLayers(self, pub_list, layers, color=None):
        header = Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = self.map_frame

        layer_points = self.VISPROTO_P.copy()
        layer_points[:, :2] += self.center

        for i in range(layers.shape[0]):
            layer_points[:, 2] = layers[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            if color is not None:
                layer_points[:, 3] = color[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            else:
                layer_points[:, 3] = 1.0
        
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            points_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, valid_points)
            pub_list[i].publish(points_msg) 

    def publishTomogram(self, layers_g, layers_t):
        header = Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = self.map_frame

        n_slice = layers_g.shape[0]
        vis_g = layers_g.copy()
        vis_t = layers_t.copy() 
        layer_points = self.VISPROTO_P.copy()
        layer_points[:, :2] += self.center

        global_points = None
        for i in range(n_slice - 1):
            mask_h = (vis_g[i + 1] - vis_g[i]) < self.slice_dh
            vis_g[i, mask_h] = np.nan
            vis_t[i + 1, mask_h] = np.minimum(vis_t[i, mask_h], vis_t[i + 1, mask_h])
            layer_points[:, 2] = vis_g[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            layer_points[:, 3] = vis_t[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            if global_points is None:
                global_points = valid_points
            else:
                global_points = np.concatenate((global_points, valid_points), axis=0)

        layer_points[:, 2] = vis_g[-1, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
        layer_points[:, 3] = vis_t[-1, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
        valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
        global_points = np.concatenate((global_points, valid_points), axis=0)
        
        points_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, global_points)
        self.tomogram_pub.publish(points_msg)


if __name__ == '__main__':
    rospy.init_node('tomogram_publisher', anonymous=True)
    
    # 默认使用 .pickle
    # pickle_file = "/rsc/tomogram/6l7l_mid360_leveled_clear.pickle"
    pickle_file = "/rsc/tomogram/6L7L_Add613_-ready-to-pctplanner.pickle"
    
    # 可以通过命令行参数指定其他 pickle 文件
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--pickle', type=str, default=pickle_file, 
                        help='Path to pickle file (relative to project root)')
    args = parser.parse_args()
    
    publisher = TomogramPublisher(args.pickle)
    
    rospy.spin()

