#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

import yaml

cam_info = CameraInfo()
    

pub = rospy.Publisher('camera_info', CameraInfo, queue_size=10) 

# new_data.height = 480
# new_data.width = 752
# new_data.distortion_model = 'plumb_bob'
# new_data.D = [-0.27959334,  0.07236902,  0.00056648,  0.00004646]
# new_data.K = [458.25468314, 0.0, 357.83920907, 0.0, 456.45047908, 228.12180948, 0.0, 0.0, 1.0]
# new_data.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# new_data.P = [458.25468314, 0.0, 357.83920907, 0.0, 0.0, 456.45047908, 228.12180948, 0.0, 0.0, 0.0, 1.0, 0.0]
# new_data.binning_x = 0
# new_data.binning_y = 0
# new_data.roi.x_offset = 0
# new_data.roi.y_offset = 0
# new_data.roi.height = 480
# new_data.roi.width = 752
# new_data.roi.do_rectify = False

def callback(data):
    global pub
    global cam_info
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)    
    cam_info.header = data.header
    pub.publish(cam_info)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    

    rospy.Subscriber("image_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    rospy.init_node('pub_camera_info_node', anonymous=True)

    calib_file = rospy.get_param('~calib')

    with file(calib_file, 'r') as f:
        params = yaml.load(f)

    cam_info.height = params['height']
    cam_info.width = params['width']
    cam_info.distortion_model = params['distortion_model']
    cam_info.K = params['K']
    cam_info.D = params['D']
    cam_info.R = params['R']
    cam_info.P = params['P']
    cam_info.binning_x = params['binning_x']
    cam_info.binning_y = params['binning_y']
    cam_info.roi.x_offset = params['roi']['x_offset']
    cam_info.roi.y_offset = params['roi']['y_offset']
    cam_info.roi.height = params['roi']['height']
    cam_info.roi.width = params['roi']['width']
    cam_info.roi.do_rectify = params['roi']['do_rectify']


    listener()