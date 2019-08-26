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

    cam_info.height = params['cam0']['resolution'][1]
    cam_info.width = params['cam0']['resolution'][0]
    cam_info.distortion_model = 'plumb_bob'
    # cam_info.K = params['K']
    # cam_info.P = params['P']
    cam_info.D = params['cam0']['distortion_coeffs']
    cam_info.P = [0] * 12
    cam_info.R = [0] * 9
    cam_info.K = [0] * 9

    cam_info.K[0] = params['cam0']['intrinsics'][0]
    cam_info.K[2] = params['cam0']['intrinsics'][2]
    cam_info.K[4] = params['cam0']['intrinsics'][1]
    cam_info.K[5] = params['cam0']['intrinsics'][3]
    cam_info.K[8] = 1

    cam_info.R[0] = 1
    cam_info.R[4] = 1
    cam_info.R[8] = 1

    cam_info.P[0] = params['cam0']['intrinsics'][0]
    cam_info.P[2] = params['cam0']['intrinsics'][2]
    cam_info.P[5] = params['cam0']['intrinsics'][1]
    cam_info.P[6] = params['cam0']['intrinsics'][3]
    cam_info.P[10] = 1



    cam_info.binning_x = 0
    cam_info.binning_y = 0
    cam_info.roi.x_offset = 0
    cam_info.roi.y_offset = 0
    cam_info.roi.height = params['cam0']['resolution'][1]
    cam_info.roi.width = params['cam0']['resolution'][0]
    cam_info.roi.do_rectify = False


    listener()