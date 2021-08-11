from math import floor

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Imu

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Vector3Stamped

from prophesee_event_msgs.msg import EventArray

import argparse
import rosbag
import gnupg

LIDAR_HZ = 100

def Parse():
    parser = argparse.ArgumentParser(description='config_io')
    parser.add_argument("-o", "--output", type=str,
                        help='set output path', default="temp.bag")
    parser.add_argument("-i", "--input", type=str,
                        help='set input path')
    parser.add_argument('--imu',type=str, help="Set IMU topic", default="/imu/data")
    parser.add_argument('--lidar',type=str, help="Set LIdar Point Cloud topic",
                        default="/os_cloud_node/points")
    parser.add_argument('--rgbl',type=str, help="Set Right RGB camera topic",
                        default="/camera/image_mono")
    return parser.parse_args()


def DumpEarlist(buffer_vec):
    buffer_vec.sort(buffer_vec, key=lambda x:x[0][0], reverse=False)
    


def main():
    args = Parse()
    bag_in = rosbag.Bag(args.input, "r")

    buffer = [[], [], []]

    lidar_start_offset_secs = 0
    lidar_start_offset_nsecs = 0
    lidar_temp_count = -1
    lidar_temp = PointCloud2()
    lidar_started = False

    imu_started_unix_time = 0
    imu_started = False

    rgbl_temp_t = 0



    with rosbag.Bag(args.output, 'w') as bag_out:
        for topic, msg, t in bag_in.read_messages():
            if topic == args.lidar:
                if not lidar_started:
                    if lidar_temp.header.stamp.secs == msg.header.stamp.secs:
                        lidar_temp_count += 1
                    else:
                        if lidar_temp_count == 0 and lidar_temp.header.stamp.nsecs < 1e8:
                            lidar_started = True
                            lidar_start_offset_secs = lidar_temp.header.stamp.secs
                            lidar_start_offset_nsecs = lidar_temp.header.stamp.nsecs * 1e-9
                            time_diff = 1 / LIDAR_HZ * (msg.header.stamp.secs - lidar_start_offset_secs) +\
                                1e-9 * (msg.header.stamp.nsecs - lidar_start_offset_nsecs)
                            if imu_started:
                                buffer[0].append([imu_started_unix_time, lidar_temp, args.lidar])
                                buffer[0].append([imu_started_unix_time + time_diff, msg, args.lidar])
                            else:
                                buffer[0].append([0, lidar_temp, args.lidar])
                                buffer[0].append([time_diff, msg, args.lidar])

                        else:
                            lidar_temp_count = 0
                            lidar_temp.header.stamp.to_sec()
                            lidar_temp = msg
                else:
                    time_diff = 1 / LIDAR_HZ * (msg.header.stamp.secs - lidar_start_offset_secs) +\
                        1e-9 * (msg.header.stamp.nsecs - lidar_start_offset_nsecs)
                    if imu_started:
                        buffer[0].append([imu_started_unix_time + time_diff, msg, args.lidar])
                    else:
                        buffer[0].append([time_diff, msg, args.lidar])

            elif topic == args.imu:
                rgbl_temp_t = msg.header.stamp.to_sec()
                if not imu_started:
                    imu_started_unix_time = rgbl_temp_t
                    for pc in buffer[0]:
                        pc[0] = pc[0] + imu_started_unix_time
                    imu_started = True
                buffer[1].append([rgbl_temp_t, msg, args.imu])
                

            elif topic == args.rgbl:
                if imu_started:
                    buffer[2].append([rgbl_temp_t, msg, args.rgbl])

            while len(buffer[0]) and len(buffer[1]) and len(buffer[2]):
                vec = [k[0][0] for k in buffer]
                dump = vec.index(min(vec))
                buffer[dump][0][1].header.stamp.secs = floor(buffer[dump][0][0])
                buffer[dump][0][1].header.stamp.nsecs =\
                    int((buffer[dump][0][0] - floor(buffer[dump][0][0])) * 1e9)
                bag_out.write(buffer[dump][0][2], buffer[dump][0][1], buffer[dump][0][1].header.stamp)
                del(buffer[dump][0])




if __name__ == "__main__":
    main()
