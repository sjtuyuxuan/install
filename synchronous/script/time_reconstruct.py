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

LIDAR_HZ = 1
LIDAR_PRE_HZ = 10
IMU_HZ = 200
RGB_HZ = 30

def Parse():
    parser = argparse.ArgumentParser(description='config_io')
    parser.add_argument("-o", "--output", type=str,
                        help='set output path', default="_processed.bag")
    parser.add_argument("-i", "--input", type=str,
                        help='set input path')
    parser.add_argument('--imu',type=str, help="Set IMU topic", default="/imu/data")
    parser.add_argument('--lidar',type=str, help="Set LIdar Point Cloud topic",
                        default="/os_cloud_node/points")
    parser.add_argument('--rgbl',type=str, help="Set Left RGB camera topic",
                        default="/stereo/left/image_mono")
    parser.add_argument('--rgbr',type=str, help="Set Right RGB camera topic",
                        default="/stereo/right/image_mono")
    parser.add_argument('--propheseel',type=str, help="Set Right Event camera topic",
                        default="/prophesee/camera/cd_events_buffer_reconstructed")
    return parser.parse_args()


def FromSec (time):
    stamp = Image().header.stamp
    stamp.secs = floor(time)
    stamp.nsecs = int((time - floor(time)) * 1e9)
    return stamp

def main():
    args = Parse()
    bag_in = rosbag.Bag(args.input, "r")

    buffer = [[], [], [], [], []]

    lidar_start_offset_secs = 0
    lidar_temp = PointCloud2()
    lidar_start_buffer = []
    lidar_started = False

    imu_started_unix_time = 0
    imu_started = False
    imu_count = 0

    rgbl_started = False
    rgbr_started = False
    rgbl_temp = Image()
    rgbr_temp = Image()
    rgbl_count = 0
    rgbr_count = 0

    propheseel_started = False

    out_path = args.output
    if out_path == "_processed.bag":
        out_path = args.input [:-4] + args.output

    with rosbag.Bag(out_path, 'w') as bag_out:
        for topic, msg, t in bag_in.read_messages():
            if topic == args.lidar:
                if not lidar_started:
                    if lidar_temp.header.stamp.secs == msg.header.stamp.secs:
                        lidar_start_buffer.append(msg)
                        if len(lidar_start_buffer) > 3:
                            lidar_started = True
                            if imu_started:
                                lidar_start_offset_secs = imu_started_unix_time - lidar_temp.header.stamp.secs
                                for lidar_msg in lidar_start_buffer:
                                    buffer[0].append([lidar_msg.header.stamp.to_sec() + lidar_start_offset_secs, lidar_msg, args.lidar])
                            else:
                                for lidar_msg in lidar_start_buffer:
                                    buffer[0].append([lidar_msg.header.stamp.to_sec(), lidar_msg, args.lidar])
                    else:
                        lidar_temp = msg
                        lidar_start_buffer = [lidar_temp]
                else:
                    if imu_started:
                        buffer[0].append([msg.header.stamp.to_sec() + lidar_start_offset_secs, msg, args.lidar])
                    else:
                        buffer[0].append([lidar_msg.header.stamp.to_sec(), msg, args.lidar])

            elif topic == args.imu:
                if not imu_started:
                    imu_started_unix_time = msg.header.stamp.to_sec()
                    if lidar_started:
                        lidar_start_offset_secs = imu_started_unix_time - buffer[0][0][0]
                        for lidar_msg in buffer[0]:
                            lidar_msg[0] += lidar_start_offset_secs

                    if rgbl_started:
                        for rgb_msg in buffer[2]:
                            rgb_msg[0] += imu_started_unix_time

                    if rgbr_started:
                        for rgb_msg in buffer[3]:
                            rgb_msg[0] += imu_started_unix_time

                    if propheseel_started:
                        for prophesee_msg in buffer[4]:
                            prophesee_msg[0] += imu_started_unix_time
                            for event in prophesee_msg[1].events:
                                event.ts = FromSec(event.ts.to_sec() + imu_started_unix_time)

                    imu_started = True
                imu_temp_t = imu_count / IMU_HZ + imu_started_unix_time
                buffer[1].append([imu_temp_t, msg, args.imu])
                imu_count += 1 
                

            elif topic == args.rgbl:
                if not rgbl_started:
                    if msg.header.stamp.to_sec() - rgbl_temp.header.stamp.to_sec() > 0.1:
                         rgbl_temp = msg
                    else:
                        rgbl_started = True
                        if imu_started: 
                            buffer[2].append([imu_started_unix_time, msg, args.rgbl])
                        else:
                            buffer[2].append([0, msg, args.rgbl])
                        rgbl_count += 1
                else:
                    if imu_started: 
                        buffer[2].append([imu_started_unix_time + rgbl_count / RGB_HZ, msg, args.rgbl])
                    else:
                        buffer[2].append([rgbl_count / RGB_HZ, msg, args.rgbl])
                    rgbl_count += 1

            elif topic == args.rgbr:
                if not rgbr_started:
                    if msg.header.stamp.to_sec() - rgbr_temp.header.stamp.to_sec() > 0.1:
                         rgbr_temp = msg
                    else:
                        rgbr_started = True
                        if imu_started: 
                            buffer[3].append([imu_started_unix_time, msg, args.rgbr])
                        else:
                            buffer[3].append([0, msg, args.rgbr])
                        rgbr_count += 1
                else:
                    if imu_started: 
                        buffer[3].append([imu_started_unix_time + rgbr_count / RGB_HZ, msg, args.rgbr])
                    else:
                        buffer[3].append([rgbr_count / RGB_HZ, msg, args.rgbr])
                    rgbr_count += 1

            elif topic == args.propheseel:
                if not propheseel_started:
                    if msg.header.stamp.secs == 0:
                        propheseel_started = True
                        if imu_started: 
                            for event in msg.events:
                                event.ts = FromSec(event.ts.to_sec() + imu_started_unix_time)
                            buffer[4].append([imu_started_unix_time + msg.header.stamp.to_sec(), msg, args.propheseel])
                        else:
                            buffer[4].append([msg.header.stamp.to_sec(), msg, args.propheseel])
                else:    
                    if imu_started: 
                        for event in msg.events:
                            event.ts = FromSec(event.ts.to_sec() + imu_started_unix_time)
                        buffer[4].append([imu_started_unix_time + msg.header.stamp.to_sec(), msg, args.propheseel])
                    else:
                        buffer[4].append([msg.header.stamp.to_sec(), msg, args.propheseel])


            while len(buffer[0]) and len(buffer[1]) and len(buffer[2]) and len(buffer[3]):
                vec = [k[0][0] for k in buffer[:4]]
                dump = vec.index(min(vec))
                buffer[dump][0][1].header.stamp = FromSec(buffer[dump][0][0])
                bag_out.write(buffer[dump][0][2], buffer[dump][0][1], buffer[dump][0][1].header.stamp)
                del(buffer[dump][0])


if __name__ == "__main__":
    main()

