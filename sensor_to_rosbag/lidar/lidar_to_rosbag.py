import os
import rclpy
import numpy as np
from datetime import datetime, timezone
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata


def txt_to_pointcloud(file_path):
    data = np.loadtxt(file_path, dtype=np.float32)
    return data


def timestamp_to_ros_time(timestamp_str):
    date_part, time_part = timestamp_str.split(' ')
    time_comp, nano_part = time_part.split('.')
    nano_part = nano_part[:9].ljust(9, '0')
    
    dt = datetime.strptime(f"{date_part} {time_comp}", "%Y-%m-%d %H:%M:%S")
    dt = dt.replace(tzinfo=timezone.utc)
    
    sec = int(dt.timestamp())
    nanosec = int(nano_part)
    
    return sec, nanosec


def create_pointcloud2_msg(points, sec, nanosec):
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
    ]
    
    header = Header()
    header.stamp = Time(sec=sec, nanosec=nanosec)
    header.frame_id = 'lidar'
    
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 16 
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = points.tobytes()
    
    return msg


def main():
    rclpy.init()
    
    data_folder = "data"
    timestamp_file = "timestamps.txt"
    bag_path = "lidar_rosbag"
    topic_name = "/lidar/points"
    
    print("Rosbag oluşturuluyor...")
    
    with open(timestamp_file, 'r') as f:
        timestamps = [line.strip() for line in f.readlines()]
    
    writer = SequentialWriter()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('cdr', 'cdr')
    writer.open(storage_options, converter_options)
    
    topic_metadata = TopicMetadata(
        id=0,
        name=topic_name,
        type='sensor_msgs/msg/PointCloud2',
        serialization_format='cdr'
    )
    writer.create_topic(topic_metadata)
    
    for i, timestamp_str in enumerate(timestamps):
        lidar_file = os.path.join(data_folder, f"{i:010d}.txt")
        
        if not os.path.exists(lidar_file):
            continue
        points = txt_to_pointcloud(lidar_file)
        sec, nanosec = timestamp_to_ros_time(timestamp_str)
        msg = create_pointcloud2_msg(points, sec, nanosec)
        
        writer.write(
            topic_name,
            serialize_message(msg),
            sec * 10**9 + nanosec
        )
    print(f"Tamamlandı! {len(timestamps)} dosya işlendi.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()