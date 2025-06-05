import os
import cv2
import rclpy
from datetime import datetime, timezone
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata


def timestamp_to_ros_time(timestamp_str):
    date_part, time_part = timestamp_str.split(' ')
    time_comp, nano_part = time_part.split('.')
    nano_part = nano_part[:9].ljust(9, '0')
    
    dt = datetime.strptime(f"{date_part} {time_comp}", "%Y-%m-%d %H:%M:%S")
    dt = dt.replace(tzinfo=timezone.utc)
    
    sec = int(dt.timestamp())
    nanosec = int(nano_part)
    
    return sec, nanosec


def create_image_msg(image, sec, nanosec, encoding='bgr8'):
    msg = Image()
    msg.header.stamp = Time(sec=sec, nanosec=nanosec)
    msg.header.frame_id = "camera"
    msg.height = image.shape[0]
    msg.width = image.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = False
    msg.step = image.shape[1] * image.shape[2]
    msg.data = image.tobytes()
    return msg


def main():
    rclpy.init()
    
    data_folder = "data"
    timestamp_file = "timestamps.txt"
    bag_path = "image_rosbag"
    topic_name = "/image_raw"
    encoding = "bgr8"

    print("Rosbag (image) oluşturuluyor...")

    with open(timestamp_file, 'r') as f:
        timestamps = [line.strip() for line in f.readlines()]
    
    writer = SequentialWriter()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('cdr', 'cdr')
    writer.open(storage_options, converter_options)
    
    topic_metadata = TopicMetadata(
        id=0,
        name=topic_name,
        type='sensor_msgs/msg/Image',
        serialization_format='cdr'
    )
    writer.create_topic(topic_metadata)
    
    count = 0
    for i, timestamp_str in enumerate(timestamps):
        image_file = os.path.join(data_folder, f"{i:010d}.png")
        
        if not os.path.exists(image_file):
            print(f"⚠️ {image_file} bulunamadı, atlanıyor.")
            continue
        
        image = cv2.imread(image_file, cv2.IMREAD_COLOR)
        if image is None:
            print(f"⚠️ {image_file} okunamadı, atlanıyor.")
            continue
        
        try:
            sec, nanosec = timestamp_to_ros_time(timestamp_str)
        except ValueError as e:
            print(f"⚠️ Geçersiz zaman damgası: {timestamp_str} - {e}")
            continue
        
        msg = create_image_msg(image, sec, nanosec, encoding)
        
        writer.write(
            topic_name,
            serialize_message(msg),
            sec * 10**9 + nanosec
        )
        count += 1
    
    print(f"\n✅ {count} görüntü başarıyla yazıldı -> {bag_path}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
