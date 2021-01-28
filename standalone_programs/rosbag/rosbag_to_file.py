import rosbag


def save_imu_msgs(rosbag, topic, filename):
    '''
   imu msgs
    Header header

    geometry_msgs/Quaternion orientation
    float64[9] orientation_covariance # Row major about x, y, z axes

    geometry_msgs/Vector3 angular_velocity
    float64[9] angular_velocity_covariance # Row major about x, y, z axes

    geometry_msgs/Vector3 linear_acceleration
    float64[9] linear_acceleration_covariance # Row major x, y z


    Timestamp,orientation, x, y, z, w, angular_velocity, x, y, z, linear_acceleration, x, y, z,
    t,       ,           , d, d, d, d,                 , d, d, d,                    , d, d, d,

    '''
    f = open('./' + filename, 'w')
    f.write('timestamp,orientation,x,y,z,w,angular_velocity,x,y,z,linear_acceleration,x,y,z\n')
    for _, msg, _ in bag.read_messages(topics=[topic]):
        csv_str = ''
        csv_str += str(msg.header.stamp.to_sec()) + ','
        csv_str += ','
        csv_str += str(msg.orientation.x) + ','
        csv_str += str(msg.orientation.y) + ','
        csv_str += str(msg.orientation.z) + ','
        csv_str += str(msg.orientation.w) + ','
        csv_str += ','
        csv_str += str(msg.angular_velocity.x) + ','
        csv_str += str(msg.angular_velocity.y) + ','
        csv_str += str(msg.angular_velocity.z) + ','
        csv_str += ','
        csv_str += str(msg.linear_acceleration.x) + ','
        csv_str += str(msg.linear_acceleration.y) + ','
        csv_str += str(msg.linear_acceleration.z) + ','
        csv_str += '\n'
        f.write(csv_str)
try:
    bag = rosbag.Bag('/home/tw/Desktop/sensor_total_revolution.bag')
    save_imu_msgs(bag, '/imu', 'imu_log.csv')
finally:
    bag.close()


