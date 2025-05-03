import rospy
import csv
from geometry_msgs.msg import PoseStamped, Pose

class PoseLogger:
    def __init__(self):
        rospy.init_node('pose_logger', anonymous=True)
        
        # Open CSV file
        self.csv_file = open('pose_data.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        self.csv_writer.writerow(["timestamp", "estimated_position_x", "estimated_position_y", "estimated_position_z", "estimated_orientation_x", "estimated_orientation_y", "estimated_orientation_z", "estimated_orientation_w", "gps_position_x", "gps_position_y", "gps_position_z", "gps_orientation_x", "gps_orientation_y", "gps_orientation_z", "gps_orientation_w"])
        
        # Initialize storage variables
        self.estimated_pose = None
        self.gps_pose = None
        
        # Subscribers
        rospy.Subscriber("localization_node/estimated_pose", PoseStamped, self.estimated_pose_callback)
        rospy.Subscriber("localization_node/gps_pose", Pose, self.gps_pose_callback)
        
    def estimated_pose_callback(self, msg):
        self.estimated_pose = msg.pose
        self.write_pose_data(msg.header.stamp.to_sec())
    
    def gps_pose_callback(self, msg):
        self.gps_pose = msg
        self.write_pose_data(rospy.get_time())
    
    def write_pose_data(self, timestamp):
        estimated = self.estimated_pose if self.estimated_pose else Pose()
        gps = self.gps_pose if self.gps_pose else Pose()
        
        self.csv_writer.writerow([
            timestamp,
            estimated.position.x, estimated.position.y, estimated.position.z,
            estimated.orientation.x, estimated.orientation.y, estimated.orientation.z, estimated.orientation.w,
            gps.position.x, gps.position.y, gps.position.z,
            gps.orientation.x, gps.orientation.y, gps.orientation.z, gps.orientation.w
        ])
        self.csv_file.flush()
    
    def run(self):
        rospy.spin()
        self.csv_file.close()

if __name__ == '__main__':
    try:
        logger = PoseLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass