import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped, Pose

class PosePlotter:
    def __init__(self):
        rospy.init_node('pose_plotter', anonymous=True)
        
        self.estimated_x = []
        self.estimated_y = []
        self.gps_x = []
        self.gps_y = []
        self.first_gps_x = None
        self.first_gps_y = None
        
        # Subscribers
        rospy.Subscriber("localization_node/estimated_pose", PoseStamped, self.estimated_pose_callback)
        rospy.Subscriber("localization_node/gps_pose", Pose, self.gps_pose_callback)
        
        plt.ion()
        self.fig, self.ax = plt.subplots()
        
        # ROS Timer to update plot at 5 Hz
        rospy.Timer(rospy.Duration(0.2), self.update_plot)
    
    def estimated_pose_callback(self, msg):
        print(f"Estimated Pose: ({msg.pose.position.x}, {msg.pose.position.y})")
        self.estimated_x.append(msg.pose.position.x)
        self.estimated_y.append(msg.pose.position.y)
    
    def gps_pose_callback(self, msg):
        # Need to fix this section!!!
        # Something is working quite right and I'm getting massive jumps in the x value
        # May be an issue with the localization node not dumping the right x value
        if self.first_gps_x is None and self.first_gps_y is None:
            self.first_gps_x = msg.position.x
            self.first_gps_y = msg.position.y
            print(f"Initial starting GPS: ({self.first_gps_x}, {self.first_gps_y})")

            self.gps_x.append(0.0)
            self.gps_y.append(0.0)
        else:
            print(f"Incoming GPS: ({msg.position.x}, {msg.position.y})")
            print(f"First GPS Point: ({self.first_gps_x}, {self.first_gps_x})")

            new_x = msg.position.x - self.first_gps_x
            new_y = msg.position.y - self.first_gps_y

            print(f"Translated GPS: ({new_x}, {new_y})")
            self.gps_x.append(new_x)
            self.gps_y.append(new_y)
    
    def update_plot(self, event):
        self.ax.clear()
        self.ax.plot(self.estimated_x, self.estimated_y, 'b-', label='Estimated Pose')
        # self.ax.plot(self.gps_x, self.gps_y, 'r-', label='GPS Pose')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.legend()
        plt.draw()
        plt.pause(0.01)
        plt.savefig("pose_plot.png")
    
    def run(self):
        rospy.spin()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    try:
        plotter = PosePlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
