#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float64



# Twist command for controlling the linear and angular velocity of the frame
VELOCITY = 0.3  # linear vel    , in m/s    , forward (+)
OMEGA = 0  # angular vel   , rad/s     , counter clock wise (+)


class TwistControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        # form the message
        self._v = VELOCITY
        self._omega = OMEGA
        self._stop = False
        self.stop_time = rospy.Time.now() + rospy.Duration(2)
        self._turn_time = rospy.Time.now()
        self._reset = rospy.Time.now()
        rospy.Timer(rospy.Duration(1), self.check_timer)
        self.continue_timer = rospy.Time.now()
        # construct publisher
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self.subscriber = rospy.Subscriber("Camera", Float64, self.receive_data)

    def check_timer(self, event):
        # Check if current time is past the stop time
        if rospy.Time.now() > self.stop_time:
            self.should_stop = True

    


    def receive_data(self, percentage):
        # print(percentage.data)
        # print(self._v * 10)
        # if self._v == 0.4:
        #     self._omega = -3.5
        # if rospy.Time.now() < self._turn_time:
        #     self._omega = -2
        if percentage.data >= 1:
            self.on_shutdown()
    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)



        message = Twist2DStamped(v=self._v, omega=self._omega)
        while not rospy.is_shutdown():
            self._publisher.publish(message)
            rate.sleep()

    def restart(self):

        return

    def on_shutdown(self):

        if self._stop == True and rospy.Time.now() > self.stop_time:
            self._v = 0.4
            self._omega = -4
            self._stop = False
            self.stop_time = rospy.Time.now() + rospy.Duration(2)
            self.continue_timer = rospy.Time.now() + rospy.Duration(1)
            self._turn_time = rospy.Time.now() + rospy.Duration(2)
            self._reset = rospy.Time.now() + rospy.Duration(2)
            print("stopped")

        else:
            if rospy.Time.now() > self.continue_timer:
                self._v = self._v * 0.92
            stop = Twist2DStamped(v=self._v, omega= self._omega)
            self._publisher.publish(stop)
            if self._v < 0.01:
                if not self._stop:
                    self.stop_time = rospy.Time.now() + rospy.Duration(2)
                self._stop = True
                # print(self.stop_time)
                # print(rospy.Time.now())


if __name__ == '__main__':
    # create the node
    node = TwistControlNode(node_name='twist_control_node')

    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()


    #!/usr/bin/env python3

# import os
# import rospy
# import time
# from duckietown.dtros import DTROS, NodeType
# from duckietown_msgs.msg import Twist2DStamped
# from std_msgs.msg import Float64

# # Twist command for controlling the linear and angular velocity of the frame
# VELOCITY = 0.2  # linear vel    , in m/s    , forward (+)
# OMEGA = 0  # angular vel   , rad/s     , counter clock wise (+)
# TARGET_PERCENTAGE = 50  # Target percentage of red at which to stop

# class TwistControlNode(DTROS):

#     def __init__(self, node_name):
#         # initialize the DTROS parent class
#         super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
#         # static parameters
#         vehicle_name = os.environ['VEHICLE_NAME']
#         twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
#         # form the message
#         self._v = VELOCITY
#         self._omega = OMEGA
#         # PID control variables
#         self.Kp = 0.1  # Proportional gain
#         self.Ki = 0.01  # Integral gain
#         self.Kd = 0.01  # Derivative gain
#         self.prev_error = 0
#         self.integral = 0
#         self.last_time = time.time()
#         # construct publisher
#         self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
#         self.subscriber = rospy.Subscriber("Camera", Float64, self.receive_data)

#     def receive_data(self, percentage):
#         current_percentage = percentage.data
#         error = TARGET_PERCENTAGE - current_percentage
#         self.apply_pid_control(error)
    
#     def apply_pid_control(self, error):
#         current_time = time.time()
#         dt = current_time - self.last_time
#         self.integral += error * dt
#         derivative = (error - self.prev_error) / dt if dt > 0 else 0

#         # PID output
#         output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

#         # Update the velocity
#         self._v = max(0, VELOCITY - output)  # Ensure velocity doesn't go negative

#         # Publish the new velocity command
#         command = Twist2DStamped(v=self._v, omega=0.0)
#         self._publisher.publish(command)

#         # Update previous error and time
#         self.prev_error = error
#         self.last_time = current_time
    
#     def run(self):
#         # publish 10 messages every second (10 Hz)
#         rate = rospy.Rate(10)
#         while not rospy.is_shutdown():
#             rate.sleep()

#     def on_shutdown(self):
#         stop = Twist2DStamped(v=0.0, omega=0.0)
#         self._publisher.publish(stop)

# if __name__ == '__main__':
#     # create the node
#     node = TwistControlNode(node_name='twist_control_node')

#     # run node
#     node.run()
#     # keep the process from terminating
#     rospy.spin()

# Explanation:

#     Error Calculation: The error is the difference between the target percentage (TARGET_PERCENTAGE) and the current percentage (current_percentage).
#     PID Control: The apply_pid_control method computes the PID output and adjusts the robot's velocity accordingly.
#     Velocity Update: The robot's velocity (self._v) is updated based on the PID output, ensuring it doesn't go negative.
#     Publishing New Velocity: A new velocity command is published in apply_pid_control.
#     Shutdown: In on_shutdown, the robot is commanded to stop by setting the velocity to zero and publishing it.

# This approach ensures the robot slows down smoothly as it approaches the desired red percentage threshold, and stops when it reaches the threshold.
