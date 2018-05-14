import rospy
import random
from ford_msgs.msg import Clusters, PlannerMode, NNActions
from geometry_msgs.msg import Vector3, Twist, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
import numpy as np
import tf

NUM_CLUSTERS = 4
DT = 0.1
KEEP_VEL_X_PF = 0.6
KEEP_VEL_Z_PF = 0.6
RADIUS = 5.0

labels = []
mean_points = []
velocities = []
min_points = []
max_points = []

class EgoAgent():
    def __init__(self):
        self.pose = PoseStamped()
        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.frame_id = 'ego_frame'
        self.pose.pose.position = Point(0, 0, 0)
        self.pose.pose.orientation = Quaternion(0, 0, 0, 1)
        
        self.vel = Twist()
        self.vel.linear.x = 0.5
        self.vel.angular.z = random.gauss(0, 1)

        self.spd = Vector3()
        self.spd.x = 0
        self.spd.y = 0

        self.pub_pose = rospy.Publisher('/JA01/pose',PoseStamped, queue_size = 10)
        self.pub_spd = rospy.Publisher('/JA01/velocity', Vector3, queue_size = 10)
        self.pub_feasible_actions = rospy.Publisher('/JA01/safe_actions', NNActions)

        self.sub_vel = rospy.Subscriber('/JA01/nn_cmd_vel', Twist, self.cbVel)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom)
    
    # def move(self, vel):
        # assert type(vel) == Twist
    def move(self):
        try:
            quat = [self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w]
            # print quat
            _, _, yaw = tf.transformations.euler_from_quaternion(quat)
        except AttributeError:
            rospy.logerr('Invalid transformation')
        yaw += self.vel.angular.z * DT
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)

        self.pose.pose.orientation = Quaternion(*q)

        self.pose.pose.position.x += self.vel.linear.x * np.cos(yaw)
        self.pose.pose.position.y += self.vel.linear.x * np.sin(yaw)

        self.pose.header.stamp = rospy.Time.now()
        self.pub_pose.publish(self.pose)

    def cbVel(self, msg):
        self.vel = msg
        self.move()

    def cbOdom(self, msg):
        vx = msg.twist.twist.linear.x
        vt = msg.twist.twist.angular.z
        self.spd.x = vx * np.cos(vt)
        self.spd.y = vx * np.sin(vt)
        self.pub_spd.publish(self.spd)
    
    def pubFeasibleActions(self):
        actions = NNActions()
        actions.angles = np.arange(-3.0, 3.0, 0.2)  # 30 in total
        actions.max_speeds = np.ones(len(actions.angles))
        # actions.path_lengths = 
        # self.pub_feasible_actions.publish()

def update(pos, pos_min, pos_max, vel):
    assert type(pos) == Vector3 and type(pos_min) == Vector3 and type(pos_max) == Vector3 and type(vel) == Vector3
    if random.random() > KEEP_VEL_X_PF:
        vel.x = random.gauss(vel.x, 0.3)
    if random.random() > KEEP_VEL_Z_PF:
        vel.y = random.gauss(vel.y, 0.3)
    diff_x = vel.x * DT
    diff_y = vel.y * DT
    pos.x += diff_x
    pos.y += diff_y
    pos_min.x += diff_x
    pos_min.y += diff_y
    pos_max.x += diff_x
    pos_max.y += diff_y

def run():
    print 'hello world from pub_agents.py'
    
    rospy.init_node('pub_agents',anonymous=False)

    pub_clusters = rospy.Publisher('/JA01/clusters', Clusters, queue_size = 10)

    ego = EgoAgent()
    
    init_angle = np.random.random(4) * 2 * 3.14159
    sizes = random.sample([0.5, 0.5, 0.4, 0.4, 0.3, 0.3], NUM_CLUSTERS)

    # print init_angle
    pos = Vector3()
    vel = Vector3()
    for id in range(NUM_CLUSTERS):
        labels.append(id)

        pos = Vector3(RADIUS * np.cos(init_angle[id]), RADIUS * np.sin(init_angle[id]), 0.0)
        mean_points.append(pos)

        vel = Vector3(random.uniform(0.1, 0.5), random.uniform(0, 3.14159 * 2), 0.0)
        velocities.append(vel)

        min_points.append(Vector3(pos.x - sizes[id] / 2.0, pos.y - sizes[id] / 2.0, 0.0))
        max_points.append(Vector3(pos.x + sizes[id] / 2.0, pos.y + sizes[id] / 2.0, 0.0))
    
    cluster = Clusters()

    rate = rospy.Rate(1 / DT)
    while not rospy.is_shutdown():
        for id in range(NUM_CLUSTERS):
            update(mean_points[id], min_points[id], max_points[id], velocities[id])
        cluster.labels = labels
        cluster.velocities = velocities
        cluster.mean_points = mean_points
        cluster.min_points = min_points
        cluster.max_points = max_points
        pub_clusters.publish(cluster)

        rate.sleep()

    rospy.spin()

 


if __name__ == '__main__':
    run()