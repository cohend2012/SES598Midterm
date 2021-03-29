#Daniel S. Cohen
# Midterm
# ASU ID:
#Goal: Complete "Rocky Times Challange"
#

"""
Midterm exam code: Robot shall take off, scan rock, deliver data to probe, move probe, land on cart
This code will fulfill the reqs outlined here
"""

import rospy
from mavros_msgs.msg import State, PositionTarget
# from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped, PoseArray, TwistStamped
from sensor_msgs.msg import CameraInfo, RegionOfInterest
import math
import numpy
from image_geometry import PinholeCameraModel
import time
# from darknet_ros_msgs.msg import BoundingBox,BoudingBoxes

from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

#from gazebo_ros_link_attacher.msg import Attach
from gazebo_ros_link_attacher.srv import Attach, AttachRequest,AttachResponse

from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy

global shutdown_robot


# our class
class OffbPosCtl:
    curr_drone_pose = PoseStamped()  # create the right structer of the msg for use
    waypointIndex = 0  # start with a way point index of 0
    distThreshold = 0.1  # how close would you like to be from the waypoit
    sim_ctr = 1

    camera = PinholeCameraModel()

    des_pose = PoseStamped()
    #des_vel = TwistStamped()
    isReadyToFly = False
    attach = False
    prob_found = False

    shutdown_robot = False
    # location
    #x_vel = numpy.linspace(0.0, 0.1, 1000)
    #y_vel = numpy.linspace(0.1, 0.0, 1000)

    # get orientation of the des_robot
    orientation = quaternion_from_euler(0, 0, 3.14 / 2 + 3.14 / 8)
    orientation2 = quaternion_from_euler(0, 0, 3.14 / 2)
    orientation3 = quaternion_from_euler(0, 0, 2.7475)
    orientation4 = quaternion_from_euler(0, 0, -2.7475)
    orientation0 = quaternion_from_euler(0, 0, 0)
    orientation5 = quaternion_from_euler(0, 0, 0)

    orientation7 = quaternion_from_euler(0, 0, 2.7475)
    orientation8 = quaternion_from_euler(0, 0, 3.0)
    orientation9 = quaternion_from_euler(0, 0, 3.4)
    orientation10 = quaternion_from_euler(0, 0, 4.1)
    orientation11 = quaternion_from_euler(0, 0, 4.8)
    orientation12 = quaternion_from_euler(0, 0, 5.5)
    orientation13 = quaternion_from_euler(0, 0, 6.2)
    orientation14 = quaternion_from_euler(0, 0, 6.9)
    orientation15 = quaternion_from_euler(0, 0, 7.3)
    orientation16 = quaternion_from_euler(0, 0, 8.3)
    orientation17 = quaternion_from_euler(0, 0, 9.0)
    orientation18 = quaternion_from_euler(0, 0, 0)

    # Store the robots location for waypoint nav system, preloaded from our reaserch out the world map


    locations = numpy.matrix([[2, 0, 1, orientation2[0], orientation2[1], orientation2[2], orientation2[3]],
                              [2, 2, 1, orientation2[0], orientation2[1], orientation2[2], orientation2[3]],
                              [0, 2, 1, orientation2[0], orientation2[1], orientation2[2], orientation2[3]],
                              [-2, 0, 1, orientation2[0], orientation2[1], orientation2[2], orientation2[3]], # end of fun box point 4

                              [65, -15, 22.0, orientation0[0], orientation0[1], orientation0[2], orientation0[3]], # start of scan and point4
                              [65, -15, 19.0, orientation3[0], orientation3[1], orientation3[2], orientation3[3]], # lower slow to ground

                              [64.6261, -16.5617, 19.5, orientation7[0], orientation7[1], orientation7[2], orientation7[3]],
                              [66.2020, -12.7721, 19.5, orientation8[0], orientation8[1], orientation8[2], orientation8[3]],
                              [64.9733, -8.8561, 19.5, orientation9[0], orientation9[1], orientation9[2], orientation9[3]],
                              [61.5149, -6.6461, 19.5, orientation10[0], orientation10[1], orientation10[2], orientation10[3]],
                              [57.4450, -7.1761, 19.5, orientation11[0], orientation11[1], orientation11[2], orientation11[3]],
                              [54.6680, -10.1982, 19.5, orientation12[0], orientation12[1], orientation12[2], orientation12[3]],
                              [54.4833, -14.2983, 19.5, orientation13[0], orientation13[1], orientation13[2], orientation13[3]],
                              [56.9773, -17.5579, 19.5, orientation14[0], orientation14[1], orientation14[2], orientation14[3]],
                              [60.9830, -18.4518, 19.5, orientation15[0], orientation15[1], orientation15[2], orientation15[3]],
                              [64.6261, -16.5617, 19.5, orientation16[0], orientation16[1], orientation16[2], orientation16[3]],

                            #16
                              [65, -15, 30.0, orientation3[0], orientation3[1], orientation3[2], orientation3[3]], # do scan and come back
                              [40.85, 3.475, 13, orientation0[0], orientation0[1], orientation0[2], orientation0[3]],
                              # drop off?
                              [84.789988, -54.251096, 25.0, orientation[0], orientation[1], orientation[2],orientation[3]],
                              [84.789988, -54.251096, 21.0, orientation[0], orientation[1], orientation[2],orientation[3]],
                              [84.789988, -54.251096, 30.0, orientation[0], orientation[1], orientation[2],orientation[3]],

                              [55, -12, 17.89, orientation0[0], orientation0[1], orientation0[2], orientation0[3]], # other

                              [10, -40, 17.89, orientation0[0], orientation0[1], orientation0[2], orientation0[3]],
                              [12.66214, -65.749400, 3.0, orientation0[0], orientation0[1], orientation0[2], orientation0[3]], # close to landing
                              #[-0, 0, 1, orientation0[0], orientation0[1], orientation0[2], orientation0[3]],
                              # [-76, 425, 1, orientation[0],orientation[1],orientation[2],orientation[3]],
                              [0, 0, 0, 0, 0, 0, 0],
                              #[0, 0, 0, 0, 0, 0, 0]
                              #[12.62140, -65.05, -3.8, orientation[0], orientation[1], orientation[2], orientation[3]],
                              ])


                            # washer loc x,y,z 84.789988,-54.251096,17.83632
    def __init__(self):
        global shutdown_robot
        shutdown_robot = False
        rospy.init_node('offboard_test', anonymous=True)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        # pose_setpoint_pub = rospy.Publisher('/mavros/setpoit_position/local geometry_msgs/PoseStamped', PoseStamped, queue_size=10)
        drone_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,callback=self.drone_pose_cb)
        #vel_pub = rospy.Publisher('mavros/local_raw/local', PositionTarget, queue_size=10)
        vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        # vel more like pose
        rover_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,callback=self.rover_pose_cb)
        state_sub = rospy.Subscriber('/mavros/state', State, callback=self.drone_state_cb)
        #attach = rospy.Publisher('/attach', String, queue_size=10)


        #attach_pub = rospy.Publisher('/link_attacher_node/attach_models',Attach, queue_size=1)

        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',Attach)
        attach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

        rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
        detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',Attach)
        detach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")






        NUM_UAV = 2
        mode_proxy = [None for i in range(NUM_UAV)]
        arm_proxy = [None for i in range(NUM_UAV)]
        x_vel = 0.0
        y_vel = 0.0
        z_vel = 0.0
        rot_speed = 0.0
        vel_moving = False

        # Comm for drones and get ready to take off the system, "preflight"
        for uavID in range(0, NUM_UAV):
            mode_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)
            arm_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/cmd/arming', CommandBool)

        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_drone_pose)
        shape = self.locations.shape

        # Keep the uav running unitll we ros gets shut down or the robot triggers a shutdown
        while not rospy.is_shutdown() and shutdown_robot is False:
            print self.sim_ctr, shape[0], self.waypointIndex
            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = mode_proxy[uavID](1, 'OFFBOARD')
                except rospy.ServiceException, e:
                    print ("mavros/set_mode service call failed: %s" % e)

            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                rospy.wait_for_service(self.mavrosTopicStringRoot(uavID) + '/cmd/arming')

            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = arm_proxy[uavID](True)
                except rospy.ServiceException, e:
                    print ("mavros1/set_mode service call failed: %s" % e)

            if self.waypointIndex is shape[0]:
                self.waypointIndex = 0
                self.sim_ctr += 1

            if self.waypointIndex is 6 and vel_moving is False:
                # another way to scan the rock on the sunny side using vel
                # no point starting on the dark side if we need to preserve power to the system
                """for index in range(250000):
                    attach.publish("At the rock ready to scan")
                    # testing that we can move robot in this sections
                    if index < 25000:
                        x_vel = -0.7
                        y_vel = 1.3
                        z_vel = 0.07
                        rot_speed = 0.18
                        print('start scan 1')

                    if index>=25000 and index<50000:
                        x_vel = -1.3
                        y_vel = 1.3
                        z_vel = 0.07
                        rot_speed = 0.16
                        print("scan 2")

                    if index>=50000 and index<75000:
                        x_vel = -0.7
                        y_vel = 0.7
                        z_vel = 0.05
                        rot_speed = 0.18
                        print("scan 3")

                    if index>=75000 and index < 100000:
                        x_vel = -0.5 # around the rocck
                        y_vel = 0.3
                        z_vel = -0.1
                        rot_speed = 0.2
                        print("scan 4")

                    if index>=150000 and index<175000:
                        x_vel = -0.5
                        y_vel = -1.8
                        z_vel = 0.0
                        rot_speed =0.0
                        print("scan 5")

                    if index>=200000 and index <250000:
                        x_vel = 0.3
                        y_vel = -0.3
                        z_vel = -0.02
                        rot_speed = 0.06
                        print("scan 6")

                    #x_vel += 0.01
                    #y_vel -= 0.01
                    #z_vel = 0.11
                    vel_pub.publish(self.vel_move(x_vel,y_vel, z_vel,rot_speed))
                    #attach.publish(" moving to target using the vel with rotate")"""
                #x_vel = 0.0
                #y_vel = 0.0
                #z_vel = 0.0
                #rot_speed = 0.0
                #vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))
                #attach.publish("ATTACH")
                #r = 3.0
                #disc = 50.0
                #counter = 0
                #rot_theta = numpy.linspace(0.0,2.0*math.pi,1000)
                #orientation = quaternion_from_euler(0, 0, 2.7475)
                #des_array = numpy.array([60.208121, -12.502033, 19.0, orientation[0], orientation[1], orientation[2], orientation[3]])
                #des_local = self.set_desired_pose_local(des_array).position
                #dist = math.sqrt((curr.x - des_local.x) * (curr.x - des_local.x) + (curr.y - des_local.y) * (
                        #curr.y - des_local.y) + (curr.z - des_local.z) * (
                              #           curr.z - des_local.z))
                #print('dist = ', dist)
                """while (counter<1000):
                    # des = self.set_desired_pose().position
                    
                    azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                         self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                    az_quat = quaternion_from_euler(0, 0, azimuth)

                    curr = self.curr_drone_pose.pose.position

                    x_coord = r*math.cos(rot_theta[counter])
                    y_coord = r*math.sin(rot_theta[counter])

                    dist = math.sqrt((curr.x - des_local.x+x_coord) * (curr.x - des_local.x+x_coord) /
                                     + (curr.y - des_local.y+y_coord) * (curr.y - des_local.y+y_coord))

                    print('dist = ', dist)
                    print('curr.x =',curr.x ,'des_local.x',des_local.x,x_coord,'x_coord')
                    print('curr.x < des_local.x+x_coord',curr.x < des_local.x+x_coord)
                    print('curr.y =', curr.y, 'des_local.y', des_local.y, y_coord, 'y_coord')
                    print('curr.y < des_local.y+y_coord', curr.y < des_local.y + y_coord)
                    print('counter',counter


                    if (curr.x < des_local.x+x_coord):
                        x_vel = 0.2
                        #z_vel = 0.005
                        rot_speed = 0.0
                    else:
                        x_vel = -0.2

                    if (curr.y < des_local.y+y_coord):

                        y_vel = 0.2
                        #z_vel = 0.005
                        rot_speed = 0.0

                    else:
                        y_vel = -0.2
                        #z_vel = 0.005

                    if(curr.z < des_local.z):
                        z_vel = 0.005

                    else:
                        z_vel = -0.005"""

                    #des_array = numpy.array([60.208121+x_coord, -12.502033+y_coord, 19.0, orientation[0], orientation[1], orientation[2], orientation[3]])

                    #des_local = self.set_desired_pose_local(des_array)


                    #pose_pub.publish(des_local)


                    #vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))

                    #print('T/F onloc')
                    #print(math.fabs(curr.y - des_local.y+y_coord) <= 0.5 and math.fabs(curr.x - des_local.x+x_coord) <= 0.5)

                    #if dist <= 0.1:
                        #counter += 1
                        #print("got it")
                        #attach.publish("ATTACH")

                        #print("MOVING........")
                    #print(index)"""

                vel_moving = True

            if self.waypointIndex is 18 and vel_moving is True:
                prob_found = False
                x_vel = 0.0
                y_vel = 0.0
                z_vel = 0.0
                rot_speed = 0.0
                vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel,rot_speed))

                orientation = quaternion_from_euler(0, 0, 0)
                des_array = numpy.array([40.8511978, 3.475391, 12.0,orientation[0], orientation[1], orientation[2], orientation[3]])
                des_local = self.set_desired_pose_local(des_array).position
                dist = math.sqrt((curr.x - des_local.x) * (curr.x - des_local.x) + (curr.y - des_local.y) * (
                                curr.y - des_local.y) + (curr.z - des_local.z) * (
                            curr.z - des_local.z))
                print('dist = ', dist)
                while(prob_found is False and dist > self.distThreshold):
                    #des = self.set_desired_pose().position

                    azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                     self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                    az_quat = quaternion_from_euler(0, 0, azimuth)

                    curr = self.curr_drone_pose.pose.position
                    dist = math.sqrt(
                        (curr.x - des_local.x) * (curr.x - des_local.x) + (curr.y - des_local.y) * (curr.y - des_local.y) + (curr.z - des_local.z) * (
                                curr.z - des_local.z))

                    print('dist = ',dist )
                    if(curr.x<des_local.x):
                        x_vel = 0.1
                        z_vel = -0.1
                        rot_speed = 0.0
                    else:
                        x_vel = -0.1

                    if(curr.y < des_local.y):

                        y_vel = 0.1
                        z_vel = -0.1
                        rot_speed = 0.0

                    else:
                        y_vel = -0.1
                        z_vel = -0.1

                    vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))

                    if dist <= self.distThreshold:
                        print("got it")
                        req = AttachRequest()
                        req.model_name_1 = "iris"
                        req.link_name_1 = "base_link"
                        req.model_name_2 = "sample_probe"
                        req.link_name_2 = "base_link"
                        attach_srv.call(req)
                        prob_found = True
                        vel_moving = False

                        #self.waypointIndex += 1


                rate.sleep()


            if self.waypointIndex is 20 and vel_moving is False:
                print("way point 20")


                x_vel = 0.0
                y_vel = 0.0
                z_vel = 0.0
                rot_speed = 0.0
                vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel,rot_speed))
                print("moving to target using the vel with rotate")
                print("MOVING........")
                vel_moving = False
                prob_ready_for_deployment = True
                x_vel = 0.0
                y_vel = 0.0
                z_vel = 0.0
                rot_speed = 0.0
                #vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))

                orientation = quaternion_from_euler(0, 0, 0)
                des_array = numpy.array([84.789988, -54.251096, 19.2, orientation[0], orientation[1], orientation[2], orientation[3]])

                des_local = self.set_desired_pose_local(des_array).position
                dist = math.sqrt((curr.x - des_local.x) * (curr.x - des_local.x) + (curr.y - des_local.y) * (
                        curr.y - des_local.y) + (curr.z - des_local.z) * (
                                         curr.z - des_local.z))
                count = 0

                ##84.789988,-54.251096,17.83632
                while (prob_ready_for_deployment is True and dist > self.distThreshold and count is 0):
                    # des = self.set_desired_pose().position
                    des_local = self.set_desired_pose_local(des_array).position
                    azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                         self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                    az_quat = quaternion_from_euler(0, 0, azimuth)

                    curr = self.curr_drone_pose.pose.position
                    dist = math.sqrt(
                        (curr.x - des_local.x) * (curr.x - des_local.x) + (curr.y - des_local.y) * (
                                    curr.y - des_local.y) + (curr.z - des_local.z) * (
                                curr.z - des_local.z))

                    print('dist = ', dist, "ready for depl", prob_ready_for_deployment)

                    if (curr.x < des_local.x):
                        x_vel = 0.1
                        z_vel = -0.2
                        rot_speed = 0.0
                    else:
                        x_vel = -0.1

                    if (curr.y < des_local.y):

                        y_vel = 0.1
                        z_vel = -0.2
                        rot_speed = 0.0

                    else:
                        y_vel = -0.1
                        z_vel = -0.2

                    if(curr.z < des_local.z):
                        z_vel = 0.2
                    vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))

                    if dist < self.distThreshold:
                        print("got it")
                        req = AttachRequest()
                        req.model_name_1 = "iris"
                        req.link_name_1 = "base_link"
                        req.model_name_2 = "sample_probe"
                        req.link_name_2 = "base_link"
                        detach_srv.call(req)
                        prob_ready_for_deployment = False
                        vel_moving = True
                        count = count + 1

                rate.sleep()

            if self.waypointIndex is 24 and vel_moving is True:
                print("way point 24")

                x_vel = 0.0
                y_vel = 0.0
                z_vel = 0.0
                rot_speed = 0.0
                vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))
                print(" moving to target using the vel ")
                print("MOVING........ to way point 14")
                vel_moving = False
                landed = False
                x_vel = 0.0
                y_vel = 0.0
                z_vel = 0.0
                rot_speed = 0.0
                vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))

                orientation = quaternion_from_euler(0,0,3.14/2)
                des_array = numpy.array([12.62140, -65.05, -3.8, orientation[0], orientation[1], orientation[2], orientation[3]])

                while (landed is False):
                    # des = self.set_desired_pose().position
                    des_local = self.set_desired_pose_local(des_array).position
                    azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                         self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                    az_quat = quaternion_from_euler(0, 0, azimuth)

                    curr = self.curr_drone_pose.pose.position
                    dist = math.sqrt(
                        (curr.x - des_local.x) * (curr.x - des_local.x) + (curr.y - des_local.y) * (
                                    curr.y - des_local.y) + (curr.z - des_local.z) * (
                                curr.z - des_local.z))
                    if (curr.x < des_local.x):
                        x_vel = 0.1
                        z_vel = -0.2
                        rot_speed = 0.0
                    else:
                        x_vel = -0.1
                        z_vel = -0.2

                    if (curr.y < des_local.y):

                        y_vel = 0.1
                        z_vel = -0.2
                        rot_speed = 0.0

                    #if(curr.z < des_local.z):
                        #z_vel = 0.2

                    else:
                        y_vel = -0.1
                        z_vel = -0.2

                    vel_pub.publish(self.vel_move(x_vel, y_vel, z_vel, rot_speed))
                    print('dist = ', dist, "Robot landed(T/F)", landed)
                    print("In the truckbed (T/F)",dist < 1.5 )
                    if dist < self.distThreshold:
                        print("landed")
                        landed = True
                        vel_moving = False
                        shutdown_robot = True

                        success = [None for i in range(NUM_UAV)]
                        for uavID in range(0, NUM_UAV):
                            rospy.wait_for_service(self.mavrosTopicStringRoot(uavID) + '/cmd/arming')

                        for uavID in range(0, NUM_UAV):
                            try:
                                success[uavID] = arm_proxy[uavID](False)
                                print ("mavros1/set_mode service call was successful",success[uavID])

                            except rospy.ServiceException, e:
                                print ("mavros1/set_mode service call failed: %s" % e)
                        # drone is on the robot ready to move to new map for scanning



                rate.sleep()




            if self.isReadyToFly and shutdown_robot is False:
                des = self.set_desired_pose().position
                azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                     self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                az_quat = quaternion_from_euler(0, 0, azimuth)

                curr = self.curr_drone_pose.pose.position
                dist = math.sqrt(
                    (curr.x - des.x) * (curr.x - des.x) + (curr.y - des.y) * (curr.y - des.y) + (curr.z - des.z) * (
                            curr.z - des.z))
                if dist < self.distThreshold:
                    self.waypointIndex += 1

            pose_pub.publish(self.des_pose)
            rate.sleep()


    def mavrosTopicStringRoot(self, uavID=0):
        mav_topic_string = '/mavros/'
        return mav_topic_string

    def set_desired_pose(self):
        self.des_pose.pose.position.x = self.locations[self.waypointIndex, 0]
        self.des_pose.pose.position.y = self.locations[self.waypointIndex, 1]
        self.des_pose.pose.position.z = self.locations[self.waypointIndex, 2]
        self.des_pose.pose.orientation.x = self.locations[self.waypointIndex, 3]
        self.des_pose.pose.orientation.y = self.locations[self.waypointIndex, 4]
        self.des_pose.pose.orientation.z = self.locations[self.waypointIndex, 5]
        self.des_pose.pose.orientation.w = self.locations[self.waypointIndex, 6]
        if self.locations[self.waypointIndex, :].sum() == 0:
            self.des_pose.pose.position.x = self.curr_rover_pose.pose.position.x
            self.des_pose.pose.position.y = self.curr_rover_pose.pose.position.y
            self.des_pose.pose.position.z = max(self.curr_rover_pose.pose.position.z, 10)
            orientation = quaternion_from_euler(0, 0, 3.14 / 2)
            self.des_pose.pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        return self.des_pose.pose

    def set_desired_pose_local(self,array_input):
        #self.des_pose= PoseStamped()

        self.des_pose.pose.position.x = array_input[0]
        self.des_pose.pose.position.y = array_input[1]
        self.des_pose.pose.position.z = array_input[2]
        self.des_pose.pose.orientation.x = array_input[3]
        self.des_pose.pose.orientation.y = array_input[4]
        self.des_pose.pose.orientation.z = array_input[5]
        self.des_pose.pose.orientation.w = array_input[6]
        orientation = quaternion_from_euler(0, 0, 3.14 / 2)
        self.des_pose.pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        return self.des_pose.pose

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def vel_move(self, x, y, z,rot_speed):
        des_vel = TwistStamped()
        des_vel.header.frame_id = 'world'
        des_vel.header.stamp = rospy.Time.from_sec(time.time())
        # des_vel.coordinate_frame = 8
        # des_vel.type_mask = 3527
        des_vel.twist.linear.x = x
        des_vel.twist.linear.y = y
        des_vel.twist.linear.z = z
        des_vel.twist.angular.z = rot_speed
        return des_vel

    def drone_pose_cb(self, msg):
        self.curr_drone_pose = msg

    def rover_pose_cb(self, msg):
        self.curr_rover_pose = msg

    def drone_state_cb(self, msg):
        print msg.mode
        if (msg.mode == 'OFFBOARD'):
            self.isReadyToFly = True
            print "readyToFly"

if __name__ == "__main__":

    OffbPosCtl()



