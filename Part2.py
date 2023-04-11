#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

def dist(x1,y1,x2,y2):
    return (sqrt(pow(x1-x2,2)+pow(y1-y2,2)))

class myturtle:

    def __init__(self):
        rospy.init_node('myturtle', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(30)
        self.err=0
        self.preverr=0
        self.sumerr=0
        self.errt=0
        self.preverrt=0
        self.sumerrt=0


    def update_pose(self, data):
        self.pose = data

    def dist(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose):
        Kp=1.2
        Kd=0.6
        Ki=0.9
        self.err=self.dist(goal_pose)
        prop=Kp*self.err
        inte=Ki*self.sumerr
        deri=Kd*(self.err-self.preverr)
        self.sumerr=self.sumerr+(self.err/30.0)
        self.preverr=self.err
        return (prop+inte+deri)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y-self.pose.y,goal_pose.x-self.pose.x)

    def angular_vel(self, goal_pose):
        Kpt=4
        Kdt=1.5
        Kit=2.4
        self.errt=self.steering_angle(goal_pose) - self.pose.theta
        prop=Kpt*self.errt
        inte=Kit*self.sumerrt
        deri=Kdt*(self.errt-self.preverrt)
        self.sumerrt=self.sumerrt+(self.errt)/30.0
        self.preverrt=self.errt
        return(prop+inte+deri)

    def move2goal(self,xcoord,ycoord):
        goal_pose = Pose()

        goal_pose.x = xcoord
        goal_pose.y = ycoord
        distance_tolerance = 0.05

        vel_msg = Twist()

        while self.dist(goal_pose) >= distance_tolerance:


            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

       
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            self.velocity_publisher.publish(vel_msg)

            self.rate.sleep()

        self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    soln=[(51.0, 43.0), (61.13142191200112, 54.061387347029466), (197.7922086052018, 115.25049212700478), (243.07830223512573, 131.45185751884898), (263.37588113136144, 142.1360208750555), (294.9852054251422, 166.55888346925983), (330.1222855814092, 187.83071213356402), (369.54370669716184, 217.04339008055246), (399.3260331766582, 235.73050125659287), (431.06234259049626, 258.3828826772639), (456.19002919126495, 278.0846417955572), (455.38198577904853, 300.40174449141773), (471.2525526828618, 340.8484606145159), (479.4718279277731, 374.25551645952703), (495.20834142153046, 420.26363997121484), (502.1868491552305, 467.464370989558), (516.9647280744992, 504.4054949034707), (528.2711562709457, 526.3804330028034), (539.0, 558.0)]
    coord=[]
    for i in range(len(soln)):
        coord.append([(soln[i][0]*11.08889)/600,((600-soln[i][1])*11.08889)/600])
    disc=[]
    disc.append((coord[0][0],coord[0][1]))
    angle=atan2(coord[1][1]-coord[0][1],coord[1][0]-coord[0][0])
    prevangle=angle
    for i in range(1,len(soln)-1):
        angle=atan2(coord[i+1][1]-coord[i][1],coord[i+1][0]-coord[i][0])
        if(abs(angle-prevangle)<=0.0525):
            continue
        else:
            disc.append([coord[i][0],coord[i][1]])
            prevangle=angle
    disc.append([coord[-1][0],coord[-1][1]])
    finalgoal=[coord[-1][0],coord[-1][1]]
    myturtle = myturtle()
    itr=0
    while(True):
        try:
            myturtle.err=0
            myturtle.preverr=0
            myturtle.sumerr=0
            myturtle.errt=0
            myturtle.preverrt=0
            myturtle.sumerrt=0
            myturtle.move2goal(disc[itr][0],disc[itr][1])
            if(itr+1<len(disc)):
                itr+=1
            else:
                break
        except rospy.ROSInterruptException:
            pass
