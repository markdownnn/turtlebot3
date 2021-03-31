import rospy
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import SensorState
PI = 3.1415926535897
angle = 20    # 시작 각도, 20도 만큼 꺽어서 이동
speed = 0.1  # 꺾을 때도 직진할 때도 0.1m/s
threshold1 = 100
threshold2 = 1000
geto_enter=0

#Converting from angles to radians
angular_speed = speed * 2 * PI / 360
relative_angle = angle * 2 * PI / 360

class Moving():
    def __init__(self):
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.odor_sub = rospy.Subscriber('sensor_state1', SensorState, self.get_odor, queue_size = 1)
        self.wind_sub = rospy.Subscriber('sensor_state2', SensorState, self.get_wind,  queue_size = 1)
        self.setting()
        self.moving()
    
    def converted_angle1(self, sensor2):
        angle_data = sensor2.wind
        if angle_data < 180:
            angle_data = (angle_data + 90) * 2 * PI / 360 # 이걸 하기 위해선 wind sensor에서 값을 0~360으로 변환하는 작업이 필요.
        else:
            angle_data = (270-angle_data)*2*PI/360
        return angle_data   

    def converted_angle2(self, sensor2):
        angle_data = sensor2.wind
        if angle_data < 180:
            angle_data = (180 - angle_data) * 2 * PI / 360 # 이걸 하기 위해선 wind sensor에서 값을 0~360으로 변환하는 작업이 필요
        else:
            angle_data = (angle_data - 180) * 2 * PI / 360
        return angle_data   

    def get_odor(self, sensor1):
        odor_data = sensor1.odor
        return odor_data

    def get_wind(self, sensor2):
        wind_data=sensor2.wind
        return wind_data

    def setting(self, sensor1, sensor2):
        twist = Twist()
        direction_data;
        odor_data;
 
        # We won't use linear components in first step
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
      
        twist.angular.z = -abs(angular_speed)
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
   
        while current_angle < relative_angle:
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)  # 초당 움직일 때마다 데이터를 받아오고 싶은 데 더 빠르게 데이터 받아올 경우
            direction_data = get_wind()
            if direction_data > 177 and direction_data < 183:
                break

        geto_enter = 0
        twist.angular.z = -abs(angular_speed)
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        left_odor = 0
        total_lo = 0

        while current_angle < relative_angle:
            self.cmd_pub.publish(twist)
            t1 = rospy.Time.now().to_sec()
            left_odor = get_odor()
            total_lo = (total_lo + left_odor) / geto_enter
            current_angle = angular_speed * (t1 - t0)
       
        twist.angular.z = 0
      
        geto_enter = 0
        twist.angular.z = abs(angular_speed)
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        right_odor = 0
        total_ro = 0
   
        while current_angle < 2 * relative_angle:
            self.cmd_pub.publish(twist)
            t1 = rospy.Time.now().to_sec()            
            right_odor = get_odor()
            total_ro = (total_ro + right_odor) / geto_enter
            current_angle = angular_speed * (t1 - t0)

        twist.angular.z = 0
        total_ro = total_ro - total_lo
     
        if total_lo < total_ro:
            self.moving()
      
        else:
            twist.angular.z = -abs(angular_speed)
            t0 = rospy.Time.now().to_sec()
            current_angle = 0
            while current_angle < 2 * relative_angle:
                self.cmd_pub.publish(twist)
                t1 = rospy.Time.now().to_sec()           
                current_angle = angular_speed*(t1-t0)
            twist.angular.z = 0
            self.moving()
   
    def moving(self):
        while True:
            twist = Twist()    
            twist.linear.x = speed

            if threshold1 < get_odor():
                twist.linear.x = 0 
                t = 0
                t0 = rospy.Time.now().to_sec()
                current_distance = 0     
        
                while t < 5:
                    twist.linear.x = speed 
                    t1 = rospy.Time.now().to_sec()
                    t = t1 - t0

                twist.linear.x = 0
                if get_wind() < 180:
                    prefer_angle = converted_angle()
                    twist.angular.z = -abs(angular_speed)
                    t0 = rospy.Time.now().to_sec()
                    current_angle = 0
        
                    while current_angle < prefer_angle:
                        self.cmd_pub.publish(twist)
                        t1 = rospy.Time.now().to_sec()
                        current_angle = angular_speed * (t1 - t0)
                    twist.angular.z = 0
                else:
                    twist.angular.z = abs(angular_speed)
                    t0 = rospy.Time.now().to_sec()
                    current_angle = 0
    
                    while current_angle < prefer_angle:
                        self.cmd_pub.publish(twist)
                        t1 = rospy.Time.now().to_sec()            
                        current_angle = angular_speed*(t1-t0)   
                    twist.angular.z = 0
        
                while get_odor() < threshold1:
                    twist.linear.x = speed
                twist.linear.x = 0
    
                if get_wind() < 180:
                    vertical_angle=converted_angle2()
                    twist.angular.z = -abs(angular_speed)
                    t0 = rospy.Time.now().to_sec()
                    current_angle = 0
    
                    while current_angle < vertical_angle:
                        self.cmd_pub.publish(twist)
                        t1 = rospy.Time.now().to_sec()          
                        current_angle = angular_speed*(t1-t0)
                    twist.angular.z = 0
        
                else:
                    vertical_angle = converted_angle2()
                    twist.angular.z = abs(angular_speed)
                    t0 = rospy.Time.now().to_sec()
                    current_angle = 0
    
                    while current_angle < vertical_angle:
                        self.cmd_pub.publish(twist)
                        t1 = rospy.Time.now().to_sec()            
                        current_angle = angular_speed*(t1-t0)   
                    twist.angular.z = 0

                twist.angular.z = -abs(angular_speed)
                t0 = rospy.Time.now().to_sec()
                current_angle = 0
                left_odor = 0
                maxl_odor = 0
                left_angle = 0
                exl_angle = 0

                while current_angle < 2 * relative_angle:
                    self.cmd_pub.publish(twist)
                    t1 = rospy.Time.now().to_sec()           
                    left_odor = get_odor()
                    current_angle = angular_speed * (t1 - t0)
                    left_angle = current_angle
                    if maxl_odor < left_odor:
                        maxl_odor = left_odor
                        exl_angle = left_angle
                twist.angular.z = 0
                twist.angular.z = abs(angular_speed)
                t0 = rospy.Time.now().to_sec()
                current_angle = 0
                right_odor = 0
                maxr_odor = 0
                right_angle =0
                exr_angle = 0
    
                while current_angle < 4 * relative_angle:
                    self.cmd_pub.publish(twist)
                    t1 = rospy.Time.now().to_sec()           
                    right_odor = get_odor()
                    current_angle = angular_speed * (t1 - t0)
                    right_angle = current_angle
                    if maxr_odor < right_odor:
                        maxr_odor = right_odor
                        exr_angle = right_angle
                twist.angular.z = 0
        
                if maxl_odor > maxr_odor:
                    twist.angular.z = -abs(angular_speed)
                    while(current_angle < exl_angle):
                        self.cmd_pub.publish(twist)
                        t1 = rospy.Time.now().to_sec()           
                        current_angle = angular_speed * (t1 - t0)
        
                else:
                    twist.angular.z = +abs(angular_speed)
                    while current_angle < exr_angle:
                        self.cmd_pub.publish(twist)
                        t1 = rospy.Time.now().to_sec()           
                        current_angle = angular_speed * (t1 - t0)

                twist.angular.z = 0

    def main():
        rospy.init_node('turtlebot3_odor')
    try:
        moving = Moving()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

    

    

      

        