import numpy as np
# from ikfastpy import ikfastpy
import robotcontrol
import time
from scipy.spatial.transform import Rotation as Rot
from multiprocessing import shared_memory
import threading
from vacuum.suction import PythonSerialDriver
# from srt_serial import PythonSerialDriver
from copy import deepcopy
import math
import serial

    
class AuboController (threading.Thread):
    def __init__(self, robot_ip_):
        self.robot = robotcontrol.Auboi5Robot()
        self.robot.initialize()
        handle = self.robot.create_context()
        port = 8899
        result = self.robot.connect(robot_ip_,port)
        if result != robotcontrol.RobotErrorType.RobotError_SUCC:
            print("connection failed!!")
        self.robot.robot_startup()
        self.robot.init_profile()
        # self.speed_scale_ = 0.7 # I CHANGED THIS! Sept 14
        self.speed_scale_ = 0.2
        joint_maxvelc = (self.speed_scale_*2.0, self.speed_scale_*2.0, self.speed_scale_*2.0, self.speed_scale_*2.0, self.speed_scale_*2.0, self.speed_scale_*2.0)
        joint_maxacc  = (self.speed_scale_*3.0, self.speed_scale_*3.0, self.speed_scale_*3.0, self.speed_scale_*3.0, self.speed_scale_*3.0, self.speed_scale_*3.0)
        line_maxvelc  = self.speed_scale_*1.0
        line_maxvelc  = self.speed_scale_*1.0

        self.robot.set_joint_maxvelc(joint_maxvelc)
        self.robot.set_joint_maxacc(joint_maxacc)
        self.robot.set_end_max_line_velc(line_maxvelc)
        self.robot.set_end_max_line_acc(line_maxvelc)

        self.suction = PythonSerialDriver()
        # self.srt = PythonSerialDriver()
        
        
        self.cTo_z_ = 0.326 # Camera to hand distance
        self.pick_z_ = 0.134 + 0.018 - 0.005 + 0.062 - 0.01# I believe this is the picking platforms, when they are being picked up (not when put down :O )
        self.sweep_z_ = 0.140 + 0.012 - 0.003 +0.063 + 0.01

        # I believe this is the dropoff
        self.medicine_box_position_origin_ = [-0.11568, -0.458656, 0.151801+0.005+0.025] # -0.11568, -0.457156, 0.151801

        # self.wms = np.full((7, 4, 6), False)
        p = 0.5
        self.wms = np.random.choice(a=[False, True], size=(7, 4, 6), p=[p, 1-p])  
        # days_per_week=7, times_per_day=4, pills_per_taking=6, grid_occupied=False

        # self.led = serial.Serial('/dev/ttyUSB1', 9600)


    def moveJ(self, joints):
        self.robot.move_joint(joints)

    def moveL(self, pose):
        xyz = (pose[0],pose[1],pose[2])
        ori = (pose[3],pose[4],pose[5],pose[6])
        self.robot.move_to_target_in_cartesian(xyz,ori)

    def getCurrentWaypoint(self,):
        current_status = self.robot.get_current_waypoint()
        # print(current_status)
        return current_status

    # def pos_srt(self, pressure = 70):
    #     self.srt.move3Fingers(True, pressure)

    # def neg_srt(self, pressure = 60):
    #     self.srt.move3Fingers(False, pressure)        

    # def zero_srt(self, pressure = 0):
    #     self.srt.move3Fingers(True, pressure)

    def opening_width_mapping(self,opening):
        if opening >= 60:
            width = 4 #4
        elif opening >= 53:
            width = 3 #3
        elif opening >= 30:
            width = 2 #2
        elif opening >= 10:
            width = 1
        else:
            width = 0
        return width

    def width_to_pressure(self,width):
        if width >= 5:
            pressure = 80
        elif width == 4:
            pressure = 50 # for pills 60 
        elif width == 3:
            pressure = 40 # for pills 55
        elif width == 2:
            pressure = 30 # for pills 45
        elif width == 1:
            pressure = 20
        else:
            pressure = 0
        # if width >= 5:
        #     pressure = 80
        # elif width == 4:
        #     pressure = 50
        # elif width == 3:
        #     pressure = 40
        # elif width == 2:
        #     pressure = 30
        # elif width == 1:
        #     pressure = 20
        # else:
        #     pressure = 0
        return pressure

    def pick_and_place_one_time(self,point,width,zoffset=0.05): 
        # auboi5_controller.moveJ(auboi5_controller.find_capture_position(pill_name[-1])) 
        # auboi5_controller.find_grasping_pose(pill_name[-1]),5, 0.08-0.003)
        pressure = self.width_to_pressure(width)
        # self.pos_srt(pressure) # Open gripper

        point_pre = deepcopy(point)
        point_pre[2] += zoffset
        self.moveL(point_pre) # Move to above grab
        self.suction.activate()
        self.moveL(point)     # Move to grab

        # self.neg_srt(60) # Grab (close gripper)

        self.moveL(point_pre) # Move back to above grab

        # def place_one_time(self,point,zoffset=0.08,pressure=60):
        # auboi5_controller.find_place_pose(pill_name[-1]), 0.04
        point_pre = deepcopy(point)
        point_pre[2] += zoffset
        
        self.moveL(point_pre)   # Move to above drop
        self.moveL(point)       # Move to drop
        # self.neg_srt(0)       # Open gripper
        # self.pos_srt(pressure)
        self.suction.deactivate()

        self.moveL(point_pre)   # Move to above drop
        # self.neg_srt(0)         # Open gripper

    def go_to_above_picking_pose(self, point, width, zoffset=0.05):
        # self.suction.deactivate() # open gripper
        point_pre = deepcopy(point)
        point_pre[2] += zoffset
        self.moveL(point_pre)

    def pick_one_time(self,point,width,zoffset=0.05): 
        pressure = self.width_to_pressure(width)
        # self.pos_srt(pressure)
        # self.suction.deactivate() # open gripper

        point_pre = deepcopy(point)
        point_pre[2] += zoffset
        self.moveL(point_pre)
        self.suction.activate() 
        # self.pos_srt()
        # print("im in point_pre")
        self.moveL(point)
        # self.pos_srt(0)
        # time.sleep(0.1)
        # self.neg_srt(60) # close gripper

        # print("im in point")

        self.moveL(point_pre)
        # print("im in point_pre")

    def place_one_time(self,point,zoffset=0.08,pressure=60):
        point_pre = deepcopy(point)
        point_pre[2] += zoffset
        self.moveL(point_pre)
        self.moveL(point)
        # self.neg_srt(0)
        # self.pos_srt(pressure) # open gripper
        self.suction.deactivate()

        self.moveL(point_pre)
        # self.neg_srt(0) # open gripper

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

    def assign_pick_point(self,x,y,rad,z):
        #the degree from left to right is pi/2 to -pi/2.    0 point to the front.
        quat = self.get_quaternion_from_euler(math.pi,0,rad)
        # print(quat)
        return [x,y,z,0.0,quat[0],quat[1],0.0]
    
    def set_aside(self,start_p_xy,finish_p_xy):
        #neg at firstc
        # self.neg_srt(60) # Set gripper to neutral before push or swiping
        #compute the degree
        k = (start_p_xy[1]-finish_p_xy[1])/(start_p_xy[0]-finish_p_xy[0])
        x_theta = math.atan(abs(k))
        if k>0:
            angle = -(math.pi/2 - x_theta) +1.5708/2
        elif k<0:
            angle = (math.pi/2 - x_theta) -1.5708/2

        start_p = self.assign_pick_point(start_p_xy[0],start_p_xy[1],angle,self.sweep_z_)
        finish_p = self.assign_pick_point(finish_p_xy[0],finish_p_xy[1],angle,self.sweep_z_)
        self.moveL(start_p)
        self.moveL(finish_p)
        # self.pos_srt(40) # Set grippe to neutral before push or swipe
        # self.moveJ(self.photo_joints_)
        # self.neg_srt(0)

    def set_sweep(self,start_p_xy,finish_p_xy):
        #neg at first
        # self.neg_srt(60)  # Set grippe to neutral before push or swipe
        #compute the degree
        k = (start_p_xy[1]-finish_p_xy[1])/(start_p_xy[0]-finish_p_xy[0])
        x_theta = math.atan(abs(k))
        if k>0:
            angle = x_theta
        elif k<0:
            angle = -x_theta

        start_p = self.assign_pick_point(start_p_xy[0],start_p_xy[1],angle,self.sweep_z_)
        finish_p = self.assign_pick_point(finish_p_xy[0],finish_p_xy[1],angle,self.sweep_z_)
        self.moveL(start_p)
        self.moveL(finish_p)
        # self.moveJ(self.photo_joints_)
        # self.pos_srt(0)  # Set grippe to neutral before push or swipe

    def quaternion_to_Tmatrix(self, quaternion):
        """Return homogeneous rotation matrix from quaternion.
        >>> R = quaternion_to_Tmatrix([x,y,z,ow,ox,oy,oz])

        """
        _EPS = np.finfo(float).eps * 4.0
        quat = [quaternion[4],quaternion[5],quaternion[6],quaternion[3]]
        q = np.array(quat, dtype=np.float64, copy=True)
        nq = np.dot(q, q)
        if nq < _EPS:
            return np.identity(4)
        q *= math.sqrt(2.0 / nq)
        q = np.outer(q, q)
        return np.array((
            (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], quaternion[0]),
            (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], quaternion[1]),
            (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], quaternion[2]),
            (                0.0,                 0.0,                 0.0, 1.0)
            ), dtype=np.float64)

    def eye_hand_transfer(self,cTo_xy):    
        # The following matrix should be modified
        # bTw = np.array((
        #     (  1.0,0.0,0.0,0.2387239779144876),
        #     (  0.0,-1.0,0.0,-0.25429673419425003),
        #     (  0.0,0.0,-1.0, 0.3106401367492817),
        #     (                0.0,                 0.0,                 0.0, 1.0)
        #     ), dtype=np.float64)
        
        current_status = self.getCurrentWaypoint()
 
        bTw = np.array((
            (  1.0,0.0,0.0,current_status['pos'][0]),
            (  0.0,-1.0,0.0,current_status['pos'][1]),
            (  0.0,0.0,-1.0, current_status['pos'][2]),
            (0.0,0.0,0.0, 1.0)
            ), dtype=np.float64)

        wTc = np.array((
            (   0.999732 , -0.0182786 , -0.0142087  ,-0.0153094-0.0016),
            (  0.0182479 ,   0.999831 ,-0.00228665  ,  0.066011-0.002),
            (  0.0142481 , 0.00202676 ,   0.999896  , 0.0101006),
            (         0.0,         0.0,         0.0,        1.0)
            ), dtype=np.float64)            
        bTc = np.dot(bTw,wTc)

        cTo = np.array((
            (  1.0,0.0,0.0,cTo_xy[0]),
            (  0.0,1.0,0.0,cTo_xy[1]),
            (  0.0,0.0,1.0, self.cTo_z_),
            (                0.0,                 0.0,                 0.0, 1.0)
            ), dtype=np.float64)
        bTo = np.dot(bTc,cTo)
        # print('the transformed bTo is: ')
        # print(bTo)
        return [bTo[0,3],bTo[1,3]]

    def rad_transfer(self,rad):
        # print('rad is: ',rad)
        if rad >= 0 and rad <= math.pi:
            # rad_transfered = -(math.pi/2 - rad)
            rad_transfered = math.pi/2 - rad
        elif rad < 0 and rad >= -math.pi:
            # rad_transfered = math.pi/2 + rad
            rad_transfered = - (math.pi/2 + rad)
        # print('rad_transfered is: ',rad_transfered)
        return rad_transfered

    def medicine_box_position(self,row,column):
        # starts from row,column=1,1, despite being stated that it starts at 0,0 ....
        X=[-0.12593, -0.12593, -0.104134, -0.081723, -0.059319, -0.0381, -0.0154, 0.005693]
        Y=[-0.455805, -0.455805, -0.421365, -0.390813, -0.3567]
        assert(row > 0)
        assert(column > 0)

        x = X[row]
        y = Y[column]
        z = self.medicine_box_position_origin_[2]
        return [x, y, z, 0.0, 1.0, 0.0, 0.0]
        
    # def medicine_box_position(self,row,column):
    #     # starts from row,column=0,0
    #     row_gap = 0.03
    #     column_gap = 0.029
    #     x = self.medicine_box_position_origin_[0] + (column) * column_gap
    #     y = self.medicine_box_position_origin_[1] + (row) * row_gap
    #     z = self.medicine_box_position_origin_[2]
    #     return [x, y, z, 0.0, 1.0, 0.0, 0.0]

    def decide_place_row_col(self, pill_class):
        # info of each pill is stroed in 3rd axis of self.wms
        wms = self.wms[:,:,pill_class-2]
        
        # Find the first unoccupied grid
        print(wms, '\n', [np.where(wms==False)[0][0],np.where(wms==False)[1][0]])
        search = np.where(wms==False)
        if search[0].size == 0 or search[1].size == 0:
            print("No available grid for placing this kind of pill.")
            return -1, -1  # could be something else
        else:
            row = search[0][0]
            col = search[1][0]
            self.wms[row, col, pill_class-2] = True  # update
            return row, col

    # def led_control(self, row, col):
    #     # Input: row, col of wms
    #     strip_id = col
    #     led_id = row

    #     num_pills = np.sum(self.wms, axis=2)[led_id, strip_id]  # 0,1,2,3
    #     print('num_pills of this cell:', num_pills)
        
    #     # Color Map 1:blue  2:yellow 3: green 0:red
    #     if num_pills >= 1:
    #         # num_pills = 5
    #         led_color = 1  # Blue
    #     else:
    #         led_color = 0  

    #     # Led Control
    #     input_str = str(strip_id) + str(led_id) + str(led_color)
    #     self.led.write(bytes(input_str, encoding = "utf8"))

    def find_place_pose(self, pill_id):
        # Initially z offset is only 0.01
        place_pose_1 = [0.07527888398880958, -0.28993198184034424, 0.21833367897001926+0.012, 0.0003226884309160187, -0.9999996130325639, -0.0006070149194710236, 0.0005489442472058018]
        place_pose_2 = [0.18401451324827875, -0.28993008521408536, 0.21833178043211302+0.012, 0.00033365920891618056, -0.9999995854364833, -0.0006427299017719509, 0.000551993358184889]
        place_pose_3 = [0.29249007993380727, -0.2899204873052338, 0.2183338261013369+0.012, 0.00032909484032954997, -0.9999995778093516, -0.0006623035118159901, 0.0005453730494115345]
        place_pose_4 = [0.4039538787798124, -0.2899226947859364, 0.21833607048550713+0.012, 0.00032430204966982066, -0.999999581121588, -0.0006621136254581226, 0.0005423931931768068]

        if pill_id == 'A':
            return place_pose_1
        elif pill_id == 'B':
            return place_pose_2
        elif pill_id == 'C':
            return place_pose_3
        elif pill_id == 'D':
            return place_pose_4
        elif pill_id == 'E':
            return place_pose_4
        elif pill_id == 'F':
            return place_pose_3
        elif pill_id == 'G':
            return place_pose_2
        elif pill_id == 'H':
            return place_pose_1
        else:
            print('Pill ID error!')
            return place_pose_1

    def find_grasping_pose(self, pill_id):
        # Grasp SOME
        # ROW 2, Y Limits: -0.15 to -0.19 
        # ROW 1, Y Limits: -0.37 to -0.43
        x_rand_range = 0.02
        y_rand_range = 0.02
        X_POS = [0.0755, 0.1827, 0.2930, 0.4012] # Col 1,2,3,4
        Y_POS = [-0.400, -0.167]# -0.147] # Row 1,2
        Z_POS = 0.1606959209601539 - 0.015 - 0.005
        # Z_POS = 0.1606959209601539
        
        
        R =  [0.00032001149681460804, -0.9998324819174745, 0.01829231174002348, 0.0005450014594178438]
        grasp_transform_1 = [X_POS[0], Y_POS[0], Z_POS]
        grasp_transform_2 = [X_POS[1], Y_POS[0], Z_POS]
        grasp_transform_3 = [X_POS[2], Y_POS[0], Z_POS]
        grasp_transform_4 = [X_POS[3], Y_POS[0], Z_POS]
        grasp_transform_5 = [X_POS[3], Y_POS[1], Z_POS]
        grasp_transform_6 = [X_POS[2], Y_POS[1], Z_POS]
        grasp_transform_7 = [X_POS[1], Y_POS[1], Z_POS]
        grasp_transform_8 = [X_POS[0], Y_POS[1], Z_POS]
        
        if pill_id == 'A':
            grasp_transform = grasp_transform_1
        elif pill_id == 'B':
            grasp_transform = grasp_transform_2
        elif pill_id == 'C':
            grasp_transform = grasp_transform_3
        elif pill_id == 'D':
            grasp_transform = grasp_transform_4
        elif pill_id == 'E':
            grasp_transform = grasp_transform_5
        elif pill_id == 'F':
            grasp_transform = grasp_transform_6
        elif pill_id == 'G':
            grasp_transform = grasp_transform_7
        elif pill_id == 'H':
            grasp_transform = grasp_transform_8
        else:
            print('Pill ID error!')
            grasp_transform = grasp_transform_1

        grasp_transform[0]+=np.random.uniform(low=-x_rand_range, high=x_rand_range, size=None)
        grasp_transform[1]+=np.random.uniform(low=-y_rand_range, high=y_rand_range, size=None)
        grasp_transform[0]
        grasp_transform.extend(R)
        return grasp_transform


        # grasp_pose_1 = [0.07906140144767754-0.01, -0.3880466268839969, 0.1627275633806139, 0.00032001149681460804, -0.9998324819174745, 0.01829231174002348, 0.0005450014594178438]
        # grasp_pose_2 = [0.18946229861861963, -0.38804696588168225+0.015, 0.16624678708473153, 0.00031929934115045516, -0.99983241504298, 0.018295950907924973, 0.0005459465429630924]
        # grasp_pose_3 = [0.2973060001262372+0.016, -0.3880447242165994+0.002, 0.16035126311643733 + 0.001, 0.00031893460275061303, -0.9998325064564134, 0.018290877638362967, 0.0005487340190974398]
        # # grasp_pose_4 = [0.4125910363366381+0.013, -0.38804541145333415-0.014, 0.15362441115264502, 0.0003204616550893969, -0.9998324963590879, 0.01829143934923484, 0.0005475173198283822]
        # grasp_pose_4 = [0.4125910363366381+0.013, -0.38804541145333415-0.014, 0.15362441115264502-0.01, 0.0003204616550893969, -0.9998324963590879, 0.01829143934923484, 0.0005475173198283822]
        # # grasp_pose_5 = [0.08325185086063296-0.016, -0.2031378212383125, 0.1646959209601539-0.01, 0.0003215350704869227, -0.9998325074999375, 0.01829086367121273, 0.0005457726149138237]
        # grasp_pose_5 = [0.08325185086063296-0.016, -0.2031378212383125, 0.1646959209601539-0.015, 0.0003215350704869227, -0.9998325074999375, 0.01829086367121273, 0.0005457726149138237]
        # grasp_pose_6 = [0.20036848183773304, -0.1933884766554534-0.017, 0.16591206511559028, 0.00032597591280512207, -0.9998324765998644, 0.018292481803730717, 0.000545513836853285]
        # grasp_pose_7 = [0.3024835164742523-0.006, -0.19338783929484285+0.01, 0.16591206511559028-0.01, 0.0003226885018341782, -0.999832571895182, 0.018287345387264026, 0.0005450213606656084]
        # grasp_pose_8 = [0.41065364670496773+0.012, -0.19339153784697233-0.008, 0.16591206511559028+0.005, 0.00032542499193981576, -0.9998324959975268, 0.018291469507588424, 0.0005442326555055663]



    def find_capture_position(self, pill_id):
        photo_joints_1 = [1.435767412185669, -0.6941367983818054, -2.22825026512146, 0.0371534563601017, -1.5719701051712036, -0.1362195611000061]
        photo_joints_2 = [1.876967430114746, -0.5009561777114868, -2.1527936458587646, -0.08011706173419952, -1.5716582536697388, 0.3049488663673401]
        photo_joints_3 = [2.179381847381592, -0.26175448298454285, -2.0107624530792236, -0.17706304788589478, -1.5713354349136353, 0.6073704361915588]
        photo_joints_4 = [2.380605936050415, -0.015562836080789566, -1.8021048307418823, -0.21452096104621887, -1.5710898637771606, 0.8085911273956299]


        if pill_id == 'A':
            return photo_joints_1
        elif pill_id == 'B':
            return photo_joints_2
        elif pill_id == 'C':
            return photo_joints_3
        elif pill_id == 'D':
            return photo_joints_4
        elif pill_id == 'E':
            return photo_joints_4
        elif pill_id == 'F':
            return photo_joints_3
        elif pill_id == 'G':
            return photo_joints_2
        elif pill_id == 'H':
            return photo_joints_1
        else:
            print('Pill ID error!')
            return photo_joints_1

if __name__ == '__main__':
    auboi5_controller = AuboController('192.168.1.115')
