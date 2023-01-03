from centermask.grasp import grasp
from demo.demo_pill import pill_segmentation, pill_segmentation_mask_first, get_parser, setup_cfg, VisualizationDemo
from figure.utils import create_axs
from detectron2.data.detection_utils import read_image
from centermask.realsense_capture import d415_frames
from auboi5_controller import AuboController
from motor_control import MotorController
import multiprocessing as mp
import time
import numpy as np
# import math
from copy import deepcopy
# import cv2
# import serial

import matplotlib.pyplot as plt

def place_manager(pick_todo_num, presc_list, day):
    target_timing = len(presc_list) - pick_todo_num
    place_row = day  # one day of a week (0:Mon, ..., 6:Sun)
    place_col = presc_list[target_timing]  # one timing of a day

    return place_row, place_col

t = time.time()
def tim(last_time, task_name):
    current_time = time.time()
    past_time = current_time - last_time
    print("Time elapsed since: ", task_name, " - ", past_time)
    return current_time

# usage
# t = tim(t)
# both prints out time elapsed and records down current time


## TODO
# 1. Tune edge cases - for both sweeping and swiping - no offset if at edge.
# 2. Replace medcine

if __name__ == "__main__":
    # robot
    auboi5_controller = AuboController('192.168.1.115')
    # waypoint = auboi5_controller.getCurrentWaypoint()
    # print(waypoint)
    # exit()

    # motor
    mc = MotorController('/dev/ttyUSB0')
    # camera
    cam = d415_frames.camera()
    for i in range(20):
        cam.capture()
    vision_x_error = -0.016
    vis_grasp = True  # visualize grasp detection results
    vis_checkexist = False
    push_experiment = False
    

    # Create obj
    mp.set_start_method("spawn", force=True)
    args = get_parser().parse_args()
    cfg = setup_cfg(args)    
    demo = VisualizationDemo(cfg, parallel=False)

    # Prescription for one day
    prescription_timing = {
                            # 'pill_A':[1,2], 'pill_B':[2], 'pill_C':[1,3], 'pill_D':[1], 
                            # 'pill_E':[3], 'pill_F':[1,4], 'pill_G':[1,4], 'pill_H':[4]
                            'pill_A':[], 'pill_B':[], 'pill_C':[1,1,1,1,1,1,1,1,1,1], 'pill_D':[], 
                            'pill_E':[], 'pill_F':[], 'pill_G':[], 'pill_H':[]
                          }  # TIMING: 0 for 8:00, 1 for 12:00, 2 for 17:00, 3 for 21:00         

    plt.close('all')
    fig, axs = create_axs(np.ones((720,1280,3)), 1)


    # Process the prescription
<<<<<<< HEAD
    for day in range(1,8):
        for pill_name, presc_list in prescription_timing.items():

            pick_todo_num = len(presc_list)
            if pick_todo_num<= 0:
                continue  # skip

            if push_experiment == False:
                # Grasp SOME without looking for the first time
                mc.go_position(mc.find_box(pill_name[-1]))
                time.sleep(0.1)
                auboi5_controller.moveJ(auboi5_controller.find_capture_position(pill_name[-1])) 
                auboi5_controller.pick_one_time(auboi5_controller.find_grasping_pose(pill_name[-1]),5, 0.08-0.003) # z_offset has nothing to do with depth that it 'digs' into canister
                auboi5_controller.place_one_time(auboi5_controller.find_place_pose(pill_name[-1]), 0.04)  # normally use this
                # auboi5_controller.place_one_time(auboi5_controller.medicine_box_position(1, 1))
                t=tim(t, "push_experiment==False stuff")
=======
    for pill_name, presc_list in prescription_timing.items():

        pick_todo_num = len(presc_list)
        if pick_todo_num<= 0:
            continue  # skip

        if push_experiment == False:
            # Grasp SOME without looking for the first time
            mc.go_position(mc.find_box(pill_name[-1]))
            time.sleep(0.1)
            auboi5_controller.moveJ(auboi5_controller.find_capture_position(pill_name[-1])) 
            auboi5_controller.pick_one_time(auboi5_controller.find_grasping_pose(pill_name[-1]),5, 0.08-0.003) # z_offset has nothing to do with depth that it 'digs' into canister
            auboi5_controller.place_one_time(auboi5_controller.find_place_pose(pill_name[-1]), 0.04)  # normally use this
            # auboi5_controller.place_one_time(auboi5_controller.medicine_box_position(1, 1))

        while True:
            # Look at the platform
            auboi5_controller.moveJ(auboi5_controller.find_capture_position(pill_name[-1])) 
            
            cam.capture()
            cam.capture()
            img = cam.color_image
            img_ins_seg, img_sem_seg, centroids = pill_segmentation_mask_first(img)
            # img_ins_seg, img_sem_seg = pill_segmentation(img)
            num_pill_on_platform = grasp.check_obj_exist(img, img_ins_seg, img_sem_seg, vis_checkexist)

            
            # Pick&Place-loop on platform
            if num_pill_on_platform > 0:
                # If there is any pill on the platform
>>>>>>> parent of 85b35a0 (Added timing in main)

            while True:
                # Look at the platform
                auboi5_controller.moveJ(auboi5_controller.find_capture_position(pill_name[-1])) 
                
<<<<<<< HEAD
                t=tim(t, "moveJ")
                cam.capture()
                cam.capture()
                img = cam.color_image
                t=tim(t, "Get color image")

                img_ins_seg, img_sem_seg, centroids = pill_segmentation_mask_first(img, demo)
                t=tim(t, "segmentation")

                # img_ins_seg, img_sem_seg = pill_segmentation(img)
                num_pill_on_platform = grasp.check_obj_exist(img, img_ins_seg, img_sem_seg, vis_checkexist)
                t=tim(t, "grasp.check_obj_exist")

=======
                if pick_todo_num <= 0:
                # Reflow (Existed but order fulfilled)
                    # reflow to storehouse from platform
                    print("\n reflow ...")
                    # If there is NO more pills requested from this pill_name (order already fulfilled)
                    # SWEEP back into the box
                    motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.push(img, centroids, pill_name[-1], auboi5_controller.cTo_z_, axs)
                
                else:
                # If there IS  more pills requested from this pill_name (order not fulfilled)

                    # if len(plt.get_fignums())==0 or len(fig.figure.axes) != 1:
                    # fig, axs = create_axs(cam.color_image, 1, 'Segmentation Results')
                    
                    # Logic included in grasp.think:
                    # If there is no good picking spots -> Push to separate
                    # If there is good picking spots -> Pick (Existed and order not fulfilled)
                    motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.think(img, img_ins_seg,
                                                                                            img_sem_seg, auboi5_controller.cTo_z_, vis_grasp, axs, centroids, pill_name[-1])
                
                fig.canvas.draw()
                fig.canvas.flush_events()
>>>>>>> parent of 85b35a0 (Added timing in main)
                
                # Pick&Place-loop on platform
                if num_pill_on_platform > 0:
                    # If there is any pill on the platform

                    
                    if pick_todo_num <= 0:
                    # Reflow (Existed but order fulfilled)
                        # reflow to storehouse from platform
                        print("\n reflow ...")
                        # If there is NO more pills requested from this pill_name (order already fulfilled)
                        # SWEEP back into the box
                        motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.push(img, centroids, pill_name[-1], auboi5_controller.cTo_z_, axs)
                        t=tim(t, "grasp.push")

                    else:
                    # If there IS  more pills requested from this pill_name (order not fulfilled)

                        # if len(plt.get_fignums())==0 or len(fig.figure.axes) != 1:
                        # fig, axs = create_axs(cam.color_image, 1, 'Segmentation Results')
                        
                        # Logic included in grasp.think:
                        # If there is no good picking spots -> Push to separate
                        # If there is good picking spots -> Pick (Existed and order not fulfilled)
                        motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.think(img, img_ins_seg,
                                                                                                img_sem_seg, auboi5_controller.cTo_z_, vis_grasp, axs, centroids, pill_name[-1])
                        t=tim(t, "grasp.think")

                    fig.canvas.draw()
                    fig.canvas.flush_events()
                    t=tim(t, "canvas.draw()")
                    
                    # revise the sys x error in vision
                    push_start[0] += vision_x_error
                    push_end[0] += vision_x_error
                    grasp_coord[0] += vision_x_error

                    # Results
                    # 0:pushing, 1:swiping, >=2:picking
                    if motion_command == 0: 
                        print('push_start:', push_start)
                        print('push_end:', push_end)
                        ps_xy = auboi5_controller.eye_hand_transfer(push_start)
                        pe_xy = auboi5_controller.eye_hand_transfer(push_end)
                        auboi5_controller.set_aside(ps_xy, pe_xy)

                    elif motion_command == 1:
                        print('sweep_start:', push_start)
                        print('sweep_end:', push_end)
                        ps_xy = auboi5_controller.eye_hand_transfer(push_start)
                        pe_xy = auboi5_controller.eye_hand_transfer(push_end)
                        auboi5_controller.set_sweep(ps_xy, pe_xy)
                        

                    elif motion_command >= 2 and motion_command<=7:
                        print('grasp_coord:', grasp_coord)
                        print('grasp_angle:', grasp_angle)
                        print('grasp_opening:', grasp_opening)
                        print('class:', motion_command)  # pill_a:2, pill_b:3, pill_c:4, pill_d:5, pill_e:6, pill_f:7

                        p1_xy = auboi5_controller.eye_hand_transfer(grasp_coord)
                        rad = auboi5_controller.rad_transfer(grasp_angle)
                        p1 = auboi5_controller.assign_pick_point(p1_xy[0], p1_xy[1], rad, auboi5_controller.pick_z_)
                        width1 = auboi5_controller.opening_width_mapping(grasp_opening)
                        print('opening width =', width1, grasp_opening)

                        auboi5_controller.pick_one_time(p1,width1)

                        # place_row, place_col = auboi5_controller.decide_place_row_col(motion_command)
                        place_row, place_col = place_manager(pick_todo_num, presc_list, day)
                        place = auboi5_controller.medicine_box_position(place_row, place_col)
                        auboi5_controller.place_one_time(place)
                        # auboi5_controller.led_control(place_row, place_col)
                        #  p1 = [0.2781040168082825, -0.24489740891213108, 0.22399999999999998, 0.0, 0.7082208269885911, 0.7059909774349783, 0.0]
                        # p1_pre = zoffset=0.05
                        # place = [-0.12593, -0.390813, 0.181801, 0.0, 1.0, 0.0, 0.0]

                        # Update
                        pick_todo_num -= 1
                    
                    t=tim(t, "Grasping or pushing")

                else:
                    # If nothing is on the platform
                    if pick_todo_num <= 0:
                        # no need to grasp SOME or reflow
                        break
                    
<<<<<<< HEAD
                    # Grasp SOME without looking
                    mc.go_position(mc.find_box(pill_name[-1]))
                    time.sleep(0.1)
                    auboi5_controller.moveJ(auboi5_controller.find_capture_position(pill_name[-1])) 
                    auboi5_controller.pick_one_time(auboi5_controller.find_grasping_pose(pill_name[-1]),5)
                    auboi5_controller.place_one_time(auboi5_controller.find_place_pose(pill_name[-1]))
                    t=tim(t, "Grasp SOME without looking")
                

=======
                    # Update
                    pick_todo_num -= 1
                    
            else:
                # If nothing is on the platform
                if pick_todo_num <= 0:
                    # no need to grasp SOME or reflow
                    break
                
                # Grasp SOME without looking
                mc.go_position(mc.find_box(pill_name[-1]))
                time.sleep(0.1)
                auboi5_controller.moveJ(auboi5_controller.find_capture_position(pill_name[-1])) 
                auboi5_controller.pick_one_time(auboi5_controller.find_grasping_pose(pill_name[-1]),5)
                auboi5_controller.place_one_time(auboi5_controller.find_place_pose(pill_name[-1]))
            


        # while pick_todo_num > 0 and platform_clear:
        #     # Realsense; Segmentation; Grasp Generation
        #     cam.capture()  # get one frame
        #     img = cam.color_image
        #     img_ins_seg, img_sem_seg = pill_segmentation(img)
        #     vis = False  # visualize grasp detection results
        #     motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.think(img, img_ins_seg,
        #                                                                                                 img_sem_seg, auboi5_controller.cTo_z_, vis)

        #     # revise the sys x error in vision
        #     push_start[0] += vision_x_error
        #     push_end[0] += vision_x_error
        #     grasp_coord[0] += vision_x_error

        #     # Results
        #     # 0:pushing, 1:swiping, >=2:picking
        #     if motion_command == 0:
        #         print('push_start:', push_start)
        #         print('push_end:', push_end)
        #         ps_xy = auboi5_controller.eye_hand_transfer(push_start)
        #         pe_xy = auboi5_controller.eye_hand_transfer(push_end)
        #         auboi5_controller.set_aside(ps_xy, pe_xy)

        #     elif motion_command == 1:
        #         print('sweep_start:', push_start)
        #         print('sweep_end:', push_end)
        #         ps_xy = auboi5_controller.eye_hand_transfer(push_start)
        #         pe_xy = auboi5_controller.eye_hand_transfer(push_end)
        #         auboi5_controller.set_sweep(ps_xy, pe_xy)

        #     elif motion_command >= 2 and motion_command<=7:
        #         print('grasp_coord:', grasp_coord)
        #         print('grasp_angle:', grasp_angle)
        #         print('grasp_opening:', grasp_opening)
        #         print('class:', motion_command)  # pill_a:2, pill_b:3, pill_c:4, pill_d:5, pill_e:6, pill_f:7

        #         p1_xy = auboi5_controller.eye_hand_transfer(grasp_coord)
        #         rad = auboi5_controller.rad_transfer(grasp_angle)
        #         p1 = auboi5_controller.assign_pick_point(p1_xy[0], p1_xy[1], rad, auboi5_controller.pick_z_)
        #         width1 = auboi5_controller.opening_width_mapping(grasp_opening)
        #         print('opening width =', width1, grasp_opening)
        #         auboi5_controller.pick_one_time(p1,width1)

        #         # place_row, place_col = auboi5_controller.decide_place_row_col(motion_command)
        #         place_row, place_col = place_manager(pick_todo_num, presc_list, day)
        #         place = auboi5_controller.medicine_box_position(place_row, place_col)
        #         auboi5_controller.place_one_time(place)
        #         # auboi5_controller.led_control(place_row, place_col)
        #         pick_todo_num -= 1  # update
                
        #     else:
        #         print('Unknown motion type.')
>>>>>>> parent of 85b35a0 (Added timing in main)

            # while pick_todo_num > 0 and platform_clear:
            #     # Realsense; Segmentation; Grasp Generation
            #     cam.capture()  # get one frame
            #     img = cam.color_image
            #     img_ins_seg, img_sem_seg = pill_segmentation(img)
            #     vis = False  # visualize grasp detection results
            #     motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = grasp.think(img, img_ins_seg,
            #                                                                                                 img_sem_seg, auboi5_controller.cTo_z_, vis)

            #     # revise the sys x error in vision
            #     push_start[0] += vision_x_error
            #     push_end[0] += vision_x_error
            #     grasp_coord[0] += vision_x_error

            #     # Results
            #     # 0:pushing, 1:swiping, >=2:picking
            #     if motion_command == 0:
            #         print('push_start:', push_start)
            #         print('push_end:', push_end)
            #         ps_xy = auboi5_controller.eye_hand_transfer(push_start)
            #         pe_xy = auboi5_controller.eye_hand_transfer(push_end)
            #         auboi5_controller.set_aside(ps_xy, pe_xy)

            #     elif motion_command == 1:
            #         print('sweep_start:', push_start)
            #         print('sweep_end:', push_end)
            #         ps_xy = auboi5_controller.eye_hand_transfer(push_start)
            #         pe_xy = auboi5_controller.eye_hand_transfer(push_end)
            #         auboi5_controller.set_sweep(ps_xy, pe_xy)

            #     elif motion_command >= 2 and motion_command<=7:
            #         print('grasp_coord:', grasp_coord)
            #         print('grasp_angle:', grasp_angle)
            #         print('grasp_opening:', grasp_opening)
            #         print('class:', motion_command)  # pill_a:2, pill_b:3, pill_c:4, pill_d:5, pill_e:6, pill_f:7

            #         p1_xy = auboi5_controller.eye_hand_transfer(grasp_coord)
            #         rad = auboi5_controller.rad_transfer(grasp_angle)
            #         p1 = auboi5_controller.assign_pick_point(p1_xy[0], p1_xy[1], rad, auboi5_controller.pick_z_)
            #         width1 = auboi5_controller.opening_width_mapping(grasp_opening)
            #         print('opening width =', width1, grasp_opening)
            #         auboi5_controller.pick_one_time(p1,width1)

            #         # place_row, place_col = auboi5_controller.decide_place_row_col(motion_command)
            #         place_row, place_col = place_manager(pick_todo_num, presc_list, day)
            #         place = auboi5_controller.medicine_box_position(place_row, place_col)
            #         auboi5_controller.place_one_time(place)
            #         # auboi5_controller.led_control(place_row, place_col)
            #         pick_todo_num -= 1  # update
                    
            #     else:
            #         print('Unknown motion type.')

                


