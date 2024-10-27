#! /usr/bin/env python
import math

from nicegui import ui
from nicegui.events import MouseEventArguments
from PIL import Image
import pandas as pd

import rospy
import rospkg
from geometry_msgs.msg import Twist
import numpy as np
import yaml

from patrol_robot.srv import *
from std_msgs.msg import Float32MultiArray, Int8

from threading import Thread
import tf
from tf import transformations


class WebUI:
    def __init__(self):
        # load params
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('patrol_robot')+'/'
        params_filename = rospy.get_param('param_yaml_file')
        with open(params_filename, 'r') as file:
            self.param = yaml.safe_load(file)

        self.task_list_file = self.pkg_path + 'traj/traj.csv'

        # load map
        map_yaml_filename = self.pkg_path+'map/'+self.param['loc']['env_map']
        with open(map_yaml_filename, 'r') as file:
            self.map_yaml = yaml.safe_load(file)
        map_pgm_filename = self.pkg_path+'map/'+self.map_yaml['image']
        self.map_pgm = Image.open(map_pgm_filename)
        self.map_size = self.map_pgm.size # w,h
        self.map_res = self.map_yaml['resolution']
        self.map_ct_m = [-self.map_yaml['origin'][0], self.map_size[1]*self.map_res+self.map_yaml['origin'][1]] # origin: The 2-D pose of the lower-left pixel in the map, as (x, y, yaw)
        self.map_ct_px = [self.map_ct_m[0]/self.map_res, self.map_ct_m[1]/self.map_res]

        self.map_show_size = [self.map_size[0]/16.0, self.map_size[1]/16.0] # in rem, 1 rem = 16 pixels

        self.bg_color = '#ddeeff' # 'bg_color' blue, '#ffeedd' orange
        ui.query('body').style(f'background-color: {self.bg_color}')

        # self.demo = Demo()
        self.grid_size = self.param['web_gui']['grid_size']
        self.ii = None
        # add html svg arrow definition to ii_fixed_content
        self.ii_content_arrowdef = '<defs><marker id="arrow" markerWidth="10" markerHeight="10" refX="0" refY="3" orient="auto" markerUnits="strokeWidth">' + \
                                '<path d="M0,0 L0,6 L9,3 z" fill="#000"></path></marker></defs>'
        self.ii_fixed_content = self.ii_content_arrowdef
        self.ii_add_content = ''
        self.ii_content_axis = ''
        self.compute_map_axis_content()
        self.ii_content_grid = ''
        self.compute_map_grid_content()
        self.ii_task_content = ''
        self.ii_robot_pose_content = ''
        self.ii_mouse_arrow_content = ''
        self.task_show_on = 0
        self.movebase_input_ck_pt = 0

        self.map_grid_on = 0
        self.map_axis_on = 0
        self.map_robot_on = 0

        self.task_list = np.zeros((10,10)) # 0: stop, 1: move_base, 2: line_track, 3: take_photo, ... 9: do nothing
        self.task_list[:, 0] = np.ones((self.task_list.shape[0],))*9

        self.map_mouse_down_pt_pix = [0.0, 0.0]
        self.map_mouse_down_pt_m = [0.0, 0.0]
        self.map_mouse_up_pt_pix = [0.0, 0.0]
        self.map_mouse_up_pt_m = [0.0, 0.0]
        self.map_mouse_is_down = 0

        self.robot_pose_cur = self.param['web_gui']['robot_pose_init']

        # self.joystick_vx_max = 0.5
        # self.joystick_w_max = 0.8

        self.loc_status = 0  # 0: lidar, 1:gps
        self.loc_state_sub = rospy.Subscriber('/loc_state', Int8, self.loc_status_cb)
        self.gps_status = 0  # 6: rtk-dual

        with ui.row():
            ui.label('5G工业巡检机器人UI控制界面')
            ui.label('Patrol Robot Web GUI (Powered by NiceGUI)')

        with ui.row():
            with ui.card().classes('w-[52rem] h-[57rem] bg-orange-3 border'):
                ui.button('Map: ')
                self.sa = ui.scroll_area().classes('w-[50rem] h-[40rem]')
                with self.sa:
                    # ii = ui.image(self.map_pgm).classes('w-[100rem]')
                    self.ii = ui.interactive_image(self.map_pgm, on_mouse=self.map_mouse_handler, events=['mousedown', 'mouseup', 'mousemove'], \
                                                   cross=True).classes(f'w-[{self.map_show_size[0]/self.param["web_gui"]["map_vis_shrink_factor"]}rem]')
                    self.ii.content += self.ii_fixed_content
                    # self.ii.content += '<line x1="50" y1="50" x2="250" y2="50" stroke="red" stroke-width="5" marker-end="url(#arrow)"/>'
                with ui.row():
                    self.map_axis_sw = ui.switch('Axis On', value=True, on_change=self.map_axis_handler)
                    self.map_axis_handler()
                    self.map_grid_sw = ui.switch('Grid On', value=True, on_change=self.map_grid_handler)
                    self.map_grid_handler()
                    self.map_robot_sw = ui.switch('Robot On', value=True, on_change=self.map_robot_handler)
                    self.map_robot_handler()
                    self.map_center_but = ui.button('Map Center', on_click=self.map_center_handler)
            with ui.card().classes('border').classes('w-[15rem] h-[57rem]'):
                ui.button('Control: ')
                control_chbox = ui.checkbox('Enable', value=True)
                with ui.column().bind_visibility_from(control_chbox, 'value'):
                    ui.label('Joystick: [send cmd_vel]')
                    with ui.row().classes('items-center'):
                        self.joystick_vx_max = ui.input('max vx', value=str(0.5)).classes('w-16')
                        self.joystick_w_max = ui.input('max w', value=str(0.8)).classes('w-16')
                    with ui.card().classes('w-[13rem] items-center'):
                        ui.joystick(color='blue', size=70,
                                    # on_move=lambda e: coordinates.set_text(f"{e.x:.3f}, {e.y:.3f}"),
                                    on_move=lambda e: self.send_cmd_vel(float(e.y)*float(self.joystick_vx_max.value),
                                                                        float(-e.x)*float(self.joystick_w_max.value)),
                                    on_end=lambda _: self.send_cmd_vel(0, 0))
                    self.joystick_label = ui.label('[vx = 0.000, w = 0.000]')
                    # ui.slider(min=1, max=3).bind_value(self.demo, 'number')
                    # ui.toggle({1: 'A', 2: 'B', 3: 'C'}).bind_value(self.demo, 'number')
                    # ui.number().bind_value(self.demo, 'number')
                ui.button('Status: ')
                status_chbox = ui.checkbox('Check', value=True)
                with ui.column().bind_visibility_from(status_chbox, 'value'):
                    ui.label('Loc Status: ')
                    with ui.row().classes('items-center'):
                        self.loc_status_radio = ui.radio({0: 'Lidar', 1: 'GPS'}, value=0).props('inline')
                    with ui.row().classes('items-center'):
                        gps_status_label_name = ui.label('GPS Status: ')
                        self.gps_status_label = ui.label(str(self.gps_status))
                    with ui.row().classes('items-center'):
                        robot_pose_label_name = ui.label('Robot Pose: ')
                        self.robot_pose_label = ui.label('[%7.3f, %7.3f, %7.3f]' % (self.robot_pose_cur[0], self.robot_pose_cur[1], self.robot_pose_cur[2]))
            with ui.card().classes('border').classes('w-[42rem] h-[57rem]'):
                ui.button('Task Client: ')
                task_chbox = ui.checkbox('Enable', value=True)
                with ui.column().bind_visibility_from(task_chbox, 'value'):
                    ui.label('Current Task:')
                    self.df = pd.DataFrame(np.block([[(np.arange(self.task_list.shape[0])+1).reshape((-1,1)), self.task_list]]), columns=['index', 'action', 'p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'p8', 'p9'])
                    self.aggrid_row = ui.row()
                    with self.aggrid_row:
                        self.aggrid = ui.aggrid.from_pandas(self.df, options={"rowSelection": "single"}).classes('w-[40rem]')
                    with ui.row().classes('items-center'):
                        ui.button('E-Stop', on_click=self.task_list_stop).classes('bg-red-5')
                        ui.button('Clear', on_click=self.task_list_clear)
                        ui.button('Send', on_click=self.task_list_send)
                        ui.button('Add Slots', on_click=self.task_list_add)
                        ui.button('Save', on_click=self.task_list_save)
                        ui.button('Load', on_click=self.task_list_load)
                        self.task_list_show_sw = ui.switch('Show On Map', value=True, on_change=self.task_list_show)
                        self.task_list_show()
                    with ui.row().classes('items-center'):
                        ui.label('Stop:       ').classes('w-24')
                        ui.button('Insert', on_click=self.task_list_insert_stop)
                    with ui.row().classes('items-center'):
                        ui.label('Move Base:  ').classes('w-24')
                        ui.button('Insert', on_click=self.task_list_insert_movebase)
                        ui.label('Param: ')
                        self.movebase_input_x = ui.input('X', value='0.0').classes('w-8')
                        self.movebase_input_y = ui.input('Y', value='0.0').classes('w-8')
                        self.movebase_input_theta = ui.input('Theta', value='0.0').classes('w-8')
                        self.movebase_input_ck_pt = ui.switch('Pre', value=False)
                    with ui.row().classes('items-center'):
                        ui.label('Simple Move Base:  ').classes('w-24')
                        ui.button('Insert', on_click=self.task_list_insert_simple_movebase)
                        ui.label('Param: ')
                        self.simple_movebase_input_x = ui.input('X', value='0.0').classes('w-8')
                        self.simple_movebase_input_y = ui.input('Y', value='0.0').classes('w-8')
                        self.simple_movebase_input_theta = ui.input('Theta', value='0.0').classes('w-8')
                        self.simple_movebase_input_mode = ui.toggle({0: 'Nor', 11: 'Pre', 12: 'Pas'}, value=0).classes('w-8')
                    with ui.row().classes('items-center'):
                        ui.label('Line Track: ').classes('w-24')
                        ui.button('Insert', on_click=self.task_list_insert_line_track)
                    with ui.row().classes('items-center'):
                        ui.label('Photo: ').classes('w-24')
                        ui.button('Insert', on_click=self.task_list_insert_wait)
                        ui.label('Param: ')
                        self.photo_input_ang_azim = ui.input('azim(rad)', value='0.0').classes('w-16')
                        self.photo_input_ang_elev = ui.input('elev(rad)', value='0.0').classes('w-16')
                        self.photo_input_t = ui.input('t(s)', value='0.0').classes('w-8')



        # ROS
        rospy.init_node('webgui_node')
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # establish client server communication
        if self.param['web_gui']['server_connection']:
            print('starting client')
            rospy.wait_for_service('TaskList')
            print('service connected.')

        self.task_list_request = rospy.ServiceProxy('TaskList', TaskList)
        self.task_list_msg = Float32MultiArray()

        self.tf_listener = tf.TransformListener()
        self.thr = Thread(target=self.robot_pose_update)
        self.thr.start()

        ui.run(title='Patrol Robot Web GUI', reload=False, show=False)

    def loc_status_cb(self, msg):
        self.loc_status = msg.data  # 0: lidar, 1:gps
        self.loc_status_radio.value = self.loc_status

    def map_mouse_handler(self, e: MouseEventArguments):
        # color = 'SkyBlue' if e.type == 'mousedown' else 'SteelBlue'
        # self.ii.content += f'<circle cx="{e.image_x}" cy="{e.image_y}" r="15" fill="none" stroke="{color}" stroke-width="4" />'
        # ui.notify(f'{e.type} at ({e.image_x:.1f}, {e.image_y:.1f})')
        if e.type == 'mousedown':
            self.map_mouse_down_pt_pix[0] = e.image_x
            self.map_mouse_down_pt_pix[1] = e.image_y
            self.map_mouse_down_pt_m[0] = (e.image_x-self.map_ct_px[0])*self.map_res
            self.map_mouse_down_pt_m[1] = -(e.image_y-self.map_ct_px[1])*self.map_res
            self.movebase_input_x.value = self.map_mouse_down_pt_m[0]
            self.movebase_input_y.value = self.map_mouse_down_pt_m[1]
            self.simple_movebase_input_x.value = self.map_mouse_down_pt_m[0]
            self.simple_movebase_input_y.value = self.map_mouse_down_pt_m[1]
            self.map_mouse_is_down = 1
        elif e.type == 'mouseup':
            self.map_mouse_up_pt_pix[0] = e.image_x
            self.map_mouse_up_pt_pix[1] = e.image_y
            self.map_mouse_up_pt_m[0] = (e.image_x - self.map_ct_px[0]) * self.map_res
            self.map_mouse_up_pt_m[1] = -(e.image_y - self.map_ct_px[1]) * self.map_res
            theta = math.atan2(self.map_mouse_up_pt_m[1]-self.map_mouse_down_pt_m[1] ,self.map_mouse_up_pt_m[0]-self.map_mouse_down_pt_m[0])
            self.movebase_input_theta.value = theta
            self.simple_movebase_input_theta.value = theta

            arrow_end_pt_pix = [0.0, 0.0]
            arrow_end_pt_pix[0] = self.map_mouse_down_pt_pix[0] + math.cos(theta)*self.param['web_gui']['map_arrow_len']/self.map_res
            arrow_end_pt_pix[1] = self.map_mouse_down_pt_pix[1] - math.sin(theta)*self.param['web_gui']['map_arrow_len']/self.map_res
            self.ii_mouse_arrow_content = f'<line x1="{self.map_mouse_down_pt_pix[0]}" y1="{self.map_mouse_down_pt_pix[1]}" ' + \
                                  f'x2="{arrow_end_pt_pix[0]}" y2="{arrow_end_pt_pix[1]}" ' + \
                                  'stroke="yellow" stroke-width="5" marker-end="url(#arrow)"/>'
            self.map_ii_content_handler()
            self.map_mouse_is_down = 0
        elif e.type == 'mousemove':
            if self.map_mouse_is_down:
                map_mouse_move_pt_pix = [0.0, 0.0]
                map_mouse_move_pt_m = [0.0, 0.0]
                map_mouse_move_pt_pix[0] = e.image_x
                map_mouse_move_pt_pix[1] = e.image_y
                map_mouse_move_pt_m[0] = (e.image_x - self.map_ct_px[0]) * self.map_res
                map_mouse_move_pt_m[1] = -(e.image_y - self.map_ct_px[1]) * self.map_res
                theta = math.atan2(map_mouse_move_pt_m[1] - self.map_mouse_down_pt_m[1], map_mouse_move_pt_m[0] - self.map_mouse_down_pt_m[0])

                arrow_end_pt_pix = [0.0, 0.0]
                arrow_end_pt_pix[0] = self.map_mouse_down_pt_pix[0] + math.cos(theta) * self.param['web_gui']['map_arrow_len'] / self.map_res
                arrow_end_pt_pix[1] = self.map_mouse_down_pt_pix[1] - math.sin(theta) * self.param['web_gui']['map_arrow_len'] / self.map_res
                self.ii_mouse_arrow_content = f'<line x1="{self.map_mouse_down_pt_pix[0]}" y1="{self.map_mouse_down_pt_pix[1]}" ' + \
                                      f'x2="{arrow_end_pt_pix[0]}" y2="{arrow_end_pt_pix[1]}" ' + \
                                      'stroke="yellow" stroke-width="5" marker-end="url(#arrow)"/>'
                self.map_ii_content_handler()

    def send_cmd_vel(self, vx, w):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = vx
        cmd_vel_msg.angular.z = w
        self.joystick_label.set_text(f"[vx = {vx:.3f}, w = {w:.3f}]")
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def map_center_handler(self):
        # scroll_to sets upper left conner, so minus half of the size of area, e.g. horizontal: 50 rem / 2 * 16
        self.sa.scroll_to(pixels=self.map_ct_px[0]/self.param['web_gui']['map_vis_shrink_factor']-25*16, axis='horizontal')
        self.sa.scroll_to(pixels=self.map_ct_px[1]/self.param['web_gui']['map_vis_shrink_factor']-20*16, axis='vertical')

    def map_axis_handler(self):
        if self.map_axis_sw.value:
            self.map_axis_on = 1
        else:
            self.map_axis_on = 0
        self.map_ii_content_handler()

    def map_grid_handler(self):
        if self.map_grid_sw.value:
            self.map_grid_on = 1
        else:
            self.map_grid_on = 0
        self.map_ii_content_handler()

    def map_robot_handler(self):
        if self.map_robot_sw.value:
            self.map_robot_on = 1
        else:
            self.map_robot_on = 0
        self.map_ii_content_handler()

    def map_ii_content_handler(self):
        self.ii_fixed_content = self.ii_content_arrowdef
        if self.map_axis_on:
            self.ii_fixed_content += self.ii_content_axis
        if self.map_grid_on:
            self.ii_fixed_content += self.ii_content_grid

        self.ii_add_content = self.ii_mouse_arrow_content + self.ii_robot_pose_content
        if self.task_show_on:
            self.ii_add_content += self.ii_task_content

        self.ii.content = self.ii_fixed_content + self.ii_add_content

    def compute_map_axis_content(self):
        center = self.map_ct_px
        x_axis_end_pt = [center[0] + 1.0/self.map_res, center[1]]
        y_axis_end_pt = [center[0], center[1] - 1.0/self.map_res]
        self.ii_content_axis = f'<line x1="{center[0]}" y1="{center[1]}" x2="{x_axis_end_pt[0]}" y2="{x_axis_end_pt[1]}" ' + \
                               'stroke="red" stroke-width="5" marker-end="url(#arrow)"/>' + \
                               f'<line x1="{center[0]}" y1="{center[1]}" x2="{y_axis_end_pt[0]}" y2="{y_axis_end_pt[1]}" ' + \
                               'stroke="green" stroke-width="5" marker-end="url(#arrow)"/>'
        #print(self.ii_content_axis)

    def compute_map_grid_content(self):
        center = self.map_ct_px
        grid_inc = self.grid_size/self.map_res
        x_min_idx = np.floor(center[0]/grid_inc)
        x_total_idx = np.ceil(self.map_size[0]/grid_inc)
        y_min_idx = np.floor(center[1] / grid_inc)
        y_total_idx = np.ceil(self.map_size[1] / grid_inc)
        x_min = 0
        x_max = self.map_size[0]
        y_min = 0
        y_max = self.map_size[1]

        self.ii_content_grid = ''
        for n in range(int(x_total_idx)):
            x = (n-x_min_idx)*grid_inc + center[0]
            if x>x_min and x<x_max:
                self.ii_content_grid += f'<line x1="{x}" y1="{y_min}" x2="{x}" y2="{y_max}" stroke-dasharray="5,5" style="stroke:rgb(55,55,55);stroke-width:1" />'
        for n in range(int(y_total_idx)):
            y = (n - y_min_idx) * grid_inc + center[1]
            if y > y_min and y < y_max:
                self.ii_content_grid += f'<line x1="{x_min}" y1="{y}" x2="{x_max}" y2="{y}" stroke-dasharray="5,5" style="stroke:rgb(55,55,55);stroke-width:1" />'

    def update_aggrid(self):
        for n in range(self.task_list.shape[0]):
            self.aggrid.options['rowData'][n]['action'] = self.task_list[n, 0]
            self.aggrid.options['rowData'][n]['p1'] = self.task_list[n, 1]
            self.aggrid.options['rowData'][n]['p2'] = self.task_list[n, 2]
            self.aggrid.options['rowData'][n]['p3'] = self.task_list[n, 3]
            self.aggrid.options['rowData'][n]['p4'] = self.task_list[n, 4]
            self.aggrid.options['rowData'][n]['p5'] = self.task_list[n, 5]
            self.aggrid.options['rowData'][n]['p6'] = self.task_list[n, 6]
            self.aggrid.options['rowData'][n]['p7'] = self.task_list[n, 7]
            self.aggrid.options['rowData'][n]['p8'] = self.task_list[n, 8]
            self.aggrid.options['rowData'][n]['p9'] = self.task_list[n, 9]
        self.aggrid.update()

    def task_list_clear(self): # do nothing
        self.task_list = self.task_list*0.0
        self.task_list[:, 0] = np.ones((self.task_list.shape[0],)) * 9
        self.update_aggrid()
        self.compute_task_content()
        self.map_ii_content_handler()

    def task_list_show(self):
        if self.task_list_show_sw.value:
            self.task_show_on = 1
        else:
            self.task_show_on = 0
        self.compute_task_content()
        self.map_ii_content_handler()

    def task_list_stop(self):  # stop
        self.task_list = self.task_list * 0.0
        self.update_aggrid()
        self.compute_task_content()
        self.map_ii_content_handler()

        if self.param['web_gui']['server_connection']:
            task_list = np.zeros((5, 10))
            task_list_flatten = task_list.reshape((1, -1))[0]
            task_list_flatten_list = task_list_flatten.tolist()

            self.task_list_msg.data = task_list_flatten_list

            rospy.loginfo('send TaskList request: ')
            resp = self.task_list_request(self.task_list_msg)
            rospy.loginfo('response is: %s' % (resp))
            ui.notify('stop cmd sent')

    def task_list_send(self):
        if self.param['web_gui']['server_connection']:
            tasks_num = self.task_list.shape[0]
            task_list = np.zeros((tasks_num, 10))

            task_list[0:tasks_num, :] = self.task_list

            task_list_flatten = task_list.reshape((1, -1))[0]
            task_list_flatten_list = task_list_flatten.tolist()

            self.task_list_msg.data = task_list_flatten_list

            rospy.loginfo('send TaskList request: ')
            resp = self.task_list_request(self.task_list_msg)
            rospy.loginfo('response is: %s' % (resp))
        ui.notify('task_list sent')

    def task_list_add(self):
        self.task_list = np.block([[self.task_list],[np.block([[np.ones((5,1))*9,np.zeros((5,9))]])]])
        self.df = pd.DataFrame(np.block([[(np.arange(self.task_list.shape[0]) + 1).reshape((-1, 1)), self.task_list]]),
                               columns=['index', 'action', 'p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'p8', 'p9'])

        self.aggrid.delete()
        with self.aggrid_row:
            self.aggrid = ui.aggrid.from_pandas(self.df, options={"rowSelection": "single"}).classes('w-[40rem]')
        self.aggrid.update()

    def task_list_save(self):
        np.savetxt(self.task_list_file, self.task_list, delimiter=",")
        # notify user
        ui.notify('task_list saved to: ' + self.task_list_file)

    def task_list_load(self):
        task_list_loaded = np.loadtxt(self.task_list_file, delimiter=',')
        self.task_list = task_list_loaded.copy()

        # update aggrid
        self.df = pd.DataFrame(np.block([[(np.arange(self.task_list.shape[0]) + 1).reshape((-1, 1)), self.task_list]]),
                               columns=['index', 'action', 'p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'p8', 'p9'])
        self.aggrid.delete()
        with self.aggrid_row:
            self.aggrid = ui.aggrid.from_pandas(self.df, options={"rowSelection": "single"}).classes('w-[40rem]')
        self.aggrid.update()
        # refresh task content on the map
        self.compute_task_content()
        self.map_ii_content_handler()

        # notify user
        ui.notify('task_list loaded from: ' + self.task_list_file)

    async def task_list_insert_stop(self): # E-stop
        row = await self.aggrid.get_selected_row()
        if row:
            row_idx = row['index']-1
            self.task_list[row_idx, :] = np.zeros((10,))
            self.update_aggrid()
            self.compute_task_content()
            self.map_ii_content_handler()
        else:
            ui.notify('No row selected!')

    async def task_list_insert_movebase(self):
        row = await self.aggrid.get_selected_row()
        if row:
            row_idx = row['index']-1
            self.task_list[row_idx, :] = np.array([1, self.movebase_input_x.value, self.movebase_input_y.value,
                                                   self.movebase_input_theta.value, float(self.movebase_input_ck_pt.value), 0, 0, 0, 0, 0])
            self.movebase_input_ck_pt.set_value(False)
            self.update_aggrid()
            self.compute_task_content()
            self.map_ii_content_handler()
        else:
            ui.notify('No row selected!')

    async def task_list_insert_simple_movebase(self):
        row = await self.aggrid.get_selected_row()
        if row:
            row_idx = row['index']-1
            self.task_list[row_idx, :] = np.array([2, self.simple_movebase_input_x.value, self.simple_movebase_input_y.value,
                                                   self.simple_movebase_input_theta.value, float(self.simple_movebase_input_mode.value), 0, 0, 0, 0, 0])
            self.update_aggrid()
            self.compute_task_content()
            self.map_ii_content_handler()
        else:
            ui.notify('No row selected!')

    async def task_list_insert_line_track(self):
        row = await self.aggrid.get_selected_row()
        if row:
            row_idx = row['index'] - 1
            self.task_list[row_idx, :] = np.array([3, 7, 0, 0, 0, 0, 0, 0, 0, 0])
            self.update_aggrid()
            self.compute_task_content()
            self.map_ii_content_handler()
        else:
            ui.notify('No row selected!')

    async def task_list_insert_wait(self):
        row = await self.aggrid.get_selected_row()
        if row:
            row_idx = row['index'] - 1
            self.task_list[row_idx, :] = np.array([9, self.photo_input_ang_azim.value, self.photo_input_ang_elev.value,
                                                   self.photo_input_t.value, 0, 0, 0, 0, 0, 0])
            self.update_aggrid()
            self.compute_task_content()
            self.map_ii_content_handler()
        else:
            ui.notify('No row selected!')

    def transform_map_to_image(self, map):
        image = [0.0, 0.0]
        image[0] = (map[0] + self.map_ct_m[0])/self.map_res
        image[1] = (-map[1] + self.map_ct_m[1]) / self.map_res
        return image

    def compute_task_content(self):
        self.ii_task_content = ''
        # ui.notify('task_list show')
        pre_pose = self.robot_pose_cur
        pre_task_is_movebase = 1

        for n in range(self.task_list.shape[0]):
            task = self.task_list[n, :]
            if round(task[0]) == 1:  # move_base
                if pre_task_is_movebase:
                    color = 'blue'
                else:
                    color = 'red'
                if task[4] >0:
                    circle_color = 'green'
                else:
                    circle_color = 'blue'
                xy_pix = self.transform_map_to_image(task[1:3])
                self.ii_task_content += f'<circle cx="{xy_pix[0]}" cy="{xy_pix[1]}" r="{15 / self.param["web_gui"]["map_vis_shrink_factor"]}"' + \
                                        f'fill="none" stroke="{circle_color}" stroke-width="4" />'
                self.ii_task_content += self.compute_arrow_content(task[1:4], 'yellow', self.param["web_gui"]['map_arrow_len'])

                pre_xy_pix = self.transform_map_to_image(pre_pose[0:2])
                self.ii_task_content += f'<line x1="{pre_xy_pix[0]}" y1="{pre_xy_pix[1]}" x2="{xy_pix[0]}" y2="{xy_pix[1]}"' + \
                                        f'stroke-dasharray="5,5" style="stroke:{color};stroke-width:2" />'
                pre_pose = [task[1], task[2], task[3]]

                pre_task_is_movebase = 1
            elif round(task[0]) == 2:  # simple_move_base
                if pre_task_is_movebase:
                    color = 'green'
                else:
                    color = 'red'
                if task[4] == 11:
                    circle_color = 'green'
                elif task[4] == 12:
                    circle_color = 'red'
                else:
                    circle_color = 'blue'
                xy_pix = self.transform_map_to_image(task[1:3])
                self.ii_task_content += f'<circle cx="{xy_pix[0]}" cy="{xy_pix[1]}" r="{15 / self.param["web_gui"]["map_vis_shrink_factor"]}"' + \
                                        f'fill="none" stroke="{circle_color}" stroke-width="4" />'
                self.ii_task_content += self.compute_arrow_content(task[1:4], 'yellow', self.param["web_gui"]['map_arrow_len'])

                pre_xy_pix = self.transform_map_to_image(pre_pose[0:2])
                self.ii_task_content += f'<line x1="{pre_xy_pix[0]}" y1="{pre_xy_pix[1]}" x2="{xy_pix[0]}" y2="{xy_pix[1]}"' + \
                                        f'stroke-dasharray="5,5" style="stroke:{color};stroke-width:2" />'
                pre_pose = [task[1], task[2], task[3]]

                pre_task_is_movebase = 1
            elif round(task[0]) == 3:  # line_track
                pre_task_is_movebase = 0

    def compute_arrow_content(self, pose, color, arrow_len):
        start_pt_pix = self.transform_map_to_image(pose[0:2])
        end_pt_pix = [0.0, 0.0]
        end_pt_pix[0] = start_pt_pix[0] + math.cos(pose[2]) * arrow_len / self.map_res
        end_pt_pix[1] = start_pt_pix[1] - math.sin(pose[2]) * arrow_len / self.map_res
        content = f'<line x1="{start_pt_pix[0]}" y1="{start_pt_pix[1]}" ' + f'x2="{end_pt_pix[0]}" y2="{end_pt_pix[1]}" ' + \
                  f'stroke="{color}" stroke-width="5" marker-end="url(#arrow)"/>'
        return content

    def robot_pose_update(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.ii_robot_pose_content = ''

            if self.map_robot_on:
                # if self.tf_listener.canTransform("/base_link", "/map", rospy.Time().now()):
                try:
                    (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0)) # returns [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]

                    # ai, aj, ak : Euler’s roll, pitch and yaw angles, axes : One of 24 axis sequences as string or encoded tuple
                    # Quaternions ix+jy+kz+w are represented as [x, y, z, w].
                    eul = transformations.euler_from_quaternion([0, 0, rot[2], rot[3]], 'rxyz')
                    self.robot_pose_cur = [trans[0], trans[1], eul[2]]
                    # rospy.logwarn('current robot pose = [%f, %f, %f]' % (self.robot_pose_cur[0], self.robot_pose_cur[1], self.robot_pose_cur[2])) # for debugging

                    # update drawing
                    x_axis_pose = self.robot_pose_cur
                    y_axis_pose = [self.robot_pose_cur[0], self.robot_pose_cur[1], self.robot_pose_cur[2]+(np.pi/2)]
                    self.ii_robot_pose_content = self.compute_arrow_content(x_axis_pose, 'red', self.param['web_gui']['map_arrow_len']/2) + \
                                                 self.compute_arrow_content(y_axis_pose, 'green', self.param['web_gui']['map_arrow_len']/2)

                    # update label
                    self.robot_pose_label.set_text('[%7.3f, %7.3f, %7.3f]' % (self.robot_pose_cur[0], self.robot_pose_cur[1], self.robot_pose_cur[2]))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

            self.map_ii_content_handler()
            rate.sleep()


# if __name__ in {"__main__", "__mp_main__"}:
# rospy.init_node('webgui_node')
# server = line_track_action()
# rospy.spin()
webui = WebUI()
rospy.signal_shutdown('exiting')

