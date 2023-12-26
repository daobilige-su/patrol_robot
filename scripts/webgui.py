import math

from nicegui import ui
from nicegui.events import MouseEventArguments
from PIL import Image

import rospy
import rospkg
from geometry_msgs.msg import Twist
import numpy as np
import yaml

from patrol_robot.srv import *
from std_msgs.msg import Float32MultiArray

class Demo:
    def __init__(self):
        self.number = 1


class WebUI:
    def __init__(self):
        # load params
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('patrol_robot')+'/'
        params_filename = self.pkg_path+'param/'+'webgui_params.yaml'
        with open(params_filename, 'r') as file:
            self.param = yaml.safe_load(file)

        # load map
        map_yaml_filename = self.pkg_path+'map/'+self.param['map']
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
        self.grid_size = self.param['grid_size']
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
        self.ii_mouse_arrow_content = ''
        self.task_show_on = 0

        self.map_grid_on = 0
        self.map_axis_on = 0

        self.task_list = np.zeros((20,10)) # 0: stop, 1: move_base, 2: line_track, 3: take_photo, ... 9: do nothing
        self.task_list[:, 0] = np.ones((20,))*9

        self.map_mouse_down_pt_pix = [0.0, 0.0]
        self.map_mouse_down_pt_m = [0.0, 0.0]
        self.map_mouse_up_pt_pix = [0.0, 0.0]
        self.map_mouse_up_pt_m = [0.0, 0.0]
        self.map_mouse_is_down = 0

        self.robot_pose_cur = self.param['robot_pose_init']

        with ui.row():
            ui.label('5G工业巡检机器人UI控制界面')
            ui.label('Patrol Robot Web GUI (Powered by NiceGUI)')

        with ui.row():
            with ui.card().classes('w-[52rem] h-[52rem] bg-orange-3 border'):
                ui.button('Map: ')
                self.sa = ui.scroll_area().classes('w-[50rem] h-[40rem]')
                with self.sa:
                    # ii = ui.image(self.map_pgm).classes('w-[100rem]')
                    self.ii = ui.interactive_image(self.map_pgm, on_mouse=self.map_mouse_handler, events=['mousedown', 'mouseup', 'mousemove'], \
                                                   cross=True).classes(f'w-[{self.map_show_size[0]/self.param["map_vis_shrink_factor"]}rem]')
                    self.ii.content += self.ii_fixed_content
                    # self.ii.content += '<line x1="50" y1="50" x2="250" y2="50" stroke="red" stroke-width="5" marker-end="url(#arrow)"/>'
                with ui.row():
                    self.map_axis_sw = ui.switch('Axis On', value=True, on_change=self.map_axis_handler)
                    self.map_axis_handler()
                    self.map_grid_sw = ui.switch('Grid On', value=True, on_change=self.map_grid_handler)
                    self.map_grid_handler()
                    self.map_center_but = ui.button('Map Center', on_click=self.map_center_handler)
            with ui.card().classes('border').classes('w-[15rem] h-[52rem]'):
                ui.button('Control: ')
                control_chbox = ui.checkbox('Enable', value=True)
                with ui.column().bind_visibility_from(control_chbox, 'value'):
                    ui.label('Joystick: [send cmd_vel]')
                    ui.joystick(color='blue', size=50,
                                # on_move=lambda e: coordinates.set_text(f"{e.x:.3f}, {e.y:.3f}"),
                                on_move=lambda e: self.send_cmd_vel(float(e.y), float(-e.x)),
                                on_end=lambda _: self.send_cmd_vel(0, 0))
                    self.joystick_label = ui.label('[vx = 0, w = 0]')
                    # ui.slider(min=1, max=3).bind_value(self.demo, 'number')
                    # ui.toggle({1: 'A', 2: 'B', 3: 'C'}).bind_value(self.demo, 'number')
                    # ui.number().bind_value(self.demo, 'number')
            with ui.card().classes('border').classes('w-[42rem] h-[52rem]'):
                ui.button('Task Client: ')
                task_chbox = ui.checkbox('Enable', value=True)
                with ui.column().bind_visibility_from(task_chbox, 'value'):
                    ui.label('Current Task:')
                    self.aggrid = ui.aggrid({
                        'defaultColDef': {'flex': 1},
                        'columnDefs': [
                            {'headerName': 'Index', 'field': 'index'}, #, 'checkboxSelection': True},
                            {'headerName': 'Action', 'field': 'action'},
                            {'headerName': 'P1', 'field': 'p1'},
                            {'headerName': 'P2', 'field': 'p2'},
                            {'headerName': 'P3', 'field': 'p3'},
                            {'headerName': 'P4', 'field': 'p4'},
                            {'headerName': 'P5', 'field': 'p5'},
                            {'headerName': 'P6', 'field': 'p6'},
                            {'headerName': 'P7', 'field': 'p7'},
                            {'headerName': 'P8', 'field': 'p8'},
                            {'headerName': 'P9', 'field': 'p9'},
                        ],
                        'rowData': [
                            {'index': 1, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 2, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 3, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 4, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 5, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 6, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 7, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 8, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 9, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 10, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 11, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 12, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 13, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 14, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 15, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 16, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 17, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 18, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 19, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                            {'index': 20, 'action': 0, 'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0, 'p8': 0, 'p9': 0},
                        ],
                        'rowSelection': 'single',
                    }).classes('w-[40rem]')
                    self.update_aggrid()
                    with ui.row():
                        ui.button('Clear', on_click=self.task_list_clear)
                        ui.button('Send', on_click=self.task_list_send)
                        ui.button('Stop', on_click=self.task_list_stop)
                        self.task_list_show_sw = ui.switch('Show On Map', on_change=self.task_list_show)
                    with ui.row():
                        ui.label('Stop:       ').classes('w-24')
                        ui.button('Insert', on_click=self.task_list_insert_stop)
                    with ui.row():
                        ui.label('Move Base:  ').classes('w-24')
                        ui.button('Insert', on_click=self.task_list_insert_movebase)
                        ui.label('Param: ')
                        self.movebase_input_x = ui.input('X', value='0.0').classes('w-8')
                        self.movebase_input_y = ui.input('Y', value='0.0').classes('w-8')
                        self.movebase_input_theta = ui.input('Theta', value='0.0').classes('w-8')
                    with ui.row():
                        ui.label('Line Track: ').classes('w-24')
                        ui.button('Insert', on_click=self.task_list_insert_line_track)



        # ROS
        rospy.init_node('webgui_node')
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # establish client server communication
        if self.param['server_connection']:
            print('starting client')
            rospy.wait_for_service('TaskList')
            print('service connected.')

        self.task_list_request = rospy.ServiceProxy('TaskList', TaskList)
        self.task_list_msg = Float32MultiArray()





        ui.run(title='Patrol Robot Web GUI', reload=False, show=False)

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
            self.map_mouse_is_down = 1
        elif e.type == 'mouseup':
            self.map_mouse_up_pt_pix[0] = e.image_x
            self.map_mouse_up_pt_pix[1] = e.image_y
            self.map_mouse_up_pt_m[0] = (e.image_x - self.map_ct_px[0]) * self.map_res
            self.map_mouse_up_pt_m[1] = -(e.image_y - self.map_ct_px[1]) * self.map_res
            theta = math.atan2(self.map_mouse_up_pt_m[1]-self.map_mouse_down_pt_m[1] ,self.map_mouse_up_pt_m[0]-self.map_mouse_down_pt_m[0])
            self.movebase_input_theta.value = theta

            arrow_end_pt_pix = [0.0, 0.0]
            arrow_end_pt_pix[0] = self.map_mouse_down_pt_pix[0] + math.cos(theta)*self.param['map_arrow_len']/self.map_res
            arrow_end_pt_pix[1] = self.map_mouse_down_pt_pix[1] - math.sin(theta)*self.param['map_arrow_len']/self.map_res
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
                arrow_end_pt_pix[0] = self.map_mouse_down_pt_pix[0] + math.cos(theta) * self.param['map_arrow_len'] / self.map_res
                arrow_end_pt_pix[1] = self.map_mouse_down_pt_pix[1] - math.sin(theta) * self.param['map_arrow_len'] / self.map_res
                self.ii_mouse_arrow_content = f'<line x1="{self.map_mouse_down_pt_pix[0]}" y1="{self.map_mouse_down_pt_pix[1]}" ' + \
                                      f'x2="{arrow_end_pt_pix[0]}" y2="{arrow_end_pt_pix[1]}" ' + \
                                      'stroke="yellow" stroke-width="5" marker-end="url(#arrow)"/>'
                self.map_ii_content_handler()


    def send_cmd_vel(self, vx, w):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = vx
        cmd_vel_msg.angular.z = w
        self.joystick_label.set_text(f"cmd_vel: [vx = {vx:.3f}, w = {w:.3f}]")
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def map_center_handler(self):
        # scroll_to sets upper left conner, so minus half of the size of area, e.g. horizontal: 50 rem / 2 * 16
        self.sa.scroll_to(pixels=self.map_ct_px[0]/self.param['map_vis_shrink_factor']-25*16, axis='horizontal')
        self.sa.scroll_to(pixels=self.map_ct_px[1]/self.param['map_vis_shrink_factor']-20*16, axis='vertical')

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

    def map_ii_content_handler(self):
        self.ii_fixed_content = self.ii_content_arrowdef
        if self.map_axis_on:
            self.ii_fixed_content += self.ii_content_axis
        if self.map_grid_on:
            self.ii_fixed_content += self.ii_content_grid

        self.ii_add_content = self.ii_mouse_arrow_content
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
        for n in range(20):
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
        self.task_list = np.zeros((20, 10))
        self.task_list[:, 0] = np.ones((20,)) * 9
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
        self.task_list = np.zeros((20, 10))
        self.update_aggrid()
        self.compute_task_content()
        self.map_ii_content_handler()

    def task_list_send(self):
        if self.param['server_connection']:
            task_list = np.zeros((20, 10))

            tasks_num = self.task_list.shape[0]
            task_list[0:tasks_num, :] = self.task_list

            task_list_flatten = task_list.reshape((1, -1))[0]
            task_list_flatten_list = task_list_flatten.tolist()

            self.task_list_msg.data = task_list_flatten_list

            rospy.loginfo('send TaskList request: ')
            resp = self.task_list_request(self.task_list_msg)
            rospy.loginfo('response is: %s' % (resp))
        ui.notify('task_list sent')

    async def task_list_insert_stop(self):
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
                                                   self.movebase_input_theta.value, 0, 0, 0, 0, 0, 0])
            self.update_aggrid()
            self.compute_task_content()
            self.map_ii_content_handler()
        else:
            ui.notify('No row selected!')

    async def task_list_insert_line_track(self):
        row = await self.aggrid.get_selected_row()
        if row:
            row_idx = row['index'] - 1
            self.task_list[row_idx, :] = np.array([2, 7, 0, 0, 0, 0, 0, 0, 0, 0])
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

        for n in range(20):
            task = self.task_list[n, :]
            if round(task[0]) == 1:  # move_base
                if pre_task_is_movebase:
                    color = 'blue'
                else:
                    color = 'red'
                xy_pix = self.transform_map_to_image(task[1:3])
                self.ii_task_content += f'<circle cx="{xy_pix[0]}" cy="{xy_pix[1]}" r="{15 / self.param["map_vis_shrink_factor"]}"' + \
                                        f'fill="none" stroke="{color}" stroke-width="4" />'
                self.ii_task_content += self.compute_arrow_content(task[1:4], 'yellow', self.param['map_arrow_len'])

                pre_xy_pix = self.transform_map_to_image(pre_pose[0:2])
                self.ii_task_content += f'<line x1="{pre_xy_pix[0]}" y1="{pre_xy_pix[1]}" x2="{xy_pix[0]}" y2="{xy_pix[1]}"' + \
                                        f'stroke-dasharray="5,5" style="stroke:{color};stroke-width:2" />'
                pre_pose = [task[1], task[2], task[3]]

                pre_task_is_movebase = 1
            elif round(task[0]) == 2:  # line_track
                pre_task_is_movebase = 0

    def compute_arrow_content(self, pose, color, arrow_len):
        start_pt_pix = self.transform_map_to_image(pose[0:2])
        end_pt_pix = [0.0, 0.0]
        end_pt_pix[0] = start_pt_pix[0] + math.cos(pose[2]) * arrow_len / self.map_res
        end_pt_pix[1] = start_pt_pix[1] - math.sin(pose[2]) * arrow_len / self.map_res
        content = f'<line x1="{start_pt_pix[0]}" y1="{start_pt_pix[1]}" ' + f'x2="{end_pt_pix[0]}" y2="{end_pt_pix[1]}" ' + \
                  f'stroke="{color}" stroke-width="5" marker-end="url(#arrow)"/>'
        return content


# if __name__ in {"__main__", "__mp_main__"}:
# rospy.init_node('webgui_node')
# server = line_track_action()
# rospy.spin()
webui = WebUI()


