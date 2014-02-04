#!/usr/bin/env python

# Copyright (c) 2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import copy
import json
import os
from PIL import Image, ImageFont

import rospy
import rospkg

from baxter_interface import Navigator, RobotEnable, CameraController, Gripper
from baxter_demo_ui import (
    BrrButton,
    BrrWindow,
    cv_to_msg,
    gen_cv,
    kill_python_procs,
    mk_process,
    overlay,
    python_proc_ids,
    RosProcess,
)
from sensor_msgs.msg import Image as ImageMsg


class BrrUi(object):
    def __init__(self, windows, btn_context, commands, share_path):

        self.img = Image.new('RGB', (1024, 600), 'white')
        self.windows = windows
        self.active_window = self.windows['demo_1']

        self._xdisp = rospy.Publisher('/robot/xdisplay', ImageMsg, latch=True)
        self._status = RobotEnable()
        self._commands = commands
        self._font = ImageFont.truetype(
                '%s/FreeSerif.ttf' % share_path, 25
        )
        self._btn_context = btn_context
        self._textHeight = self.font.getsize('W')[1]
        self._active_example = False

        self._navigators = {'left': Navigator('left'), 
                           'right': Navigator('right')}

        # Navigator OK Button
        self._navigators['left'].button0_changed.connect(self.left_ok_pressed)
        self._navigators['right'].button0_changed.connect(self.right_ok_pressed)

        # Navigator Wheel
        self._navigators['left'].wheel_changed.connect(self.left_wheel_moved)
        self._navigators['right'].wheel_changed.connect(self.right_wheel_moved)

        # Navigator Baxter Button
        self._navigators['left'].button2_changed.connect(self.enable)
        self._navigators['right'].button2_changed.connect(self.enable)

        # Navigator Back Button
        self._navigators['left'].button1_changed.connect(self.back)
        self._navigators['right'].button1_changed.connect(self.back)

        self._wheel_ok = True
        self._wheel_states = {'left': self.navigators['left'].wheel,
                             'right': self.navigators['right'].wheel}

        self.cameras = {'left_hand': CameraController('left_hand_camera'),
                        'right_hand': CameraController('right_hand_camera'),
                        'head': CameraController('head_camera')}
        self.cam_sub = None

        self._l_grip = {'interface': Gripper('left'), 'type': 'custom'}
        self._r_grip = {'interface': Gripper('right'), 'type': 'custom'}
        rospy.Timer(rospy.Duration(.5), self._update_grippers)

        self.enable()
        mk_process('rosrun baxter_tools tuck_arms.py -u')

    

    def selected(self):
        return self.active_window.selected_btn()

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Main Draw function.
    # Converts the appropriate frame to a ros message and sends
    #     it to the screen.
    # Also sets the current_frame parameter, in expectation of
    #     future hooks to merge images into the current view
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''

    def draw(self):
        img = Image.new('RGB', (1024, 600), 'white')
        print '--@UI.draw():  window = %s' % self.active_window.name
        img = gen_cv(self._draw_window(img, self.active_window.name))
        self.img = img
        msg = cv_to_msg(img)
        self.xdisp.publish(msg)
        rospy.sleep(.1)

    def _draw_window(self, img, window, selected=True):
        if self.windows[window].parent:
            img = self._draw_window(img,
                                   window=self.windows[window].parent, 
                                   selected=False)
        return self.windows[window].draw(img, selected)

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Functions linking wheel turns with scrolling in the UI
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''

    def _left_wheel_moved(self, v):
        self._wheel_moved(v, 'left')

    def _right_wheel_moved(self, v):
        self._wheel_moved(v, 'right')

    def _wheel_moved(self, v, side):
        if not self._active_example and self.wheel_ok:
            wheel = self._wheel_states[side]
            if v > wheel and v - wheel < 100:
                self.scroll(1)
            else:
                self.scroll(-1)
            self._wheel_states[side] = v
            self._wheel_ok = False
            rospy.Timer(rospy.Duration(.01), self._set_wheel_ok, oneshot=True)

    def scroll(self, direction):
        self.active_window.scroll(direction)

    def _set_wheel_ok(self, event):
        self._wheel_ok = True

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Functions linking pressing the OK button on either arm with
    #     the currently selected example
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''

    def _left_ok_pressed(self, v):
        self.ok_pressed(v, 'left')

    def _right_ok_pressed(self, v):
        self.ok_pressed(v, 'right')

    def ok_pressed(self, v, side):
        if v == True:
            context = self._btn_context[self.selected().name]
            func = self._btn_context[self.selected().name]['function']
            if func == "Back":
                self.kill_examples()
            self.active_window = self.windows[context['nextWindow']]
            self.draw()
            if func and func != "Back":
                globals()[func](self, side)

    def back(self, v):
        if v == True:
            if self.active_window.parent:
                self.kill_examples()
                self.active_window = self.windows[self.active_window.parent]
                self.draw()

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Commands to enable the robot (if it is disabled when the demo
    #     starts) and to kill all currently running examples.
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''

    def kill_examples(self, v=1):
        print '--@kill_examples'
        self._active_example = False
        self.selected()._status = 'selected'
        for cmd in self._commands:
            kill_python_procs(cmd)
        for camera in self.cameras:
            self.cameras[camera].close()
        if self.cam_sub != None:
            self.cam_sub.unregister()
        self.draw()
        self._enable()

    def _enable(self, v=1):
        if v == 1:
            try:
                self._status.enable()
            except:
                self.error_screen('stopped')
                return False
            if not self._status.state().enabled:
                self.error_screen('no_enable')
        self._enable_cuff()

    def error_screen(self, error):
        self.windows[error].parent = self.active_window.name
        self.active_window = self.windows(error)
        self.draw()

    def _enable_cuff(self):
        if len(python_proc_ids('gripper_cuff_control')) == 0:
            RosProcess('rosrun baxter_examples gripper_cuff_control.py')

    def _update_grippers(self, event):
        new_l = self._l_grip['interface'].type()
        new_r = self._r_grip['interface'].type()
        if new_l != self._l_grip['type']:
            self._l_grip['type'] = new_l
            if new_l == 'electric':
                self._l_grip['interface'].calibrate()
        if new_r != self._r_grip['type']:
            self._r_grip['type'] = new_r
            if new_r == 'electric':
                self._r_grip['interface'].calibrate()
        self._enable_cuff()


def cam_right(ui, side):
    camera_disp(ui, 'right_hand')

def cam_left(ui, side):
    camera_disp(ui, 'left_hand')

def cam_head(ui, side):
    camera_disp(ui, 'head')

def camera_disp(ui, side):
    def _display(camera, name):
        camera.close()
        camera.resolution = (640, 400)
        camera.open()

    def _cam_to_screen(msg):
        newMsg = overlay(ui.img, msg, (1024, 600), (205, 140, 640, 400))
        ui.xdisp.publish(newMsg)

    ui.cam_sub = rospy.Subscriber(
        'cameras/%s_camera/image' % side,
        ImageMsg,
        _cam_to_screen)

    camera = ui.cameras[side]
    _display(camera, '%s_camera' % side)

def springs(ui, side):
    proc = RosProcess('rosrun baxter_examples '
                       'joint_torque_springs.py -l %s' % side)

def puppet(ui, side):
    proc = RosProcess('rosrun baxter_examples '
                       'joint_velocity_puppet.py -l %s' % side)

def wobbler(ui, side):
    proc = RosProcess('rosrun baxter_examples '
                       'joint_velocity_wobbler.py')
    proc.process.stdin.close()

def record(ui, side):
    proc = RosProcess('rosrun baxter_examples' 
                       'joint_recorder.py -f recording')
    ui.windows['record_submenu'].set_btn_selectable(2, True)

def play(ui, side):
    proc1 = RosProcess('rosrun baxter_interface '
                        'joint_trajectory_action_server.py &')
    rospy.sleep(1)
    proc2 = RosProcess('rosrun baxter_examples' 
                        'joint_trajectory_file_playback.py -f recording -l 0')

def tare(ui, side):
    calib()

def reboot(ui, side):
    mk_process('shutdown -r now')

def shutdown(ui, side):
    mk_process('shutdown -h now')

def calib(ui, stage=0):
    print stage
    if stage == 0 or stage == 1:
        run_calibs(stage)
    else:
        mk_process('rm -rf /var/tmp/hlr/calib.txt')

def run_calibs(stage):
    f = open('/var/tmp/hlr/calib.txt', 'w')
    f.write('stage %s' % (stage + 1))
    for side in ['left', 'right']:
        if run_calib(stage, side) == 0:
            ui.error_screen('calib_error')
            return 0
    exit_with_return_code('EXIT_REBOOT')

def run_calib(stage, side):
    if stage == 0:
        return mk_process('rosrun baxter_tools calibrate_arm.py -l %s' % side)
    elif stage == 1:
        return mk_process('rosrun baxter_tools tare.py -l %s' % side)

def check_calib():
    try:
        f = open('/var/tmp/hlr/calib.txt', 'r')
        stage = f.read()
        calib(int(stage.split()[1]))
    except IOError:
        pass

def main():
    rospy.init_node('rsdk_demo_ui')
    rp = rospkg.RosPack()
    pack_path = rp.get_path('baxter_demo_ui') + '/share'

    f = open('%s/config.json' % pack_path).read()
    conf_data = json.loads(f)


    windows = {}
    btn_context = {}
    for window in conf_data['Windows']:
        buttons = {}
        if window['back']:
            name = '%s_back' % window['name']
            size = window['back']['size']
            offset = window['back']['offset']
            if window['parent']:
                img_pref = 'Back'
                inner = True
            else:
                img_pref = 'MainBack'
                inner = False
            buttons[name] = BrrButton(name, size, offset, 0,
                                      img_pref, inner,
                                      '', True, pack_path)
            btn_context[name] = {'nextWindow': window['parent'],
                                     'function': 'Back'}
        try:
            for btn in window['Buttons']:
                buttons[btn['name']] = BrrButton(btn['name'], btn['size'],
                                                 btn['offset'], btn['index'],
                                                 btn['image_prefix'], btn['inner'],
                                                 '', btn['selectable'], pack_path)
                btn_context[btn['name']] = {'nextWindow': btn['nextWindow'],
                                            'function': btn['function']}
        except:
            pass

        windows[window['name']] = BrrWindow(window, buttons, pack_path)

    commands = ['joint_torque', 'wobbler', 
                'puppet', 'joint', 
                'baxter_interface', 'baxter_examples']
    ui = BrrUi(windows, btn_context, commands, pack_path)
    ui.draw()
    check_calib()

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
