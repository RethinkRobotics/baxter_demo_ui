#!/usr/bin/python

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

from PIL import (
    Image,
    ImageFont
)

import rospy
import rospkg

from baxter_interface import (
  Navigator,
  RobotEnable,
  CameraController,
  Gripper
)
from sensor_msgs.msg import Image as ImageMsg

from .baxter_procs import (
    kill_python_procs,
    mk_process,
    python_proc_ids,
    RosProcess,
)

from .demo_buttons import BrrButton
from .demo_windows import BrrWindow

from .img_proc import (
    cv_to_msg,
    gen_cv,
    overlay,
)

import demo_functions


'''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# UI wrapper class for Baxter Research Robot.
# This class has 2 purposes:
#    1) To handle user interaction with the robot and interpret
#            those interactions in a UI context
#    2) To run robot utility functions and button_press functions
#
#
# Initializiation aguments:
#    windows - a dictionary of BrrWindow objects
#              (all of the windows configured in the UI)
#    btn_context - a dictionary containing metadata for the buttons
#                  (what functions they run and what window they take you to
#    commands - a list of strings to be searched through when killing
#                  running example programs
#    share_path - path to this package's share/ folder
#
# Public Parameters:
#    img - The current image being displayed by the UI, in cv format
#    windows - Dict with the full list of windows in the UI
#    active_window - The BrrWindow object currently selected in the UI
#    xdisp - Publisher for xdisplay topic
#    cameras - Dict of Camera objects from baxter_interface
#    camera_sub - Camera subscriber.
#
# Public Methods:
#    selected(self) - Returns the BrrButton selected in the current active
#                         BrrWindow object.
#    draw(self) - Draws windows recursively, sets self.img, and publishes
#                     to the screen.
#    scroll(self, direction) - Calls the scroll function for the active
#                                  window passing the scroll direction.
#    ok_pressed(self, v, side) - Enacts the function for the selected button
#                                    in the active window,
#    back(self, v) - If in a window with a back button, this will kill all
#                        examples and set the previous window as active.
#    kill_examples(self, v) - Kills all processes matching the criteria in
#                                 self.commands
#    error_screen(self, error) - Will display the selected error screen on
#                                    top of the current display.
#                                Sets the error window's parent to preserve
#                                    "back" functionality
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''
class BrrUi(object):
    def __init__(self, share_path, conf_path, commands):
        self.share_path = share_path
        self.conf_path = conf_path
        self.windows = dict()
        self._btn_context = dict()

        self._functions = demo_functions
        self._load_config()

        self.img = Image.new('RGB', (1024, 600), 'white')
        self.active_window = self.windows['demo_1']
        self.xdisp = rospy.Publisher('/robot/xdisplay', ImageMsg, latch=True)

        self._status = RobotEnable()
        self._commands = commands
        self._font = ImageFont.truetype(
                '%s/FreeSerif.ttf' % share_path, 25
        )
        self._textHeight = self._font.getsize('W')[1]
        self._active_example = False

        self._navigators = {'left': Navigator('left'),
                           'right': Navigator('right')}

        # Navigator OK Button
        self._navigators['left'].button0_changed.connect(self._left_ok_pressed)
        self._navigators['right'].button0_changed.connect(
            self._right_ok_pressed)

        # Navigator Wheel
        self._navigators['left'].wheel_changed.connect(self._left_wheel_moved)
        self._navigators['right'].wheel_changed.connect(
            self._right_wheel_moved)

        # Navigator Baxter Button
        self._navigators['left'].button2_changed.connect(self._enable)
        self._navigators['right'].button2_changed.connect(self._enable)

        # Navigator Back Button
        self._navigators['left'].button1_changed.connect(self.back)
        self._navigators['right'].button1_changed.connect(self.back)

        self._wheel_ok = True
        self._wheel_states = {'left': self._navigators['left'].wheel,
                             'right': self._navigators['right'].wheel}

        self.cameras = dict()
        for cam in ['left_hand', 'right_hand', 'head']:
            try:
                self.cameras[cam] = CameraController('%s_camera' % cam)
            except AttributeError:
                self.windows['cam_submenu'].set_btn_selectable('cam_%s' % cam,
                                                               False)
                sel = self.windows['cam_submenu'].selected_btn()
                bad_cam = self.windows['cam_submenu'].get_btn('cam_%s' % cam)
                if (sel == bad_cam and
                    not self.windows['cam_submenu'].scroll(1)):
                    self.windows['cam_submenu'].selected_btn_index = 0

        self.cam_sub = None

        self._l_grip = {'interface': Gripper('left'), 'type': 'custom'}
        self._r_grip = {'interface': Gripper('right'), 'type': 'custom'}
        rospy.Timer(rospy.Duration(.5), self._update_grippers)

        self._enable()
        mk_process('rosrun baxter_tools tuck_arms.py -u')

    def _load_config(self):
        f = open(self.conf_path).read()
        conf_data = json.loads(f)
        print "?"
        for window in conf_data['Windows']:
            buttons = dict()
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
                                          '', True, self.share_path)
                self._btn_context[name] = {'nextWindow': window['parent'],
                                         'function': 'Back'}
            try:
                for btn in window['Buttons']:
                    buttons[btn['name']] = BrrButton(
                                    btn['name'], btn['size'],
                                    btn['offset'], btn['index'],
                                    btn['image_prefix'],
                                    btn['inner'], '',
                                    btn['selectable'], self.share_path)
                    self._btn_context[btn['name']] = {
                                            'nextWindow': btn['nextWindow'],
                                            'function': btn['function']}
            except:
                pass

            self.windows[window['name']] = BrrWindow(window,
                                                     buttons,
                                                     self.share_path)

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
        if not self._active_example and self._wheel_ok:
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
        self.draw()

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
            if func == 'Back':
                self.kill_examples()
            self.active_window = self.windows[context['nextWindow']]
            self.draw()
            if func and func != 'Back':
                getattr(self._functions, func)(self, side)

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
        error_screen = '%s_error' % error
        self.windows[error_screen].parent = self.active_window.name
        self.active_window = self.windows(error_screen)
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
