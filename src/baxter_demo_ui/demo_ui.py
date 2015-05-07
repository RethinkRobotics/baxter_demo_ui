#!/usr/bin/python

# Copyright (c) 2015, Rethink Robotics
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
from sensor_msgs.msg import Image as ImageMsg

from baxter_interface import (
  Navigator,
  RobotEnable,
  CameraController,
  Gripper
)
from baxter_core_msgs.msg import AssemblyState

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
        self.xdisp = rospy.Publisher('/robot/xdisplay', ImageMsg,
                                     latch=True, queue_size=1)

        self._status = RobotEnable()

        self._commands = commands
        self._font = ImageFont.truetype(
                '%s/HelveticaLight.ttf' % share_path, 30
        )
        self._textHeight = self._font.getsize('W')[1]
        self._active_example = False

        self._navigators = {'left': Navigator('left'),
                           'torso_left': Navigator('torso_left'),
                           'right': Navigator('right'),
                           'torso_right': Navigator('torso_right')}

        self._listeners_connected = False
        self._connect_listeners()

        self._estop_state = False
        self._estop_sub = rospy.Subscriber('robot/state', AssemblyState,
                                           self._estop_callback)

        self._wheel_ok = True

        self.cameras = dict()
        camera_list = ['left_hand', 'right_hand', 'head']
        for idx, cam in enumerate(camera_list):
            try:
                self.cameras[cam] = CameraController('%s_camera' % cam)
            except AttributeError:
                try:
                    # This camera might not be powered
                    # Turn off the power to the last camera
                    # this will turn power on to the current camera
                    CameraController('%s_camera' % camera_list[idx-1]).close()
                    # And try again to locate the camera service
                    self.cameras[cam] = CameraController('%s_camera' % cam)
                except AttributeError:
                    # This camera is unavailable (might be broken)
                    # Disable camera button in the UI
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
        rospy.Timer(rospy.Duration(.5), self._check_enable)

        self.error_state = False
        self._enable()
        self.calib_stage = 0
        self.draw()
        mk_process('rosrun baxter_tools tuck_arms.py -u')

    def _estop_callback(self, msg):
        if self._estop_state != msg.stopped:
            self._estop_state = msg.stopped
            if msg.stopped and self._listeners_connected:
                self._disconnect_listeners()
            elif not msg.stopped and not self._listeners_connected:
                self._connect_listeners()

    def _connect_listeners(self):
        # Navigator OK Button
        self._navigators['left'].button0_changed.connect(self._left_ok_pressed)
        self._navigators['torso_left'].button0_changed.connect(
            self._left_ok_pressed)
        self._navigators['right'].button0_changed.connect(
            self._right_ok_pressed)
        self._navigators['torso_right'].button0_changed.connect(
            self._right_ok_pressed)

        # Navigator Wheel
        self._navigators['left'].wheel_changed.connect(self._left_wheel_moved)
        self._navigators['torso_left'].wheel_changed.connect(
            self._left_wheel_moved)
        self._navigators['right'].wheel_changed.connect(
            self._right_wheel_moved)
        self._navigators['torso_right'].wheel_changed.connect(
            self._right_wheel_moved)

        # Navigator Baxter Button
        self._navigators['left'].button2_changed.connect(self._enable)
        self._navigators['torso_left'].button2_changed.connect(self._enable)
        self._navigators['right'].button2_changed.connect(self._enable)
        self._navigators['torso_right'].button2_changed.connect(self._enable)

        # Navigator Back Button
        self._navigators['left'].button1_changed.connect(self.back)
        self._navigators['torso_left'].button1_changed.connect(self.back)
        self._navigators['right'].button1_changed.connect(self.back)
        self._navigators['torso_right'].button1_changed.connect(self.back)

        self._listeners_connected = True

    def _disconnect_listeners(self):
        # Navigator OK Button
        self._navigators['left'].button0_changed.disconnect(
            self._left_ok_pressed)
        self._navigators['torso_left'].button0_changed.disconnect(
            self._left_ok_pressed)
        self._navigators['right'].button0_changed.disconnect(
            self._right_ok_pressed)
        self._navigators['torso_right'].button0_changed.disconnect(
            self._right_ok_pressed)

        # Navigator Wheel
        self._navigators['left'].wheel_changed.disconnect(
            self._left_wheel_moved)
        self._navigators['torso_left'].wheel_changed.disconnect(
            self._left_wheel_moved)
        self._navigators['right'].wheel_changed.disconnect(
            self._right_wheel_moved)
        self._navigators['torso_right'].wheel_changed.disconnect(
            self._right_wheel_moved)

        # Navigator Baxter Button
        self._navigators['left'].button2_changed.disconnect(self._enable)
        self._navigators['torso_left'].button2_changed.disconnect(self._enable)
        self._navigators['right'].button2_changed.disconnect(self._enable)
        self._navigators['torso_right'].button2_changed.disconnect(self._enable)

        # Navigator Back Button
        self._navigators['left'].button1_changed.disconnect(self.back)
        self._navigators['torso_left'].button1_changed.disconnect(self.back)
        self._navigators['right'].button1_changed.disconnect(self.back)
        self._navigators['torso_right'].button1_changed.disconnect(self.back)

        self._listeners_connected = False

    def _load_config(self):
        f = open(self.conf_path).read()
        conf_data = json.loads(f)
        for window in conf_data['Windows']:
            buttons = dict()
            if window['back']:
                name = '%s_back' % window['name']
                size = window['back']['size']
                offset = window['back']['offset']
                icon_prefix = 'Inner_Back'
                icon_offset = window['back']['icon_offset']
                buttons[name] = BrrButton(name, size, offset, 0,
                                          icon_prefix, 'TopSmall', icon_offset,
                                          '', 0, True, self.share_path)
                self._btn_context[name] = {'nextWindow': window['parent'],
                                         'function': 'Back'}
            if 'Buttons' in window.keys():
                for btn in window['Buttons']:
                    buttons[btn['name']] = BrrButton(
                                                     btn['name'],
                                                     btn['size'],
                                                     btn['offset'],
                                                     btn['index'],
                                                     btn['icon_prefix'],
                                                     btn['button'],
                                                     btn['icon_offset'],
                                                     btn['label'],
                                                     btn['label_y'],
                                                     btn['selectable'],
                                                     self.share_path
                                                    )

                    self._btn_context[btn['name']] = {
                        'nextWindow': btn['nextWindow'],
                        'function': btn['function']
                    }

            self.windows[window['name']] = BrrWindow(window,
                                                     buttons,
                                                     self.share_path)

        errors = conf_data['Error']
        for error in errors['errors']:
            name = error['name']
            buttons = dict()
            buttons['OK'] = BrrButton(
                                      '%s_OK' % name,  # name
                                      [200, 60],  # size
                                      errors['OK']['offset'],  # button offset
                                      0,  # index
                                      None,  # icon prefix
                                      "Wide",  # button type
                                      [0, 0],  # icon offset
                                      "OK",  # label
                                      16,  # label y-offset
                                      True,  # selectable?
                                      self.share_path
                                     )
            self._btn_context["%s_OK" % name] = {
                'nextWindow': None,
                'function': 'Back'
            }
            window = {
                'name': '%s_error' % name,
                'bg': errors['bg'],
                'back': False,
                'offset': errors['offset'],
                'parent': False,
                'default': '%s_OK' % name,
                'no_scroll': False,
                'text': [{'text': error['text'],
                           'text_y': error['text_y']}],
            }
            self.windows['%s_error' % name] = BrrWindow(window, buttons,
                                                        self.share_path)

        for win in conf_data['Confirmation']['Windows']:
            conf = conf_data['Confirmation']
            name = win['name']
            labels = list()
            for text in win['messages']:
                labels.append({'text': text['text'],
                              'text_y': text['text_y']})

            buttons = dict()
            buttons['OK'] = BrrButton(
                                      '%s_OK' % name,  # name
                                      [200, 60],  # size
                                      conf['OK']['offset'],  # button offset
                                      1,  # index
                                      None,  # icon prefix
                                      "Wide",  # button type
                                      [0, 0],  # icon offset
                                      win['conf_text'],  # label
                                      16,  # label y-offset
                                      True,  # selectable?
                                      self.share_path)
            self._btn_context['%s_OK' % name] = {
                                        'nextWindow': win['nextWindow'],
                                        'function': win['function']}
            buttons['Cancel'] = BrrButton(
                                          '%s_Back' % name,
                                          [200, 60],
                                          conf['Cancel']['offset'],
                                          0,
                                          None,
                                          "Wide",
                                          [0, 0],
                                          "Cancel",
                                          16,
                                          True,
                                          self.share_path
                                         )
            self._btn_context['%s_Back' % name] = {
                                        'nextWindow': win['parent'],
                                        'function': 'Back'}

            window = {
                    'name': '%s_conf' % win['name'],
                    'bg': conf['bg'],
                    'back': False,
                    'offset': conf['offset'],
                    'parent': win['parent'],
                    'default': '%s_OK' % name,
                    'no_scroll': False,
                    'text': labels
            }
            self.windows['%s_conf' % name] = BrrWindow(window, buttons,
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
        img = gen_cv(self._draw_window(img, self.active_window.name))
        self.img = img
        msg = cv_to_msg(img)
        self.xdisp.publish(msg)
        rospy.sleep(.1)

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Simple method that sets the active window based on the window's name
    #     and re-draws the UI.
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''

    def set_active_window(self, name):
        self.active_window = self.windows[name]
        self.draw()

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
            if v > 0:
                self.scroll(1)
            else:
                self.scroll(-1)
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
                self.error_state = False
                self.kill_examples()
                mk_process('rm -rf /var/tmp/hlr/demo_calib.txt')
            self.set_active_window(context['nextWindow'])
            self.draw()
            if func and func != 'Back':
                getattr(self._functions, func)(self, side)

    def back(self, v):
        if v == True:
            self.error_state = False
            if self.active_window.parent:
                self.kill_examples()
                self.set_active_window(self.active_window.parent)
                self.draw()

    '''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Commands to enable the robot (if it is disabled when the demo
    #     starts) and to kill all currently running examples.
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''

    def kill_examples(self, v=1):
        rospy.loginfo('--@kill_examples')
        self._active_example = False
        self.selected()._status = 'selected'
        for cmd in self._commands:
            kill_python_procs(cmd)
        if self.cam_sub != None:
            self.cam_sub.unregister()
        self.draw()
        if not self.error_state:
            self._enable()

    def _check_enable(self, event):
        self._enable()

    def _enable(self, v=1):
        if v == 1 and not self._status.state().enabled:
            try:
                self._status.enable()
            except:
                self.error_screen('stopped')
                return False
            if not self._status.state().enabled:
                self.error_screen('no_enable')
                return False
        self._enable_cuff()
        return True

    def error_screen(self, error):
        if self.error_state == False:
            self.error_state = error
            self.kill_examples()
            error_screen = '%s_error' % error
            if self.active_window.name.startswith('run'):
                new_parent = self.active_window.parent
            elif not self.active_window.name.endswith('_error'):
                new_parent = self.active_window.name
            self._btn_context['%s_OK' % error]['nextWindow'] = new_parent
            self.windows[error_screen].parent = new_parent
            self.set_active_window(error_screen)
            self.draw()

    def _enable_cuff(self):
        if len(python_proc_ids('gripper_cuff_control')) == 0:
            RosProcess('rosrun baxter_examples gripper_cuff_control.py')
