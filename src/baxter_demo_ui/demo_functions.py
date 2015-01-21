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

import actionlib
import rospy

from .baxter_procs import (
    mk_process,
    RosProcess,
)

from control_msgs.msg import FollowJointTrajectoryAction

from sensor_msgs.msg import Image as ImageMsg

from .img_proc import (
    cv_to_msg,
    gen_cv,
    overlay,
)

'''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Button Functions
# These functions are called by the buttons in the UI, as referenced
#     in the UI's btn_context dictionary
# Each function is passed a reference to the UI object and the arm/side
#     that OK was pressed on.
# **cam_<>(ui, side) - All call camera_disp for the selected camera.
# **camera_disp(ui, cam_side) - Overlays the output of the selected camera
#                                   on top of the UI's current img.
# **springs(ui, side) - Runs joint_torque_springs for the given arm.
# **puppet(ui, side) - Runs joint_velocity_puppet for the given arm.
# **record(ui, side) - Runs the joint_recorder and sets the play button
#                          as selectable
# **play(ui, side) - Runs joint_trajectory_file_playback on a recorded file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''


def cam_right(ui, side):
    camera_disp(ui, 'right_hand')


def cam_left(ui, side):
    camera_disp(ui, 'left_hand')


def cam_head(ui, side):
    camera_disp(ui, 'head')


def camera_disp(ui, cam_side):
    def _display(camera, name):
        other_cameras = [cam for cam in ui.cameras if name != cam]
        if other_cameras:
            ui.cameras[other_cameras.pop()].close()
        camera.resolution = (640, 400)
        camera.open()

    def _cam_to_screen(msg):
        newMsg = overlay(ui.img, msg,
                         x_overlay_offset=205,
                         y_overlay_offset=140)
        ui.xdisp.publish(newMsg)

    ui.cam_sub = rospy.Subscriber(
        'cameras/%s_camera/image' % cam_side,
        ImageMsg,
        _cam_to_screen)

    camera = ui.cameras[cam_side]
    _display(camera, cam_side)


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
    proc = RosProcess('rosrun baxter_examples '
                       'joint_recorder.py -f recording')
    ui.windows['record_submenu'].set_btn_selectable('play', True)


def play(ui, side):
    left_client = actionlib.SimpleActionClient(
        'robot/limb/left/follow_joint_trajectory',
        FollowJointTrajectoryAction,
    )
    right_client = actionlib.SimpleActionClient(
        'robot/limb/right/follow_joint_trajectory',
        FollowJointTrajectoryAction,
    )
    proc1 = RosProcess('rosrun baxter_interface '
                       'joint_trajectory_action_server.py -m velocity &')
    proc1.run()
    server_online = False
    while not rospy.is_shutdown():
        rospy.sleep(1)
        if (left_client.wait_for_server(rospy.Duration(1.0)) and
            right_client.wait_for_server(rospy.Duration(1.0))):
            server_online = True
            break
    if server_online:
        proc2 = RosProcess('rosrun baxter_examples '
                           'joint_trajectory_file_playback.py -f recording')


'''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Utility Button Functions
# These functions are called by the options in the MoreOptions screen
#     and perform system functions
# **reboot(ui, side) - Reboots the robot using "shutdown -r now"
#                          allowed in sudo-ers file.
# **shutdown(ui, side) - Shuts down the robot using "shutdown -h now"
#                            allowed in the sudo-ers file.
# **calib(ui, side, stage) - Calls run_calibs if the stage is 1 or 0.
#                      Otherwise removes the calibration flag temp file.
# **run_calibs(stage) - Calls run_calib for the appropriate stage,
#                           for each arm.
#                       Writes the new calibration stage to a temp file
#                           and reboots the robot
# **run_calib(stage, side) - Runs the appropriate calibration on the
#                                specified arm.
#                            0 -> tare.py
#                            1 -> calibrate_arm.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''


def reboot(ui=None, side=None):
    mk_process('sudo shutdown -r now')


def shutdown(ui=None, side=None):
    mk_process('sudo shutdown -h now')


'''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Note on calibration functions:
#   The three 'stages' of calibration are:
#   * stage 1: Calibration
#   * stage 2: Tare
#   * stage 3: Done
# When running calibration, the user will run the calib() function from
#   the UI.  At this point, the calib_stage will be 0, so it will call
#   run_calibs() which will run Calibrate on both arms, and write a file
#   setting the stage to 1.
# Upon reboot, the UI will check that file with check_calibs() and note
#   that it is now at stage 1.  At this point, it will change to the
#   run_calib window and run Tare on both arms, re-write the file to
#   set the new stage to 2 and reboot again
# On this final run, check_calibs() will note that we are on stage 2
#   and will not navigate to the run_calib screen.  It will just call
#   cailb() which will note the stage, and simply delete the stage file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''


def calib(ui, side=None):
    if ui.calib_stage < 2:
        run_calibs(ui)
    else:
        mk_process('rm -rf /var/tmp/hlr/demo_calib.txt')
        ui.calib_stage = 0


def run_calibs(ui):
    for side in ['left', 'right']:
        if run_calib(ui.calib_stage, side) != 0:
            ui.set_active_window('retry_calib_conf')
            return 0
    f = open('/var/tmp/hlr/demo_calib.txt', 'w')
    f.write('stage %s' % (ui.calib_stage + 1))
    reboot()


def run_calib(stage, side):
    if stage == 0:
        return mk_process('rosrun baxter_tools calibrate_arm.py -l %s' % side)
    elif stage == 1:
        return mk_process('rosrun baxter_tools tare.py -l %s' % side)


# Checks for temp calibration file on startup and runs calibrations if found.
def check_calib(ui):
    try:
        f = open('/var/tmp/hlr/demo_calib.txt', 'r')
        stage = f.read()
        ui.calib_stage = int(stage.split()[1])
        if ui.calib_stage < 2:
            ui.set_active_window('run_calib')
        calib(ui)
    except IOError:
        pass
