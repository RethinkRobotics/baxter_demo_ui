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

import rospy
import rospkg
from baxter_demo_ui import BrrUi
from baxter_demo_ui.demo_functions import check_calib

'''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Main loop for the demo mode module.
# Generates UI elements in the following order:
#     Buttons, then Windows, then the full UI.
# For each window:
#     Will generate a back button if the window is configured to need one.
#     Next, each button registered to the Window will be instantiated.
#     Finally, the window will be instantiated with a list of buttons
# After all windows are instantiated, the BrrUi class will be instantiated.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''


def main():
    rospy.init_node('rsdk_demo_ui')
    rp = rospkg.RosPack()
    pack_path = rp.get_path('baxter_demo_ui') + '/share'
    conf_path = '%s/config.json' % pack_path

    commands = ['baxter_interface', 'record', 'playback', 'puppet',
                'tare', 'calibrate', 'joint_trajectory']
    ui = BrrUi(pack_path, conf_path, commands)
    check_calib(ui)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()
