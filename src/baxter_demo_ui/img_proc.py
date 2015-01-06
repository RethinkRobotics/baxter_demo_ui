#!/usr/bin/env python

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

import cv2
import cv_bridge
import PIL
import numpy as np


'''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# PIL/cv2 interaction
# The functions in this secion handle interactions between
#   PIL and cvmat formats.
# **gen_msg() converts an RGB PIL image to a BGR cvmat image
# **rgb_to_bgr() converts a RGB PIL image to a BGR PIL image
# **PIL_to_cv() converts a BGR PIL image to a cvmat image
# **cv_to_msg() converts a cvmat image to a rosmsg format
# **msg_to_cv() converts a rosmsg image to a cvmat image
# **overlay() takes an original image, a new image, and an
#    x, y offset as the origin position for the upper left 
#    hand corner (0,0) in terms of the original image
#    and returns the original image with the new image 
#    overlayed in the defined rectangle.
#    For the moment, this is mainly used for the Camera
#       Display demo, overlaying the selected camera's image
#       onto the UI display
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''


def gen_cv(img):
    return PIL_to_cv(rgb_to_bgr(img))


def rgb_to_bgr(img):
    r, g, b = img.split()
    return PIL.Image.merge('RGB', (b, g, r))


def PIL_to_cv(img):
    return np.array(img)


def cv_to_msg(img):
    return cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding='bgr8')


def msg_to_cv(img):
    return cv_bridge.CvBridge().imgmsg_to_cv2(img, desired_encoding='bgr8')


def overlay(old_img, new_img, x_overlay_offset=0, y_overlay_offset=0):
    """
    # Overlays the ROS message new_img on top of the opencv2 old_img
    # using the following coordinate system to map the new_img's
    # origin (0,0) to an (x,y) position on the old image.
    # The coordinate frame for both images the opencv2 standard:
    # 0-->
    # |  x
    # v y
    """
    # Copy out the original image
    tmp = np.copy(old_img)
    # Convert the new image ROS message to an opencv2 image
    sub = msg_to_cv(new_img)
    # Guard against running over the bounds of temp image
    y_overlay_end = min(y_overlay_offset + sub.shape[0], tmp.shape[0])
    y_delta = y_overlay_end - y_overlay_offset
    x_overlay_end = min(x_overlay_offset + sub.shape[1], tmp.shape[1])
    x_delta = x_overlay_end - x_overlay_offset
    # Insert the image in overlay location
    tmp[y_overlay_offset:y_overlay_end, x_overlay_offset:x_overlay_end] = sub[:y_delta,:x_delta]
    return cv_to_msg(tmp)
