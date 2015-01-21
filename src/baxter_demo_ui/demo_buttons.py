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

from PIL import (
  Image,
  ImageDraw,
  ImageFont
)
import rospkg

'''~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Button UI class for Baxter Research Robot Demo Mode
# This class defines and controls the dislpay of a single button
#   within the context of the UI.
#
# Initialization arguments:
#     name - The configured name for this button
#     size - The size (in pixels) of this button's images
#     offset - The x,y location in pixels of the top-right of this button
#     index - The index of the button within its Window's context
#             This is only used for the window generation.  The button
#               never does anything with this information.
#     image_prefix - Filename prefix for image assets
#     inner - Refers to types of button assets.  Inner buttons only have
#               2 distinct images, while non-inner buttons have 4
#     label - The configured label for the button
#     selectable - Bool: should the user be able to scroll to this button?
#     share_path - path to this package's share/ folder
#
# Public Parameters:
#     name - the configured name for this button
#     index - the size (in pixels) of this button's images
#     selectable - Bool: should the user be able to scroll to this button
#
# Public Methods:
#     get_image(self, state) - Returns a resized, antialiased version of the
#                                image for a given state of this button
#     draw_label(self, draw) - Use the passed draw item to add label text
#                                to the button image.
#     draw(self, img, draw, state) - Draws the image at the correct size
#                                      and offset onto the passed img.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'''


class BrrButton(object):
    def __init__(self, name, size, offset,
                 index, icon_prefix, btn_type, icon_offset,
                 label, label_y, selectable=True, share_path=''):
        self.name = name
        self.index = index
        self.selectable = selectable

        self._size = tuple(size)
        self._offset = tuple(offset)
        self._font = ImageFont.truetype('%s/HelveticaLight.ttf' %
                                           share_path, 30)

        btn_path = '%s/Buttons/Buttons_%s' % (share_path, btn_type)
        if icon_prefix == 'None':
            icon_prefix = None
        elif btn_type == 'Inner':
            icon_path = '%s/Icons/Icons_Inner_%s' % (share_path, icon_prefix)
        else:
            icon_path = '%s/Icons/Icons_%s' % (share_path, icon_prefix)

        base_bg = Image.open('%s.png' % btn_path)
        pressed_bg = Image.open('%s_Pressed.png' % btn_path)
        if btn_type in ['Main', 'Inner']:
            dis_bg = Image.open('%s_Dis.png' % btn_path)
        else:
            dis_bg = Image.open('%s.png' % btn_path)

        if icon_prefix == None:
            dis_icon = None
            base_icon = None
        else:
            base_icon = Image.open('%s.png' % icon_path)
            if icon_prefix == 'Inner_Back':
                dis_icon = Image.open('%s.png' % icon_path)
            else:
                dis_icon = Image.open('%s_Dis.png' % icon_path)

        self._imgs = dict()
        self._imgs['idle'] = self.gen_image(base_bg, base_icon, icon_offset)
        self._imgs['selected'] = self.gen_image(pressed_bg,
                                                base_icon,
                                                icon_offset)
        self._imgs['disabled'] = self.gen_image(dis_bg,
                                                dis_icon,
                                                icon_offset)
        self._imgs['pressed'] = self.gen_image(pressed_bg,
                                               dis_icon,
                                               icon_offset)

        self._label = label
        self._label_y = label_y

    def gen_image(self, btn, icon, offset):
        img = btn.resize(self._size, Image.ANTIALIAS)
        if icon != None:
            img.paste(icon, tuple(offset), icon)
        return img

    def draw_label(self, draw):
        label_x = (self._offset[0] + self._size[0] / 2 -
                   draw.textsize(self._label, self._font)[0] / 2)
        label_y = self._offset[1] + self._label_y
        draw.text((label_x, label_y), self._label,
                  fill='white', font=self._font)

    def draw(self, img, draw, state):
        tmp = self._imgs[state]
        img.paste(tmp, self._offset, tmp)
        self.draw_label(draw)
