from PIL import Image, ImageDraw, ImageFont
import rospkg


class BrrButton(object):
    def __init__(self, name, size, offset,
                 index, image_prefix, inner,
                 label='', selectable=True, share_path=''):
        self.name = name
        self.index = index
        self.selectable = selectable
       
        self._size = tuple(size)
        self._offset = tuple(offset)
        self._font = ImageFont.truetype('%s/FreeSerif.ttf' %
                                           share_path, 25)
        # "inner" refers to a group of image assets that only have 2 images)
        if inner:
            base_path = ('%s/Buttons/Buttons_Inner_%s' %
                         (self.share_path, image_prefix))
        else:
            base_path = ('%s/Buttons/Buttons_%s' %
                         (self.share_path, image_prefix))
        self._imgs = dict()
        self._imgs['idle'] = Image.open('%s.png' % base_path)
        self._imgs['selected'] = Image.open('%s_Pressed.png' % base_path)
        if inner:
            self._imgs['disabled'] = self._imgs['idle']
            self._imgs['pressed'] = self._imgs['selected']
        else:
            self._imgs['disabled'] = Image.open('%s_Dis.png' % base_path)
            self._imga['pressed'] = Image.open('%s_ON.png' % base_path)
        self._label = label

    def get_image(self, state):
        return self._imgs[state].resize(self._size, Image.ANTIALIAS)

    def draw_label(self, draw):
        label_x = (self._offset[0] + self._size[0]/2 -
                   draw.textsize(self._label, self._font)[0]/2)
        label_y = self._offset[1] + self._size[1]
        draw.text((label_x, label_y), self.i_label, fill='white', font = self._font)

    def draw(self, img, draw, state):
        tmp = self.get_image(state)
        self.draw_label(draw)
        img.paste(tmp, self.offset)

