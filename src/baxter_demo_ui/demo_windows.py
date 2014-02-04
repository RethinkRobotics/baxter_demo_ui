from copy import copy
from PIL import Image, ImageDraw, ImageFont
import rospkg


class BrrWindow(object):
    def __init__(self, window_data, buttons, share_path):
        self.name = window_data['name']
        self.parent = window_data['parent']
        if self.parent and len(self._buttons) > 1:
            self._selected_btn_index = 1
        else:
            self._selected_btn_index = 0

        self._no_scroll = window_data['no_scroll'] 
        self._selected = window_data['default']

        self._font = ImageFont.truetype('%s/FreeSerif.ttf' %
                                           share_path, 25)
        self._bg = dict()
        self._bg['normal'] = Image.open('%s/Panels/%s.png' %
                                           (share_path,
                                            window_data['bg']))
        self._bg['offset'] = (window_data['offset'][0],
                             window_data['offset'][1])
        self._bg['size'] = self._bg['normal'].size
        if window_data['bg'] == 'Panels_Main':
            self._bg['disabled'] = Image.open('%s/Panels/'
                                                 '%s_DarkBkg.png' %
                                                  (share_path, 
                                                   window_data['bg']))
        else:
            self._bg['disabled'] = self.bg['normal']
        self._buttons = {} 
        for name, btn in buttons.items():
            self._buttons[btn.index] = btn
        
        self._states = {}
        self._states['normal'] = {}
        self._states['disabled'] = {}
        for id in self._buttons:
            name = self._buttons[id].name
            self._states['normal'][name] = self._gen_img(name, disabled=False)
            self._states['disabled'][name] = self._gen_img(name, disabled=True)

    def draw(self, img, selected=True):
        if selected:
            tmp = self._states['normal'][self.selected_btn().name]
        else:
            tmp = self._states['disabled'][self.selected_btn().name]
        img.paste(tmp, self._bg['offset'])
        return img

    def selected_btn(self):
        return self.buttons[self.selected_btn_index]
    
    def set_btn_selectable(self, index, sel):
        self._buttons[i].selectable = True

    def scroll(self, direction):
        print '--@scroll():  direction=%s' % direction
        if not self._no_scroll:
            i = self._selected_btn_index + direction
            while (i >= 0 and i < len(self.buttons)):
                if self._buttons[i].selectable:
                    self._selected_btn_index = i
                    break
                i += direction

    def _gen_img(self, sel_btn, disabled = False):
        if disabled: 
            tmp = copy(self._bg['disabled'])
        else: 
            tmp = copy(self._bg['normal'])
        d = ImageDraw.Draw(tmp)
        for name, btn in self._buttons.items():
            if btn.name == sel_btn:
                if disabled:
                    state = 'pressed'
                else:
                    state = 'selected'
            else:
                if disabled:
                    state = 'disabled'
                else:
                    state = 'idle'
            btn.draw(tmp, d, state)
        return tmp
