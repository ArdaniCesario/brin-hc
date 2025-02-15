import os
os.environ['KIVY_WINDOW'] = 'sdl2'

from kivy.app import App
from kivy.config import Config


import main_dashboard as dashboard_v0

class mainApp(App):
    def build(self):
        return dashboard_v0.Dashboard()

if __name__ == '__main__':
    Config.set('graphics', 'fullscreen', 1)
    Config.set('graphics', 'window_state', 'visible')
    Config.set('graphics', 'width', '1280')
    Config.set('graphics', 'height', '720')
    Config.set('kivy', 'keyboard_mode', 'system')
    Config.set('kivy', 'exit_on_escape', 1)
    Config.write()

    mainApp().run()
