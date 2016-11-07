# view.py
'''
view class map walker
'''
import Tkinter


class MapWalkerView():

    def __init__(self, logger, cmder):
        self.logger = logger
        self.degree = 0
        self.start_name = ''
        self.end_name = ''
        self.cmder = cmder
        self.init_map_panel()

    def go(self):
        self.start_name = str(self.start_name_entry.get())
        self.end_name = str(self.end_entry.get())
        self.cmder.walk_to(start=self.start_name,
                           dst=self.end_name)

    def routines(self):
        self.panel.update()

    def init_map_panel(self):
        self.panel = Tkinter.Tk()
        self.go_btn = Tkinter.Button(self.panel, text="Go",
                                     command=self.go)
        self.go_btn.pack()

        start_sign = Tkinter.StringVar()
        start_label = Tkinter.Label(self.panel,
                                    textvariable=start_sign,
                                    relief=Tkinter.RAISED)
        start_sign.set("start name")
        start_label.pack()

        self.start_name_set = Tkinter.StringVar()
        self.start_name_entry = Tkinter.Entry(self.panel,
                                              textvariable=self.start_name)
        self.start_name_entry.pack()

        end_sign = Tkinter.StringVar()
        end_label = Tkinter.Label(self.panel,
                                  textvariable=end_sign,
                                  relief=Tkinter.RAISED)
        end_sign.set("end name")
        end_label.pack()

        self.end_set = Tkinter.StringVar()
        self.end_entry = Tkinter.Entry(self.panel,
                                       textvariable=self.end_name)
        self.end_entry.pack()
