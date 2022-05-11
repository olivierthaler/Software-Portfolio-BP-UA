from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import *
# https://matplotlib.org/stable/api/index.html
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import matplotlib.patches as patches
# import matplotlib.image as mpimg
# from matplotlib import colors
import numpy as np
import time


class Graph(QWidget):
    CommandUpdate = pyqtSignal([list])

    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        self.fig = Figure()
        self.widget = FigureCanvasQTAgg(self.fig)
        self.layoutvertical = QVBoxLayout(self)
        self.layoutvertical.addWidget(self.widget)
        self.axes = self.widget.figure.add_subplot(111)
        self.widget.show()
        self.position_marker = None
        self.start_marker = None
        self.end_marker = None
        self.grid_marker = None
        self.widget.mpl_connect('button_press_event', self.onClick)

        self.command_queue = []  #commando wachtrij
        self.NewSeq_toggling = False
        self.EditSeq_toggling = False
        self.type = Type(self.axes)

    def setup_axes(self):
        self.axes.set_xlim([0, 800])
        self.axes.set_ylim([0, 150])
        self.axes.set_xlabel("X axis position (mm)")
        self.axes.set_ylabel("Y axis position (mm)")
        self.axes.grid()

    def setup_robotstation(self):
        # --- Draw contour of BaseStation and LiquidStation ---
        self.rect_BaseSt = patches.Rectangle((self.type.BaseSt[0], self.type.BaseSt[1]), self.type.BaseSt[2], self.type.BaseSt[3],
                                             linewidth=2,
                                             edgecolor='black',
                                             facecolor=[0.8275, 0.8275, 0.8275])
        self.axes.add_patch(self.rect_BaseSt)
        self.rect_LiqSt = patches.Rectangle((self.type.LiquidSt[0], self.type.LiquidSt[1]), self.type.LiquidSt[2], self.type.LiquidSt[3],
                                            linewidth=2,
                                            edgecolor='black',
                                            facecolor=[0.8275, 0.8275, 0.8275])  # RGB-waarden lichtgrijs
        self.axes.add_patch(self.rect_LiqSt)

        # --- Draw liquid boxes ---
        for j in range(12):
            self.axes.add_patch(patches.Rectangle((self.type.LiquidSt[0] + self.type.LiqBox_edgeX +
                                                   (self.type.LiqBox_distancex + self.type.LiqBox_width)*j, self.type.LiqBox_edgeY),
                                                    self.type.LiqBox_width, self.type.LiqBox_depth,
                                                    facecolor=[0.3010, 0.7450, 0.9330],  # RGB-waarden lichtblauw
                                                    edgecolor="black",
                                                    linewidth=1))
        self.widget.draw()

    def drawBaskets(self):
        for i in range(len(self.type.zuurkast_setup)):
            if self.type.zuurkast_setup[i]['Basket'] is not None:
                baseX, baseY = self.type.location(i)
                if i == 0:
                    self.axes.add_patch(patches.Rectangle((baseX + (self.type.BaseSt[2] - self.type.Basket_width) / 2,
                                                           baseY + (self.type.BaseSt[3] - self.type.Basket_depth) / 2),
                                                            self.type.Basket_width, self.type.Basket_depth,
                                                            facecolor=[0.6706, 0.9294, 0.8275],  # appelblauwzeegroen
                                                            edgecolor="green",
                                                            linewidth=1))
                else:
                    self.axes.add_patch(patches.Rectangle((baseX + (self.type.LiqBox_width - self.type.Basket_width) / 2,
                                                           baseY + (self.type.LiqBox_depth - self.type.Basket_depth) / 2),
                                                            self.type.Basket_width, self.type.Basket_depth,
                                                            facecolor=[0.6706, 0.9294, 0.8275],  # appelblauwzeegroen
                                                            edgecolor="green",
                                                            linewidth=1))
        self.widget.draw()

    def drawLid(self):
        for i in range(len(self.type.zuurkast_setup)):
            baseX, baseY = self.type.location(i)
            if self.type.zuurkast_setup[i]['Lid'] == 'Closed':
                self.axes.add_patch(patches.Rectangle((baseX + (self.type.LiqBox_width - self.type.LidHandle_width) / 2,
                                                       baseY + (self.type.LiqBox_depth - self.type.LidHandle_depth) / 2),
                                                        self.type.LidHandle_width, self.type.LidHandle_depth,
                                                        facecolor=[0.3010, 0.7450, 0.9330],  # lichtblauw
                                                        edgecolor="black",
                                                        linewidth=1))

            elif self.type.zuurkast_setup[i]['Lid'] == 'Opened':
                self.axes.add_patch(patches.Rectangle((baseX + (self.type.LiqBox_width - self.type.LiqBoxIn_width) / 2,
                                                       baseY + (self.type.LiqBox_depth - self.type.LiqBoxIn_depth) / 2),
                                                      self.type.LiqBoxIn_width, self.type.LiqBoxIn_depth,
                                                      facecolor=[0.0471, 0.6392, 0.8745],  # lichtblauw
                                                      edgecolor="blue",
                                                      linewidth=1))
        self.widget.draw()


    def onClick(self, event):
        if event.button == 1 or event.button == 3:
            if event.inaxes:
                ax_idx = np.where([ax is event.inaxes for ax in self.fig.get_axes()])[0][0]
                print(ax_idx)
                x, y = event.xdata, event.ydata
                print("clicked on position", x, y)
                self.motor_com(x, y)

    def motor_com(self, x, y):
        if not self.NewSeq_toggling and not self.EditSeq_toggling:   # if "make new sequence" or "Edit sequence" is not checked:
            commandx = bytearray("M_abs 0 " + str(round(x)) + "\r", "utf8")
            print(commandx)
            commandy = bytearray("M_abs 1 " + str(round(y)) + "\r", "utf8")
            print(commandy)
            self.command_list(commandx, commandy)
        else:
            self.analyse_location(x,y)

    def command_list(self, xcommand, ycommand):
        list = []
        list.append(xcommand)
        list.append(ycommand)
        self.run_commands(list)

    def analyse_location(self, x, y):
        print("analysing")
        move_commands = []
        if self.type.BaseSt[0] < x < self.type.BaseSt[2] and self.type.BaseSt[1] < y < self.type.BaseSt[3]:
            index = 0
            move_commands.extend(self.type.move_to_fixed_pos(index))
            self.type.sequence_list[self.type.SeqNum].extend(move_commands)
        else:
            index = 1
            for i in range(12):
                basex = self.type.LiquidSt[0] + self.type.LiqBox_edgeX + (self.type.LiqBox_distancex + self.type.LiqBox_width) * i
                basey = self.type.LiquidSt[1] + self.type.LiqBox_edgeY
                if basex < x < basex+self.type.LiqBox_width and basey < y < basey+self.type.LiqBox_depth:
                    move_commands.extend(self.type.move_to_fixed_pos(index))
                    self.type.sequence_list[self.type.SeqNum].extend(move_commands)
                    index += 1

        if move_commands == []:
            print("Click on the base station or one of the liquid boxes.")
        else:
            self.run_commands(move_commands)


    def run_commands(self, new_commands):
        self.command_queue.extend(new_commands)
        #print(self.command_queue)
        self.CommandUpdate.emit(new_commands)

    def redraw(self):
        self.setup_axes()
        self.setup_robotstation()
        self.drawLid()
        self.drawBaskets()

    def do(self):
        self.widget.draw()
        self.widget.flush_events()

class Type():
    def __init__(self, axs):
        # zuurkast_setup as a list of dictionaries, accessible via index  (Lid = deksel)
        self.zuurkast_setup = [{'name':'BaseStation', 'Lid': 'No_Lid', 'Basket': None},
                                {'name':'LiqBox_1', 'Lid': 'Closed', 'Basket': None},
                                {'name':'LiqBox_2', 'Lid': 'Closed', 'Basket': None},
                                {'name':'LiqBox_3', 'Lid': 'Closed', 'Basket': None},
                                {'name':'LiqBox_4', 'Lid': 'Closed', 'Basket': None},
                                {'name':'LiqBox_5', 'Lid': 'Closed', 'Basket': None},
                                {'name':'LiqBox_6', 'Lid': 'Closed', 'Basket': None},
                                {'name':'LiqBox_7', 'Lid': 'Closed', 'Basket': None},
                                {'name':'LiqBox_8', 'Lid': 'Closed', 'Basket': None},
                                {'name':'LiqBox_9', 'Lid': 'Closed', 'Basket': None},
                                {'name':'LiqBox_10', 'Lid': 'Closed', 'Basket': None},
                                {'name':'LiqBox_11', 'Lid': 'Closed', 'Basket': None},
                                {'name':'LiqBox_12', 'Lid': 'Closed', 'Basket': None}]

        self.BaseSt = [0, 0, 63, 126]       # self.BaseSt = [originX, originY, limitX, limitY]
        self.LiquidSt = [100, 0, 690, 126, 110]  # self.LiquidSt = [originX, originY, limitX, limitY, height]
        # 690x126mm -> outside dimensions liquidstation,  63x126mm -> outside dimensions base station

        # --- parameters Basket:
        self.Basket_width = 30
        self.Basket_depth = 92.2
        self.Basket_height = 50.5
        self.BasketHandle_heigt = 107

        # --- parameters liquidstation boxes:
        self.LiqBox_edgeX = 6.5
        self.LiqBox_edgeY = 13
        self.LiqBox_width = 50
        self.LiqBox_depth = 100
        self.LiqBoxIn_width = 45
        self.LiqBoxIn_depth = 95
        self.LiqBoxIn_height = 98
        self.LiqBox_distancex = (self.LiquidSt[2] - self.LiqBox_edgeX * 2 - 12 * self.LiqBox_width) / 11  # tussenafstand liquid boxes

        # --- parameters Lid:
        self.LidHandle_width = 10
        self.LidHandle_depth = 50
        self.LidHandle_thickness = 5

        self.sequence_list = []
        self.SeqNum = 0

        self.simulate = []
        self.axes = axs


    def add_basket(self, name):
        if self.zuurkast_setup[0]['Basket'] != None:
            raise ValueError("Base station (position 0) is already occupied by basket " + str(self.zuurkast_setup[0]['Basket']))
        self.zuurkast_setup[0]['Basket'] = name
        print("Basket " + str(name) + " added to base station: \n", self.zuurkast_setup[0])


    def re_address_basket(self, index_old, index_new):
        if self.zuurkast_setup[index_old]['Basket'] is None:
            raise ValueError("No basket on position " + str(index_old) + " to move")

        if self.zuurkast_setup[index_new]['Basket'] is not None:
            raise ValueError("Position " + str(index_new) + " is already occupied by basket " + str(self.zuurkast_setup[index_new]['Basket']))

        if self.zuurkast_setup[index_new]['Lid'] == 'Closed':
            raise ValueError("Can't place the basket on position " + str(index_new) + ", liquid box is closed.")

        if self.zuurkast_setup[index_old]['Lid'] == 'Closed':
            raise ValueError("Can't pick up the basket on position " + str(index_old) + ", liquid box is closed.")

        self.zuurkast_setup[index_new]['Basket'] = self.zuurkast_setup[index_old]['Basket']
        self.zuurkast_setup[index_old]['Basket'] = None
        print("moving basket " + str(self.zuurkast_setup[index_new]['Basket']) + " from position " + str(index_old) + " to " + str(index_new),
              '\n', self.zuurkast_setup[index_old], '\n', self.zuurkast_setup[index_new])


    def change_lid_state(self, index, new_state):
        if new_state == 'Open' and self.zuurkast_setup[index]['Lid'] == 'Closed':
            self.zuurkast_setup[index]['Lid'] = 'Opened'
            print(self.zuurkast_setup[index])

        elif new_state == 'Close' and self.zuurkast_setup[index]['Lid'] == 'Opened':
            self.zuurkast_setup[index]['Lid'] = 'Closed'
            print(self.zuurkast_setup[index])


    def location(self, index):
        #print("locating index")
        current = 1
        if index == 0:
            basex = self.BaseSt[0]
            basey = self.BaseSt[1]
            return [basex, basey]
        else:
            for i in range(12):
                basex = self.LiquidSt[0] + self.LiqBox_edgeX + (self.LiqBox_distancex + self.LiqBox_width) * i
                basey = self.LiquidSt[1] + self.LiqBox_edgeY
                if current == index:
                    return [basex, basey]
                current += 1


    def move_to_fixed_pos(self, index):
        #print("Moving to position ", index)
        basex, basey = self.location(index)
        if index == 0:
            Xpos = basex + self.BaseSt[2] / 2
            Ypos = basey + self.BaseSt[3] / 2
        else:
            Xpos = basex + self.LiqBox_width / 2
            Ypos = basey + self.LiqBox_depth / 2

        #return [bytearray("M_abs 0 " + str(Xpos) + "\r", "utf8"), bytearray("M_abs 1 " + str(Ypos) + "\r", "utf8")]    Use this return for a robot that can move in Y-direction
        return [bytearray("M_abs 0 " + str(Xpos) + "\r", "utf8")]

#    def save_in_textfile(self):