#MAIN PROGRAM
#RUN THIS ONE

import sys
import time
import serial
import serial.tools.list_ports as port_list
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from GUI_robot_controls import Ui_MainWindow
from inspect import isfunction


def connect_to_ports(find_name):
    all_ports = list(port_list.comports())
    print("print all_ports: ", all_ports)
    pos_ports = [p.device for p in all_ports if "Arduino" in p.description]
    #pos_ports = [p.device for p in all_ports if "USB-SERIAL CH340" in p.description]    # use this for a Chinese Arduino
    print("print pos_ports: ", pos_ports)
    [print(p.description) for p in all_ports]

    if pos_ports == []:
        all_ports = list(port_list.comports())
        ard = all_ports[0].device
        return ard

    ## Search for Suitable Port
    print(pos_ports)
    for port in pos_ports:
        print(".")
        try:
            ard = serial.Serial(port, 9600, timeout=0.1)
        except:
            continue
        print("trying", port, "...", end="")
        response = read_info(ard)
        print(response, "...", end="")
        if response == find_name:
            print("Port Found: ", port)
            break
        else:
            ard.close()
            ard = None
    print("")

    return port

def read_info(ard):
    for _ in range(10):
        response = ard.readline().decode("utf-8").split("\r")[0]
        if response == "":
            print(".", end="")
        if response == "Startup":
            print("Starting up device")
            time.sleep(.1)
            break
    ard.write(b"Info\r")
    Info = ard.readline().decode("utf-8").split("\r")[0]
    print("Device Info: " + Info)
    return Info

def wait_for_response(device):

    for _ in range(10):
        response = device.readline().decode("utf-8").split("\r")[0]
        if response == "":
            print(".",end="")
        else:
            print(response)
            return response
    return ""


class GUI_Robot_Zuurkast(QMainWindow, Ui_MainWindow):
    def __init__(self, port=None):
        super(GUI_Robot_Zuurkast, self).__init__()

        self.setupUi(self)

        # move related commands/buttons/parameters robot:
        self.stopping.clicked.connect(self.stop)
        self.Homing.clicked.connect(self.home)
        self.moveX.clicked.connect(self.xcom)
        self.moveY.clicked.connect(self.ycom)
        self.moveZ.clicked.connect(self.zcom)
        self.gripper.clicked.connect(self.grippercom)
        self.move_command.textEdited.connect(self.command)
        self.move = 0
        self.relative_absolute.valueChanged.connect(self.RelAbsMoveSelection)
        self.HomeClicked = False
        self.RelAbs = "rel"
        self.joystick_control.stateChanged.connect(self.stick)

        # task related commands/buttons/parameters robot:
        self.CurrentPositionBasket.valueChanged.connect(self.ID_value_current)
        self.ID_current = 0
        self.NewPositionBasket.valueChanged.connect(self.ID_value_new)
        self.ID_new = 0
        self.LiqBoxNr.valueChanged.connect(self.ID_value_Lid)
        self.LidNr = 1
        self.OpenLid.clicked.connect(self.open_lid)
        self.CloseLid.clicked.connect(self.close_lid)
        self.AddNewBasket.clicked.connect(self.new_basket)
        self.ApplyMovement.clicked.connect(self.MoveBasket)
        self.clear_all.clicked.connect(self.all_cleared)
        self.basketnumber = 1  # variable for numbering the different baskets

        self.originZgripper = 105
        self.GripperOpen = -25
        self.GripperMaxOpen = -45   # maximum opening distance gripper
        self.GripperClamp = 60  # distance to clamp with force

        self.GrabBasket_Zdist = self.widget_visu.type.LiquidSt[4] - self.widget_visu.type.LiqBoxIn_depth + \
                                self.widget_visu.type.Basket_height + self.widget_visu.type.BasketHandle_heigt - self.originZgripper
        self.MoveBasket_Zdist = self.GrabBasket_Zdist + self.widget_visu.type.Basket_height + self.widget_visu.type.BasketHandle_heigt + 50
        self.GrabLid_Zdist = self.widget_visu.type.LiquidSt[4] + self.widget_visu.type.LidHandle_thickness - self.originZgripper
        # After every command for opening/closing a liquidbox or moving baskets,
        # the robot returns to the MoveLid_Zdist and the gripper is open
        self.MoveLid_Zdist = self.GrabBasket_Zdist + 50
        self.StoreLid_Xdist = 400  # at the moment no realistic value because robot can't move over a big enough x distance

        # sequence related buttons:
        self.FinishNewSequence.clicked.connect(self.save_new_seq)
        self.DeleteLastCommand.clicked.connect(self.del_last_com)
        self.sequence_toggle.stateChanged.connect(self.sequence)
        self.EditSeq.stateChanged.connect(self.edit_seq)
        self.SeqNum = 0
        self.SequenceSelection.activated.connect(self.disp_seq)
        self.sim.clicked.connect(self.simming)
        self.delete_seq.clicked.connect(self.DeleteSeq)

        # draw visualisation
        self.widget_visu.setup_axes()
        self.widget_visu.setup_robotstation()
        self.widget_visu.drawBaskets()
        self.widget_visu.drawLid()

        # connect pyqtSignal "commandUpdate" to the function run_tasks:
        self.widget_visu.CommandUpdate.connect(self.run_tasks)

        if port is None:
            port = connect_to_ports("Robot Zuurkast CORE-lab")
            self.device = serial.Serial(port, 9600, timeout=0.1)
            print(self.device)
        else:
            self.device = serial.Serial(port, 9600, timeout=0.1)
        self.device.isMoving = False


    def run_tasks(self, queue):
        while queue:
            print("queue: ", queue)
            try:
                is_busy = self.check_finished()
            except Exception as e:
                print(e)
            print("..", is_busy)
            if not is_busy:
                queue_item = queue.pop(0)
                if isinstance(queue_item, bytearray):
                    print("Executing Command")
                    cmd = queue_item
                    print(cmd)
                    self.device.write(cmd)
                    self.device.isMoving = True
                    #print(self.device.readline())
                elif isfunction(queue_item):
                    print("Executing Task")
                    fun = queue_item
                    print(fun)
                    fun()
            time.sleep(0.1)
        print(queue)
        self.wait_for_empty(device=self.device)

    def check_finished(self):
        #print(self.device)
        cmd = bytearray("Ready" + "\r", "utf8")
        self.device.write(cmd)
        device = self.device
        response = wait_for_response(device)
        # print("Ready?",end="")
        if "Ready" in response:
            self.device.isMoving = False
            print("Ready!")
        elif "Busy" in response:
            print("Busy")
            self.device.isMoving = True
        elif "" == response:
            print("No Response")
            self.device.isMoving = True

        return self.device.isMoving

    def wait_for_empty(self, device):
        for _ in range(15):
            response = device.readline().decode("utf-8").split("\r")[0]
            if response != "":
                print(".", end="")
            else:
                return
        return ""


    def command(self):
        print("command value reading")
        self.move = self.move_command.text()
        print(self.move)

    def RelAbsMoveSelection(self):
        if not self.HomeClicked:
            self.relative_absolute.setValue(0)
            print("First click 'Home' before using absolute movements")
        else:
            if self.relative_absolute.value() == 0:
                self.RelAbs = "rel"
            elif self.relative_absolute.value() == 1:
                self.RelAbs = "abs"
            print(self.RelAbs)

    def stick(self):
        if self.joystick_control.checkState():
            self.device.write(bytearray("J 1" + "\r", "utf8"))
            print("Joystick activated")
        else:
            self.device.write(bytearray("J 0" + "\r", "utf8"))
            print("Joystick deactivated")

    def ID_value_current(self):
        print("check current position basket")
        self.ID_current = self.CurrentPositionBasket.value()
        print(self.ID_current)

    def ID_value_new(self):
        print("check new position basket")
        self.ID_new = self.NewPositionBasket.value()
        print(self.ID_new)

    def ID_value_Lid(self):
        print("check current liquid box number")
        self.LidNr= self.LiqBoxNr.value()
        print(self.LidNr)

    def all_cleared(self):
        print("clearing all baskets:")
        for i in range(13):
            self.widget_visu.type.zuurkast_setup[i]['Basket'] = None
            print(self.widget_visu.type.zuurkast_setup[i])
        self.widget_visu.redraw()
        self.basketnumber = 1

    def new_basket(self):
        try:
            self.widget_visu.type.add_basket("B" + str(self.basketnumber))
            self.widget_visu.redraw()
            self.basketnumber += 1
        except Exception as e:
            print(e)

    def sequence(self):
        print("sequence chosen")
        if self.sequence_toggle.checkState():
            self.widget_visu.NewSeq_toggling = True
            self.label_3.setEnabled(True)
            self.SequenceNumber.setEnabled(True)
            self.FinishNewSequence.setEnabled(True)
            self.DeleteLastCommand.setEnabled((True))
            self.sequence_toggle.setEnabled(False)
            self.sim.setEnabled(False)
            self.delete_seq.setEnabled(False)
            self.SequenceSelection.setEnabled(False)
            self.EditSeq.setEnabled(False)
            try:
                self.SeqNum = self.widget_visu.type.sequence_list.index([])     # if there is an empty slot in the sequence_list caused by
                                                                                # deleting, store the commands for the new sequence on this index
                self.widget_visu.type.SeqNum = self.SeqNum
            except:
                self.SeqNum = len(self.widget_visu.type.sequence_list)
                self.widget_visu.type.SeqNum = self.SeqNum
                self.widget_visu.type.sequence_list.append([])     # if there is no empty slot in the sequence_list,
                                                                   # add an empty list to store the commands
            self.SequenceSelection.insertItem(self.SeqNum, "Seq " + str(self.SeqNum))
            self.SequenceNumber.display(str(self.SeqNum))
        else:
            self.widget_visu.NewSeq_toggling = False
            self.label_3.setEnabled(False)
            self.SequenceNumber.setEnabled(False)
            self.FinishNewSequence.setEnabled(False)
            self.DeleteLastCommand.setEnabled(False)
            self.sim.setEnabled(True)
            self.delete_seq.setEnabled(True)
            self.SequenceSelection.setEnabled(True)
        print(self.widget_visu.type.sequence_list)

    def save_new_seq(self):
        if self.widget_visu.type.sequence_list[self.SeqNum] == []:
            print("First, add some commands.")
            return
        self.sequence_toggle.setEnabled(True)
        self.sequence_toggle.setChecked(False)
        self.EditSeq.setEnabled(True)
        self.EditSeq.setChecked(False)
        print("Seq " + str(self.SeqNum) + ":  ", self.widget_visu.type.sequence_list[self.SeqNum])

    def del_last_com(self):
        try:
            del self.widget_visu.type.sequence_list[self.SeqNum][-1]
            print("Seq " + str(self.SeqNum) + ": ", self.widget_visu.type.sequence_list[self.SeqNum])
        except:
            print("Deleting last command not possible, sequence is empty")

    def disp_seq(self):
        print("Seq " + str(self.SequenceSelection.currentIndex()) + " selected:  ",
              self.widget_visu.type.sequence_list[self.SequenceSelection.currentIndex()])

    def edit_seq(self):
        if self.EditSeq.checkState():
            self.widget_visu.EditSeq_toggling = True
            self.label_3.setEnabled(True)
            self.SequenceNumber.setEnabled(True)
            self.FinishNewSequence.setEnabled(True)
            self.DeleteLastCommand.setEnabled((True))
            self.sequence_toggle.setEnabled(False)
            self.EditSeq.setEnabled(False)
            self.sim.setEnabled(False)
            self.delete_seq.setEnabled(False)
            self.SequenceSelection.setEnabled(False)

            self.SeqNum = self.SequenceSelection.currentIndex()
            self.widget_visu.type.SeqNum = self.SeqNum
            self.SequenceNumber.display(str(self.SeqNum))
        else:
            self.widget_visu.EditSeq_toggling = False
            self.label_3.setEnabled(False)
            self.SequenceNumber.setEnabled(False)
            self.FinishNewSequence.setEnabled(False)
            self.DeleteLastCommand.setEnabled(False)
            self.EditSeq.setEnabled(True)
            self.sim.setEnabled(True)
            self.delete_seq.setEnabled(True)
            self.SequenceSelection.setEnabled(True)

    def DeleteSeq(self):
        self.widget_visu.type.sequence_list[self.SequenceSelection.currentIndex()] = []  # clear the selected sequence in the sequence_list
        self.SequenceSelection.removeItem(self.SequenceSelection.currentIndex())  # delete the selected sequence out of the combo box
        self.delete_seq.setChecked(False)
        print("Sequence list:  ", self.widget_visu.type.sequence_list)

    def simming(self):
        if self.sim.isChecked():
            self.delete_seq.setEnabled(False)
            self.EditSeq.setEnabled(False)
            self.SequenceSelection.setEnabled(False)
            self.sequence_toggle.setEnabled(False)

            print("Simulating Seq " + str(self.SequenceSelection.currentIndex()) + ": ")
            simulate_seq = self.widget_visu.type.sequence_list[self.SequenceSelection.currentIndex()]
            print(simulate_seq)
            self.run_tasks(simulate_seq)
        else:
            self.stop()
            print("Simulating stopped.")
            self.delete_seq.setEnabled(True)
            self.EditSeq.setEnabled(True)
            self.SequenceSelection.setEnabled(True)
            self.sequence_toggle.setEnabled(True)


    #######################################################################
    #### Move controls ####################################################
    #######################################################################

    def stop(self):
        print("stopping")
        self.device.write(bytearray("STOP" + "\r", "utf8"))
        if self.widget_visu.NewSeq_toggling or self.widget_visu.EditSeq_toggling:  # if we are making or editing a sequence, add the command to the sequence
            self.widget_visu.type.sequence_list[self.SeqNum].append(bytearray("STOP" + "\r", "utf8"))

    def home(self):
        self.HomeClicked = True
        print("homing")
        self.device.write(bytearray("Home" + "\r", "utf8"))
        if self.widget_visu.NewSeq_toggling or self.widget_visu.EditSeq_toggling:  # if we are making or editing a sequence, add the command to the sequence
            self.widget_visu.type.sequence_list[self.SeqNum].append(bytearray("Home" + "\r", "utf8"))

    def xcom(self):
        print("executing x command")
        self.device.write(bytearray("M_" + str(self.RelAbs) +  " 0 " + str(self.move) + "\r", "utf8"))
        if self.widget_visu.NewSeq_toggling or self.widget_visu.EditSeq_toggling:  # if we are making or editing a sequence, add the command to the sequence
            self.widget_visu.type.sequence_list[self.SeqNum].append(bytearray("M_" + str(self.RelAbs) +  " 0 " + str(self.move) + "\r", "utf8"))

    def ycom(self):
        print("executing y command")
        self.device.write(bytearray("M_" + str(self.RelAbs) + " 1 " + str(self.move) + "\r", "utf8"))
        if self.widget_visu.NewSeq_toggling or self.widget_visu.EditSeq_toggling:  # if we are making or editing a sequence, add the command to the sequence
            self.widget_visu.type.sequence_list[self.SeqNum].append(bytearray("M_" + str(self.RelAbs) + " 1 " + str(self.move) + "\r", "utf8"))

    def zcom(self):
        print("executing z command")
        self.device.write(bytearray("M_" + str(self.RelAbs) + " 2 " + str(self.move) + "\r", "utf8"))
        if self.widget_visu.NewSeq_toggling or self.widget_visu.EditSeq_toggling:  # if we are making or editing a sequence, add the command to the sequence
            self.widget_visu.type.sequence_list[self.SeqNum].append(bytearray("M_" + str(self.RelAbs) + " 2 " + str(self.move) + "\r", "utf8"))

    def grippercom(self):
        print("executing gripper command")
        self.device.write(bytearray("M_" + str(self.RelAbs) + " 3 " + str(self.move) + "\r", "utf8"))
        if self.widget_visu.NewSeq_toggling or self.widget_visu.EditSeq_toggling:  # if we are making or editing a sequence, add the command to the sequence
            self.widget_visu.type.sequence_list[self.SeqNum].append(bytearray("M_" + str(self.RelAbs) + " 3 " + str(self.move) + "\r", "utf8"))

    def MoveBasket(self):
        try:
            self.widget_visu.type.re_address_basket(self.ID_current, self.ID_new)
            self.widget_visu.redraw()

            command_list = []
            command_list.extend(self.widget_visu.type.move_to_fixed_pos(self.ID_current) +
                                [bytearray("M_abs 2 " + str(self.GrabBasket_Zdist) + "\r", "utf8"),
                                 bytearray("M_rel 3 " + str(self.GripperClamp) + "\r", "utf8"),
                                 bytearray("M_abs 2 " + str(self.MoveBasket_Zdist) + "\r", "utf8")] +
                                 self.widget_visu.type.move_to_fixed_pos(self.ID_new) +
                                [bytearray("M_abs 2 " + str(self.GrabBasket_Zdist) + "\r", "utf8"),
                                 bytearray("M_rel 3 " + str(self.GripperOpen) + "\r", "utf8"),
                                 bytearray("M_abs 2 " + str(self.MoveLid_Zdist) + "\r", "utf8")])
            self.run_tasks(command_list)

            if self.widget_visu.NewSeq_toggling or self.widget_visu.EditSeq_toggling:  # if we are making or editing a sequence, add the command to the sequence
                self.widget_visu.type.sequence_list[self.SeqNum].extend(self.MoveBasket())

        except Exception as e:
            print(e)


    def CheckLidOpen(self, number_LiqBox):
        if self.LidIsOpen.checkState(): # dit moet via camera herkening van QR-code aanwezig of niet aanwezig zijn
            return True
        else:
            return False

    def open_lid(self):
        if not self.CheckLidOpen(self.LidNr):
            print("Opening lid of box " + str(self.LidNr))
            self.widget_visu.type.change_lid_state(self.LidNr, 'Open')
            self.widget_visu.redraw()

            command_list = []
            command_list.extend(self.widget_visu.type.move_to_fixed_pos(self.LidNr) +
                [bytearray("M_abs 2 " + str(self.GrabLid_Zdist) + "\r", "utf8"),
                 bytearray("M_rel 3 " + str(self.GripperClamp) + "\r", "utf8"),
                 bytearray("M_abs 2 " + str(self.MoveLid_Zdist) + "\r", "utf8"),
                 bytearray("M_abs 0 " + str(self.StoreLid_Xdist) + "\r", "utf8"),
                 bytearray("M_abs 2 " + str(0) + "\r", "utf8"),
                 bytearray("M_rel 3 " + str(self.GripperOpen) + "\r", "utf8"),
                 bytearray("M_abs 2 " + str(self.MoveLid_Zdist) + "\r", "utf8")])
            self.run_tasks(command_list)

            if self.widget_visu.NewSeq_toggling or self.widget_visu.EditSeq_toggling:  # if we are making or editing a sequence, add the command to the sequence
                self.widget_visu.type.sequence_list[self.SeqNum].extend([self.open_lid])  # self.open_lid refers to the name of the function,
                                                                                          # self.open_lid() with parentheses calls the function
                print(self.widget_visu.type.sequence_list)

        else:
            print("Lid of box " + str(self.LidNr) + " is already opened.")

    def close_lid(self):
        if self.CheckLidOpen(self.LidNr):
            print("Closing lid of box " + str(self.LidNr))
            self.widget_visu.type.change_lid_state(self.LidNr, 'Close')
            self.widget_visu.redraw()

            command_list = []
            command_list.extend([bytearray("M_abs 0 " + str(self.StoreLid_Xdist) + "\r", "utf8"),
                                 bytearray("M_abs 2 " + str(0) + "\r", "utf8"),
                                 bytearray("M_rel 3 " + str(self.GripperClamp) + "\r", "utf8"),
                                 bytearray("M_abs 2 " + str(self.MoveLid_Zdist) + "\r", "utf8")] +
                                 self.widget_visu.type.move_to_fixed_pos(self.LidNr) +
                                 [bytearray("M_abs 2 " + str(self.GrabLid_Zdist) + "\r", "utf8"),
                                 bytearray("M_rel 3 " + str(self.GripperOpen) + "\r", "utf8"),
                                 bytearray("M_abs 2 " + str(self.MoveLid_Zdist) + "\r", "utf8")])
            self.run_tasks(command_list)

            if self.widget_visu.NewSeq_toggling or self.widget_visu.EditSeq_toggling:  # if we are making or editing a sequence, add the command to the sequence
                self.widget_visu.type.sequence_list[self.SeqNum].extend(self.close_lid)

        else:
            print("Lid of box " + str(self.LidNr) + " is already closed.")



if __name__ == "__main__":
    app = QApplication(sys.argv)
    main = GUI_Robot_Zuurkast()
    main.show()
    sys.exit(app.exec_())