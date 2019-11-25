from PyQt5 import QtCore, QtGui, QtWidgets
from pymavlink import mavutil
import time


class MainThread(QtCore.QThread):

    def run(self):


        master = mavutil.mavlink_connection('udpin:0.0.0.0:15000')
        master.wait_heartbeat()
        master.arducopter_arm()

        yaw_final = 100
        ui.label_52.setText(str(yaw_final))
        Kp = 0
        Ki = 0
        Kd = 0
        I = 0
        initial = time.time()
        prev_error = 0
        while True:
            try:
                Kd = ui.value_d/10
                Ki = ui.value_i/10
                Kp = ui.value_p/10

                ui.label_d.setText("Kd:"+str(Kd))
                ui.label_i.setText("Ki:"+str(Ki))
                ui.label_p.setText("Kp:"+str(Kp))

                msg = master.recv_match(type="ATTITUDE").to_dict()
                yaw = msg["yaw"] * 180/3.14
                roll = msg["roll"] * 180/3.14
                pitch = msg["pitch"] * 180/3.14

                ui.label_32.setText(str(round(yaw, 3)))
                ui.label_22.setText(str(round(roll, 3)))
                ui.label_12.setText(str(round(pitch, 3)))

                error = yaw_final - yaw

                if error > 180:
                    error = error - 360
                elif error < -180:
                    error = error + 360

                dt = time.time() - initial
                initial = time.time()
                error_diff = error - prev_error
                prev_error = error

                ui.label_42.setText(str(round(error, 3)))

                P = Kp * error

                I = I + (Ki * error * dt)

                D = (Kd * error_diff)/dt

                pid = P+I+D

                ui.label_pv.setText("P:"+str(round(P, 3)))
                ui.label_iv.setText("I:"+str(round(I, 3)))
                ui.label_dv.setText("D:"+str(round(D, 3)))
                ui.label_62.setText(str(round(pid, 3)))

                if (pid >600):
                    pid = 600

                if (pid < -600):
                    pid = -600


                master.mav.manual_control_send(
                            master.target_system,
                            0,
                            0,
                            500,
                            0,
                            0)
            except Exception as msg:
                #print(msg)
                pass

class Ui_PIDControl(object):
    def setupUi(self, PIDControl):
        PIDControl.setObjectName("PIDControl")
        PIDControl.resize(663, 474)
        self.centralwidget = QtWidgets.QWidget(PIDControl)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.mainGL = QtWidgets.QGridLayout()
        self.mainGL.setObjectName("mainGL")
        self.labelsGL = QtWidgets.QGridLayout()
        self.labelsGL.setObjectName("labelsGL")
        self.label_82 = QtWidgets.QLabel(self.centralwidget)
        self.label_82.setObjectName("label_82")
        self.labelsGL.addWidget(self.label_82, 7, 2, 1, 1)
        self.label_22 = QtWidgets.QLabel(self.centralwidget)
        self.label_22.setObjectName("label_22")
        self.labelsGL.addWidget(self.label_22, 1, 2, 1, 1)
        self.label_42 = QtWidgets.QLabel(self.centralwidget)
        self.label_42.setObjectName("label_42")
        self.labelsGL.addWidget(self.label_42, 3, 2, 1, 1)
        self.label_62 = QtWidgets.QLabel(self.centralwidget)
        self.label_62.setObjectName("label_62")
        self.labelsGL.addWidget(self.label_62, 5, 2, 1, 1)
        self.label_51 = QtWidgets.QLabel(self.centralwidget)
        self.label_51.setObjectName("label_51")
        self.labelsGL.addWidget(self.label_51, 4, 1, 1, 1)
        self.label_81 = QtWidgets.QLabel(self.centralwidget)
        self.label_81.setObjectName("label_81")
        self.labelsGL.addWidget(self.label_81, 7, 1, 1, 1)
        self.label_31 = QtWidgets.QLabel(self.centralwidget)
        self.label_31.setObjectName("label_31")
        self.labelsGL.addWidget(self.label_31, 2, 1, 1, 1)
        self.label_72 = QtWidgets.QLabel(self.centralwidget)
        self.label_72.setObjectName("label_72")
        self.labelsGL.addWidget(self.label_72, 6, 2, 1, 1)
        self.label_52 = QtWidgets.QLabel(self.centralwidget)
        self.label_52.setObjectName("label_52")
        self.labelsGL.addWidget(self.label_52, 4, 2, 1, 1)
        self.label_21 = QtWidgets.QLabel(self.centralwidget)
        self.label_21.setObjectName("label_21")
        self.labelsGL.addWidget(self.label_21, 1, 1, 1, 1)
        self.label_71 = QtWidgets.QLabel(self.centralwidget)
        self.label_71.setObjectName("label_71")
        self.labelsGL.addWidget(self.label_71, 6, 1, 1, 1)
        self.label_41 = QtWidgets.QLabel(self.centralwidget)
        self.label_41.setObjectName("label_41")
        self.labelsGL.addWidget(self.label_41, 3, 1, 1, 1)
        self.label_11 = QtWidgets.QLabel(self.centralwidget)
        self.label_11.setObjectName("label_11")
        self.labelsGL.addWidget(self.label_11, 0, 1, 1, 1)
        self.label_12 = QtWidgets.QLabel(self.centralwidget)
        self.label_12.setObjectName("label_12")
        self.labelsGL.addWidget(self.label_12, 0, 2, 1, 1)
        self.label_32 = QtWidgets.QLabel(self.centralwidget)
        self.label_32.setObjectName("label_32")
        self.labelsGL.addWidget(self.label_32, 2, 2, 1, 1)
        self.label_61 = QtWidgets.QLabel(self.centralwidget)
        self.label_61.setObjectName("label_61")
        self.labelsGL.addWidget(self.label_61, 5, 1, 1, 1)
        self.mainGL.addLayout(self.labelsGL, 1, 1, 1, 1)
        self.pidGL = QtWidgets.QGridLayout()
        self.pidGL.setObjectName("pidGL")
        self.label_pv = QtWidgets.QLabel(self.centralwidget)
        self.label_pv.setObjectName("label_pv")
        self.pidGL.addWidget(self.label_pv, 2, 0, 1, 1)
        self.label_i = QtWidgets.QLabel(self.centralwidget)
        self.label_i.setObjectName("label_i")
        self.pidGL.addWidget(self.label_i, 0, 1, 1, 1)
        self.label_d = QtWidgets.QLabel(self.centralwidget)
        self.label_d.setObjectName("label_d")
        self.pidGL.addWidget(self.label_d, 0, 2, 1, 1)
        self.label_dv = QtWidgets.QLabel(self.centralwidget)
        self.label_dv.setObjectName("label_dv")
        self.pidGL.addWidget(self.label_dv, 2, 2, 1, 1)
        self.lineEdit_p = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_p.setText("")
        self.lineEdit_p.setObjectName("lineEdit_p")
        self.pidGL.addWidget(self.lineEdit_p, 3, 0, 1, 1)
        self.label_iv = QtWidgets.QLabel(self.centralwidget)
        self.label_iv.setObjectName("label_iv")
        self.pidGL.addWidget(self.label_iv, 2, 1, 1, 1)
        self.label_p = QtWidgets.QLabel(self.centralwidget)
        self.label_p.setObjectName("label_p")
        self.pidGL.addWidget(self.label_p, 0, 0, 1, 1)
        self.lineEdit_i = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_i.setObjectName("lineEdit_i")
        self.pidGL.addWidget(self.lineEdit_i, 3, 1, 1, 1)
        self.lineEdit_d = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_d.setObjectName("lineEdit_d")
        self.pidGL.addWidget(self.lineEdit_d, 3, 2, 1, 1)
        self.verticalSlider_p = QtWidgets.QSlider(self.centralwidget)
        self.verticalSlider_p.setOrientation(QtCore.Qt.Vertical)
        self.verticalSlider_p.setObjectName("verticalSlider_p")
        self.pidGL.addWidget(self.verticalSlider_p, 1, 0, 1, 1)
        self.verticalSlider_i = QtWidgets.QSlider(self.centralwidget)
        self.verticalSlider_i.setOrientation(QtCore.Qt.Vertical)
        self.verticalSlider_i.setObjectName("verticalSlider_i")
        self.pidGL.addWidget(self.verticalSlider_i, 1, 1, 1, 1)
        self.verticalSlider_d = QtWidgets.QSlider(self.centralwidget)
        self.verticalSlider_d.setOrientation(QtCore.Qt.Vertical)
        self.verticalSlider_d.setObjectName("verticalSlider_d")
        self.pidGL.addWidget(self.verticalSlider_d, 1, 2, 1, 1)
        self.pidGL.setRowStretch(0, 10)
        self.pidGL.setRowStretch(1, 60)
        self.pidGL.setRowStretch(2, 10)
        self.pidGL.setRowStretch(3, 10)
        self.mainGL.addLayout(self.pidGL, 1, 0, 1, 1)
        self.mainGL.setColumnStretch(0, 90)
        self.mainGL.setColumnStretch(1, 50)
        self.verticalLayout.addLayout(self.mainGL)
        PIDControl.setCentralWidget(self.centralwidget)

        self.retranslateUi(PIDControl)
        QtCore.QMetaObject.connectSlotsByName(PIDControl)

        # -----------------------------------------------
        self.value_i=0
        self.value_d=0
        self.value_p=0
        self.lineEdit_d.editingFinished.connect(self.edit_d)
        self.lineEdit_i.editingFinished.connect(self.edit_i)
        self.lineEdit_p.editingFinished.connect(self.edit_p)

        self.mainThread = MainThread(PIDControl)
        self.mainThread.start()


        """self.verticalSlider_d.setMinimum(0)
        self.verticalSlider_d.setMaximum(100)
        self.verticalSlider_d.setValue(0)
        self.verticalSlider_d.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.verticalSlider_d.setTickInterval(5)
        self.verticalSlider_d.valueChanged.connect(self.valuechange_d)

        self.verticalSlider_i.setMinimum(0)
        self.verticalSlider_i.setMaximum(100)
        self.verticalSlider_i.setValue(0)
        self.verticalSlider_i.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.verticalSlider_i.setTickInterval(5)
        self.verticalSlider_i.valueChanged.connect(self.valuechange_i)


        self.verticalSlider_p.setMinimum(0)
        self.verticalSlider_p.setMaximum(100)
        self.verticalSlider_p.setValue(0)
        self.verticalSlider_p.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.verticalSlider_p.setTickInterval(5)
        self.verticalSlider_p.valueChanged.connect(self.valuechange_p)"""

    def edit_d(self):
        try:
            self.value_d = float(self.lineEdit_d.text())*10
        except Exception as msg:
            print(msg)
            pass

    def edit_i(self):
        try:
            self.value_i = float(self.lineEdit_i.text())*10
        except Exception as msg:
            print(msg)
            pass

    def edit_p(self):
        try:
            self.value_p = float(self.lineEdit_p.text())*10
        except Exception as msg:
            print(msg)
            pass


    def valuechange_d(self,value):
        self.value_d = value
    def valuechange_i(self,value):
        self.value_i = value
    def valuechange_p(self,value):
        self.value_p = value

    def retranslateUi(self, PIDControl):
        _translate = QtCore.QCoreApplication.translate
        PIDControl.setWindowTitle(_translate("PIDControl", "PID Control ui"))
        self.label_82.setText(_translate("PIDControl", "82"))
        self.label_22.setText(_translate("PIDControl", "22"))
        self.label_42.setText(_translate("PIDControl", "42"))
        self.label_62.setText(_translate("PIDControl", "62"))
        self.label_51.setText(_translate("PIDControl", "yaw_final:"))
        self.label_81.setText(_translate("PIDControl", "81"))
        self.label_31.setText(_translate("PIDControl", "yaw:"))
        self.label_72.setText(_translate("PIDControl", "72"))
        self.label_52.setText(_translate("PIDControl", "52"))
        self.label_21.setText(_translate("PIDControl", "roll:"))
        self.label_71.setText(_translate("PIDControl", "71"))
        self.label_41.setText(_translate("PIDControl", "yaw_error:"))
        self.label_11.setText(_translate("PIDControl", "pitch:"))
        self.label_12.setText(_translate("PIDControl", "12"))
        self.label_32.setText(_translate("PIDControl", "32"))
        self.label_61.setText(_translate("PIDControl", "total_pid:"))
        self.label_pv.setText(_translate("PIDControl", "p_value"))
        self.label_i.setText(_translate("PIDControl", "Integral"))
        self.label_d.setText(_translate("PIDControl", "Derivative"))
        self.label_dv.setText(_translate("PIDControl", "d_value"))
        self.label_iv.setText(_translate("PIDControl", "i_value"))
        self.label_p.setText(_translate("PIDControl", "Proportional"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    PIDControl = QtWidgets.QMainWindow()
    ui = Ui_PIDControl()
    ui.setupUi(PIDControl)
    PIDControl.show()
    sys.exit(app.exec_())
