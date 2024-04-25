"""
Laboratory for Computational Motor Control, Johns Hopkins School of Medicine
@author: Jay Pi <jay.s.314159@gmail.com>
"""
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import QApplication, QLabel, QCheckBox, QWidget, QFileDialog, QPushButton, QLineEdit, QAction,\
                            QHBoxLayout, QDoubleSpinBox, QComboBox
from PyQt5.QtCore import Qt, pyqtSlot
from PyQt5.QtGui import QColor
import pyqtgraph as pg

from fsm_gui import FsmGui
from data_manager import DataManager
import app_lib as lib


import sys, zmq, math, os, json, pathlib, shutil, ctypes
import numpy as np
from collections import deque

class PlotGui(FsmGui):
    def __init__(self,x):
        super(PlotGui,self).__init__(x)
        # Init. option to ctrl Open Ephys
        empty_QWidget = QWidget()
        empty_QWidget.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        self.toolbar.addWidget(empty_QWidget)
        self.open_ephys_restart_QAction = QAction(QtGui.QIcon(os.path.join('.', 'icon', 'restart.png')),'')
        self.open_ephys_restart_QAction.setToolTip('Restart network connection to Open Ephys')
        self.toolbar.addAction(self.open_ephys_restart_QAction)
        self.open_ephys_QLabel = QLabel(' Open Ephys control: ')
        self.open_ephys_QCheckBox = QCheckBox()
        self.toolbar.addWidget(self.open_ephys_QLabel)
        self.toolbar.addWidget(self.open_ephys_QCheckBox)
        port_num = 5555
        try:
            self.open_ephys_socket = self.init_open_ephys_connection(port_num)
            self.open_ephys_socket.send_string('IsAcquiring')
            self.open_ephys_socket.recv()
        except:
            err_msg = f'Connection to Open Ephys failed. Change port to {port_num} and restart connection.'
            self.log_QPlainTextEdit.appendPlainText(err_msg)

        # Set the parent path to save data in lab server format
        self.data_path_QLabel = QLabel('   File Path: ')
        self.data_path_QLineEdit = QLineEdit()
        self.data_path_QLineEdit.setReadOnly(True)
        self.data_path_QPushButton = QPushButton('Browse')
        self.toolbar.addWidget(self.data_path_QLabel)
        self.toolbar.addWidget(self.data_path_QLineEdit)
        self.toolbar.addWidget(self.data_path_QPushButton)
        self.data_path_QFileDialog = QFileDialog()
        self.data_path_QFileDialog.setFileMode(QFileDialog.FileMode.DirectoryOnly)
        sys_parameter, _ = lib.load_parameter('','sys_parameter.json', False, False, lib.set_default_sys_parameter)# load system parameter
        self.data_path_QLineEdit.setText(sys_parameter['data_path'])
        self.data_path_QFileDialog.setDirectory(self.data_path_QLineEdit.text())

        self.data_manager = DataManager()
        # Create socket for ZMQ
        try:
            context = zmq.Context()
            self.fsm_to_plot_socket = context.socket(zmq.SUB)
            self.fsm_to_plot_socket.setsockopt(zmq.CONFLATE,1)
            self.fsm_to_plot_socket.connect("tcp://192.168.0.2:5556")
            self.fsm_to_plot_socket.subscribe("")
            self.fsm_to_plot_poller = zmq.Poller()
            self.fsm_to_plot_poller.register(self.fsm_to_plot_socket, zmq.POLLIN)

            self.fsm_to_plot_priority_socket = context.socket(zmq.SUB)
            self.fsm_to_plot_priority_socket.connect("tcp://192.168.0.2:5557")
            self.fsm_to_plot_priority_socket.subscribe("")
            self.fsm_to_plot_priority_poller = zmq.Poller()
            self.fsm_to_plot_priority_poller.register(self.fsm_to_plot_priority_socket, zmq.POLLIN)

            self.plot_to_fsm_socket = context.socket(zmq.PUB)
            self.plot_to_fsm_socket.bind("tcp://192.168.0.1:5558")

        except Exception as error:
            self.log_QPlainTextEdit.appendPlainText('Error in starting zmq sockets:')
            self.log_QPlainTextEdit.appendPlainText(str(error) + '.')
            self.toolbar_run_QAction.setDisabled(True)
            self.toolbar_connect_QAction.setDisabled(True)
        # Signals
        self.data_manager.signals.to_main_thread.connect(self.data_manager_signalled)
        self.receiver_QTimer.timeout.connect(self.receiver_QTimer_timeout)
        self.toolbar_connect_QAction.triggered.connect(self.toolbar_connect_QAction_triggered)
        self.toolbar_run_QAction.triggered.connect(self.toolbar_run_QAction_triggered)
        self.toolbar_stop_QAction.triggered.connect(self.toolbar_stop_QAction_triggered)
        self.data_path_QPushButton.clicked.connect(self.data_path_QPushButton_clicked)

        # Disable target widgets
        self.tgt.deleteLater()
        self.pd_tgt.deleteLater()

        # Customiaze plots
        self.plot_1_cue = self.plot_1_PlotWidget.plot(np.zeros((0)), np.zeros((0)), pen = None,\
            symbolBrush=None, symbolPen='g', symbol='+',symbolSize=14,name='cue')
        self.plot_1_end = self.plot_1_PlotWidget.plot(np.zeros((0)), np.zeros((0)), pen = None,\
            symbolBrush=None, symbolPen='r', symbol='+',symbolSize=14,name='end')
        self.plot_1_tgt = self.plot_1_PlotWidget.\
            plot(np.zeros((0)), np.zeros((0)), pen = None,\
            symbolBrush=None, symbolPen='b', symbol='+',symbolSize=14,name='tgt')
        self.plot_1_eye = self.plot_1_PlotWidget.\
            plot(np.zeros((0)), np.zeros((0)), pen = None,\
            symbolBrush='k', symbolPen='k', symbol='o',symbolSize=10,name='eye',connect='finite')

        # Digital input/out channels
        self.dio_title_QLabel = QLabel('<b>Digital input/output display</b> (low, high)')
        self.dio_title_QLabel.setAlignment(Qt.AlignCenter)
        self.sidepanel_custom_QVBoxLayout.addWidget(self.dio_title_QLabel)
        self.num_dio_ch = 8
        self.din_data_dict = {}
        self.dout_data_dict = {}
        self.dio_plot_dict = {}
        self.dio_QLabel_dict = {}
        self.dio_QCheckBox_dict = {}
        self.dio_low_QDoubleSpinBox_dict = {}
        self.dio_high_QDoubleSpinBox_dict = {}
        self.dio_color_QComboBox_dict = {}
        self.dio_choice_QComboBox_dict = {}
        self.dio_QHBoxLayout = {}
        for ch_idx in range(self.num_dio_ch):
            self.dio_QHBoxLayout['ch_' + str(ch_idx)] = QHBoxLayout()
            self.sidepanel_custom_QVBoxLayout.addLayout(self.dio_QHBoxLayout['ch_' + str(ch_idx)])
            self.din_data_dict['ch_' + str(ch_idx)] = deque(maxlen=self.data_length)   
            self.dout_data_dict['ch_' + str(ch_idx)] = deque(maxlen=self.data_length)   
            self.dio_QLabel_dict['ch_' + str(ch_idx)] = QLabel('Ch. ' + str(ch_idx) + '.')
            self.dio_QCheckBox_dict['ch_' + str(ch_idx)] = QCheckBox()
            self.dio_low_QDoubleSpinBox_dict['ch_' + str(ch_idx)] = QDoubleSpinBox()
            self.dio_low_QDoubleSpinBox_dict['ch_' + str(ch_idx)].setDecimals(1)
            self.dio_low_QDoubleSpinBox_dict['ch_' + str(ch_idx)].setValue(0)
            self.dio_high_QDoubleSpinBox_dict['ch_' + str(ch_idx)] = QDoubleSpinBox()
            self.dio_high_QDoubleSpinBox_dict['ch_' + str(ch_idx)].setDecimals(1)
            self.dio_high_QDoubleSpinBox_dict['ch_' + str(ch_idx)].setValue(1+ch_idx/10)
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)] = QComboBox()
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].addItem('black')
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].setItemData(0,QColor(Qt.black),Qt.BackgroundRole)
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].addItem('cyan')
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].setItemData(1,QColor(Qt.cyan),Qt.BackgroundRole)
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].addItem('blue')
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].setItemData(2,QColor(Qt.blue),Qt.BackgroundRole)
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].addItem('red')
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].setItemData(3,QColor(Qt.darkRed),Qt.BackgroundRole)
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].addItem('magenta')
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].setItemData(4,QColor(Qt.magenta),Qt.BackgroundRole)
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].addItem('yellow')
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].setItemData(5,QColor(Qt.yellow),Qt.BackgroundRole)
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].addItem('green')
            self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].setItemData(6,QColor(Qt.green),Qt.BackgroundRole)
            self.dio_choice_QComboBox_dict['ch_' + str(ch_idx)] = QComboBox()
            self.dio_choice_QComboBox_dict['ch_' + str(ch_idx)].addItem('input')
            self.dio_choice_QComboBox_dict['ch_' + str(ch_idx)].addItem('output')
            self.dio_QHBoxLayout['ch_' + str(ch_idx)].addWidget(self.dio_QLabel_dict['ch_' + str(ch_idx)])
            self.dio_QHBoxLayout['ch_' + str(ch_idx)].addWidget(self.dio_QCheckBox_dict['ch_' + str(ch_idx)])
            self.dio_QHBoxLayout['ch_' + str(ch_idx)].addWidget(self.dio_choice_QComboBox_dict['ch_' + str(ch_idx)])
            self.dio_QHBoxLayout['ch_' + str(ch_idx)].addWidget(self.dio_low_QDoubleSpinBox_dict['ch_' + str(ch_idx)])
            self.dio_QHBoxLayout['ch_' + str(ch_idx)].addWidget(self.dio_high_QDoubleSpinBox_dict['ch_' + str(ch_idx)])
            self.dio_QHBoxLayout['ch_' + str(ch_idx)].addWidget(self.dio_color_QComboBox_dict['ch_' + str(ch_idx)])
            self.dio_plot_dict['ch_' + str(ch_idx)] = self.plot_2_PlotWidget.\
                plot(np.zeros((0)), np.zeros((0)), pen = pg.mkPen(color='w', width=2.5),connect='finite')

    @pyqtSlot()
    def toolbar_run_QAction_triggered(self):
        # Control Open Ephys
        if self.open_ephys_QCheckBox.isChecked():
            try:
                open_ephys_msg = f'StartRecord RecordNode=1 CreateNewDir=1 RecDir={self.data_path_QLineEdit.text()}'
                self.open_ephys_socket.send_string(open_ephys_msg)
                self.open_ephys_socket.recv()
            except Exception as error:
                self.log_QPlainTextEdit.appendPlainText('Error in controlling Open Ephys.')
                self.log_QPlainTextEdit.appendPlainText(str(error) + '.')
        # Check to see if FSM process ready
        self.plot_to_fsm_socket.send_pyobj(('confirm_connection',0))
        # Wait for confirmation for 5 sec.
        if self.fsm_to_plot_priority_poller.poll(2000):
            msg = self.fsm_to_plot_priority_socket.recv_pyobj(flags=zmq.NOBLOCK)
            if msg[0] == 0:
                self.toolbar_run_QAction.setDisabled(True)
                self.toolbar_stop_QAction.setEnabled(True)
                # Start FSM
                self.plot_to_fsm_socket.send_pyobj(('run',0))
                # Reset data
                self.eye_x_data.clear()
                self.eye_y_data.clear()
                self.tgt_x_data.clear()
                self.tgt_y_data.clear()
                self.t_data.clear()
                for ch_idx in range(self.num_dio_ch):
                    self.din_data_dict['ch_' + str(ch_idx)].clear()
                    self.dout_data_dict['ch_' + str(ch_idx)].clear()
        else:
            self.log_QPlainTextEdit.appendPlainText('No connection with FSM computer.')
        # Disable file path search
        self.data_path_QPushButton.setDisabled(True)
    @pyqtSlot()
    def toolbar_stop_QAction_triggered(self):
        # Control Open Ephys
        if self.open_ephys_QCheckBox.isChecked():
            try:
                open_ephys_msg = 'StopRecord'
                self.open_ephys_socket.send_string(open_ephys_msg)
                self.open_ephys_socket.recv()
                # Find the latest recording folder and rename subfolder to 'raw_data'
                rec_dir = self.data_path_QLineEdit.text()
                recent_rec_dir = max([os.path.join(rec_dir,d) for d in os.listdir(rec_dir)], key=os.path.getmtime)
                os.rename(os.path.join(recent_rec_dir,os.listdir(recent_rec_dir)[0]), os.path.join(recent_rec_dir,'raw_data'))
            except:
                self.log_QPlainTextEdit.appendPlainText('Error in controlling Open Ephys')
        self.toolbar_run_QAction.setEnabled(True)
        self.toolbar_stop_QAction.setDisabled(True)
        # Stop FSM
        self.plot_to_fsm_socket.send_pyobj(('stop',0))
        # Convert the data of the current recording
        self.data_manager.convert_data()
        # If controlling Open Ephys, copy the behavior files to Open Ephys folder
        if self.open_ephys_QCheckBox.isChecked():
            try:
                self.open_ephys_socket.send_string('IsAcquiring') # dummy check to see Open Ephys comm. works
                self.open_ephys_socket.recv()
                shutil.copy(os.path.join(self.data_manager.data_file_path +'.hdf5'),os.path.join(recent_rec_dir,'raw_data')) # rec. path from above
                shutil.copy(os.path.join(self.data_manager.data_file_path +'.mat'),os.path.join(recent_rec_dir,'raw_data'))
            except Exception as error:
                self.log_QPlainTextEdit.appendPlainText(str(error) + '.')
        # Enable file path search
        self.data_path_QPushButton.setEnabled(True)
    @pyqtSlot()
    def toolbar_connect_QAction_triggered(self):
        '''
        when triggered, start to receive messages from another computer
        '''
        self.receiver_QTimer.start(10)
        self.toolbar_connect_QAction.setDisabled(True)

    @pyqtSlot()
    def open_ephys_restart_QAction_triggered(self):
        '''
        try to restart ZMQ connection to Open Ephys
        '''
        port_num = 5555
        try:
            self.open_ephys_socket = self.init_open_ephys_connection(port_num)
            self.open_ephys_socket.send_string('IsAcquiring')
            self.open_ephys_socket.recv()
        except:
            err_msg = f'Connection to Open Ephys unsuccessful. Change port to {port_num} and restart connection.'
            self.log_QPlainTextEdit.appendPlainText(err_msg)

    @pyqtSlot()
    def receiver_QTimer_timeout(self):
        # Non-priority channel - only data for real-time plotting
        if self.fsm_to_plot_poller.poll(0):
            msg = self.fsm_to_plot_socket.recv_pyobj(flags=zmq.NOBLOCK)
            t,eye_x,eye_y,tgt_x,tgt_y,din,dout = msg
            if not math.isnan(t):
                self.eye_x_data.append(eye_x)
                self.eye_y_data.append(eye_y)
                self.tgt_x_data.append(tgt_x)
                self.tgt_y_data.append(tgt_y)
                self.t_data.append(t)
                # Digital input/output
                din_bin = '{0:08b}'.format(int(din))
                dout_bin = '{0:08b}'.format(int(dout))
                for ch_idx in range(self.num_dio_ch):
                    low_val = self.dio_low_QDoubleSpinBox_dict['ch_' + str(ch_idx)].value()
                    high_val = self.dio_high_QDoubleSpinBox_dict['ch_' + str(ch_idx)].value()
                    din_ch_bin = din_bin[-ch_idx-1]
                    dout_ch_bin = dout_bin[-ch_idx-1]
                    if din_ch_bin == '0':
                        self.din_data_dict['ch_' + str(ch_idx)].append(low_val)
                    elif din_ch_bin == '1':
                        self.din_data_dict['ch_' + str(ch_idx)].append(high_val)
                    if dout_ch_bin == '0':
                        self.dout_data_dict['ch_' + str(ch_idx)].append(low_val)
                    elif dout_ch_bin == '1':
                        self.dout_data_dict['ch_' + str(ch_idx)].append(high_val)
                    dio_color_str = self.dio_color_QComboBox_dict['ch_' + str(ch_idx)].currentText()
                    if dio_color_str == 'black':
                        dio_color = 'k'
                    elif dio_color_str == 'cyan':
                        dio_color = 'c'
                    elif dio_color_str == 'blue':
                        dio_color = 'b'
                    elif dio_color_str == 'red':
                        dio_color= 'r'
                    elif dio_color_str == 'magenta':
                        dio_color = 'm'
                    elif dio_color_str == 'yellow':
                        dio_color = 'y'
                    elif dio_color_str == 'green':
                        dio_color = 'g'
                    if self.dio_QCheckBox_dict['ch_' + str(ch_idx)].isChecked():    
                        self.dio_plot_dict['ch_' + str(ch_idx)].setPen(pg.mkPen(color=dio_color, width=2.5))
                        dio_choice = self.dio_choice_QComboBox_dict['ch_' + str(ch_idx)].currentText()
                        if dio_choice == 'input':
                            self.dio_plot_dict['ch_' + str(ch_idx)].setData(self.t_data,self.din_data_dict['ch_' + str(ch_idx)])
                        elif dio_choice == 'output':
                            self.dio_plot_dict['ch_' + str(ch_idx)].setData(self.t_data,self.dout_data_dict['ch_' + str(ch_idx)])
                    else:
                        self.dio_plot_dict['ch_' + str(ch_idx)].clear()
                # Plot
                self.plot_1_eye.setData([eye_x],[eye_y])
                self.plot_1_tgt.setData([tgt_x],[tgt_y])
                self.plot_2_eye_x.setData(self.t_data,self.eye_x_data)
                self.plot_2_eye_y.setData(self.t_data,self.eye_y_data)
                self.plot_2_tgt_x.setData(self.t_data,self.tgt_x_data)
                self.plot_2_tgt_y.setData(self.t_data,self.tgt_y_data)
        if self.fsm_to_plot_priority_poller.poll(0):
            msg = self.fsm_to_plot_priority_socket.recv_pyobj()
            msg_title = msg[0]
            if msg_title == 'tgt_data':
                if len(msg[1]) == 2: # only cue tgt
                    cue_x, cue_y = msg[1]
                elif len(msg[1]) == 4: # cue and end tgt
                    cue_x, cue_y, end_x, end_y = msg[1]
                    self.plot_1_end.setData([end_x],[end_y])
                self.plot_1_cue.setData([cue_x],[cue_y])
            if msg_title == 'trial_data':
                self.data_manager.trial_num = msg[1]
                self.data_manager.trial_data = msg[2]
                self.data_manager.save_data()
            if msg_title == 'pump_1':
                self.pump_1.pump_once_QPushButton_clicked()
            if msg_title == 'pump_2':
                self.pump_2.pump_once_QPushButton_clicked()
            if msg_title == 'log':
                self.log_QPlainTextEdit.appendPlainText(msg[1])
            if msg_title == 'confirm_connection':
                self.plot_to_fsm_socket.send_pyobj((0,0))
            if msg_title == 'init_data':
                _, exp_name, exp_parameter = msg
                self.data_manager.init_data(exp_name,exp_parameter)
            if msg_title == 'run':
                self.toolbar_run_QAction.setDisabled(True)
                self.toolbar_stop_QAction.setEnabled(True)
                # Control Open Ephys
                if self.open_ephys_QCheckBox.isChecked():
                    try:
                        open_ephys_msg = f'StartRecord RecordNode=1 CreateNewDir=1 RecDir={self.data_path_QLineEdit.text()}'
                        self.open_ephys_socket.send_string(open_ephys_msg)
                        self.open_ephys_socket.recv()
                    except:
                        self.log_QPlainTextEdit.appendPlainText('Error in controlling Open Ephys')
                # Disable file path search
                self.data_path_QPushButton.setDisabled(True)
            if msg_title == 'stop':
                self.toolbar_run_QAction.setEnabled(True)
                self.toolbar_stop_QAction.setDisabled(True)
                # Control Open Ephys
                if self.open_ephys_QCheckBox.isChecked():
                    try:
                        open_ephys_msg = 'StopRecord'
                        self.open_ephys_socket.send_string(open_ephys_msg)
                        self.open_ephys_socket.recv()
                        # Find the latest recording folder and rename subfolder to 'raw_data'
                        rec_dir = self.data_path_QLineEdit.text()
                        recent_rec_dir = max([os.path.join(rec_dir,d) for d in os.listdir(rec_dir)], key=os.path.getmtime)
                        os.rename(os.path.join(recent_rec_dir,os.listdir(recent_rec_dir)[0]), os.path.join(recent_rec_dir,'raw_data'))
                    except:
                        self.log_QPlainTextEdit.appendPlainText('Error in controlling Open Ephys')
                        self.toolbar_run_QAction.setEnabled(True)
                        self.toolbar_stop_QAction.setDisabled(True)
                # Convert the data of the current recording
                self.data_manager.convert_data()
                # If controlling Open Ephys, copy the behavior files to Open Ephys folder
                if self.open_ephys_QCheckBox.isChecked():
                    try:
                        self.open_ephys_socket.send_string('IsAcquiring') # dummy check to see Open Ephys comm. works
                        self.open_ephys_socket.recv()
                        shutil.copy(os.path.join(self.data_manager.data_file_path +'.hdf5'),os.path.join(recent_rec_dir,'raw_data')) # rec. path from above
                        shutil.copy(os.path.join(self.data_manager.data_file_path +'.mat'),os.path.join(recent_rec_dir,'raw_data'))
                    except Exception as error:
                        self.log_QPlainTextEdit.appendPlainText(str(error) + '.')
                # Enable file path search
                self.data_path_QPushButton.setEnabled(True)
    @pyqtSlot()
    def data_path_QPushButton_clicked(self):
        if self.data_path_QFileDialog.exec_():
            data_path = self.data_path_QFileDialog.selectedFiles()
            # Save data path and display
            with open('sys_parameter.json','r') as file:
                sys_parameter = json.load(file)
            sys_parameter['data_path'] = data_path[0]
            with open('sys_parameter.json','w') as file:
                json.dump(sys_parameter, file, indent=4)
            self.data_path_QLineEdit.setText(data_path[0])

    @pyqtSlot(object)
    def data_manager_signalled(self, signal):
        message = signal[0]
        if message == 'log':
            self.log_QPlainTextEdit.appendPlainText(signal[1])

    def init_open_ephys_connection(self, port_num):
        open_ephys_context = zmq.Context()
        open_ephys_socket = open_ephys_context.socket(zmq.REQ)
        open_ephys_socket.setsockopt(zmq.CONFLATE, 1)
        open_ephys_socket.RCVTIMEO = 1000
        open_ephys_socket.connect(f"tcp://127.0.0.1:{port_num}")

        return open_ephys_socket

if __name__ == '__main__':
    if sys.flags.interactive != 1 or not hasattr(QtCore, 'PYQT_VERSION'):
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(u'_') # explicitly setting id to have a different icon in the task bar
        app = QApplication(sys.argv)
        app.setWindowIcon(QtGui.QIcon(os.path.join('.', 'icon', 'experiment_window.png')))
        app_gui = PlotGui(0)
        app_gui.show()
        sys.exit(app.exec())
