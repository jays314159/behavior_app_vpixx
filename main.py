"""
Laboratory for Computational Motor Control, Johns Hopkins School of Medicine
@author: Jay Pi <jay.s.314159@gmail.com>
"""
from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow, QToolBar, QAction, QWidget, QVBoxLayout,\
                            QLabel, QHBoxLayout, QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox, QFrame,\
                            QPushButton, QPlainTextEdit
from PyQt5.QtCore import Qt
from psychopy import monitors, visual, core

from calibration.calibration import CalFsmProcess, CalGuiProcess
from calibration.refinement import CalRefineFsmProcess, CalRefineGuiProcess
from experiment.simple_saccade import SimpleSacGuiProcess, SimpleSacFsmProcess
from experiment.corr_saccade import CorrSacGuiProcess, CorrSacFsmProcess

from target import TargetWidget
import app_lib as lib

import sys, multiprocessing, os, json, time, traceback
from pathlib import Path
import pyqtgraph as pg
class MainGui(QMainWindow):
    def __init__(self, parent = None):
        super(MainGui,self).__init__(parent)
        multiprocessing.set_start_method('spawn')
        
        # Build menu
        self.menubar = self.menuBar()
        self.menu_exp = self.menubar.addMenu('Experiment')
        self.menu_cal = self.menubar.addMenu('Calibration')
        
        # Actions
        self.simple_sac_QAction = QAction('Simple Saccade',self)
        self.menu_exp.addAction(self.simple_sac_QAction)
        self.corr_sac_QAction = QAction('Corrective Saccade', self)
        self.menu_exp.addAction(self.corr_sac_QAction)
        self.cal_QAction = QAction('Calibration',self)
        self.menu_cal.addAction(self.cal_QAction)
        self.refine_cal_QAction = QAction('Refinement',self)
        self.menu_cal.addAction(self.refine_cal_QAction)
        
        # Setting GUI
        self.main_QWidget = QWidget()
        self.setCentralWidget(self.main_QWidget)
        self.main_QVBoxLayout = QVBoxLayout()
        self.main_QWidget.setLayout(self.main_QVBoxLayout)
        self.lock_unlock_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.lock_unlock_QHBoxLayout)
        self.unlock_QPushButton = QPushButton('Unlock')
        self.lock_unlock_QHBoxLayout.addWidget(self.unlock_QPushButton)
        self.lock_QPushButton = QPushButton('Lock')
        self.lock_QPushButton.setDisabled(True)
        self.lock_unlock_QHBoxLayout.addWidget(self.lock_QPushButton)
        
        self.main_separator_0_QFrame = QFrame()
        self.main_QVBoxLayout.addWidget(self.main_separator_0_QFrame)
        self.main_separator_0_QFrame.setFrameShape(QFrame.HLine)
        self.main_separator_0_QFrame.setFrameShadow(QFrame.Raised)
        
        self.monitor_QLabel = QLabel("<b>Monitor info</b>")
        self.main_QVBoxLayout.addWidget(self.monitor_QLabel)
        self.monitor_QLabel.font().setPointSize(12)
        self.monitor_QLabel.setAlignment(Qt.AlignCenter)
        self.monitor_name_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.monitor_name_QHBoxLayout)
        self.monitor_name_QLabel = QLabel("Monitor name: ")
        self.monitor_name_QHBoxLayout.addWidget(self.monitor_name_QLabel)
        self.monitor_name_QLabel.setAlignment(Qt.AlignRight)
        self.monitor_name_QLineEdit = QLineEdit()
        self.monitor_name_QHBoxLayout.addWidget(self.monitor_name_QLineEdit)
        self.monitor_name_QLineEdit.setText('behave')
        self.monitor_name_QLineEdit.setDisabled(True)
        self.monitor_name_QLineEdit.setToolTip('No need to change')
        self.monitor_total_num_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.monitor_total_num_QHBoxLayout)
        self.monitor_total_num_QLabel = QLabel('# of monitors: ')
        self.monitor_total_num_QLabel.setAlignment(Qt.AlignRight)
        self.monitor_total_num_QHBoxLayout.addWidget(self.monitor_total_num_QLabel)
        self.monitor_total_num_QSpinBox = QSpinBox()
        self.monitor_total_num_QHBoxLayout.addWidget(self.monitor_total_num_QSpinBox)
        self.monitor_total_num_QSpinBox.setMinimum(1)
        self.monitor_num_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.monitor_num_QHBoxLayout)
        self.monitor_num_QLabel = QLabel('Monitor #: ')
        self.monitor_num_QLabel.setAlignment(Qt.AlignRight)
        self.monitor_num_QHBoxLayout.addWidget(self.monitor_num_QLabel)
        self.monitor_num_QLabel.setAlignment(Qt.AlignRight)
        self.monitor_num_QComboBox = QComboBox()
        self.monitor_num_QHBoxLayout.addWidget(self.monitor_num_QComboBox)
        self.monitor_num_test_QPushButton = QPushButton('Test')
        self.monitor_num_QHBoxLayout.addWidget(self.monitor_num_test_QPushButton)
        self.monitor_num_close_QPushButton = QPushButton('Close')
        self.monitor_num_QHBoxLayout.addWidget(self.monitor_num_close_QPushButton)
        self.monitor_num_close_QPushButton.setDisabled(True)
        self.monitor_size_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.monitor_size_QHBoxLayout)
        self.monitor_size_QLabel = QLabel('Monitor size; H, V (px): ')
        self.monitor_size_QLabel.setAlignment(Qt.AlignRight)
        self.monitor_size_QHBoxLayout.addWidget(self.monitor_size_QLabel)
        self.monitor_size_horz_QDoubleSpinBox = QDoubleSpinBox()
        self.monitor_size_QHBoxLayout.addWidget(self.monitor_size_horz_QDoubleSpinBox)
        self.monitor_size_horz_QDoubleSpinBox.setMinimum(1)
        self.monitor_size_horz_QDoubleSpinBox.setMaximum(4000)
        self.monitor_size_horz_QDoubleSpinBox.setDecimals(0)
        self.monitor_size_vert_QDoubleSpinBox = QDoubleSpinBox()
        self.monitor_size_QHBoxLayout.addWidget(self.monitor_size_vert_QDoubleSpinBox)
        self.monitor_size_vert_QDoubleSpinBox.setMinimum(1)
        self.monitor_size_vert_QDoubleSpinBox.setMaximum(4000)
        self.monitor_size_vert_QDoubleSpinBox.setDecimals(0)
        self.monitor_dist_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.monitor_dist_QHBoxLayout)
        self.monitor_dist_QLabel = QLabel('Monitor dist. (cm): ')
        self.monitor_dist_QHBoxLayout.addWidget(self.monitor_dist_QLabel)
        self.monitor_dist_QLabel.setAlignment(Qt.AlignRight)
        self.monitor_dist_QDoubleSpinBox = QDoubleSpinBox()
        self.monitor_dist_QHBoxLayout.addWidget(self.monitor_dist_QDoubleSpinBox)
        self.monitor_dist_QDoubleSpinBox.setMinimum(1)
        self.monitor_dist_QDoubleSpinBox.setMaximum(1000)
        self.monitor_dist_QDoubleSpinBox.setDecimals(0)
        self.monitor_width_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.monitor_width_QHBoxLayout)
        self.monitor_width_QLabel = QLabel('Monitor width (cm): ')
        self.monitor_width_QHBoxLayout.addWidget(self.monitor_width_QLabel)
        self.monitor_width_QLabel.setAlignment(Qt.AlignRight)
        self.monitor_width_QDoubleSpinBox = QDoubleSpinBox()
        self.monitor_width_QHBoxLayout.addWidget(self.monitor_width_QDoubleSpinBox)
        self.monitor_width_QDoubleSpinBox.setMinimum(0)
        self.monitor_width_QDoubleSpinBox.setMaximum(1000)
        self.monitor_width_QDoubleSpinBox.setDecimals(0)
        self.monitor_rate_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.monitor_rate_QHBoxLayout)
        
        self.main_separator_1_QFrame = QFrame()
        self.main_QVBoxLayout.addWidget(self.main_separator_1_QFrame)
        self.main_separator_1_QFrame.setFrameShape(QFrame.HLine)
        self.main_separator_1_QFrame.setFrameShadow(QFrame.Raised)
        
        self.which_eye_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.which_eye_QHBoxLayout)
        self.which_eye_QLabel = QLabel('Which eye to track: ')
        self.which_eye_QHBoxLayout.addWidget(self.which_eye_QLabel)
        self.which_eye_QLabel.setAlignment(Qt.AlignRight)
        self.which_eye_QComboBox = QComboBox()
        self.which_eye_QHBoxLayout.addWidget(self.which_eye_QComboBox)
        self.which_eye_QComboBox.addItems(['Left', 'Right'])
        self.which_eye_QComboBox.setToolTip('Depending on device and mirror configuration, the side may be flipped')
        
        self.main_separator_2_QFrame = QFrame()
        self.main_QVBoxLayout.addWidget(self.main_separator_2_QFrame)
        self.main_separator_2_QFrame.setFrameShape(QFrame.HLine)
        self.main_separator_2_QFrame.setFrameShadow(QFrame.Raised)
        
        self.monkey_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.monkey_QHBoxLayout)
        self.monkey_QLabel = QLabel('Monkey: ')
        self.monkey_QHBoxLayout.addWidget(self.monkey_QLabel)
        self.monkey_QLabel.setAlignment(Qt.AlignRight)
        self.monkey_QComboBox = QComboBox()
        self.monkey_QHBoxLayout.addWidget(self.monkey_QComboBox)
        self.monkey_delete_QPushButton = QPushButton('Delete')
        self.monkey_QHBoxLayout.addWidget(self.monkey_delete_QPushButton)
        self.monkey_QLineEdit = QLineEdit()
        self.monkey_QHBoxLayout.addWidget(self.monkey_QLineEdit)
        self.monkey_add_QPushButton = QPushButton('Add')
        self.monkey_QHBoxLayout.addWidget(self.monkey_add_QPushButton)
        
        self.main_separator_3_QFrame = QFrame()
        self.main_QVBoxLayout.addWidget(self.main_separator_3_QFrame)
        self.main_separator_3_QFrame.setFrameShape(QFrame.HLine)
        self.main_separator_3_QFrame.setFrameShadow(QFrame.Raised)
        
        self.sys_password_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.sys_password_QHBoxLayout)
        self.sys_password_QLabel = QLabel('System password: ')
        self.sys_password_QLabel.setToolTip('Needed to empty Linux log files')
        self.sys_password_QHBoxLayout.addWidget(self.sys_password_QLabel)
        self.sys_password_QLabel.setAlignment(Qt.AlignRight)
        self.sys_password_QLineEdit = QLineEdit()
        self.sys_password_QHBoxLayout.addWidget(self.sys_password_QLineEdit)
        self.sys_password_QLineEdit.setToolTip('Needed to empty Linux log files')
        
        self.main_separator_4_QFrame = QFrame()
        self.main_QVBoxLayout.addWidget(self.main_separator_4_QFrame)
        self.main_separator_4_QFrame.setFrameShape(QFrame.HLine)
        self.main_separator_4_QFrame.setFrameShadow(QFrame.Raised)
        
        self.main_QVBoxLayout.addWidget(QLabel('Restart exp. window to apply the settings; they are automatically saved.'))
        
        self.log_QPlainTextEdit = QPlainTextEdit()
        self.log_QPlainTextEdit.setReadOnly(True)
        self.main_QVBoxLayout.addWidget(self.log_QPlainTextEdit)
        
        self.lock_parameter()
        
        # Load parameters
        self.mon_parameter, self.mon_parameter_file_path = lib.load_parameter('','monitor_setting.json',False,False,self.set_default_mon_parameter)
        self.monitor_name_QLineEdit.setText(self.mon_parameter['monitor_name'])
        self.monitor_total_num_QSpinBox.setValue(self.mon_parameter['num_monitor'])
        for i in range(self.monitor_total_num_QSpinBox.value()):
            self.monitor_num_QComboBox.addItem(str(i))
        self.monitor_num_QComboBox.setCurrentText(str(self.mon_parameter['monitor_num']))
        self.monitor_size_horz_QDoubleSpinBox.setValue(self.mon_parameter['monitor_size'][0])
        self.monitor_size_vert_QDoubleSpinBox.setValue(self.mon_parameter['monitor_size'][1])
        self.monitor_dist_QDoubleSpinBox.setValue(self.mon_parameter['monitor_distance'])
        self.monitor_width_QDoubleSpinBox.setValue(self.mon_parameter['monitor_width'])
        
        
        self.main_parameter, self.main_parameter_file_path = lib.load_parameter('','main_parameter.json',False,False,self.set_default_parameter)
        for monkey_id in self.main_parameter['monkey']:
            self.monkey_QComboBox.addItem(monkey_id)
        self.monkey_QComboBox.setCurrentText(self.main_parameter['current_monkey'])
        self.sys_password_QLineEdit.setText(self.main_parameter['sys_password'])
        
        self.cal_parameter,self.cal_parameter_file_path = lib.load_parameter('calibration','cal_parameter.json',True,True,lib.set_default_cal_parameter,'calibration',self.monkey_QComboBox.currentText())
        self.which_eye_QComboBox.setCurrentText(self.cal_parameter['which_eye_tracked'])
    #%% Signals
        self.simple_sac_QAction.triggered.connect(self.simple_sac_QAction_triggered)
        self.corr_sac_QAction.triggered.connect(self.corr_sac_QAction_triggered)
        self.cal_QAction.triggered.connect(self.cal_QAction_triggered)
        self.refine_cal_QAction.triggered.connect(self.refine_cal_QAction_triggered)
        
        self.monitor_total_num_QSpinBox.valueChanged.connect(self.monitor_total_num_QSpinBox_valueChanged)
        self.monitor_num_test_QPushButton.clicked.connect(self.monitor_num_test_QPushButton_clicked)
        self.monitor_num_close_QPushButton.clicked.connect(self.monitor_num_close_QPushButton_clicked)
        
        self.monkey_QComboBox.currentTextChanged.connect(self.monkey_QComboBox_currentTextChanged)
        self.monkey_add_QPushButton.clicked.connect(self.monkey_add_QPushButton_clicked)
        self.monkey_delete_QPushButton.clicked.connect(self.monkey_delete_QPushButton_clicked)
        self.lock_QPushButton.clicked.connect(self.lock_parameter)
        self.unlock_QPushButton.clicked.connect(self.unlock_parameter)
    #%% Slots
    def simple_sac_QAction_triggered(self):
        # Empty Linux log files; communication with tracker fills up the files, eventually crashing
        sys_password = self.sys_password_QLineEdit.text()
        cmd_output = os.system("echo %s | sudo -S sh -c 'echo > /var/log/syslog'" % (sys_password))
        os.system("echo %s | sudo -S sh -c 'echo > /var/log/syslog.1'" % (sys_password))
        if cmd_output != 0:
            self.log_QPlainTextEdit.appendPlainText("Input correct password to clear log files and try again")
            return
        
        self.save_parameter()
        
        stop_exp_Event = multiprocessing.Event()
        stop_exp_Event.set()
        stop_fsm_process_Event = multiprocessing.Event()
        fsm_to_gui_rcvr, fsm_to_gui_sndr = multiprocessing.Pipe(duplex=False)
        gui_to_fsm_rcvr, gui_to_fsm_sndr = multiprocessing.Pipe(duplex=False)
        
        real_time_data_Array = multiprocessing.Array('d', range(5))
        exp_name = 'simple_saccade'
        fsm_process = SimpleSacFsmProcess(exp_name, fsm_to_gui_sndr, gui_to_fsm_rcvr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array, self.main_parameter, self.mon_parameter)
        gui_process = SimpleSacGuiProcess(exp_name, fsm_to_gui_rcvr, gui_to_fsm_sndr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array, self.main_parameter)
        
        fsm_process.start()
        time.sleep(0.25)
        gui_process.start()
        
    def corr_sac_QAction_triggered(self):
        # Empty Linux log files; communication with tracker fills up the files, eventually crashing
        sys_password = self.sys_password_QLineEdit.text()
        cmd_output = os.system("echo %s | sudo -S sh -c 'echo > /var/log/syslog'" % (sys_password))
        os.system("echo %s | sudo -S sh -c 'echo > /var/log/syslog.1'" % (sys_password))
        if cmd_output != 0:
            self.log_QPlainTextEdit.appendPlainText("Input correct password to clear log files and try again")
            return
        
        self.save_parameter()
        
        stop_exp_Event = multiprocessing.Event()
        stop_exp_Event.set()
        stop_fsm_process_Event = multiprocessing.Event()
        fsm_to_gui_rcvr, fsm_to_gui_sndr = multiprocessing.Pipe(duplex=False)
        gui_to_fsm_rcvr, gui_to_fsm_sndr = multiprocessing.Pipe(duplex=False)
        
        real_time_data_Array = multiprocessing.Array('d', range(5))
        exp_name = 'random_corrective_saccades'
        fsm_process = CorrSacFsmProcess(exp_name, fsm_to_gui_sndr, gui_to_fsm_rcvr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array, self.main_parameter, self.mon_parameter)
        gui_process = CorrSacGuiProcess(exp_name, fsm_to_gui_rcvr, gui_to_fsm_sndr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array, self.main_parameter)
                            
        fsm_process.start()
        time.sleep(0.25)
        gui_process.start()
    
    def cal_QAction_triggered(self):
        # Empty Linux log files; communication with tracker fills up the files, eventually crashing
        sys_password = self.sys_password_QLineEdit.text()
        cmd_output = os.system("echo %s | sudo -S sh -c 'echo > /var/log/syslog'" % (sys_password))
        os.system("echo %s | sudo -S sh -c 'echo > /var/log/syslog.1'" % (sys_password))
        if cmd_output != 0:
            self.log_QPlainTextEdit.appendPlainText("Input correct password to clear log files and try again")
            return
        
        self.save_parameter()
        
        stop_exp_Event = multiprocessing.Event()
        stop_exp_Event.set()
        stop_fsm_process_Event = multiprocessing.Event()
        fsm_to_gui_rcvr, fsm_to_gui_sndr = multiprocessing.Pipe(duplex=False)
        gui_to_fsm_rcvr, gui_to_fsm_sndr = multiprocessing.Pipe(duplex=False)
        
        real_time_data_Array = multiprocessing.Array('d', range(4))
        cal_name = 'calibration'
        fsm_process = CalFsmProcess(cal_name, fsm_to_gui_sndr, gui_to_fsm_rcvr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array, self.main_parameter, self.mon_parameter)
        gui_process = CalGuiProcess(cal_name, fsm_to_gui_rcvr, gui_to_fsm_sndr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array, self.main_parameter)
        
        fsm_process.start()
        time.sleep(0.25)
        gui_process.start()
    
    def refine_cal_QAction_triggered(self):    
        # Empty Linux log files; communication with tracker fills up the files, eventually crashing
        sys_password = self.sys_password_QLineEdit.text()
        cmd_output = os.system("echo %s | sudo -S sh -c 'echo > /var/log/syslog'" % (sys_password))
        os.system("echo %s | sudo -S sh -c 'echo > /var/log/syslog.1'" % (sys_password))
        if cmd_output != 0:
            self.log_QPlainTextEdit.appendPlainText("Input correct password to clear log files and try again")
            return
        
        self.save_parameter()
        
        stop_exp_Event = multiprocessing.Event()
        stop_exp_Event.set()
        stop_fsm_process_Event = multiprocessing.Event()
        fsm_to_gui_rcvr, fsm_to_gui_sndr = multiprocessing.Pipe(duplex=False)
        gui_to_fsm_rcvr, gui_to_fsm_sndr = multiprocessing.Pipe(duplex=False)
        
        real_time_data_Array = multiprocessing.Array('d', range(5))
        cal_name = 'refinement'
        fsm_process = CalRefineFsmProcess(cal_name, fsm_to_gui_sndr, gui_to_fsm_rcvr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array, self.main_parameter, self.mon_parameter)
        gui_process = CalRefineGuiProcess(cal_name, fsm_to_gui_rcvr, gui_to_fsm_sndr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array, self.main_parameter)
        
        fsm_process.start()
        time.sleep(0.25)
        gui_process.start()
        
    def monitor_total_num_QSpinBox_valueChanged(self):
        self.monitor_num_QComboBox.clear()
        for i in range(self.monitor_total_num_QSpinBox.value()):
            self.monitor_num_QComboBox.addItem(str(i+1))
        self.monitor_num_QComboBox.setCurrentText('1')
        
    def monitor_num_test_QPushButton_clicked(self):
        self.monitor_num_test_QPushButton.setDisabled(True)
        self.monitor_num_close_QPushButton.setEnabled(True)
        this_monitor = monitors.Monitor(self.mon_parameter['monitor_name'], width=self.mon_parameter['monitor_width'], distance=self.mon_parameter['monitor_distance'])
        this_monitor.save()
        this_monitor.setSizePix(self.mon_parameter['monitor_size'])
        self.window = visual.Window(screen=self.monitor_num_QComboBox.currentIndex(), allowGUI=False, color='white',monitor=this_monitor,
                                    units='deg',winType='pyglet', fullscr=True, checkTiming=False, waitBlanking=False)
    def monitor_num_close_QPushButton_clicked(self):
        self.monitor_num_test_QPushButton.setEnabled(True)
        self.monitor_num_close_QPushButton.setDisabled(True)
        self.window.close()
    
    def monkey_add_QPushButton_clicked(self):
        monkey_id = self.monkey_QLineEdit.text().upper()
        # Add if not already present
        is_present = False
        for counter_id in range(self.monkey_QComboBox.count()):
            if monkey_id == self.monkey_QComboBox.itemText(counter_id):
                is_present = True
        if not is_present:
            self.monkey_QComboBox.addItem(monkey_id)
            self.monkey_QLineEdit.setText(monkey_id)
            _,_ = lib.load_parameter('calibration','cal_parameter.json',True,True,lib.set_default_cal_parameter,'calibration',monkey_id)
        else:
            self.log_QPlainTextEdit.appendPlainText("ID already present")
    def monkey_delete_QPushButton_clicked(self):
        if self.monkey_QComboBox.count() > 1:
            with open(self.cal_parameter_file_path,'r') as file:
                all_parameter = json.load(file)
            all_parameter.pop(self.monkey_QComboBox.currentText(),'')
            with open(self.cal_parameter_file_path,'w') as file:
                json.dump(all_parameter, file, indent=4)
                
            try:
                parameter_path = os.path.join(str(Path().absolute()),'experiment','exp_parameter.json')
                with open(parameter_path, 'r') as file:
                    all_parameter = json.load(file)
                all_parameter.pop(self.monkey_QComboBox.currentText(),'')    
                with open(parameter_path, 'w') as file:
                    json.dump(all_parameter, file, indent=4)
            except:
                pass
            
            self.monkey_QComboBox.removeItem(self.monkey_QComboBox.currentIndex())
        else:
            self.log_QPlainTextEdit.appendPlainText('At least one ID required')
    def monkey_QComboBox_currentTextChanged(self):
        self.cal_parameter,self.cal_parameter_file_path = lib.load_parameter('calibration','cal_parameter.json',True,True,lib.set_default_cal_parameter,'calibration',self.monkey_QComboBox.currentText())
        self.which_eye_QComboBox.setCurrentText(self.cal_parameter['which_eye_tracked'])
        self.main_parameter['current_monkey'] = self.monkey_QComboBox.currentText()
        
    def save_parameter(self):
        self.mon_parameter['monitor_name'] = self.monitor_name_QLineEdit.text()
        self.mon_parameter['num_monitor'] = self.monitor_total_num_QSpinBox.value()
        self.mon_parameter['monitor_num'] = self.monitor_num_QComboBox.currentIndex()
        self.mon_parameter['monitor_size'][0] = self.monitor_size_horz_QDoubleSpinBox.value()
        self.mon_parameter['monitor_size'][1] = self.monitor_size_vert_QDoubleSpinBox.value()
        self.mon_parameter['monitor_distance'] = self.monitor_dist_QDoubleSpinBox.value()
        self.mon_parameter['monitor_width'] = self.monitor_width_QDoubleSpinBox.value()
        with open(self.mon_parameter_file_path,'w') as file:
            json.dump(self.mon_parameter, file, indent=4)
        which_eye_tracked = self.which_eye_QComboBox.currentText()
        self.main_parameter['current_monkey'] = self.monkey_QComboBox.currentText()
        self.main_parameter['sys_password'] = self.sys_password_QLineEdit.text()
        with open(self.cal_parameter_file_path,'r') as file:
            all_parameter = json.load(file)
        all_parameter[self.main_parameter['current_monkey']]['calibration']['which_eye_tracked'] = which_eye_tracked
        with open(self.cal_parameter_file_path,'w') as file:
            json.dump(all_parameter, file, indent=4)
        self.main_parameter['monkey'] = []
        for counter_id in range(self.monkey_QComboBox.count()):
            self.main_parameter['monkey'].append(self.monkey_QComboBox.itemText(counter_id))
        
        with open(self.main_parameter_file_path,'w') as file:
            json.dump(self.main_parameter, file, indent=4)
    
    def lock_parameter(self):
        self.lock_QPushButton.setDisabled(True)
        self.unlock_QPushButton.setEnabled(True)
        self.monitor_total_num_QSpinBox.setDisabled(True)
        self.monitor_num_QComboBox.setDisabled(True)
        self.monitor_size_horz_QDoubleSpinBox.setDisabled(True)
        self.monitor_size_vert_QDoubleSpinBox.setDisabled(True)
        self.monitor_dist_QDoubleSpinBox.setDisabled(True)
        self.monitor_width_QDoubleSpinBox.setDisabled(True)
        self.which_eye_QComboBox.setDisabled(True)
        self.monkey_QComboBox.setDisabled(True)
        self.monkey_delete_QPushButton.setDisabled(True)
        self.monkey_add_QPushButton.setDisabled(True)
        self.sys_password_QLineEdit.setDisabled(True)
        
    def unlock_parameter(self):
        self.lock_QPushButton.setEnabled(True)
        self.unlock_QPushButton.setDisabled(True)
        self.monitor_total_num_QSpinBox.setEnabled(True)
        self.monitor_num_QComboBox.setEnabled(True)
        self.monitor_size_horz_QDoubleSpinBox.setEnabled(True)
        self.monitor_size_vert_QDoubleSpinBox.setEnabled(True)
        self.monitor_dist_QDoubleSpinBox.setEnabled(True)
        self.monitor_width_QDoubleSpinBox.setEnabled(True)
        self.which_eye_QComboBox.setEnabled(True)
        self.monkey_QComboBox.setEnabled(True)
        self.monkey_delete_QPushButton.setEnabled(True)
        self.monkey_add_QPushButton.setEnabled(True)
        self.sys_password_QLineEdit.setEnabled(True)
        
    def set_default_mon_parameter(self):
        parameter = {
                 'monitor_name': 'behave',
                 'num_monitor': 2,
                 'monitor_num': 1,
                 'monitor_size': [1920,1080],
                 'monitor_distance': 85.0,
                 'monitor_width': 70.0,
                 }
    
        return parameter
    
    def set_default_parameter(self):
        parameter = {
                     'monkey': ['0000'],
                     'current_monkey': '0000',
                     'sys_password': '123456'
                     }
        return parameter
    
    def closeEvent(self,event):
        try:
            pg.exit() # this should come at the end 
        except:
            pass
if __name__ == '__main__':
    if sys.flags.interactive != 1 or not hasattr(QtCore, 'PYQT_VERSION'):        
        main_app = QApplication(sys.argv)
        main_app.setWindowIcon(QtGui.QIcon(os.path.join('.', 'icon', 'marmoset.png')))
        main_app_gui = MainGui()
        main_app_gui.show()
        sys.exit(main_app.exec())