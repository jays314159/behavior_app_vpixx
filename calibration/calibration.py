"""
Laboratory for Computational Motor Control, Johns Hopkins School of Medicine
@author: Jay Pi <jay.s.314159@gmail.com>
"""
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QComboBox, QPushButton, QLabel, QHBoxLayout, QDoubleSpinBox, QCheckBox, QPlainTextEdit,\
                            QDialog, QShortcut, QGridLayout
from PyQt5.QtCore import QRunnable, QThreadPool, pyqtSignal, pyqtSlot, QObject, Qt, QTimer
from PyQt5.QtGui import QKeySequence
from psychopy import monitors, visual, core

import pyqtgraph as pg

from pypixxlib import tracker
from pypixxlib._libdpx import DPxOpen, TPxSetupTPxSchedule,TPxEnableFreeRun,DPxSelectDevice,DPxUpdateRegCache, DPxSetTPxAwake,\
                              TPxDisableFreeRun, DPxGetReg16,DPxGetTime,TPxBestPolyGetEyePosition, DPxSetTPxSleep,DPxClose

import multiprocessing, sys, os, json, random, time, copy, ctypes, math, zmq
sys.path.append('../app')
from pathlib import Path
import numpy as np

from fsm_gui import FsmGui
from target import TargetWidget
import app_lib as lib

class Fsm_calThread(multiprocessing.Process):
    def __init__(self, exp_name, fsm_to_gui_sndr, gui_to_fsm_Q, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array):
        super().__init__()
        self.exp_name = exp_name
        self.cal_matrix = []
        self.fsm_to_gui_sndr = fsm_to_gui_sndr
        self.gui_to_fsm_Q = gui_to_fsm_Q
        self.stop_exp_Event = stop_exp_Event
        self.stop_fsm_process_Event = stop_fsm_process_Event
        self.real_time_data_Array = real_time_data_Array
        
        # Init var.
        self.eye_raw_x = 0
        self.eye_raw_y = 0
        self.tgt_x = 0
        self.tgt_y = 0
        self.t = math.nan
        self.tgt_num = 1
        
    @pyqtSlot()
    def run(self):    
        # Set up exp. screen
        file_path = os.path.join(str(Path().absolute()), 'monitor_setting.json')
        with open(file_path,'r') as file:
            setting = json.load(file)
        refresh_rate = setting['monitor_refresh_rate']
        this_monitor = monitors.Monitor(setting['monitor_name'], width=setting['monitor_width'], distance=setting['monitor_distance'])
        this_monitor.setSizePix(setting['monitor_size'])
        self.window = visual.Window(size=setting['monitor_size'],screen=setting['monitor_num'], allowGUI=False, color='white', monitor=this_monitor,
                                units='deg', winType='pyglet', fullscr=True, checkTiming=False, waitBlanking=False)
        self.window.flip()
        
        # Make targets
        self.update_target()
        
        # Check if VPixx available; if so, open
        DPxOpen()
        tracker.TRACKPixx3().open() # this throws error if not device not open           
        DPxSetTPxAwake()
        DPxSelectDevice('DATAPIXX3')   
        DPxUpdateRegCache()
        # Turn on VPixx schedule
        lib.VPixx_turn_on_schedule()
        # Get pointers to store data from device
        cal_data, raw_data = lib.VPixx_get_pointers_for_data()
        
        run_exp = False
        # Process loop
        while not self.stop_fsm_process_Event.is_set():
            if not self.stop_exp_Event.is_set():
                # Update targets
                self.update_target()
                # Load exp parameter
                fsm_parameter, parameter_file_path = lib.load_parameter('calibration','cal_parameter.json',True,self.init_fsm_parameter,self.exp_name)
                # Init. var
                self.t = DPxGetTime()
                run_exp = True  
            # Trial loop
            while not self.stop_fsm_process_Event.is_set() and run_exp: 
                if self.stop_exp_Event.is_set():
                    run_exp = False
                    # Remove all targets
                    self.window.flip()
                    break
                for tgt_idx in range(self.fsm_parameter['num_cal_tgt']):
                    tgt_num_temp = tgt_idx + 1
                    if not self.fsm_parameter['tgt_'+str(tgt_num_temp)][2]:
                        continue
                    self.tgt_num = tgt_num_temp
                    state = 'INIT'
                    # FSM loop
                    while not self.stop_fsm_process_Event.is_set() and run_exp:
                        if self.stop_exp_Event.is_set():
                            break
                        # Get time       
                        self.t = TPxBestPolyGetEyePosition(cal_data, raw_data) # this calls 'DPxUpdateRegCache' as well
                        # Get eye status (blinking)
                        eye_status = DPxGetReg16(0x59A)
                        left_eye_blink = bool(eye_status & (1 << 1)) # right blink, left blink
                        if not left_eye_blink:
                            raw_data_left = [raw_data[2], raw_data[3],1] # [left x, left y, right x, right y]
                            self.eye_raw_x = raw_data_left[0]
                            self.eye_raw_y = raw_data_left[1]
                        else:
                            self.eye_raw_x = 9999 # invalid values; more stable than nan values for plotting purposes in pyqtgraph
                            self.eye_raw_y = 9999 
                        # FSM
                        if state == 'INIT':
                            tgt_p = np.array(self.fsm_parameter['tgt_'+str(self.tgt_num)][0:2])
                            if self.tgt_num != 5:
                                pursuit_start_p = np.array([self.fsm_parameter['start_x'], self.fsm_parameter['start_y']])
                                pursuit_dir = (tgt_p - pursuit_start_p)/np.linalg.norm(tgt_p - pursuit_start_p)
                                pursuit_p = pursuit_dir*self.fsm_parameter['pursuit_amp']
                                pursuit_v = pursuit_p/self.fsm_parameter['pursuit_dur']
                                state_start_time = self.t
                                state_inter_time = self.t
                                state = 'STR_TARGET_PURSUIT'
                            else:
                                state_start_time = self.t
                                state_inter_time = self.t                   
                                state = 'CUE_TARGET_PRESENT'
                                self.fsm_to_screen_sndr.send(('tgt','draw',tgt_p))
                            lib.playSound(1000,0.1) # neutral beep
                        if state == 'STR_TARGET_PURSUIT':
                            if self.fsm_parameter['is_pursuit_tgt']:
                                self.tgt.pos = pursuit_v*(self.t-state_start_time) + pursuit_start_p
                                self.tgt.draw()
                                self.window.flip()
                            if (self.t-state_start_time) > self.fsm_parameter['pursuit_dur'] or not self.fsm_parameter['is_pursuit_tgt']:
                                state_start_time = self.t
                                state_inter_time = self.t                   
                                state = 'CUE_TARGET_PRESENT'
                        if state == 'CUE_TARGET_PRESENT':    
                            self.tgt.pos = tgt_p
                            self.tgt.draw()
                            self.window.flip()
                            if (self.t-state_start_time) >= self.fsm_parameter['cal_dur']:
                                state_start_time = self.t
                                state_inter_time = state_start_time
                                state = 'ITI'
                        if state == 'ITI':
                            self.window.flip()
                            if (self.t-state_start_time) >= self.fsm_parameter['ITI']:
                                break # move onto next target
                        # Update shared real time data
                        with self.real_time_data_Array.get_lock():
                            self.real_time_data_Array[0] = self.tgt_num
                            self.real_time_data_Array[1] = self.t
                            self.real_time_data_Array[2] = self.eye_raw_x
                            self.real_time_data_Array[3] = self.eye_raw_y
        # Turn off VPixx schedule
        lib.VPixx_turn_off_schedule()
        # Close VPixx devices
        DPxSetTPxSleep()
        DPxSelectDevice('DATAPIXX3')
        DPxUpdateRegCache()  
        DPxClose()        
        tracker.TRACKPixx3().close()  
        # Signal completion
        self.signals.to_main_thread.emit(('fsm_done',0))
        # Reset time
        self.t = math.nan
        
    def update_target(self):
        tgt_parameter, _ = lib.load_parameter('','tgt_parameter.json',True,TargetWidget.set_default_parameter,'tgt')
        self.tgt = visual.Rect(win=self.window, width=tgt_parameter['size'],height=tgt_parameter['size'], units='deg', 
                      lineColor=tgt_parameter['line_color'],fillColor=tgt_parameter['fill_color'],
                      lineWidth=tgt_parameter['line_width'])
        self.tgt.draw() # draw once already, because the first draw may be slower - Poth, 2018   
        self.window.clearBuffer() # clear the back buffer of previously drawn stimuli - Poth, 2018
        
    def init_fsm_parameter(self):
        self.fsm_parameter = {'run': False,
                              'start_x': 0,
                              'start_y': 0,
                              'pursuit_amp': 0.7,
                              'pursuit_dur': 0.7,
                              'is_pursuit_tgt': True,
                              'start_dur': 1,
                              'cal_dur': 1,
                              'ITI': 1,
                              'num_cal_tgt': 9,
                              'tgt_1': [-5,5,True],
                              'tgt_2': [0,5,True],
                              'tgt_3': [5,5,True],
                              'tgt_4': [-5,0,True],
                              'tgt_5': [0,0,True],
                              'tgt_6': [5,0,True],
                              'tgt_7': [-5,-5,True],
                              'tgt_8': [0,-5,True],
                              'tgt_9': [5,-5,True]
                              }
        
class Fsm_calGui(FsmGui):
    def __init__(self,exp_name, fsm_to_gui_rcvr, gui_to_fsm_sndr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array):
        self.exp_name = exp_name
        self.fsm_to_gui_rcvr = fsm_to_gui_rcvr
        self.gui_to_fsm_sndr = gui_to_fsm_sndr
        self.stop_exp_Event = stop_exp_Event
        self.stop_fsm_process_Event = stop_fsm_process_Event
        self.real_time_data_Array = real_time_data_Array
        super(Fsm_calGui,self).__init__(self.stop_fsm_process_Event)
        self.init_gui()
        
        # Create socket for ZMQ
        try:
            context = zmq.Context()
            self.fsm_to_plot_priority_socket = context.socket(zmq.PUB)
            self.fsm_to_plot_priority_socket.bind("tcp://192.168.0.2:5557")
        except Exception as error:
            self.log_QPlainTextEdit.appendPlainText('Error in starting zmq sockets:')
            self.log_QPlainTextEdit.appendPlainText(str(error) + '.')
            self.toolbar_run_QAction.setDisabled(True)
            self.toolbar_connect_QAction.setDisabled(True)
            
        # Init. extra shortcuts
        self.tgt_num_QComboBox_up_QShortcut = QShortcut(Qt.Key_Up,self)
        self.tgt_num_QComboBox_down_QShortcut = QShortcut(Qt.Key_Down,self)
        self.tgt_QCheckBox_QShortcut = QShortcut(Qt.Key_Space,self)
        QShortcut(QKeySequence('Ctrl+D'), self.delete_selected_QPushButton, self.delete_selected_QPushButton.animateClick)
        
        # Init. var
        self.data_dict = {}
        for tgt_idx in range(self.num_cal_tgt):
            tgt_num = tgt_idx + 1
            self.reset_data_for_tgt(tgt_num)
        self.plot_1_dict['active'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_x'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_selected_x'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_y'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_selected_y'].setData(np.zeros(0),np.zeros(0))
            
        self.currently_selected_tgt_ind = np.zeros(self.num_cal_tgt, dtype=bool) # indicates which tgt data are collected for calibration
        self.min_tgt_for_cal = 5 # min. num. of tgts whose data are selected to compute calibration
        
        self.cal_eye_x = 0 # values to be used for cal.
        self.cal_eye_y = 0
        self.cal_tgt_x = 0
        self.cal_tgt_y = 0
        self.new_cal = False # indicates whether new calibration done
                
        # ROI plots
        self.plot_1_ROI = self.plot_1_PlotWidget.plot(np.zeros((0)), np.zeros((0)),\
            pen=pg.mkPen(color='m', width=2, style=QtCore.Qt.SolidLine),symbol='o', symbolSize=3, symbolBrush='m', symbolPen=None)
        self.plot_1_ROI_firstToLast = self.plot_1_PlotWidget.plot(np.zeros((0)), np.zeros((0)),\
            pen=pg.mkPen(color='m', width=2, style=QtCore.Qt.DotLine), symbol=None, symbolSize=None, symbolBrush=None, symbolPen=None) # plot to connect first and last points of ROI
        self.plot_2_ROI = self.plot_2_PlotWidget.plot(np.zeros((0)), np.zeros((0)),\
            pen=pg.mkPen(color='m', width=2, style=QtCore.Qt.SolidLine),symbol='o', symbolSize=3, symbolBrush='m', symbolPen=None)
        self.plot_2_ROI_firstToLast = self.plot_2_PlotWidget.plot(np.zeros((0)), np.zeros((0)),\
            pen=pg.mkPen(color='m', width=2, style=QtCore.Qt.DotLine), symbol=None, symbolSize=None, symbolBrush=None, symbolPen=None)
        self.ROI_x_plot_1 = np.zeros(0) # ROI to select calibration points
        self.ROI_y_plot_1 = np.zeros(0)
        self.ROI_x_plot_2 = np.zeros(0)
        self.ROI_y_plot_2 = np.zeros(0)
                
        # Load parameters
        self.cal_parameter, self.parameter_file_path = lib.load_parameter('calibration','cal_parameter.json',True,lib.set_default_cal_parameter,'calibration')
        self.tgt_widgets_dict['tgt_5_horz_QDoubleSpinBox'].setValue(self.cal_parameter['start_x'])
        self.tgt_widgets_dict['tgt_5_vert_QDoubleSpinBox'].setValue(self.cal_parameter['start_y'])
        self.pursuit_amp_QDoubleSpinBox.setValue(self.cal_parameter['pursuit_amp'])
        self.pursuit_dur_QDoubleSpinBox.setValue(self.cal_parameter['pursuit_dur'])
        self.pursuit_QCheckBox.setChecked(self.cal_parameter['is_pursuit_tgt'])
        self.cal_tgt_dur_QDoubleSpinBox.setValue(self.cal_parameter['cal_dur'])
        self.tgt_ITI_QDoubleSpinBox.setValue(self.cal_parameter['ITI'])
        for tgt_idx in range(self.num_cal_tgt):
            tgt_num = tgt_idx + 1
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_horz_QDoubleSpinBox'].setValue(self.cal_parameter['tgt_'+str(tgt_num)][0])
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_vert_QDoubleSpinBox'].setValue(self.cal_parameter['tgt_'+str(tgt_num)][1]),
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'].setChecked(self.cal_parameter['tgt_'+str(tgt_num)][2])
        if self.cal_parameter['cal_status']:
            self.cal_status_QLabel.setText("Calibrated")
            self.cal_status_QLabel.setStyleSheet('background-color: rgb(0,255,0)')   
        else:
            self.cal_status_QLabel.setText("Uncalibrated")
            self.cal_status_QLabel.setStyleSheet('background-color: rgb(255,0,0)')
        
        # Initialize fsm thread        
        self.fsm_thread = Fsm_calThread(self.fsm_to_screen_sndr)
    #%% SIGNALS
        self.data_QTimer.timeout.connect(self.data_QTimer_timeout)
        self.fsm_thread.signals.to_main_thread.connect(self.receive_fsm_signal)
        # Plots
        self.plot_1_ROI_signal = pg.SignalProxy(self.plot_1_PlotWidget.scene().sigMouseClicked, rateLimit=60, slot=self.plot_1_PlotWidget_mouseClicked)
        self.plot_2_ROI_signal = pg.SignalProxy(self.plot_2_PlotWidget.scene().sigMouseClicked, rateLimit=60, slot=self.plot_2_PlotWidget_mouseClicked)
        # Toolbar
        self.toolbar_run_QAction.triggered.connect(self.toolbar_run_QAction_triggered)
        self.toolbar_stop_QAction.triggered.connect(self.toolbar_stop_QAction_triggered)
        
        self.cal_QPushButton.clicked.connect(self.cal_QPushButton_clicked)
        self.select_ROI_QPushButton.clicked.connect(self.select_ROI_QPushButton_clicked)
        self.deselect_QPushButton.clicked.connect(self.deselect_QPushButton_clicked)
        self.clear_ROI_QPushButton.clicked.connect(self.clear_ROI_QPushButton_clicked)
        self.delete_selected_QPushButton.clicked.connect(self.delete_selected_QPushButton_clicked)
        self.delete_all_QPushButton.clicked.connect(self.delete_all_QPushButton_clicked)
        self.delete_QPushButton.clicked.connect(self.delete_QPushButton_clicked)
        self.tgt_num_QComboBox.currentTextChanged.connect(self.tgt_num_QComboBox_currentTextChanged)
        self.clear_cal_QPushButton.clicked.connect(self.clear_cal_QPushButton_clicked)
        self.save_QPushButton.clicked.connect(self.save_QPushButton_clicked)
        self.auto_tgt_QPushButton.clicked.connect(self.auto_tgt_QPushButton_clicked)
        # Shortcuts
        self.tgt_num_QComboBox_up_QShortcut.activated.connect(self.tgt_num_QComboBox_up_QShortcut_activated)
        self.tgt_num_QComboBox_down_QShortcut.activated.connect(self.tgt_num_QComboBox_down_QShortcut_activated) # down arrow
        self.tgt_QCheckBox_QShortcut.activated.connect(self.tgt_QCheckBox_QShortcut_activated) # space bar 
    #%% SLOTS
    @pyqtSlot()
    def toolbar_run_QAction_triggered(self):
        # Save used parameters
        self.save_QPushButton_clicked()  
        # Set GUI elements
        self.toolbar_run_QAction.setDisabled(True)
        self.toolbar_stop_QAction.setEnabled(True)    
        # Disable some user functions
        self.disable_manual_fncs()
        self.sidepanel_parameter_QWidget.setDisabled(True)
        self.tgt.setDisabled(True)
        self.pd_tgt.setDisabled(True)       
        # Update FSM parameters
        self.fsm_thread.fsm_parameter['run'] = True
        self.fsm_thread.fsm_parameter['start_x'] = self.tgt_widgets_dict['tgt_5_horz_QDoubleSpinBox'].value()
        self.fsm_thread.fsm_parameter['start_y'] = self.tgt_widgets_dict['tgt_5_vert_QDoubleSpinBox'].value()
        self.fsm_thread.fsm_parameter['pursuit_amp'] = self.pursuit_amp_QDoubleSpinBox.value()
        self.fsm_thread.fsm_parameter['pursuit_dur'] = self.pursuit_dur_QDoubleSpinBox.value()
        self.fsm_thread.fsm_parameter['is_pursuit_tgt'] = self.pursuit_QCheckBox.isChecked()
        self.fsm_thread.fsm_parameter['cal_dur'] = self.cal_tgt_dur_QDoubleSpinBox.value()
        self.fsm_thread.fsm_parameter['ITI'] = self.tgt_ITI_QDoubleSpinBox.value()
        self.fsm_thread.fsm_parameter['num_cal_tgt'] = self.num_cal_tgt
        for tgt_idx in range(self.num_cal_tgt):
            tgt_num = tgt_idx + 1
            self.fsm_thread.fsm_parameter['tgt_'+str(tgt_num)] = [self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_horz_QDoubleSpinBox'].value(),
                                                                  self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_vert_QDoubleSpinBox'].value(),
                                                                  self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'].isChecked()]
        # Reinitialize data and plots for the targets whose data are being recollected
        for tgt_idx in range(self.num_cal_tgt):
            tgt_num = tgt_idx + 1
            if self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'].isChecked():
                self.reset_data_for_tgt(tgt_num)    
        self.plot_1_dict['active'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_x'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_selected_x'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_selected_y'].setData(np.zeros(0),np.zeros(0))
        # Clear ROI and selected points
        self.clear_ROI_QPushButton_clicked()      
        # Start FSM
        self.stop_exp_Event.clear()       
        # Start timer to get data from fsm thread
        self.data_QTimer.start(self.data_rate)        
        
    @pyqtSlot()
    def toolbar_stop_QAction_triggered(self):
        self.toolbar_run_QAction.setEnabled(True)
        self.toolbar_stop_QAction.setDisabled(True)
        # Enable some user functions
        self.enable_manual_fncs()
        self.sidepanel_parameter_QWidget.setEnabled(True)
        self.tgt.setEnabled(True)
        self.pd_tgt.setEnabled(True)
        # Ask FSM to stop
        self.stop_exp_Event.set()
        # Stop timer to stop getting data from fsm thread
        self.data_QTimer.stop()
    
    # Select data points in ROI
    @pyqtSlot()
    def select_ROI_QPushButton_clicked(self):
        tgt_num = int(self.tgt_num_QComboBox.currentText())
        x_span_data = np.array(self.data_dict['eye_raw_x_tgt_'+str(tgt_num)])
        y_span_data = np.array(self.data_dict['eye_raw_y_tgt_'+str(tgt_num)])
        t_span_data = np.array(self.data_dict['t_tgt_'+str(tgt_num)])
        # Check to see if any ROI chosen; also which plot active
        if len(self.ROI_x_plot_1) > 1 and len(x_span_data) > 0:
            x_span_ROI = np.append(self.ROI_x_plot_1, self.ROI_x_plot_1[0])
            y_span_ROI = np.append(self.ROI_y_plot_1, self.ROI_y_plot_1[0])      
            data_selected = lib.inpolygon(x_span_data,y_span_data,x_span_ROI,y_span_ROI) # in boolean, which data points selected
        elif len(self.ROI_x_plot_2) > 1 and len(x_span_data)>0:
            x_span_ROI = np.append(self.ROI_x_plot_2, self.ROI_x_plot_2[0])
            y_span_ROI = np.append(self.ROI_y_plot_2, self.ROI_y_plot_2[0]) 
            data_selected_x = lib.inpolygon(t_span_data,x_span_data,x_span_ROI,y_span_ROI) # in boolean, which data points selected
            data_selected_y = lib.inpolygon(t_span_data,y_span_data,x_span_ROI,y_span_ROI)
            data_selected = data_selected_x | data_selected_y    
            
        if (len(self.ROI_x_plot_1) > 1 or len(self.ROI_x_plot_2) > 1) and len(x_span_data)>0:
            # Saving selected data
            self.data_dict['eye_raw_selected_tgt_'+str(tgt_num)] = data_selected
            # If any data selected,
            if np.any(data_selected):
                # Disable checkbox for that tgt
                self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'].setChecked(False)
                self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'].setEnabled(False)
                # Highlight selected data
                self.plot_1_dict['tgt_'+str(tgt_num)+'_selected'].setData(x_span_data[data_selected], y_span_data[data_selected])
                self.tgt_num_QComboBox_currentTextChanged()
                # Indicate data selected
                self.currently_selected_tgt_ind[tgt_num-1] = True
                # Clear ROI
                self.clear_ROI_QPushButton_clicked()
                
            # Enable computing calibration matrix, if the number of tgts whose data are selected exceed the specified num.
            if np.sum(self.currently_selected_tgt_ind) >= self.min_tgt_for_cal:
                self.cal_QPushButton.setEnabled(True)  
    
    # Deselect the data points selected for calibration
    @pyqtSlot()
    def deselect_QPushButton_clicked(self):
        tgt_num = int(self.tgt_num_QComboBox.currentText())
        # Check if any selected
        if self.currently_selected_tgt_ind[tgt_num-1] == True:
            # Clear data that store selected points
            self.data_dict['eye_raw_selected_tgt_'+str(tgt_num)] = np.zeros(0, dtype=bool)
            # Clear plots displaying selected points
            self.plot_1_dict['tgt_'+str(tgt_num)+'_selected'].setData(np.zeros(0),np.zeros(0))
            self.plot_2_dict['tgt_selected_x'].setData(np.zeros(0),np.zeros(0))
            self.plot_2_dict['tgt_selected_y'].setData(np.zeros(0),np.zeros(0))
            # Indicate no data are selected for this target
            self.currently_selected_tgt_ind[tgt_num-1] = False
            # Enable checkboxes
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'].setChecked(True)
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'].setEnabled(True) 
            
        # Disable calibration, if the number of tgts of whose data are selected is less than the specified num.
        if np.sum(self.currently_selected_tgt_ind) < self.min_tgt_for_cal:
            self.cal_QPushButton.setEnabled(False) 

    @pyqtSlot()
    def cal_QPushButton_clicked(self):        
        eye_p_all = np.zeros(shape=(0,3)) # init. of var. to contain eye_x,eye_y,1 columns
        tgt_p_all = np.zeros(shape=(0,3)) # init. of var. to contain target_x,target_y,1 columns
        selected_tgt_idc = np.where(self.currently_selected_tgt_ind)[0]
        for tgt_idx in range(np.sum(self.currently_selected_tgt_ind)):
            tgt_num = selected_tgt_idc[tgt_idx] + 1
            eye_raw_x = self.data_dict['eye_raw_x_tgt_'+str(tgt_num)]
            eye_raw_y = self.data_dict['eye_raw_y_tgt_'+str(tgt_num)]
            selected_data = self.data_dict['eye_raw_selected_tgt_'+str(tgt_num)]
            ################################### TEMPORARY FIX ######################################
            if len(eye_raw_y) > len(selected_data):
                eye_raw_x = eye_raw_x[:len(selected_data)]
            else:
                selected_data = selected_data[:len(eye_raw_x)]
            if len(eye_raw_y) > len(selected_data):
                eye_raw_y = eye_raw_y[:len(selected_data)]
            else:
                selected_data = selected_data[:len(eye_raw_y)]    
            ################################### TEMPORARY FIX ######################################    
            eye_raw_x_selected = np.array(eye_raw_x)[selected_data]
            mean_eye_raw_x_selected = np.mean(eye_raw_x_selected)
            eye_raw_y_selected = np.array(eye_raw_y)[selected_data]
            mean_eye_raw_y_selected = np.mean(eye_raw_y_selected)
            eye_raw_p = np.hstack([mean_eye_raw_x_selected,mean_eye_raw_y_selected,1]) # horz. concat. of x,y,1 
            eye_p_all = np.vstack([eye_p_all, eye_raw_p])
            tgt_x = self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_horz_QDoubleSpinBox'].value()
            tgt_y = self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_vert_QDoubleSpinBox'].value()
            tgt_p = np.array([tgt_x,tgt_y,1])
            tgt_p_all = np.vstack([tgt_p_all, tgt_p])
        
        cal_matrix = np.linalg.inv(eye_p_all.T@eye_p_all)@eye_p_all.T@tgt_p_all
        self.cal_parameter['cal_matrix'] = cal_matrix.tolist()
        tgt_p_pred = eye_p_all @ self.cal_parameter['cal_matrix']
        tgt_p_pred = tgt_p_pred[:,[0,1]]
        self.cal_parameter['RMSE'] = np.sqrt(np.sum(np.square(tgt_p_all[:,[0,1]]-tgt_p_pred))/len(selected_tgt_idc))
        self.log_QPlainTextEdit.appendPlainText("RMSE: " + str(self.cal_parameter['RMSE']) + " deg")
        self.cal_parameter['cal_status'] = True
        self.cal_status_QLabel.setText("Calibrated")
        self.cal_status_QLabel.setStyleSheet('background-color: rgb(0,255,0)')   
     
    @pyqtSlot()
    def save_QPushButton_clicked(self):
        try:
            with open(self.parameter_file_path,'r') as file:
                all_parameter = json.load(file)
            self.cal_parameter['start_x'] = self.tgt_widgets_dict['tgt_5_horz_QDoubleSpinBox'].value()
            self.cal_parameter['start_y'] = self.tgt_widgets_dict['tgt_5_vert_QDoubleSpinBox'].value()
            self.cal_parameter['pursuit_amp'] = self.pursuit_amp_QDoubleSpinBox.value()
            self.cal_parameter['pursuit_dur'] = self.pursuit_dur_QDoubleSpinBox.value()
            self.cal_parameter['is_pursuit_tgt'] = self.pursuit_QCheckBox.isChecked()
            self.cal_parameter['cal_dur'] = self.cal_tgt_dur_QDoubleSpinBox.value()
            self.cal_parameter['ITI'] = self.tgt_ITI_QDoubleSpinBox.value()
            self.cal_parameter['num_cal_tgt'] = self.num_cal_tgt
            for tgt_idx in range(self.num_cal_tgt):
                tgt_num = tgt_idx + 1
                self.cal_parameter['tgt_'+str(tgt_num)] = [self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_horz_QDoubleSpinBox'].value(),
                                                                      self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_vert_QDoubleSpinBox'].value(),
                                                                      self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'].isChecked()]
            all_parameter[self.exp_name] = self.cal_parameter    
            with open(self.parameter_file_path,'w') as file:
                json.dump(all_parameter, file, indent=4)   
            self.log_QPlainTextEdit.appendPlainText('Saved calibration along with parameters')
        except Exception as e:
            self.log_QPlainTextEdit.appendPlainText('Calibration save failed')
            self.log_QPlainTextEdit.appendPlainText(e)
        
    @pyqtSlot()
    def data_QTimer_timeout(self):
        '''
        getting data from fsm process
        '''   
        with self.real_time_data_Array.get_lock():
            tgt_num = self.real_time_data_Array[0]
            t = self.real_time_data_Array[1]
            eye_raw_x = self.real_time_data_Array[2]
            eye_raw_y = self.real_time_data_Array[3]
        if not math.isnan(t):
            self.data_dict['eye_raw_x_tgt_'+str(tgt_num)].append(eye_raw_x)
            self.data_dict['eye_raw_y_tgt_'+str(tgt_num)].append(eye_raw_y)
            self.data_dict['t_abs_tgt_'+str(tgt_num)].append(t)
            self.data_dict['t_tgt_'+str(tgt_num)].append(\
                    self.data_dict['t_abs_tgt_'+str(tgt_num)][-1] - 
                    self.data_dict['t_abs_tgt_'+str(tgt_num)][0])
            # Plot data
            self.plot_1_dict['tgt_'+str(tgt_num)].\
                setData(np.array(self.data_dict['eye_raw_x_tgt_'+str(tgt_num)]),np.array(self.data_dict['eye_raw_y_tgt_'+str(tgt_num)]))
            self.plot_1_dict['active'].\
                setData(np.array(self.data_dict['eye_raw_x_tgt_'+str(tgt_num)]),np.array(self.data_dict['eye_raw_y_tgt_'+str(tgt_num)]))
            self.plot_2_dict['tgt_x'].\
                setData(np.array(self.data_dict['t_tgt_'+str(tgt_num)]), np.array(self.data_dict['eye_raw_x_tgt_'+str(tgt_num)]))
            self.plot_2_dict['tgt_y'].\
                setData(np.array(self.data_dict['t_tgt_'+str(tgt_num)]), np.array(self.data_dict['eye_raw_y_tgt_'+str(tgt_num)]))
            self.plot_2_PlotWidget_ViewBox.setRange(xRange=(self.data_dict['t_tgt_'+str(tgt_num)][0],self.data_dict['t_tgt_'+str(tgt_num)][-1]))
            self.tgt_num_QComboBox.setCurrentText(str(tgt_num))
    
    @pyqtSlot()
    def clear_ROI_QPushButton_clicked(self):
        self.ROI_x_plot_1 = np.zeros(0)
        self.ROI_y_plot_1 = np.zeros(0)
        self.ROI_x_plot_2 = np.zeros(0)
        self.ROI_y_plot_2 = np.zeros(0)
        self.plot_1_ROI.setData(np.zeros(0), np.zeros(0))
        self.plot_1_ROI_firstToLast.setData(np.zeros(0), np.zeros(0))
        self.plot_2_ROI.setData(np.zeros(0), np.zeros(0))
        self.plot_2_ROI_firstToLast.setData(np.zeros(0), np.zeros(0))
        
    # Delete selected data, if any selected
    @pyqtSlot()
    def delete_selected_QPushButton_clicked(self):
        tgt_num = int(self.tgt_num_QComboBox.currentText())
        # Check if any selected
        if self.currently_selected_tgt_ind[tgt_num-1] == True:
            # Keep non-selected points from the data
            not_selected_data_ind = ~self.data_dict['eye_raw_selected_tgt_'+str(tgt_num)]
            self.data_dict['eye_raw_x_tgt_'+str(tgt_num)] = np.array(self.data_dict['eye_raw_x_tgt_'+str(tgt_num)])[not_selected_data_ind]   
            self.data_dict['eye_raw_x_tgt_'+str(tgt_num)] = self.data_dict['eye_raw_x_tgt_'+str(tgt_num)].tolist() # converting to list, to use append   
            self.data_dict['eye_raw_y_tgt_'+str(tgt_num)] = np.array(self.data_dict['eye_raw_y_tgt_'+str(tgt_num)])[not_selected_data_ind]
            self.data_dict['eye_raw_y_tgt_'+str(tgt_num)] = self.data_dict['eye_raw_y_tgt_'+str(tgt_num)].tolist()
            self.data_dict['t_tgt_'+str(tgt_num)] =  np.array(self.data_dict['t_tgt_'+str(tgt_num)])[not_selected_data_ind]
            self.data_dict['t_tgt_'+str(tgt_num)] = self.data_dict['t_tgt_'+str(tgt_num)].tolist()
            self.data_dict['t_abs_tgt_'+str(tgt_num)] = np.array(self.data_dict['t_abs_tgt_'+str(tgt_num)])[not_selected_data_ind]
            self.data_dict['t_abs_tgt_'+str(tgt_num)] = self.data_dict['t_abs_tgt_'+str(tgt_num)].tolist()
            # Show the remaining data
            self.plot_1_dict['tgt'+str(tgt_num)].\
                setData(self.data_dict['eye_raw_x_tgt_'+str(tgt_num)],self.data_dict['eye_raw_y_tgt_'+str(tgt_num)])
            self.plot_1_dict['active'].\
                setData(self.data_dict['eye_raw_x_tgt_'+str(tgt_num)],self.data_dict['eye_raw_y_tgt_'+str(tgt_num)])
            self.plot_2_dict['tgt_x'].\
                setData(self.data_dict['t_tgt_'+str(tgt_num)], self.data_dict['eye_raw_x_tgt_'+str(tgt_num)])
            self.plot_2_dict['tgt_y'].\
                setData(self.data_dict['t_tgt_'+str(tgt_num)],self.data_dict['eye_raw_y_tgt_'+str(tgt_num)])
            # Clear plots showing selected points
            self.plot_1_dict['tgt_'+str(tgt_num)+'_selected'].setData(np.zeros(0),np.zeros(0))
            self.plot_2_dict['tgt_selected_x'].setData(np.zeros(0),np.zeros(0))
            self.plot_2_dict['tgt_selected_y'].setData(np.zeros(0),np.zeros(0))
            # Clear data that store selected points
            self.data_dict['eye_raw_selected_tgt_'+str(tgt_num)] = np.zeros(0, dtype=bool)
            # Indicate no data are selected for this target
            self.currently_selected_tgt_ind[tgt_num-1] = False
            # Enable checkboxes
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'].setChecked(True)
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'].setEnabled(True) 
        # Disable computing calibration matrix, if number of targes whose data are selected less than the specified num.
        if np.sum(self.currently_selected_tgt_ind) < self.min_tgt_for_cal:
            self.cal_QPushButton.setEnabled(False)
            
    # Delete all collected data
    @pyqtSlot()
    def delete_all_QPushButton_clicked(self):
        for tgt_idx in range(self.num_cal_tgt):
            tgt_num = tgt_idx + 1
            self.reset_data_for_tgt(tgt_num)
        self.plot_1_dict['active'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_x'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_selected_x'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_y'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_selected_y'].setData(np.zeros(0),np.zeros(0))
        
        self.clear_ROI_QPushButton_clicked()
        self.cal_QPushButton.setEnabled(False)
        # Reinitialize checkboxes
        for tgt_idx in range(self.num_cal_tgt):
            tgt_num = tgt_idx + 1
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'].setEnabled(True)
        
    # Delete collected data for specific target
    @pyqtSlot()
    def delete_QPushButton_clicked(self):
        tgt_num = self.tgt_num_QComboBox.currentIndex()+1
        self.reset_data_for_tgt(tgt_num)
        self.plot_1_dict['active'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_x'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_selected_x'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_y'].setData(np.zeros(0),np.zeros(0))
        self.plot_2_dict['tgt_selected_y'].setData(np.zeros(0),np.zeros(0))
        self.currently_selected_tgt_ind[tgt_num-1] = False
        self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'].setEnabled(True)
        self.clear_ROI_QPushButton_clicked()
        # Disable computing calibration matrix, if number of targes whose data are selected less than some number
        if np.sum(self.currently_selected_tgt_ind) < self.min_tgt_for_cal:
            self.cal_QPushButton.setEnabled(False)
     
    # Highlight the data for the current target in Plot 1 and show them in Plot 2    
    @pyqtSlot()
    def tgt_num_QComboBox_currentTextChanged(self):
        tgt_num = self.tgt_num_QComboBox.currentText()       
        x_span_data = np.array(self.data_dict['eye_raw_x_tgt_'+str(tgt_num)])
        y_span_data = np.array(self.data_dict['eye_raw_y_tgt_'+str(tgt_num)])
        t_span_data = np.array(self.data_dict['t_tgt_'+str(tgt_num)])
        
        self.plot_1_dict['active'].setData(x_span_data,y_span_data)
        self.plot_2_dict['tgt_x'].setData(t_span_data,x_span_data)
        self.plot_2_dict['tgt_y'].setData(t_span_data,y_span_data)

        selected_data = self.data_dict['eye_raw_selected_tgt_'+str(tgt_num)]
        ################################### TEMPORARY FIX ######################################   
        x_span_data = x_span_data[:len(selected_data)]
        y_span_data = y_span_data[:len(selected_data)]
        t_span_data = t_span_data[:len(selected_data)]
        ################################### TEMPORARY FIX ######################################  
        self.plot_2_dict['tgt_selected_x'].setData(t_span_data[selected_data],x_span_data[selected_data])
        self.plot_2_dict['tgt_selected_y'].setData(t_span_data[selected_data],y_span_data[selected_data])
    
    # Clear calibration
    @pyqtSlot()
    def clear_cal_QPushButton_clicked(self):
        self.cal_parameter['cal_status'] = False
        self.cal_status_QLabel.setText("Uncalibrated")
        self.cal_status_QLabel.setStyleSheet('background-color: rgb(255,0,0)')
        
    @pyqtSlot(object)
    def receive_fsm_signal(self, signal):
        message = signal[0]
        if message == 'pump_1':
            self.pump_1.pump_once_QPushButton_clicked()     
        if message == 'pump_2':
            self.pump_2.pump_once_QPushButton_clicked()    
        if message == 'fsm_done':  
            self.sidepanel_parameter_QWidget.setEnabled(True)
            self.tgt.setEnabled(True)
            self.pd_tgt.setEnabled(True)
            self.toolbar_run_QAction.setEnabled(True)
            self.toolbar_stop_QAction.setDisabled(True)
            self.enable_manual_fncs()
            # Stop timer to stop getting data from fsm thread
            self.data_QTimer.stop()

    def plot_1_PlotWidget_mouseClicked(self, mouseEvent):
        if mouseEvent[0].button() == QtCore.Qt.LeftButton and self.toolbar_run_QAction.isEnabled():
            pos = mouseEvent[0].scenePos()
            if self.plot_1_PlotWidget.sceneBoundingRect().contains(pos):
                # Clear ROI in plot 2
                self.ROI_x_plot_2 = np.zeros(0)
                self.ROI_y_plot_2 = np.zeros(0)
                self.plot_2_ROI.setData(np.zeros(0), np.zeros(0))
                self.plot_2_ROI_firstToLast.setData(np.zeros(0), np.zeros(0))
                # Plot ROI in plot 1
                mousePoint = self.plot_1_PlotWidget_ViewBox.mapSceneToView(pos)
                self.ROI_x_plot_1 = np.append(self.ROI_x_plot_1,[mousePoint.x()])
                self.ROI_y_plot_1 = np.append(self.ROI_y_plot_1,[mousePoint.y()])
                self.plot_1_ROI.setData(self.ROI_x_plot_1, self.ROI_y_plot_1)
                if self.ROI_x_plot_1.size > 2:
                    self.plot_1_ROI_firstToLast.setData(self.ROI_x_plot_1[[0,-1],],self.ROI_y_plot_1[[0,-1],])
                    
    def plot_2_PlotWidget_mouseClicked(self,mouseEvent):
        if mouseEvent[0].button() == QtCore.Qt.LeftButton and self.toolbar_run_QAction.isEnabled():
            pos = mouseEvent[0].scenePos()
            if self.plot_1_PlotWidget.sceneBoundingRect().contains(pos):
                # Clear ROI in plot 1
                self.ROI_x_plot_1 = np.zeros(0)
                self.ROI_y_plot_1 = np.zeros(0)
                self.plot_1_ROI.setData(np.zeros(0), np.zeros(0))
                self.plot_1_ROI_firstToLast.setData(np.zeros(0), np.zeros(0))
                # Plot ROI in plot 2
                mousePoint = self.plot_2_PlotWidget_ViewBox.mapSceneToView(pos)
                self.ROI_x_plot_2 = np.append(self.ROI_x_plot_2,[mousePoint.x()])
                self.ROI_y_plot_2 = np.append(self.ROI_y_plot_2,[mousePoint.y()])
                self.plot_2_ROI.setData(self.ROI_x_plot_2, self.ROI_y_plot_2)
                if self.ROI_x_plot_2.size > 2:
                    self.plot_2_ROI_firstToLast.setData(self.ROI_x_plot_2[[0,-1],],self.ROI_y_plot_2[[0,-1],])
          
    @pyqtSlot()
    def tgt_num_QComboBox_up_QShortcut_activated(self):
        current_tgt_num = int(self.tgt_num_QComboBox.currentText())
        if current_tgt_num < self.num_cal_tgt:
            current_tgt_num += 1
        self.tgt_num_QComboBox.setCurrentIndex(current_tgt_num-1)
        if len(self.data_dict['t_tgt_'+str(current_tgt_num)]) > 0:
            self.plot_2_PlotWidget_ViewBox.setRange(xRange=(self.data_dict['t_tgt_'+str(current_tgt_num)][0],self.data_dict['t_tgt_'+str(current_tgt_num)][-1]))
        
    @pyqtSlot()
    def tgt_num_QComboBox_down_QShortcut_activated(self):
        current_tgt_num = int(self.tgt_num_QComboBox.currentText())
        if current_tgt_num > 1:
            current_tgt_num -= 1
        self.tgt_num_QComboBox.setCurrentIndex(current_tgt_num-1)
        if len(self.data_dict['t_tgt_'+str(current_tgt_num)]) > 0:
            self.plot_2_PlotWidget_ViewBox.setRange(xRange=(self.data_dict['t_tgt_'+str(current_tgt_num)][0],self.data_dict['t_tgt_'+str(current_tgt_num)][-1]))
        
    @pyqtSlot()
    def tgt_QCheckBox_QShortcut_activated(self):
        '''
        When called, check the state of the checkbox of the current target (as indicated in QComboBox)
        and toggle the state
        '''
        current_tgt_num = int(self.tgt_num_QComboBox.currentText())
        if self.tgt_widgets_dict['tgt_'+str(current_tgt_num)+'_QCheckBox'].isChecked():
            self.tgt_widgets_dict['tgt_'+str(current_tgt_num)+'_QCheckBox'].setChecked(False)
        else:
            self.tgt_widgets_dict['tgt_'+str(current_tgt_num)+'_QCheckBox'].setChecked(True)
                
            
    @pyqtSlot()
    def auto_tgt_QPushButton_clicked(self):
        '''
        Set the positions of targets around the center target, basing on 
        specified distance from the center target
        '''
        center_tgt_num = 5 # center target in 9-point target grid
        center_tgt_x = self.tgt_widgets_dict['tgt_'+str(center_tgt_num)+'_horz_QDoubleSpinBox'].value()
        center_tgt_y = self.tgt_widgets_dict['tgt_'+str(center_tgt_num)+'_vert_QDoubleSpinBox'].value()
        dist_from_center = self.auto_tgt_dist_QDoubleSpinBox.value()
        
        # Iterate thru multipliers and add the multiplied distance to the center 
        # and assign the position to relevant target
        tgt_num = 0
        for m_y in [1,0,-1]:
            for m_x in [-1,0,1]:
                tgt_num += 1
                self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_horz_QDoubleSpinBox'].setValue(\
                       center_tgt_x + m_x*dist_from_center)
                self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_vert_QDoubleSpinBox'].setValue(\
                       center_tgt_y + m_y*dist_from_center)
                    
    #%% GUI
    def init_gui(self):
        # Disable pumps
        self.pump_1.deleteLater()
        self.pump_2.deleteLater()
        # Customize plots
        self.plot_1_PlotWidget.setLabel('left', "Vertical position", units = 'device raw')
        self.plot_1_PlotWidget.setLabel('bottom', "Horizontal position", units = 'device raw')
        self.plot_1_dict = {} # init. of dict. that contains plots of data for 
                              # each calibration target and plots of selected data
        self.num_cal_tgt = 9
        color_list = [[128, 0, 0, 255],
                      [170, 110, 40, 255],
                      [128, 128, 0, 255],
                      [0, 0, 128, 255],
                      [0, 130, 200, 255],
                      [230, 25, 75, 255],
                      [0, 128, 128, 255],
                      [245, 130, 48, 255],
                      [255, 225, 25, 255]]
        for tgt_idx in range(self.num_cal_tgt):
            tgt_num = tgt_idx + 1
            self.plot_1_dict['tgt_'+str(tgt_num)] =\
                self.plot_1_PlotWidget.plot(np.zeros((0)), np.zeros((0)), pen = None,\
                symbolBrush=pg.mkBrush(color=color_list[tgt_idx],width=0.1),symbolPen=pg.mkPen(color='k', width=1.5))  
            self.plot_1_dict['tgt_'+str(tgt_num)+'_selected'] =\
                self.plot_1_PlotWidget.plot(np.zeros((0)), np.zeros((0)), pen=None,
                    symbol='o', symbolBrush=None, symbolPen=pg.mkPen('m', width=3) ) 
        self.plot_1_dict['active'] = self.plot_1_PlotWidget.\
            plot(np.zeros((0)), np.zeros((0)), pen = None,\
            symbolBrush=None,symbol='o',symbolPen=pg.mkPen(color=[210,245,60,255], width=2))    
        
        self.plot_2_dict = {} # init. of dict. that contains plots of data for 
                              # each calibration target and plots of selected data    
        self.plot_2_dict['tgt_x'] = self.plot_2_PlotWidget.plot(np.zeros((0)), np.zeros((0)), pen = pg.mkPen(color='r', width=0.8),\
                symbolBrush=pg.mkBrush(color='r',width=0.3), symbolPen=pg.mkPen(color='k', width=1),name='x')
        self.plot_2_dict['tgt_y'] = self.plot_2_PlotWidget.plot(np.zeros((0)), np.zeros((0)), pen = pg.mkPen(color='b', width=0.8),\
                symbolBrush=pg.mkBrush(color='b',width=0.3), symbolPen=pg.mkPen(color='k', width=1),name='y')  
        self.plot_2_dict['tgt_selected_x'] = self.plot_2_PlotWidget.plot(np.zeros((0)), np.zeros((0)), pen=None,\
                symbol='o', symbolBrush=None,symbolPen=pg.mkPen('m', width=2) ) 
        self.plot_2_dict['tgt_selected_y'] = self.plot_2_PlotWidget.plot(np.zeros((0)), np.zeros((0)), pen=None,\
                symbol='o', symbolBrush=None,symbolPen=pg.mkPen('m', width=2) ) 
        # Toolbar
        self.cal_status_QLabel = QLabel('Uncalibrated')
        self.cal_status_QLabel.setAlignment(Qt.AlignCenter)
        self.cal_status_QLabel.setStyleSheet('background-color: rgb(255,0,0)')
        self.toolbar.addWidget(self.cal_status_QLabel)
        
        self.cal_QPushButton = QPushButton('Calibrate')
        self.cal_QPushButton.setEnabled(False)
        self.cal_QPushButton.setToolTip('Calibrate using picked points (<b>CTRL+C</b>)')
        QShortcut(QKeySequence('Ctrl+C'), self.cal_QPushButton, self.cal_QPushButton.animateClick) 
        self.toolbar.addWidget(self.cal_QPushButton)
        
        self.select_ROI_QPushButton = QPushButton('Select')
        self.select_ROI_QPushButton.setToolTip('Select ROI (<b>S</b>)')
        QShortcut(Qt.Key_S, self.select_ROI_QPushButton, self.select_ROI_QPushButton.animateClick)
        self.toolbar.addWidget(self.select_ROI_QPushButton)
        
        self.deselect_QPushButton = QPushButton('Deselct')
        self.deselect_QPushButton.setToolTip('Deselct picked points (<b>D</b>)')
        QShortcut(Qt.Key_D, self.deselect_QPushButton, self.deselect_QPushButton.animateClick)
        self.toolbar.addWidget(self.deselect_QPushButton)
        
        self.clear_ROI_QPushButton = QPushButton('Clear')
        self.clear_ROI_QPushButton.setToolTip('Clear ROI (<b>C</b>)')
        QShortcut(Qt.Key_C, self.clear_ROI_QPushButton, self.clear_ROI_QPushButton.animateClick)
        self.toolbar.addWidget(self.clear_ROI_QPushButton)
        
        self.delete_selected_QPushButton = QPushButton('Delete selected')
        self.toolbar.addWidget(self.delete_selected_QPushButton)
        
        self.delete_all_QPushButton = QPushButton('Delete All')
        self.delete_all_QPushButton.setToolTip('Delete all collected data points')
        self.toolbar.addWidget(self.delete_all_QPushButton)
        
        self.delete_QPushButton = QPushButton('Delete')
        self.delete_QPushButton.setToolTip('Delete the collected data points for the current target')
        self.toolbar.addWidget(self.delete_QPushButton)
        
        self.tgt_num_QLabel = QLabel('Target #:')
        self.tgt_num_QLabel.setAlignment(Qt.AlignCenter|Qt.AlignRight)
        self.toolbar.addWidget(self.tgt_num_QLabel)
        self.tgt_num_QComboBox = QComboBox()
        for tgt_idx in range(self.num_cal_tgt):
            tgt_num = tgt_idx + 1
            self.tgt_num_QComboBox.addItem(str(tgt_num))
        self.toolbar.addWidget(self.tgt_num_QComboBox)
        
        self.clear_cal_QPushButton = QPushButton('Clear calibration')
        self.toolbar.addWidget(self.clear_cal_QPushButton)
                           
        self.save_QPushButton = QPushButton('Save')
        self.save_QPushButton.setToolTip('Save calibration and parameters (<b>CTRL+S</b>)')
        self.save_QPushButton.setDisabled(True) # initially disable as there is no new calibration
        QShortcut(QKeySequence('Ctrl+S'), self.save_QPushButton, self.save_QPushButton.animateClick) 
        self.toolbar.addWidget(self.save_QPushButton)
        
        # Side panel
        self.tgt_pos_QLabel = QLabel('<b>Target positions</b> (x,y) (deg)')
        self.tgt_pos_QLabel.setAlignment(Qt.AlignCenter)
        self.sidepanel_custom_QVBoxLayout.addWidget(self.tgt_pos_QLabel)
        
        self.tgt_widgets_dict = {}
        init_tgt_x_pos_list = [-1,0,1]
        init_tgt_y_pos_list = [-1,0,1]
        init_tgt_pos_list = []
        for y_idx in range(len(init_tgt_y_pos_list)):
            for x_idx in range(len(init_tgt_x_pos_list)):
                init_tgt_pos_list.append([init_tgt_x_pos_list[x_idx], init_tgt_x_pos_list[y_idx]])
        for tgt_idx in range(self.num_cal_tgt):
            tgt_num = tgt_idx + 1
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QLabel'] = QLabel(str(tgt_num)+'.')
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QLabel'].setAlignment(Qt.AlignCenter)
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'] = QCheckBox()
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'].setChecked(True)
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_horz_QDoubleSpinBox'] = QDoubleSpinBox()
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_horz_QDoubleSpinBox'].setRange(-20,20)
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_horz_QDoubleSpinBox'].setValue(init_tgt_pos_list[tgt_idx][0])
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_horz_QDoubleSpinBox'].setDecimals(1)
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_vert_QDoubleSpinBox'] = QDoubleSpinBox()
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_vert_QDoubleSpinBox'].setRange(-20,20)
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_vert_QDoubleSpinBox'].setValue(init_tgt_pos_list[tgt_idx][1])
            self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_vert_QDoubleSpinBox'].setDecimals(1)
        self.tgt_QGridLayout = QGridLayout()
        self.sidepanel_custom_QVBoxLayout.addLayout(self.tgt_QGridLayout)
        self.dict_tgt_QHBoxLayouts = {}
        for tgt_idx in range(self.num_cal_tgt):
            tgt_num = tgt_idx + 1
            self.dict_tgt_QHBoxLayouts[str(tgt_num)] = QHBoxLayout()
        # Target position grid
        for tgt_idx in range(self.num_cal_tgt):
            tgt_num = tgt_idx + 1
            self.dict_tgt_QHBoxLayouts[str(tgt_num)].addWidget\
                (self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QLabel'])
            self.dict_tgt_QHBoxLayouts[str(tgt_num)].addWidget\
                (self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_QCheckBox'])
            self.dict_tgt_QHBoxLayouts[str(tgt_num)].addWidget\
                (self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_horz_QDoubleSpinBox'])
            self.dict_tgt_QHBoxLayouts[str(tgt_num)].addWidget\
                (self.tgt_widgets_dict['tgt_'+str(tgt_num)+'_vert_QDoubleSpinBox'])
        tgt_num = 1
        for i in [1,2,3]:
            for j in [1,2,3]:
                self.tgt_QGridLayout.addLayout\
                    (self.dict_tgt_QHBoxLayouts[str(tgt_num)],i,j,1,1)
                tgt_num += 1  
        
        # Auto-populate target positions around the center target
        self.auto_tgt_QHBoxLayout = QHBoxLayout()
        self.sidepanel_custom_QVBoxLayout.addLayout(self.auto_tgt_QHBoxLayout)
        self.auto_tgt_QLabel = QLabel('Distance from center target:')
        self.auto_tgt_QLabel.setAlignment(Qt.AlignRight)
        self.auto_tgt_QHBoxLayout.addWidget(self.auto_tgt_QLabel)
        self.auto_tgt_dist_QDoubleSpinBox = QDoubleSpinBox()
        self.auto_tgt_dist_QDoubleSpinBox.setRange(0.1, 20)
        self.auto_tgt_dist_QDoubleSpinBox.setSingleStep(0.1)
        self.auto_tgt_dist_QDoubleSpinBox.setValue(3)
        self.auto_tgt_QHBoxLayout.addWidget(self.auto_tgt_dist_QDoubleSpinBox)
        self.auto_tgt_QPushButton = QPushButton('Auto-populate')
        self.auto_tgt_QHBoxLayout.addWidget(self.auto_tgt_QPushButton)
        
        # Inter-target interval
        self.tgt_ITI_QHBoxLayout = QHBoxLayout()
        self.sidepanel_custom_QVBoxLayout.addLayout(self.tgt_ITI_QHBoxLayout)
        self.tgt_ITI_QLabel = QLabel("Inter-target interval:")
        self.tgt_ITI_QLabel.setAlignment(Qt.AlignRight)
        self.tgt_ITI_QHBoxLayout.addWidget(self.tgt_ITI_QLabel)
        self.tgt_ITI_QDoubleSpinBox = QDoubleSpinBox()
        self.tgt_ITI_QDoubleSpinBox.setMaximum(10)
        self.tgt_ITI_QDoubleSpinBox.setMinimum(0) 
        self.tgt_ITI_QDoubleSpinBox.setValue(2)
        self.tgt_ITI_QHBoxLayout.addWidget(self.tgt_ITI_QDoubleSpinBox)
        
        # Calibration target duration
        self.cal_tgt_dur_QHBoxLayout = QHBoxLayout()
        self.sidepanel_custom_QVBoxLayout.addLayout(self.cal_tgt_dur_QHBoxLayout)
        self.cal_tgt_dur_QLabel = QLabel("Cal. target duration (s):")
        self.cal_tgt_dur_QLabel.setAlignment(Qt.AlignRight)
        self.cal_tgt_dur_QHBoxLayout.addWidget(self.cal_tgt_dur_QLabel)
        self.cal_tgt_dur_QDoubleSpinBox = QDoubleSpinBox()
        self.cal_tgt_dur_QDoubleSpinBox.setMaximum(10)
        self.cal_tgt_dur_QDoubleSpinBox.setMinimum(0.01) 
        self.cal_tgt_dur_QDoubleSpinBox.setValue(3)
        self.cal_tgt_dur_QHBoxLayout.addWidget(self.cal_tgt_dur_QDoubleSpinBox)
        
        # Pursuit target option
        self.pursuit_QHBoxLayout = QHBoxLayout()
        self.sidepanel_custom_QVBoxLayout.addLayout(self.pursuit_QHBoxLayout)
        self.pursuit_QLabel = QLabel("Pursuit target:")
        self.pursuit_QLabel.setAlignment(Qt.AlignRight)
        self.pursuit_QHBoxLayout.addWidget(self.pursuit_QLabel)
        self.pursuit_QCheckBox = QCheckBox()
        self.pursuit_QCheckBox.setChecked(True)
        self.pursuit_QHBoxLayout.addWidget(self.pursuit_QCheckBox)
        self.pursuit_amp_QDoubleSpinBox = QDoubleSpinBox()
        self.pursuit_amp_QDoubleSpinBox.setMaximum(10)
        self.pursuit_amp_QDoubleSpinBox.setMinimum(0)
        self.pursuit_QHBoxLayout.addWidget(self.pursuit_amp_QDoubleSpinBox)
        self.pursuit_amp_QLabel = QLabel("(amp)(deg):")
        self.pursuit_QHBoxLayout.addWidget(self.pursuit_amp_QLabel)
        self.pursuit_dur_QDoubleSpinBox = QDoubleSpinBox()
        self.pursuit_dur_QDoubleSpinBox.setMaximum(10)
        self.pursuit_dur_QDoubleSpinBox.setMinimum(0.01) 
        self.pursuit_QHBoxLayout.addWidget(self.pursuit_dur_QDoubleSpinBox)
        self.pursuit_dur_QLabel = QLabel("(t)(s):")
        self.pursuit_QHBoxLayout.addWidget(self.pursuit_dur_QLabel) 
    #%% FUNCTIONS
    def reset_data_for_tgt(self,tgt_num):
        '''
        Resetting data for specified target
        '''
        self.data_dict['eye_raw_x_tgt_'+str(tgt_num)] = []
        self.data_dict['eye_raw_y_tgt_'+str(tgt_num)] = []
        self.data_dict['t_tgt_'+str(tgt_num)] = []
        self.data_dict['t_abs_tgt_'+str(tgt_num)] = []
        self.data_dict['eye_raw_selected_tgt_'+str(tgt_num)] = np.zeros(0,dtype=bool)
        self.plot_1_dict['tgt_'+str(tgt_num)].setData(np.zeros(0),np.zeros(0))
        self.plot_1_dict['tgt_'+str(tgt_num)+'_selected'].setData(np.zeros(0),np.zeros(0))
    
    def disable_manual_fncs(self):
        self.cal_QPushButton.setDisabled(True)
        self.select_ROI_QPushButton.setDisabled(True)
        self.clear_ROI_QPushButton.setDisabled(True)
        self.deselect_QPushButton.setDisabled(True)
        self.delete_selected_QPushButton.setDisabled(True)
        self.delete_all_QPushButton.setDisabled(True)
        self.delete_QPushButton.setDisabled(True)
        self.clear_cal_QPushButton.setDisabled(True)
        self.save_QPushButton.setDisabled(True)
                
    def enable_manual_fncs(self):
        self.select_ROI_QPushButton.setEnabled(True)
        self.clear_ROI_QPushButton.setEnabled(True)
        self.deselect_QPushButton.setEnabled(True)
        self.delete_selected_QPushButton.setEnabled(True)
        self.delete_all_QPushButton.setEnabled(True)
        self.delete_QPushButton.setEnabled(True)
        self.clear_cal_QPushButton.setEnabled(True)
        self.save_QPushButton.setEnabled(True)
           
class Fsm_calProcess(multiprocessing.Process):
    def __init__(self, exp_name, fsm_to_gui_rcvr, gui_to_fsm_sndr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array,parent=None):
        super(Fsm_calProcess,self).__init__(parent)
        self.exp_name = exp_name
        self.fsm_to_gui_rcvr = fsm_to_gui_rcvr
        self.gui_to_fsm_sndr = gui_to_fsm_sndr
        self.stop_exp_Event = stop_exp_Event
        self.real_time_data_Array = real_time_data_Array
        self.stop_fsm_process_Event = stop_fsm_process_Event
    def run(self):  
        fsm_app = QApplication(sys.argv)
        fsm_app_gui = Fsm_calGui(self.exp_name, self.fsm_to_gui_rcvr, self.gui_to_fsm_sndr, self.stop_exp_Event, self.stop_fsm_process_Event, self.real_time_data_Array)
        fsm_app_gui.show()
        sys.exit(fsm_app.exec())
        
        