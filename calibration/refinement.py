"""
Laboratory for Computational Motor Control, Johns Hopkins School of Medicine
@author: Jay Pi <jay.s.314159@gmail.com>
"""
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QComboBox, QPushButton, QLabel, QHBoxLayout, QDoubleSpinBox, QCheckBox, QPlainTextEdit,\
                            QDialog, QShortcut
from PyQt5.QtCore import QRunnable, QThreadPool, pyqtSignal, pyqtSlot, QObject, Qt, QTimer
from PyQt5.QtGui import QKeySequence
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

class Fsm_calRefineThreadSignals(QObject):
    to_main_thread = pyqtSignal(object)

class Fsm_calRefineThread(QRunnable):
    def __init__(self, fsm_to_screen_sndr):
        super().__init__()
        self.cal_matrix = []
        self.init_fsm_parameter()
        self.fsm_to_screen_sndr = fsm_to_screen_sndr
        self.signals = Fsm_calRefineThreadSignals()
        self.setAutoDelete(False) # if True, the object of this thread worker is deleted after finishing thread
    
        # Init var.
        self.eye_x = 0
        self.eye_y = 0
        self.tgt_x = 0
        self.tgt_y = 0
        self.t = math.nan
        self.cal_eye_pos = [] # list of eye positions to be used for calibration
                              # that correspond to target positions
        
    @pyqtSlot()
    def run(self):           
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
        
        num_tgt = np.array(self.fsm_parameter['tgt_pos']).shape[0]
        for counter_tgt in range(num_tgt):
            state = 'INIT'
            while self.fsm_parameter['run']:
                DPxUpdateRegCache() # need to call each loop to update buffer from device
                # Get time       
                self.t = DPxGetTime()
                # Get eye status (blinking)
                eye_status = DPxGetReg16(0x59A)
                left_eye_blink = bool(eye_status & (1 << 1)) # right blink, left blink
                if not left_eye_blink:
                    TPxBestPolyGetEyePosition(cal_data, raw_data)
                    raw_data_left = [raw_data[2], raw_data[3],1] # [left x, left y, right x, right y]
                    eye_pos = lib.raw_to_deg(raw_data_left,self.cal_matrix)
                    self.eye_x = eye_pos[0]
                    self.eye_y = eye_pos[1]
                else:
                    self.eye_x = 9999 # invalid values; more stable than nan values for plotting purposes in pyqtgraph
                    self.eye_y = 9999 
                # FSM
                if state == 'INIT':
                    tgt_pos = self.fsm_parameter['tgt_pos'][counter_tgt]      
                    state = 'CUE_TARGET_PRESENT'
                if state == 'CUE_TARGET_PRESENT':
                    lib.playSound(1000,0.1) # neutral beep
                    self.fsm_to_screen_sndr.send(('tgt','draw',tgt_pos))
                    self.tgt_x = tgt_pos[0]
                    self.tgt_y = tgt_pos[1]
                    state_start_time = self.t
                    state_inter_time = state_start_time
                    state = 'DETECT_SACCADE_END'
                if state == 'DETECT_SACCADE_END':
                    # If in manual mode, erase target after some time
                    if ((self.fsm_parameter['mode'] == 'Manual') and ((self.t-state_start_time) > self.fsm_parameter['tgt_dur'])):
                            self.fsm_to_screen_sndr.send(('all','rm'))
                            break
                    # Check for eye distance to target
                    if not left_eye_blink:
                        eye_dist_from_tgt = np.sqrt((tgt_pos[0]-eye_pos[0])**2 + (tgt_pos[1]-eye_pos[1])**2)
                                        
                        # Conditional statements below only apply to 'Auto' and 'Test' mode
                        # If not fixating at the target, reset the fixation duration timer                   
                        if eye_dist_from_tgt > self.fsm_parameter['rew_area']/2 or left_eye_blink:
                            state_inter_time = self.t 
                        # Min. fixation time req.    
                        if ((self.fsm_parameter['mode'] == 'Auto') or (self.fsm_parameter['mode'] == 'Test'))\
                            and ((self.t-state_inter_time) >= 0.1): 
                            # Erase target
                            self.fsm_to_screen_sndr.send(('all','rm'))
                            state = 'DELIVER_REWARD'
                if state == 'DELIVER_REWARD':
                    lib.playSound(2000,0.1) # reward beep
                    if self.fsm_parameter['auto_pump']:
                        self.signals.to_main_thread.emit(('pump',0))
                    if self.fsm_parameter['mode'] == 'Auto':
                        # Save data for bias computation
                        self.cal_eye_pos.append([self.eye_x,self.eye_y])
                    state_start_time = self.t
                    state = 'ITI'
                if state == 'ITI':
                    if (self.t-state_start_time) > 0.2:
                        break # move onto next target
        
        # If auto mode, compute bias. Use average bias for all targets 
        # Skip if user stopped.
        if self.fsm_parameter['run']: 
            if self.fsm_parameter['mode'] == 'Auto':       
                if self.fsm_parameter['auto_mode'] == 'Full':
                    # Concatenate 1s to target and eye arrays
                    tgt_pos = np.array(self.fsm_parameter['tgt_pos'])
                    eye_pos = np.array(self.cal_eye_pos)
                    tgt_pos_mat = np.hstack([tgt_pos,np.ones((tgt_pos.shape[0],1))])
                    eye_pos_mat = np.hstack([eye_pos,np.ones((eye_pos.shape[0],1))])
                    # Compute new calibration matrix that goes from the already calibrated eye data to target data
                    new_cal_matrix = np.linalg.inv(eye_pos_mat.T @ eye_pos_mat) @ eye_pos_mat.T @ tgt_pos_mat
                    updated_cal_matrix = np.array(self.cal_matrix) @ new_cal_matrix
                    eye_pos_pred = eye_pos_mat @ new_cal_matrix
                    eye_pos_pred = eye_pos_pred[:,[0,1]]
                    RMSE = np.sqrt(np.sum(np.square(tgt_pos-eye_pos_pred))/num_tgt)
                    new_cal_matrix = new_cal_matrix.tolist()
                    self.signals.to_main_thread.emit(('auto_mode_result',self.fsm_parameter['auto_mode'], [new_cal_matrix,RMSE]))
                elif self.fsm_parameter['auto_mode'] == 'Bias':
                    bias = np.empty((num_tgt,2))
                    for counter_tgt in range(num_tgt):
                        tgt_pos = np.array(self.fsm_parameter['tgt_pos'][counter_tgt])     
                        eye_pos = np.array(self.cal_eye_pos[counter_tgt])
                        print(tgt_pos)
                        print(eye_pos)
                        bias[counter_tgt] = tgt_pos - eye_pos
                    mean_bias = np.mean(bias, axis=0)
                    self.signals.to_main_thread.emit(('auto_mode_result',self.fsm_parameter['auto_mode'], [mean_bias]))
                    print(mean_bias)
        # Remove all targets
        self.fsm_to_screen_sndr.send(('all','rm'))
        
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
        # Init. var
        self.cal_eye_pos = []
    def init_fsm_parameter(self):
        self.fsm_parameter = {}
        self.fsm_parameter['run'] = False
        self.fsm_parameter['mode'] = 'Auto'
        self.fsm_parameter['auto_mode'] = 'Full'
        self.fsm_parameter['tgt_dur'] = 1
        self.fsm_parameter['tgt_pos'] = [[0,0]]
        self.fsm_parameter['rew_area'] = 2
        self.fsm_parameter['auto_pump'] = True
        
        
class Fsm_calRefineGui(FsmGui):
    def __init__(self,cal_name, fsm_to_screen_sndr, stop_fsm_process_Event):
        self.cal_name = cal_name
        self.fsm_to_screen_sndr = fsm_to_screen_sndr
        self.stop_fsm_process_Event = stop_fsm_process_Event
        super(Fsm_calRefineGui,self).__init__(self.fsm_to_screen_sndr, self.stop_fsm_process_Event)
        
        self.init_gui()

        # Init. var
        self.tgt_fsm_list = []
        self.tgt_auto_list = []
        self.cal_eye_x = 0 # values to be used for cal.
        self.cal_eye_y = 0
        self.cal_tgt_x = 0
        self.cal_tgt_y = 0
        self.new_cal = False # indicates whether new calibration done
        self.new_RMSE = 0.0        
        
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
        self.refine_parameter, self.parameter_file_path = lib.load_parameter('calibration','cal_parameter.json',True,lib.set_default_cal_parameter, self.cal_name)
        self.cal_parameter, self.parameter_file_path = lib.load_parameter('calibration','cal_parameter.json',True,lib.set_default_cal_parameter, 'calibration')
        self.old_cal_matrix = self.cal_parameter['cal_matrix']
        self.new_cal_matrix = copy.deepcopy(self.old_cal_matrix)
        self.tgt_pos_x_QDoubleSpinBox.setValue(self.refine_parameter['tgt_pos'][0])
        self.tgt_pos_y_QDoubleSpinBox.setValue(self.refine_parameter['tgt_pos'][1])
        self.tgt_dur_QDoubleSpinBox.setValue(self.refine_parameter['tgt_dur'])
        self.default_center_pos_x_QDoubleSpinBox.setValue(self.refine_parameter['center_tgt_pos'][0])
        self.default_center_pos_y_QDoubleSpinBox.setValue(self.refine_parameter['center_tgt_pos'][1])
        self.default_dist_QDoubleSpinBox.setValue(self.refine_parameter['dist_from_center'])
        self.default_rew_area_QDoubleSpinBox.setValue(self.refine_parameter['rew_area'])
        self.tgt_auto_list = self.refine_parameter['tgt_auto_list']
        self.auto_pump_QCheckBox.setChecked(self.refine_parameter['auto_pump'])
        if len(self.tgt_auto_list) > 0: 
            for x,y in self.tgt_auto_list:
                self.tgt_fsm_list_QPlainTextEdit.appendPlainText('(' + '{:.1f}'.format(x) + ',' + '{:.1f}'.format(y) + ')') # for display purpose only
        if not self.cal_parameter['cal_status']:
            self.toolbar_run_QAction.setDisabled(True)
            self.log_QPlainTextEdit.appendPlainText('No calibration found. Please calibrate first.')
        
        # Initialize fsm thread        
        self.fsm_thread = Fsm_calRefineThread(self.fsm_to_screen_sndr)
    #%% SIGNALS
        self.data_QTimer.timeout.connect(self.data_QTimer_timeout)
        self.fsm_thread.signals.to_main_thread.connect(self.receive_fsm_signal)
        # Plots
        self.plot_1_ROI_signal = pg.SignalProxy(self.plot_1_PlotWidget.scene().sigMouseClicked, rateLimit=60, slot=self.plot_1_PlotWidget_mouseClicked)
        self.plot_2_ROI_signal = pg.SignalProxy(self.plot_2_PlotWidget.scene().sigMouseClicked, rateLimit=60, slot=self.plot_2_PlotWidget_mouseClicked)
        # Toolbar
        self.toolbar_run_QAction.triggered.connect(self.toolbar_run_QAction_triggered)
        self.toolbar_stop_QAction.triggered.connect(self.toolbar_stop_QAction_triggered)
        self.cal_mode_QComboBox.currentTextChanged.connect(self.cal_mode_QComboBox_currentTextChanged)
        self.cal_auto_mode_QComboBox.currentTextChanged.connect(self.cal_auto_mode_QComboBox_currentTextChanged)
        self.clear_ROI_QPushButton.clicked.connect(self.clear_ROI_QPushButton_clicked)
        self.select_ROI_QPushButton.clicked.connect(self.select_ROI_QPushButton_clicked)
        self.deselect_QPushButton.clicked.connect(self.deselect_QPushButton_clicked)
        self.calibrate_QPushButton.clicked.connect(self.calibrate_QPushButton_clicked)
        self.reset_cal_QPushButton.clicked.connect(self.reset_cal_QPushButton_clicked)
        self.save_cal_QPushButton.clicked.connect(self.save_cal_QPushButton_clicked)
        # Sidepanel 
        self.tgt_pos_add_QPushButton.clicked.connect(self.tgt_pos_add_QPushButton_clicked)
        self.add_default_tgt_QPushButton.clicked.connect(self.add_default_tgt_QPushButton_clicked)
        self.reset_tgt_fsm_list_QPushButton.clicked.connect(self.reset_tgt_fsm_list_QPushButton_clicked)
    #%% SLOTS
    @pyqtSlot()
    def toolbar_run_QAction_triggered(self):
        # Save parameter
        self.refine_parameter['mode'] = self.cal_mode_QComboBox.currentText()
        self.refine_parameter['tgt_pos'] = [self.tgt_pos_x_QDoubleSpinBox.value(),self.tgt_pos_y_QDoubleSpinBox.value()]
        self.refine_parameter['tgt_dur'] = self.tgt_dur_QDoubleSpinBox.value()
        self.refine_parameter['center_tgt_pos'] = [self.default_center_pos_x_QDoubleSpinBox.value(),self.default_center_pos_y_QDoubleSpinBox.value()]
        self.refine_parameter['dist_from_center'] = self.default_dist_QDoubleSpinBox.value()
        self.refine_parameter['rew_area'] = self.default_rew_area_QDoubleSpinBox.value()
        self.refine_parameter['tgt_auto_list'] = self.tgt_auto_list
        self.refine_parameter['auto_pump'] = self.auto_pump_QCheckBox.isChecked()
        with open(self.parameter_file_path,'r') as file:
            all_parameter = json.load(file)
        all_parameter[self.cal_name] = self.refine_parameter  
        with open(self.parameter_file_path,'w') as file:
            json.dump(all_parameter, file, indent=4)   
        self.log_QPlainTextEdit.appendPlainText('Saved parameters')
        # If auto or test mode and no target specified, return
        cal_mode = self.cal_mode_QComboBox.currentText()
        if ((cal_mode == 'Auto') or (cal_mode == 'Test')) and (len(self.tgt_auto_list) == 0):
            self.log_QPlainTextEdit.appendPlainText('No target added.')
            return
        # Auto, full calibration mode needs more than 5 targets
        elif (cal_mode == 'Auto') and (self.cal_auto_mode_QComboBox.currentText() == 'Full') and (len(self.tgt_auto_list) < 5):
            self.log_QPlainTextEdit.appendPlainText('For full, auto calibration, need more than or equal to 5 targets')
            return
        else:
            self.tgt_fsm_list = self.tgt_auto_list
        self.toolbar_run_QAction.setDisabled(True)
        self.toolbar_stop_QAction.setEnabled(True)
        # Add target (manual mode)
        if cal_mode == 'Manual':
            self.tgt_fsm_list.append((self.tgt_pos_x_QDoubleSpinBox.value(),self.tgt_pos_y_QDoubleSpinBox.value()))
            self.cal_tgt_x = self.tgt_fsm_list[0][0]
            self.cal_tgt_y = self.tgt_fsm_list[0][1]
        # Set parameters
        self.fsm_thread.fsm_parameter['run'] = True
        self.fsm_thread.fsm_parameter['mode'] = self.cal_mode_QComboBox.currentText()
        self.fsm_thread.fsm_parameter['auto_mode'] = self.cal_auto_mode_QComboBox.currentText()
        self.fsm_thread.fsm_parameter['tgt_dur'] = self.tgt_dur_QDoubleSpinBox.value()
        self.fsm_thread.fsm_parameter['tgt_pos'] = self.tgt_fsm_list
        self.fsm_thread.fsm_parameter['rew_area'] = self.default_rew_area_QDoubleSpinBox.value()
        self.fsm_thread.fsm_parameter['auto_pump'] = self.auto_pump_QCheckBox.isChecked()
        self.fsm_thread.cal_matrix = self.new_cal_matrix
        # Reset target list (manual mode)
        if self.cal_mode_QComboBox.currentText() == 'Manual':
            self.tgt_fsm_list = []
        # Reset data
        self.eye_x_data.clear()
        self.eye_y_data.clear()
        self.tgt_x_data.clear()
        self.tgt_y_data.clear()
        self.t_data.clear()
        # Disable some user functions
        self.init_auto_mode()
        self.sidepanel_parameter_QWidget.setDisabled(True)
        self.tgt.setDisabled(True)
        self.pd_tgt.setDisabled(True)
        self.reset_cal_QPushButton.setDisabled(True)
        self.save_cal_QPushButton.setDisabled(True)
        # Clear ROI and selected points
        self.clear_ROI_QPushButton_clicked()
        self.plot_1_eye_selected.setData(np.zeros(0), np.zeros(0))
        self.plot_2_eye_x_selected.setData(np.zeros(0), np.zeros(0))
        self.plot_2_eye_y_selected.setData(np.zeros(0), np.zeros(0))
        # Start fsm thread
        self.thread_pool.start(self.fsm_thread)    
        # Start timer to get data from fsm thread
        self.data_QTimer.start(self.data_rate)
        
    @pyqtSlot()
    def toolbar_stop_QAction_triggered(self):
        self.toolbar_run_QAction.setEnabled(True)
        self.toolbar_stop_QAction.setDisabled(True)
        # Enable some user functions
        self.sidepanel_parameter_QWidget.setEnabled(True)
        self.tgt.setEnabled(True)
        self.pd_tgt.setEnabled(True)
        self.fsm_thread.fsm_parameter['run'] = False
        # Stop timer to stop getting data from fsm thread
        self.data_QTimer.stop()
        # Restore reset/save functions if new cal. available
        if self.new_cal == True:
            self.reset_cal_QPushButton.setEnabled(True)
            self.save_cal_QPushButton.setEnabled(True)
   
    @pyqtSlot()
    def cal_mode_QComboBox_currentTextChanged(self):
        self.init_mode_state()
        # Remove everything from plots
        self.erase_data_plots()
        self.clear_ROI_QPushButton_clicked()
        self.erase_selected_data_plots()
        # Disable Calibration
        self.calibrate_QPushButton.setDisabled(True) 
        
    @pyqtSlot()
    def cal_auto_mode_QComboBox_currentTextChanged(self):
        pass 
    @pyqtSlot()
    def select_ROI_QPushButton_clicked(self):
        x_span_data = np.array(self.eye_x_data)
        y_span_data = np.array(self.eye_y_data)
        t_span_data = np.array(self.t_data)
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
            # If any data selected,
            if np.any(data_selected):
                # Highlight selected data
                self.plot_1_eye_selected.setData(x_span_data[data_selected], y_span_data[data_selected])
                self.plot_2_eye_x_selected.setData(t_span_data[data_selected], x_span_data[data_selected])
                self.plot_2_eye_y_selected.setData(t_span_data[data_selected], y_span_data[data_selected])
                # Set values for calibration
                self.cal_eye_x = np.mean(x_span_data[data_selected])
                self.cal_eye_y = np.mean(y_span_data[data_selected])
                # Clear ROI
                self.clear_ROI_QPushButton_clicked()
                # Enable Calibration
                self.calibrate_QPushButton.setEnabled(True)  
    
    @pyqtSlot()
    def deselect_QPushButton_clicked(self):
        # Disable Calibration
        self.calibrate_QPushButton.setDisabled(True)  
        # Remove highlighted data
        self.erase_selected_data_plots()
    @pyqtSlot()
    def calibrate_QPushButton_clicked(self):        
        self.reset_cal_QPushButton.setEnabled(True)
        self.save_cal_QPushButton.setEnabled(True)
        self.new_cal = True
        bias_x = self.cal_tgt_x - self.cal_eye_x
        bias_y = self.cal_tgt_y - self.cal_eye_y
        self.new_cal_matrix[2][0] = self.new_cal_matrix[2][0] + bias_x
        self.new_cal_matrix[2][1] = self.new_cal_matrix[2][1] + bias_y
        self.log_QPlainTextEdit.appendPlainText('Bias change: '+'({:.1f}'.format(bias_x) + ',' + '{:.1f}'.format(bias_y) + ')')
        # Disable cal. button
        self.calibrate_QPushButton.setDisabled(True)
    @pyqtSlot()
    def reset_cal_QPushButton_clicked(self):
        # Disable reset/save button
        self.reset_cal_QPushButton.setDisabled(True)
        self.save_cal_QPushButton.setDisabled(True)
        self.new_cal = False
        # Revert to old matrix
        self.new_cal_matrix = copy.deepcopy(self.old_cal_matrix)
        self.log_QPlainTextEdit.appendPlainText('Reverted to original calibration matrix')
    @pyqtSlot()
    def save_cal_QPushButton_clicked(self):
        # Disable reset/save button
        self.reset_cal_QPushButton.setDisabled(True)
        self.save_cal_QPushButton.setDisabled(True)
        self.new_cal = False
        # Update 'old' cal matrix
        self.old_cal_matrix = copy.deepcopy(self.new_cal_matrix)
        self.cal_parameter['cal_matrix'] = self.old_cal_matrix
        self.cal_parameter['RMSE'] = self.new_RMSE
        
        with open(self.parameter_file_path,'r') as file:
            all_parameter = json.load(file)
        all_parameter[self.cal_name] = self.refine_parameter
        all_parameter['calibration'] = self.cal_parameter  
        with open(self.parameter_file_path,'w') as file:
            json.dump(all_parameter, file, indent=4)   
            
        self.log_QPlainTextEdit.appendPlainText('Saved calibration and parameters')
    @pyqtSlot()
    def data_QTimer_timeout(self):
        '''
        start getting data from fsm thread
        '''
        if not math.isnan(self.fsm_thread.t):
            self.eye_x_data.append(self.fsm_thread.eye_x)
            self.eye_y_data.append(self.fsm_thread.eye_y)
            self.tgt_x_data.append(self.fsm_thread.tgt_x)
            self.tgt_y_data.append(self.fsm_thread.tgt_y)
            self.t_data.append(self.fsm_thread.t)
            # Plot data
            self.plot_1_eye.setData(self.eye_x_data,self.eye_y_data)
            self.plot_1_tgt.setData([self.fsm_thread.tgt_x],[self.fsm_thread.tgt_y])
            self.plot_2_eye_x.setData(self.t_data,self.eye_x_data)
            self.plot_2_eye_y.setData(self.t_data,self.eye_y_data)
            self.plot_2_tgt_x.setData(self.t_data,self.tgt_x_data)
            self.plot_2_tgt_y.setData(self.t_data,self.tgt_y_data)
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
        
    @pyqtSlot(object)
    def receive_fsm_signal(self, signal):
        message = signal[0]
        if message == 'pump':
            self.pump_1.pump_once_QPushButton_clicked()       
        if message == 'fsm_done':  
            self.sidepanel_parameter_QWidget.setEnabled(True)
            self.tgt.setEnabled(True)
            self.pd_tgt.setEnabled(True)
            self.toolbar_run_QAction.setEnabled(True)
            self.toolbar_stop_QAction.setDisabled(True)
            self.init_mode_state()
            # Stop timer to stop getting data from fsm thread
            self.data_QTimer.stop()
        if message == 'auto_mode_result':
            if signal[1] == 'Full':
                self.reset_cal_QPushButton.setEnabled(True)
                self.save_cal_QPushButton.setEnabled(True)
                self.new_cal = True
                self.new_cal_matrix = signal[2][0]
                self.new_RMSE = signal[2][1]
                self.log_QPlainTextEdit.appendPlainText('Old RMSE: ' + '{:.2f}'.format(self.cal_parameter['RMSE']) + ' New RMSE: '+ '{:.2f}'.format(self.new_RMSE)) 
            elif signal[1] == 'Bias':
                self.reset_cal_QPushButton.setEnabled(True)
                self.save_cal_QPushButton.setEnabled(True)
                self.new_cal = True
                # Update new calibration
                bias = signal[2][0] 
                self.new_cal_matrix[2][0] = self.new_cal_matrix[2][0] + bias[0]
                self.new_cal_matrix[2][1] = self.new_cal_matrix[2][1] + bias[1]
                self.log_QPlainTextEdit.appendPlainText('Auto bias change: '+'({:.2f}'.format(bias[0]) + ',' + '{:.2f}'.format(bias[1]) + ')')
            
    def plot_1_PlotWidget_mouseClicked(self, mouseEvent):
        if mouseEvent[0].button() == QtCore.Qt.LeftButton and self.cal_mode_QComboBox.currentText() == 'Manual'\
            and self.toolbar_run_QAction.isEnabled():
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
        if mouseEvent[0].button() == QtCore.Qt.LeftButton and self.cal_mode_QComboBox.currentText() == 'Manual'\
            and self.toolbar_run_QAction.isEnabled():
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
    def tgt_pos_add_QPushButton_clicked(self):
        if not self.cal_mode_QComboBox.currentText() == 'Manual':
            tgt_x = self.tgt_pos_x_QDoubleSpinBox.value()
            tgt_y = self.tgt_pos_y_QDoubleSpinBox.value()
            self.add_tgt_to_list(tgt_x,tgt_y)
    @pyqtSlot()
    def add_default_tgt_QPushButton_clicked(self):
        '''
        add 9 targets in a grid 
        '''
        c_x = self.default_center_pos_x_QDoubleSpinBox.value()
        c_y = self.default_center_pos_y_QDoubleSpinBox.value()
        a = self.default_dist_QDoubleSpinBox.value()
        i_list = [1,0,-1] # multiplier list
        j_list = [1,0,-1]
        random.shuffle(i_list)
        random.shuffle(j_list)
        for i in i_list:
            for j in j_list:
                self.add_tgt_to_list(c_x + a*i, c_y + a*j)
                
    @pyqtSlot()
    def reset_tgt_fsm_list_QPushButton_clicked(self):
        self.tgt_auto_list = []
        self.tgt_fsm_list_QPlainTextEdit.clear()
    #%% GUI
    def init_gui(self):
        # Customize plots
        self.plot_1_eye = self.plot_1_PlotWidget.\
            plot(np.zeros((0)), np.zeros((0)), pen = None,\
            symbolBrush='k', symbolPen='k', symbol='o',symbolSize=10,name='eye',connect='finite') 
        self.plot_1_eye.setSymbolSize(5)
        self.plot_1_tgt = self.plot_1_PlotWidget.\
            plot(np.zeros((0)), np.zeros((0)), pen = None,\
            symbolBrush=None, symbolPen='b', symbol='+',symbolSize=14,name='tgt')
        self.plot_1_eye_selected = self.plot_1_PlotWidget.plot(np.zeros((0)), np.zeros((0)), pen=None,\
                    symbol='o', symbolBrush=None,symbolPen=pg.mkPen('m', width=3))
        self.plot_2_eye_x_selected = self.plot_2_PlotWidget.plot(np.zeros((0)), np.zeros((0)), pen=None,\
                    symbol='o', symbolBrush=None,symbolPen=pg.mkPen('m', width=3))    
        self.plot_2_eye_y_selected = self.plot_2_PlotWidget.plot(np.zeros((0)), np.zeros((0)), pen=None,\
                    symbol='o', symbolBrush=None,symbolPen=pg.mkPen('m', width=3))  
        # Toolbar
        self.cal_mode_QComboBox = QComboBox()
        self.cal_mode_QComboBox.addItem('Auto')
        self.cal_mode_QComboBox.addItem('Manual')
        self.cal_mode_QComboBox.addItem('Test')
        self.toolbar.addWidget(self.cal_mode_QComboBox)
        
        self.cal_auto_mode_QComboBox = QComboBox()
        self.cal_auto_mode_QComboBox.addItem('Full')
        self.cal_auto_mode_QComboBox.addItem('Bias')
        self.toolbar.addWidget(self.cal_auto_mode_QComboBox)
        
        self.select_ROI_QPushButton = QPushButton('Select')
        self.select_ROI_QPushButton.setToolTip('Select ROI (<b>S</b>)')
        QShortcut(Qt.Key_S, self.select_ROI_QPushButton, self.select_ROI_QPushButton.animateClick)
        self.toolbar.addWidget(self.select_ROI_QPushButton)
        
        self.clear_ROI_QPushButton = QPushButton('Clear')
        self.clear_ROI_QPushButton.setToolTip('Clear ROI (<b>C</b>)')
        QShortcut(Qt.Key_C, self.clear_ROI_QPushButton, self.clear_ROI_QPushButton.animateClick)
        self.toolbar.addWidget(self.clear_ROI_QPushButton)
        
        self.deselect_QPushButton = QPushButton('Deselct')
        self.deselect_QPushButton.setToolTip('Deselct picked points (<b>D</b>)')
        QShortcut(Qt.Key_D, self.deselect_QPushButton, self.deselect_QPushButton.animateClick)
        self.toolbar.addWidget(self.deselect_QPushButton)
        
        self.calibrate_QPushButton = QPushButton('Calibrate')
        self.calibrate_QPushButton.setToolTip('Calibrate using picked points (<b>CTRL+C</b>)')
        self.calibrate_QPushButton.setDisabled(True)
        QShortcut(QKeySequence('Ctrl+C'), self.calibrate_QPushButton, self.calibrate_QPushButton.animateClick) 
        self.toolbar.addWidget(self.calibrate_QPushButton)
        
        self.reset_cal_QPushButton = QPushButton('Reset')
        self.reset_cal_QPushButton.setToolTip('Go back to original calibration (<b>R</b>)')
        self.reset_cal_QPushButton.setDisabled(True) # initially disable as there is no new calibration
        QShortcut(Qt.Key_R, self.reset_cal_QPushButton, self.reset_cal_QPushButton.animateClick)
        self.toolbar.addWidget(self.reset_cal_QPushButton)
        
        self.save_cal_QPushButton = QPushButton('Save')
        self.save_cal_QPushButton.setToolTip('Save calibration (<b>CTRL+S</b>)')
        self.save_cal_QPushButton.setDisabled(True) # initially disable as there is no new calibration
        QShortcut(QKeySequence('Ctrl+S'), self.save_cal_QPushButton, self.save_cal_QPushButton.animateClick) 
        self.toolbar.addWidget(self.save_cal_QPushButton)
        
        # Side panel
        self.tgt_pos_QHBoxLayout = QHBoxLayout()
        self.tgt_pos_QLabel = QLabel('Target Pos. (x,y) (deg): ')
        self.tgt_pos_QHBoxLayout.addWidget(self.tgt_pos_QLabel)
        self.tgt_pos_x_QDoubleSpinBox = QDoubleSpinBox()
        self.tgt_pos_x_QDoubleSpinBox.setValue(0)
        self.tgt_pos_x_QDoubleSpinBox.setMinimum(-50)
        self.tgt_pos_x_QDoubleSpinBox.setMaximum(50)
        self.tgt_pos_x_QDoubleSpinBox.setDecimals(1)
        self.tgt_pos_x_QDoubleSpinBox.setSingleStep(0.1)
        self.tgt_pos_QHBoxLayout.addWidget(self.tgt_pos_x_QDoubleSpinBox)
        self.tgt_pos_y_QDoubleSpinBox = QDoubleSpinBox()
        self.tgt_pos_y_QDoubleSpinBox.setValue(0)
        self.tgt_pos_y_QDoubleSpinBox.setMinimum(-50)
        self.tgt_pos_x_QDoubleSpinBox.setMaximum(50)
        self.tgt_pos_y_QDoubleSpinBox.setDecimals(1)
        self.tgt_pos_y_QDoubleSpinBox.setSingleStep(0.1)
        self.tgt_pos_QHBoxLayout.addWidget(self.tgt_pos_y_QDoubleSpinBox)
        self.tgt_pos_add_QPushButton = QPushButton('Add')
        self.tgt_pos_QHBoxLayout.addWidget(self.tgt_pos_add_QPushButton)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.tgt_pos_QHBoxLayout)
        
        self.tgt_dur_QHBoxLayout = QHBoxLayout()
        self.tgt_dur_QLabel = QLabel('Target Duration (s)')
        self.tgt_dur_QLabel.setToolTip('Applicable only in "manual" mode')
        self.tgt_dur_QHBoxLayout.addWidget(self.tgt_dur_QLabel)
        self.tgt_dur_QDoubleSpinBox = QDoubleSpinBox()
        self.tgt_dur_QDoubleSpinBox.setValue(1)
        self.tgt_dur_QDoubleSpinBox.setDecimals(1)
        self.tgt_dur_QDoubleSpinBox.setSingleStep(0.25)
        self.tgt_dur_QHBoxLayout.addWidget(self.tgt_dur_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.tgt_dur_QHBoxLayout)
        
        self.default_tgt_QLabel = QLabel('<b>Default Auto Targets</b>')
        self.default_tgt_QLabel.setAlignment(Qt.AlignCenter)
        self.sidepanel_custom_QVBoxLayout.addWidget(self.default_tgt_QLabel)
        
        self.default_center_pos_QHBoxLayout = QHBoxLayout()
        self.default_center_pos_QHBoxLayout.addWidget(QLabel('Center Pos (x,y) (deg): '))
        self.default_center_pos_x_QDoubleSpinBox = QDoubleSpinBox()
        self.default_center_pos_x_QDoubleSpinBox.setValue(0)
        self.default_center_pos_x_QDoubleSpinBox.setMinimum(-50)
        self.default_center_pos_x_QDoubleSpinBox.setMaximum(50)
        self.default_center_pos_x_QDoubleSpinBox.setDecimals(1)
        self.default_center_pos_x_QDoubleSpinBox.setSingleStep(0.1)
        self.default_center_pos_QHBoxLayout.addWidget(self.default_center_pos_x_QDoubleSpinBox)
        self.default_center_pos_y_QDoubleSpinBox = QDoubleSpinBox()
        self.default_center_pos_y_QDoubleSpinBox.setValue(0)
        self.default_center_pos_y_QDoubleSpinBox.setMinimum(-50)
        self.default_center_pos_y_QDoubleSpinBox.setMaximum(50)
        self.default_center_pos_y_QDoubleSpinBox.setDecimals(1)
        self.default_center_pos_y_QDoubleSpinBox.setSingleStep(0.1)
        self.default_center_pos_QHBoxLayout.addWidget(self.default_center_pos_y_QDoubleSpinBox)        
        self.sidepanel_custom_QVBoxLayout.addLayout(self.default_center_pos_QHBoxLayout)
        
        self.default_dist_QHBoxLayout = QHBoxLayout()
        self.default_dist_QHBoxLayout.addWidget(QLabel('Distance from Center (deg): '))
        self.default_dist_QDoubleSpinBox = QDoubleSpinBox()
        self.default_dist_QDoubleSpinBox.setValue(3)
        self.default_dist_QDoubleSpinBox.setDecimals(1)
        self.default_dist_QDoubleSpinBox.setSingleStep(0.1)
        self.default_dist_QHBoxLayout.addWidget(self.default_dist_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.default_dist_QHBoxLayout)
        
        self.default_rew_area_QHBoxLayout = QHBoxLayout()
        self.default_rew_area_QHBoxLayout.addWidget(QLabel('Reward Area (deg): '))
        self.default_rew_area_QDoubleSpinBox = QDoubleSpinBox()
        self.default_rew_area_QDoubleSpinBox.setValue(2)
        self.default_rew_area_QDoubleSpinBox.setDecimals(1)
        self.default_rew_area_QDoubleSpinBox.setSingleStep(0.1)
        self.default_rew_area_QHBoxLayout.addWidget(self.default_rew_area_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.default_rew_area_QHBoxLayout)
        
        self.add_default_tgt_QPushButton = QPushButton('Add Default Targets')
        self.add_default_tgt_QPushButton.setToolTip('Adding 9 target grid\naround center position')
        self.sidepanel_custom_QVBoxLayout.addWidget(self.add_default_tgt_QPushButton)
        
        self.auto_pump_QHBoxLayout = QHBoxLayout()
        self.auto_pump_QLabel = QLabel('Auto Pump?')
        self.auto_pump_QLabel.setToolTip('Checking this will pump food\nwhenever an animal completes a target')
        self.auto_pump_QHBoxLayout.addWidget(self.auto_pump_QLabel)
        self.auto_pump_QCheckBox = QCheckBox()
        self.auto_pump_QHBoxLayout.addWidget(self.auto_pump_QCheckBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.auto_pump_QHBoxLayout)
        
        self.tgt_fsm_list_QLabel = QLabel('<b>Target List</b>')
        self.tgt_fsm_list_QLabel.setAlignment(Qt.AlignCenter)
        self.sidepanel_custom_QVBoxLayout.addWidget(self.tgt_fsm_list_QLabel)
        self.tgt_fsm_list_QPlainTextEdit = QPlainTextEdit()
        self.tgt_fsm_list_QPlainTextEdit.setReadOnly(True)
        self.sidepanel_custom_QVBoxLayout.addWidget(self.tgt_fsm_list_QPlainTextEdit)
        self.reset_tgt_fsm_list_QPushButton = QPushButton('Reset Target List')
        self.sidepanel_custom_QVBoxLayout.addWidget(self.reset_tgt_fsm_list_QPushButton)
        
        # Initialize some functions
        if self.cal_mode_QComboBox.currentText() == 'Manual':
            self.init_manual_mode()
        else:
            self.init_auto_mode()
    #%% FUNCTIONS
    def add_tgt_to_list(self,x,y):
        self.tgt_fsm_list_QPlainTextEdit.appendPlainText('(' + '{:.1f}'.format(x) + ',' + '{:.1f}'.format(y) + ')') # for display purpose only
        self.tgt_auto_list.append((x,y))
    def init_auto_mode(self):
        self.cal_auto_mode_QComboBox.setEnabled(True)
        self.select_ROI_QPushButton.setDisabled(True)
        self.clear_ROI_QPushButton.setDisabled(True)
        self.deselect_QPushButton.setDisabled(True)
        self.tgt_pos_add_QPushButton.setEnabled(True)
        self.tgt_dur_QDoubleSpinBox.setEnabled(False)
        self.default_center_pos_x_QDoubleSpinBox.setEnabled(True)
        self.default_center_pos_y_QDoubleSpinBox.setEnabled(True)
        self.default_dist_QDoubleSpinBox.setEnabled(True)
        self.default_rew_area_QDoubleSpinBox.setEnabled(True)
        self.add_default_tgt_QPushButton.setEnabled(True)
        self.auto_pump_QCheckBox.setEnabled(True)
        self.reset_tgt_fsm_list_QPushButton.setEnabled(True)
        
    def init_manual_mode(self):
        self.cal_auto_mode_QComboBox.setEnabled(False)
        self.select_ROI_QPushButton.setEnabled(True)
        self.clear_ROI_QPushButton.setEnabled(True)
        self.deselect_QPushButton.setEnabled(True)
        self.tgt_pos_add_QPushButton.setEnabled(False)
        self.tgt_dur_QDoubleSpinBox.setEnabled(True)
        self.default_center_pos_x_QDoubleSpinBox.setEnabled(False)
        self.default_center_pos_y_QDoubleSpinBox.setEnabled(False)
        self.default_dist_QDoubleSpinBox.setEnabled(False)
        self.default_rew_area_QDoubleSpinBox.setEnabled(False)
        self.add_default_tgt_QPushButton.setEnabled(False)
        self.auto_pump_QCheckBox.setEnabled(False)
        self.reset_tgt_fsm_list_QPushButton.setEnabled(False)
    def init_mode_state(self):
        # Initialize some functions depending on mode state
        if self.cal_mode_QComboBox.currentText() == 'Manual':
            self.init_manual_mode()
            # Clear target list log
            self.tgt_fsm_list_QPlainTextEdit.clear()
            
        else:
            self.init_auto_mode()
            
    def erase_data_plots(self):
        self.plot_1_eye.setData(np.zeros(0),np.zeros(0))
        self.plot_1_tgt.setData(np.zeros(0),np.zeros(0))
        self.plot_2_eye_x.setData(np.zeros(0),np.zeros(0))
        self.plot_2_eye_y.setData(np.zeros(0),np.zeros(0))
        self.plot_2_tgt_x.setData(np.zeros(0),np.zeros(0))
        self.plot_2_tgt_y.setData(np.zeros(0),np.zeros(0))
    def erase_selected_data_plots(self):
        self.plot_1_eye_selected.setData(np.zeros(0),np.zeros(0))
        self.plot_2_eye_x_selected.setData(np.zeros(0), np.zeros(0))
        self.plot_2_eye_y_selected.setData(np.zeros(0), np.zeros(0))
        
class Fsm_calRefineProcess(multiprocessing.Process):
    def __init__(self, cal_name, fsm_to_screen_sndr, stop_fsm_process_Event, parent=None):
        super(Fsm_calRefineProcess,self).__init__(parent)
        self.cal_name = cal_name
        self.fsm_to_screen_sndr = fsm_to_screen_sndr
        self.stop_fsm_process_Event = stop_fsm_process_Event
    def run(self):  
        fsm_app = QApplication(sys.argv)
        fsm_app_gui = Fsm_calRefineGui(self.cal_name, self.fsm_to_screen_sndr, self.stop_fsm_process_Event)
        fsm_app_gui.show()
        sys.exit(fsm_app.exec())
        
        