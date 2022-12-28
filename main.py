"""
Laboratory for Computational Motor Control, Johns Hopkins School of Medicine
@author: Jay Pi <jay.s.314159@gmail.com>
"""
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QToolBar, QAction
from psychopy import monitors, visual, core

from calibration.calibration import Fsm_calProcess
from calibration.refinement import Fsm_calRefineProcess
from experiment.simple_saccade import SimpleSacGuiProcess, SimpleSacFsmProcess, SimpleSacGui
from experiment.corr_saccade import CorrSacGuiProcess, CorrSacFsmProcess, CorrSacGui

from target import TargetWidget

import sys, multiprocessing, os, json, time, traceback
from pathlib import Path

class MainGui(QMainWindow):
    def __init__(self,main_to_screen_sndr, fsm_to_screen_sndr, parent = None):
        super(MainGui,self).__init__(parent)
        multiprocessing.set_start_method('spawn')
        
        self.main_to_screen_sndr = main_to_screen_sndr
        self.fsm_to_screen_sndr = fsm_to_screen_sndr
        
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
        
    #%% Signals
        self.simple_sac_QAction.triggered.connect(self.simple_sac_QAction_triggered)
        self.corr_sac_QAction.triggered.connect(self.corr_sac_QAction_triggered)
        self.cal_QAction.triggered.connect(self.cal_QAction_triggered)
        self.refine_cal_QAction.triggered.connect(self.refine_cal_QAction_triggered)
        
    #%% Slots
    def simple_sac_QAction_triggered(self):
        stop_exp_Event = multiprocessing.Event()
        stop_exp_Event.set()
        stop_fsm_process_Event = multiprocessing.Event()
        fsm_to_gui_rcvr, fsm_to_gui_sndr = multiprocessing.Pipe(duplex=False)
        gui_to_fsm_rcvr, gui_to_fsm_sndr = multiprocessing.Pipe(duplex=False)
        
        real_time_data_Array = multiprocessing.Array('d', range(5))
        exp_name = 'simple_saccade'
        fsm_process = SimpleSacFsmProcess(exp_name, SimpleSacGui.set_default_exp_parameter, self.fsm_to_screen_sndr, fsm_to_gui_sndr, gui_to_fsm_rcvr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array)
        gui_process = SimpleSacGuiProcess(exp_name,self.fsm_to_screen_sndr, fsm_to_gui_rcvr, gui_to_fsm_sndr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array)
                   
        fsm_process.start()
        time.sleep(0.25)
        gui_process.start()
        
    def corr_sac_QAction_triggered(self):
        stop_exp_Event = multiprocessing.Event()
        stop_exp_Event.set()
        stop_fsm_process_Event = multiprocessing.Event()
        fsm_to_gui_rcvr, fsm_to_gui_sndr = multiprocessing.Pipe(duplex=False)
        gui_to_fsm_rcvr, gui_to_fsm_sndr = multiprocessing.Pipe(duplex=False)
        
        real_time_data_Array = multiprocessing.Array('d', range(5))
        exp_name = 'corrective_saccade'
        fsm_process = CorrSacFsmProcess(exp_name, CorrSacGui.set_default_exp_parameter, self.fsm_to_screen_sndr, fsm_to_gui_sndr, gui_to_fsm_rcvr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array)
        gui_process = CorrSacGuiProcess(exp_name,self.fsm_to_screen_sndr, fsm_to_gui_rcvr, gui_to_fsm_sndr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array)
                   
        fsm_process.start()
        time.sleep(0.25)
        gui_process.start()
    
    def cal_QAction_triggered(self):
        stop_fsm_process_Event = multiprocessing.Event()
        cal_name = 'calibration'
        fsm_process = Fsm_calProcess(cal_name,self.fsm_to_screen_sndr, stop_fsm_process_Event)
        fsm_process.start()
    
    def refine_cal_QAction_triggered(self):    
        stop_fsm_process_Event = multiprocessing.Event()
        cal_name = 'refinement'
        fsm_process = Fsm_calRefineProcess(cal_name, self.fsm_to_screen_sndr, stop_fsm_process_Event)
        fsm_process.start()
       
if __name__ == '__main__':
    if sys.flags.interactive != 1 or not hasattr(QtCore, 'PYQT_VERSION'):
        main_to_screen_rcvr, main_to_screen_sndr = multiprocessing.Pipe(duplex=False)
        fsm_to_screen_rcvr, fsm_to_screen_sndr = multiprocessing.Pipe(duplex=False)
        
        main_app = QApplication(sys.argv)
        main_app_gui = MainGui(main_to_screen_sndr,fsm_to_screen_sndr)
        main_app_gui.show()
        sys.exit(main_app.exec())