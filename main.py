"""
Laboratory for Computational Motor Control, Johns Hopkins School of Medicine
@author: Jay Pi <jay.s.314159@gmail.com>
"""
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QToolBar, QAction
from psychopy import monitors, visual, core

from calibration.bias import Fsm_calBiasProcess
from experiment.simple_saccade import SimpleSacGuiProcess, SimpleSacFsmProcess, SimpleSacGui
from experiment.corr_saccade import CorrSacGuiProcess, CorrSacFsmProcess, CorrSacGui

from target import TargetWidget

import sys, multiprocessing, os, json, time, traceback
from pathlib import Path

class MainGui(QMainWindow):
    def __init__(self,main_to_screen_sndr, fsm_to_screen_sndr, parent = None):
        super(MainGui,self).__init__(parent)
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
        self.bias_cal_QAction = QAction('Bias',self)
        self.menu_cal.addAction(self.bias_cal_QAction)
        
    #%% Signals
        self.simple_sac_QAction.triggered.connect(self.simple_sac_QAction_triggered)
        self.corr_sac_QAction.triggered.connect(self.corr_sac_QAction_triggered)
        self.bias_cal_QAction.triggered.connect(self.bias_cal_QAction_triggered)
        
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
    
    
    def bias_cal_QAction_triggered(self):
              
        stop_fsm_process_Event = multiprocessing.Event()
        fsm_process = Fsm_calBiasProcess(self.fsm_to_screen_sndr, stop_fsm_process_Event)
        fsm_process.start()
    
        
    #%% FUNCTIONS
    def closeEvent(self, event):
        self.main_to_screen_sndr.send(('quit',0))
        time.sleep(0.1)
        ######
        # # Wait for signal for successful exit 
        # while True:
        #     if (not self.main_to_screen_Q.empty()) and (self.main_to_screen_Q.get() == 1):
        #         time.sleep(0.1)
        #         break
        ######
class MainProcess(multiprocessing.Process):
    def __init__(self, main_to_screen_sndr, fsm_to_screen_sndr, parent=None):
        super(MainProcess,self).__init__(parent)
        self.main_to_screen_sndr = main_to_screen_sndr
        self.fsm_to_screen_sndr = fsm_to_screen_sndr
    def run(self):
        main_app = QApplication(sys.argv)
        main_app_gui = MainGui(self.main_to_screen_sndr, self.fsm_to_screen_sndr)
        main_app_gui.show()
        sys.exit(main_app.exec())
        
    def build_gui(self):
        self.main_QMainWindow = QMainWindow()
        
        # Build toolbar
        self.toolbar = QToolBar()
        self.toolbar.setIconSize(QtCore.QSize(30,30))
        self.main_QMainWindow.addToolBar(self.toolbar)  
        
class ScreenProcess(multiprocessing.Process):
    def __init__(self, main_to_screen_rcvr, fsm_to_screen_rcvr, parent=None):
        super(ScreenProcess,self).__init__(parent)
        self.main_to_screen_rcvr = main_to_screen_rcvr
        self.fsm_to_screen_rcvr = fsm_to_screen_rcvr
    def init_monitor(self):
        '''
        load PsychoPy monitor settings from .json file and start the screen.
        Units of distances are in cm.

        '''
        file_path = os.path.join(str(Path().absolute()), 'monitor_setting.json')
        with open(file_path,'r') as file:
            setting = json.load(file)
        self.refresh_rate = setting['monitor_refresh_rate']
        this_monitor = monitors.Monitor(setting['monitor_name'], width=setting['monitor_width'], distance=setting['monitor_distance'])
        this_monitor.setSizePix(setting['monitor_size'])
        self.window = visual.Window(size=setting['monitor_size'],screen=setting['monitor_num'], allowGUI=False, color='white', monitor=this_monitor,
                                    units='deg', winType='glfw', fullscr=False, checkTiming=False, waitBlanking=False)
        self.window.flip()
        # Make targets
        self.tgt = {}
        self.tgt['tgt'] = visual.Rect(win=self.window, width=2,height=2, units='deg', lineColor='black',fillColor='black',lineWidth=1.5)
        self.tgt['tgt'].pos = (0,0)
        self.tgt['tgt'].draw() # draw once already, because the first draw may be slower - Poth, 2018                 
        self.tgt['pd_tgt'] = visual.Rect(win=self.window, width=6,height=6, units='deg',lineColor='black',fillColor='black',lineWidth=1.5)
        self.tgt['pd_tgt'].pos = (26,-15)
        self.tgt['pd_tgt'].draw()
        self.window.clearBuffer() # clear the back buffer of previously drawn stimuli - Poth, 2018
    def run(self):
        self.init_monitor()
        # loop_period = 1000/self.refresh_rate # in ms
        loop_period = 0.05
        t = time.perf_counter_ns()/(10**6)
        while True:
            if time.perf_counter_ns()/(10**6) - t >= loop_period:
                t = time.perf_counter_ns()/(10**6)
                if self.main_to_screen_rcvr.poll():
                    message = self.main_to_screen_rcvr.recv()
                    message_title = message[0]
                    
                    if message_title == 'quit':
                        # self.main_to_screen_Q.put(1)
                        core.quit()                                     
                        self.window.close()
                        time.sleep(0.2)   
                        
                        break
                # Screen command    
                if self.fsm_to_screen_rcvr.poll():
                    message = self.fsm_to_screen_rcvr.recv()
                    tgt_name = message[0]
                    tgt_command = message[1]
                                  
                    # Draw targets
                    if tgt_command == 'draw':
                        tgt_pos = message[2]
                        if tgt_name == 'all':
                            for key in self.tgt:
                                if key != 'pd_tgt':
                                    self.tgt[key].pos = tgt_pos
                                self.tgt[key].draw()         
                        else:
                            if tgt_name != 'pd_tgt':
                                self.tgt[tgt_name].pos = tgt_pos
                            self.tgt[tgt_name].draw()
                        self.window.flip()
                    # Remove targets
                    if tgt_command == 'rm':
                        if tgt_name != 'all':
                            for key in self.tgt:
                                if key != tgt_name:
                                    self.tgt[key].draw()
                        self.window.flip()
                    # Change targets
                    if tgt_command == 'change':
                        tgt_prop = message[2]
                        tgt_prop_value = message[3]
                        if tgt_prop == 'pos':
                            self.tgt[tgt_name].pos = tgt_prop_value
                        elif tgt_prop == 'size':
                            self.tgt[tgt_name].width = tgt_prop_value
                            self.tgt[tgt_name].height = tgt_prop_value
                        elif tgt_prop == 'line_width':
                            self.tgt[tgt_name].lineWidth = tgt_prop_value
                        elif tgt_prop == 'fill_color':
                            self.tgt[tgt_name].fillColor = tgt_prop_value
                        elif tgt_prop == 'line_color':
                            self.tgt[tgt_name].lineColor = tgt_prop_value 
                    

                
                
                
                
                
                
                
                

if __name__ == '__main__':
    if sys.flags.interactive != 1 or not hasattr(QtCore, 'PYQT_VERSION'):
        # Queues to communicate btwn processes
        ######
        # main_to_screen_Q = multiprocessing.Queue()
        # fsm_to_screen_Q = multiprocessing.Queue()
        main_to_screen_rcvr, main_to_screen_sndr = multiprocessing.Pipe(duplex=False)
        fsm_to_screen_rcvr, fsm_to_screen_sndr = multiprocessing.Pipe(duplex=False)
        ######
        try:
            multiprocessing.set_start_method('spawn')
            screen_process = ScreenProcess(main_to_screen_rcvr,fsm_to_screen_rcvr)
            main_process = MainProcess(main_to_screen_sndr,fsm_to_screen_sndr)
            main_process.start()        
            time.sleep(0.25)
            screen_process.start()

        except Exception as e:
            traceback.print_exc()
            print('main_error')
            print(e)
        ######