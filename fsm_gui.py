"""
Laboratory for Computational Motor Control, Johns Hopkins School of Medicine
@author: Jay Pi <jay.s.314159@gmail.com>
"""
from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QSplitter, QWidget, QTabWidget, QToolBar, QAction,\
                            QShortcut, QPlainTextEdit          
from PyQt5.QtCore import Qt, QThreadPool,QTimer
from PyQt5.QtGui import QKeySequence
import pyqtgraph as pg

from pypixxlib import tracker
from pypixxlib._libdpx import DPxOpen, DPxClose, TPxSetupTPxSchedule, TPxDisableFreeRun, DPxSelectDevice, DPxUpdateRegCache,\
                              DPxSetTPxSleep, DPxGetError, DPxSetTPxAwake

from pump import PumpWidget
from target import TargetWidget
from sound import Sound

import time
import numpy as np
from collections import deque

class FsmGui(QMainWindow):
    def __init__(self,fsm_to_screen_sndr, stop_fsm_process_Event):        
        super(FsmGui,self).__init__(parent=None)
        self.thread_pool = QThreadPool()  
        self.data_QTimer = QTimer()
        
        self.fsm_to_screen_sndr = fsm_to_screen_sndr
        self.stop_fsm_process_Event = stop_fsm_process_Event
        self.main_QWidget = QWidget()
        self.setCentralWidget(self.main_QWidget)
        self.main_QVBoxLayout = QVBoxLayout()
        self.main_QWidget.setLayout(self.main_QVBoxLayout)
        self.main_horz_QSplitter = QSplitter(Qt.Horizontal)
        self.main_QVBoxLayout.addWidget(self.main_horz_QSplitter)
        self.main_horz_QSplitter.setChildrenCollapsible(False)
        self.main_vert_QSplitter = QSplitter(Qt.Vertical)
        self.main_horz_QSplitter.addWidget(self.main_vert_QSplitter)
        self.main_vert_QSplitter.setChildrenCollapsible(False)
        
        #%% TOOLBAR
        self.toolbar = QToolBar()
        self.toolbar.setContextMenuPolicy(Qt.PreventContextMenu)
        self.toolbar.setIconSize(QtCore.QSize(25,25))
        self.addToolBar(self.toolbar)  
        self.toolbar_run_QAction = QAction(QtGui.QApplication.style().\
            standardIcon(QtGui.QStyle.SP_MediaPlay), "Run", self)
        self.toolbar_run_QAction.setToolTip("Run (<b>R</b>)")
        self.toolbar_run_QAction.setShortcut(Qt.Key_R)
        self.toolbar.addAction(self.toolbar_run_QAction)
        self.toolbar_stop_QAction = QAction(QtGui.QApplication.style().\
            standardIcon(QtGui.QStyle.SP_MediaStop), "Stop", self)
        self.toolbar_stop_QAction.setDisabled(True)
        self.toolbar_stop_QAction.setToolTip("Stop (<b>CTRL+S</b>)")
        self.toolbar_stop_QAction.setShortcut(QKeySequence('Ctrl+S'))
        self.toolbar.addAction(self.toolbar_stop_QAction)
        #%% PLOTS    
        self.plot_1_PlotWidget = pg.PlotWidget() # Scatter plot
        self.main_vert_QSplitter.addWidget(self.plot_1_PlotWidget)
        self.plot_1_PlotWidget.setBackground('w')
        font = QtGui.QFont()
        font.setPixelSize(10)
        self.plot_1_PlotWidget.setLabel('left', "Vertical position", units = 'deg')
        self.plot_1_PlotWidget.getAxis('left').setPen(pg.mkPen(color = 'k', width = 1))
        self.plot_1_PlotWidget.getAxis('left').setStyle(tickFont = font)
        self.plot_1_PlotWidget.setTitle('Position scatter plot')
        self.plot_1_PlotWidget.setLabel('bottom', "Horizontal position", units = 'deg')
        self.plot_1_PlotWidget.getAxis('bottom').setPen(pg.mkPen(color = 'k', width = 1))
        self.plot_1_PlotWidget.getAxis('bottom').setStyle(tickFont = font)
        self.plot_1_PlotWidget_ViewBox = self.plot_1_PlotWidget.getViewBox()
        self.plot_1_PlotWidget_ViewBox.setRange(xRange=(0,20),yRange=(0,20))
        self.plot_1_PlotWidget_PlotItem = self.plot_1_PlotWidget.getPlotItem()
        self.plot_1_PlotWidget_PlotItem.addLegend(offset=(-20,5))      
                
        self.plot_2_PlotWidget = pg.PlotWidget() # Position vs. time plot 
        self.main_vert_QSplitter.addWidget(self.plot_2_PlotWidget)
        self.plot_2_PlotWidget.setBackground('w')
        font = QtGui.QFont()
        font.setPixelSize(10)
        self.plot_2_PlotWidget.setLabel('left', "Position", units = 'deg')
        self.plot_2_PlotWidget.getAxis('left').setPen(pg.mkPen(color = 'k', width = 1))
        self.plot_2_PlotWidget.getAxis('left').setStyle(tickFont = font)
        self.plot_2_PlotWidget.setTitle('Time plot')
        self.plot_2_PlotWidget.setLabel('bottom', "Time", units = 's')
        self.plot_2_PlotWidget.getAxis('bottom').setPen(pg.mkPen(color = 'k', width = 1))
        self.plot_2_PlotWidget.getAxis('bottom').setStyle(tickFont = font)
        self.plot_2_PlotWidget_ViewBox = self.plot_2_PlotWidget.getViewBox()
        self.plot_2_PlotWidget_ViewBox.setRange(xRange=(0,20),yRange=(0,20))
        self.plot_2_PlotWidget_ViewBox.enableAutoRange(self.plot_2_PlotWidget_ViewBox.XAxis)   
        self.plot_2_PlotWidget_PlotItem = self.plot_2_PlotWidget.getPlotItem()
        self.plot_2_PlotWidget_PlotItem.addLegend(offset=(-20,5))
        
        self.plot_2_tgt_x = self.plot_2_PlotWidget.\
            plot(np.zeros((0)), np.zeros((0)), pen = pg.mkPen(color=(245,130,48), width=2.5,style=QtCore.Qt.DashLine),name='tgt x',connect='finite')
        self.plot_2_tgt_y = self.plot_2_PlotWidget.\
            plot(np.zeros((0)), np.zeros((0)), pen = pg.mkPen(color=(210,245,60), width=2.5,style=QtCore.Qt.DashLine),name='tgt y',connect='finite')
        self.plot_2_eye_x = self.plot_2_PlotWidget.\
            plot(np.zeros((0)), np.zeros((0)), pen = pg.mkPen(color=(230,25,75), width=2.5),name='eye x',connect='finite') # plotdataitem     
        self.plot_2_eye_y = self.plot_2_PlotWidget.\
            plot(np.zeros((0)), np.zeros((0)), pen = pg.mkPen(color=(60,180,75), width=2.5),name='eye y',connect='finite') # plotdataitem
        
        
        # Housekeeping var. to plot eye data
        self.data_rate = int(1/60*1000) # how often to get eye and time data from fsm (ms)
        # self.data_rate = 1
        data_duration = 5 # how long to store eye and time data (s)
        data_length = int(data_duration*1000/int(1/60*1000))
        self.eye_x_data = deque(maxlen=data_length)
        self.eye_y_data = deque(maxlen=data_length)
        self.tgt_x_data = deque(maxlen=data_length)
        self.tgt_y_data = deque(maxlen=data_length)   
        self.t_data = deque(maxlen=data_length)
        #%% LOG
        self.log_QPlainTextEdit = QPlainTextEdit()
        self.log_QPlainTextEdit.setReadOnly(True)
        self.main_vert_QSplitter.addWidget(self.log_QPlainTextEdit)
        #%% PARAMETER
        self.sidepanel_QWidget = QWidget()
        self.main_horz_QSplitter.addWidget(self.sidepanel_QWidget)
        self.sidepanel_QVBoxLayout = QVBoxLayout()
        self.sidepanel_QWidget.setLayout(self.sidepanel_QVBoxLayout)
        self.sidepanel_parameter_QWidget = QWidget()
        self.sidepanel_QVBoxLayout.addWidget(self.sidepanel_parameter_QWidget)
        self.sidepanel_custom_QVBoxLayout = QVBoxLayout() # place to add custom parameters/widgets      
        self.sidepanel_parameter_QWidget.setLayout(self.sidepanel_custom_QVBoxLayout)
        self.sidepanel_QVBoxLayout.addStretch(1)
        #%% WIDGETS
        self.sidepanel_QTabWidget = QTabWidget()
        self.sidepanel_default_QVBoxLayout = QVBoxLayout() # place to add default parameters/widgets
        self.sidepanel_default_QVBoxLayout.addWidget(self.sidepanel_QTabWidget)
        self.pump_1 = PumpWidget(1)
        self.sidepanel_QTabWidget.addTab(self.pump_1, 'Pump 1')
        self.pump_2 = PumpWidget(2)
        self.sidepanel_QTabWidget.addTab(self.pump_2, 'Pump 2')
        self.sidepanel_QVBoxLayout.addLayout(self.sidepanel_default_QVBoxLayout)
        self.tgt = TargetWidget('tgt',self.fsm_to_screen_sndr)
        self.tgt.tgt_pos_x_QDoubleSpinBox.setDisabled(True)
        self.tgt.tgt_pos_y_QDoubleSpinBox.setDisabled(True)
        self.tgt.tgt_pos_QPushButton.setDisabled(True)
        self.sidepanel_QTabWidget.addTab(self.tgt,'Target')
        self.pd_tgt = TargetWidget('pd_tgt',self.fsm_to_screen_sndr)
        self.sidepanel_QTabWidget.addTab(self.pd_tgt,'PD Target')

        #%% SOUND
        self.neutral_beep = Sound(1000,0.1)
        self.pun_beep = Sound(200,0.1)
        self.reward_beep = Sound(2000,0.1)
    def closeEvent(self,event):
        self.stop_fsm_process_Event.set()
        time.sleep(0.1)
        ######
        # # Wait for signal for successful exit 
        # while True:
        #     if (not self.fsm_to_gui_rcvr.empty()) and (self.fsm_to_gui_rcvr.get() == 1):
        #         time.sleep(0.1)
        #         break
        ######
        self.pump_1.clean_exit()
        self.pump_2.clean_exit()

        pg.exit() # this should come at the end 
        
        