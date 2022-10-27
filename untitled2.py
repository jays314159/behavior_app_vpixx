# -*- coding: utf-8 -*-
"""
Created on Fri Sep 16 15:59:48 2022

@author: ChamberB_Behavior
"""
from PyQt5 import QtCore
from PyQt5.QtWidgets import QMainWindow, QApplication, QComboBox, QPushButton, QLabel, QHBoxLayout, QDoubleSpinBox, QCheckBox, QPlainTextEdit,\
                            QDialog, QShortcut
from PyQt5.QtCore import QRunnable, QThreadPool, pyqtSignal, pyqtSlot, QObject, Qt, QTimer

from pypixxlib import tracker
from pypixxlib._libdpx import DPxOpen, TPxSetupTPxSchedule,TPxEnableFreeRun,DPxSelectDevice,DPxUpdateRegCache, DPxSetTPxAwake,\
                              TPxDisableFreeRun, DPxGetReg16,DPxGetTime,TPxBestPolyGetEyePosition, DPxSetDoutValue, TPxReadTPxData,\
                              TPxGetTrackerScheduleFrameLength, TPxGetBuffWriteAddr
import sys, time                              
import numpy as np          

import app_lib as lib                 

# class TimeLoop(QRunnable):
#     def __init__(self):
#         super().__init__()
#         self.setAutoDelete(False)
#     @pyqtSlot()
#     def run(self):
#         DPxOpen()
#         tracker.TRACKPixx3().open() # this throws error if not device not open           
#         DPxSetTPxAwake()
#         DPxSelectDevice('DATAPIXX3')   
#         DPxUpdateRegCache()
        
#         DPxSelectDevice('DATAPIXX3')
#         TPxSetupTPxSchedule()
#         TPxEnableFreeRun()
#         DPxUpdateRegCache()
        
#         old_time = 0
#         # Get pointers to store data from device
#         cal_data, raw_data = lib.VPixx_get_pointers_for_data()
        
#         while True:
#             DPxUpdateRegCache()

#             t = DPxGetTime()
#             print((t-old_time)*1000)
#             old_time = t
#             tpxData = TPxReadTPxData(0)
#             print(len(tpxData[0]))
#             TPxSetupTPxSchedule()
#             DPxUpdateRegCache()
# class Data(QRunnable):
#     def __init__(self):
#         super().__init__()
#         self.setAutoDelete(False)
#     @pyqtSlot()
#     def run(self):
#         tpxData = TPxReadTPxData(0)
#         print(len(tpxData[0]))
#         # TPxSetupTPxSchedule()
#         # DPxUpdateRegCache()
#         # TPxEnableFreeRun()
# class GUI(QMainWindow):
#     def __init__(self):
#         super().__init__()
#         self.threadpool = QThreadPool()
#         self.button = QPushButton('button')
#         self.button.clicked.connect(self.button_clicked)
#         self.setCentralWidget(self.button)   
#         self.timeloop = TimeLoop()
#         self.data = Data()
#         self.threadpool.start(self.timeloop)
#     @pyqtSlot()
#     def button_clicked(self):
#         self.threadpool.start(self.data)
        
        
    
# app = QApplication(sys.argv)
# gui = GUI() 
# gui.show()
# sys.exit(app.exec_())
cal_data, raw_data = lib.VPixx_get_pointers_for_data()
DPxOpen()
tracker.TRACKPixx3().open() # this throws error if not device not open           
DPxSetTPxAwake()
DPxSelectDevice('DATAPIXX3')   
DPxUpdateRegCache()
            
DPxSelectDevice('DATAPIXX3')
TPxSetupTPxSchedule()
TPxEnableFreeRun()
DPxUpdateRegCache()
t_data = []
t_break = time.time()
old_time=0
bitMask = 0xffffff # for VPixx dout
while True:
    # DPxUpdateRegCache()
    t = TPxBestPolyGetEyePosition(cal_data, raw_data)
    t_data.extend([t])
    eye_status = DPxGetReg16(0x59A)
    DPxSetDoutValue(0, bitMask)
    # t = DPxGetTime()
    # print((t-old_time)*1000)
    old_time = t
    # writeAddr = TPxGetBuffWriteAddr()
    # tpxData = TPxReadTPxData(0)
    # print(len(tpxData[0]))
    # t = tpxData[0][0::22]
    # t_data.extend(t)
    # print(np.diff(t))
    # TPxSetupTPxSchedule() # resets data
    # DPxUpdateRegCache()
    if time.time() - t_break > 10:
        print('done')
        break
# tpxData = TPxReadTPxData(0,30)
# t = tpxData[0][0::22]
# t_data.extend(t)
diff_t = np.diff(t_data)
print(diff_t)

frameSize = TPxGetTrackerScheduleFrameLength()
writeAddr = TPxGetBuffWriteAddr()
# TPxEnableFreeRun()
# DPxUpdateRegCache()
# DPxUpdateRegCache()
# t = DPxGetTime()            
# tpxData = TPxReadTPxData(0)
# print(len(tpxData[0]))


# TPxSetupTPxSchedule() # resets data
# TPxEnableFreeRun()

# TPxDisableFreeRun()
# t = tpxData[0][0::22]