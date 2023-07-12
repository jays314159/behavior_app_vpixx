"""
Laboratory for Computational Motor Control, Johns Hopkins School of Medicine
@author: Jay Pi <jay.s.314159@gmail.com>
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox, QPushButton,\
                            QLineEdit, QPlainTextEdit, QShortcut
from PyQt5.QtCore import Qt, QThreadPool, QRunnable, pyqtSlot, QTimer, QObject, pyqtSignal

import app_lib as lib

import serial, os, json, time,traceback
from pathlib import Path


class PumpSignal(QObject):
    vol_dispensed = pyqtSignal(float)
    thread_finished = pyqtSignal()
    error = pyqtSignal()
class Pump(QRunnable):
    def __init__(self,serial_comm):
        super().__init__()
        self.serial_comm = serial_comm
        self.signal = PumpSignal()
        self.setAutoDelete(False) # if True, the object of this Worker is deleted
                                  # when starting it from QThreadPool
    @pyqtSlot()
    def run(self):
        pump_start = self.serial_comm.write(str.encode('RUN 1\r'))
        self.serial_comm.read_until(str.encode('ETX'))
        time.sleep(0.4) # Need to wait to dispense before querying how much dispensed
        self.signal.thread_finished.emit()
        
    def read_vol_dispensed(self):
        # Read volume dispensed by pump
        self.serial_comm.write(str.encode('DIS\r'))
        pump_msg =self.serial_comm.read_until(str.encode('ETX'))
        pump_msg = pump_msg.decode() # from byte to str
        pump_msg = pump_msg.lower()
        inf_idx = pump_msg.rfind('i')
        vol_disp = float(pump_msg[inf_idx+1:inf_idx+6])
        self.signal.vol_dispensed.emit(vol_disp)

            
    def reset_vol_dispensed(self):
        # Reset volume in pump 
        self.serial_comm.write(str.encode('CLD INF\r'))
        self.serial_comm.read_until(str.encode('ETX'))
        
class PumpWidget(QWidget):
    def __init__(self, pump_num, parent=None):
        super(PumpWidget,self).__init__(parent)
        self.pump_num = pump_num
        self.pump_name = 'pump_' + str(self.pump_num)
        self.thread_pool = QThreadPool() 
        self.is_pump_ready = True
        
        self.init_gui()
        # Load parameter or set default values
        self.parameter, self.parameter_file_path = lib.load_parameter('','pump_parameter.json',True,False,self.set_default_parameter,self.pump_name)
        self.update_parameter()
        
        # Init. serial comm.
        self.serial_comm = serial.Serial()
        self.init_serial_comm()
        # Init. pump
        self.pump = Pump(self.serial_comm)
        self.set_program()
        self.init_signals()
    def init_gui(self):
        self.main_QVBoxLayout = QVBoxLayout()
        self.setLayout(self.main_QVBoxLayout)
        
        self.port_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.port_QHBoxLayout)
        self.port_QLabel = QLabel('Port:')
        self.port_QLabel.setAlignment(Qt.AlignRight)
        self.port_QHBoxLayout.addWidget(self.port_QLabel)
        self.port_QLineEdit = QLineEdit()
        self.port_QLineEdit.setText('COM1')
        self.port_QHBoxLayout.addWidget(self.port_QLineEdit)
        self.port_apply_QPushButton = QPushButton('Apply')
        self.port_QHBoxLayout.addWidget(self.port_apply_QPushButton)
        
        self.vol_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.vol_QHBoxLayout)
        self.vol_QLabel = QLabel('Volume per Pump (mL):')
        self.vol_QLabel.setAlignment(Qt.AlignRight)
        self.vol_QHBoxLayout.addWidget(self.vol_QLabel)
        self.vol_QDoubleSpinBox = QDoubleSpinBox()
        self.vol_QDoubleSpinBox.setRange(0.001, 1)
        self.vol_QDoubleSpinBox.setDecimals(3)
        self.vol_QDoubleSpinBox.setSingleStep(0.001)
        self.vol_QDoubleSpinBox.setValue(0.02)
        self.vol_QHBoxLayout.addWidget(self.vol_QDoubleSpinBox)
        self.vol_apply_QPushButton = QPushButton('Apply')
        self.vol_QHBoxLayout.addWidget(self.vol_apply_QPushButton)
        
        self.pump_once_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.pump_once_QHBoxLayout)
        self.pump_once_QPushButton = QPushButton('Pump Once')
        self.pump_once_QPushButton.setToolTip('Pump whenever serial comm. is ready (<b>P</b>)')
        QShortcut(Qt.Key_P, self.pump_once_QPushButton, self.pump_once_QPushButton.animateClick)
        self.pump_once_QHBoxLayout.addWidget(self.pump_once_QPushButton)
        
        self.log_QPlainTextEdit = QPlainTextEdit()
        self.main_QVBoxLayout.addWidget(self.log_QPlainTextEdit)
        self.log_QPlainTextEdit.setReadOnly(True)
        
        self.vol_disp_QPushButton = QPushButton('Read volume dispensed')
        self.main_QVBoxLayout.addWidget(self.vol_disp_QPushButton)
        
        self.reset_vol_QPushButton = QPushButton('Reset volume dispensed')
        self.main_QVBoxLayout.addWidget(self.reset_vol_QPushButton)
        
    
    #%% Signals
    def init_signals(self):
        self.port_apply_QPushButton.clicked.connect(self.port_apply_QPushButton_clicked)
        self.port_QLineEdit.textEdited.connect(self.port_QLineEdit_textEdited)
        self.vol_apply_QPushButton.clicked.connect(self.vol_apply_QPushButton_clicked)
        self.vol_QDoubleSpinBox.valueChanged.connect(self.vol_QDoubleSpinBox_valueChanged)
        self.pump_once_QPushButton.clicked.connect(self.pump_once_QPushButton_clicked)
        self.vol_disp_QPushButton.clicked.connect(self.vol_disp_QPushButton_clicked)
        self.reset_vol_QPushButton.clicked.connect(self.reset_vol_QPushButton_clicked)
        self.pump.signal.vol_dispensed.connect(self.pump_vol_dispensed_received)
        self.pump.signal.error.connect(self.pump_error_received)
        self.pump.signal.thread_finished.connect(self.pump_thread_finished)
    #%% Slots    
    @pyqtSlot()
    def port_apply_QPushButton_clicked(self):
        '''
        change port address; e.g., 'COM1'

        '''
        try:
            self.parameter['port_address'] = self.port_QLineEdit.text()
            state = self.init_serial_comm()
            if state == 1:
                return
            with open(self.parameter_file_path,'r+') as file:
                parameter = json.load(file)
                parameter[self.pump_name] = self.parameter
                file.seek(0)
                json.dump(parameter, file, indent=4)
            self.set_program()
            self.log_QPlainTextEdit.appendPlainText('Port change success!')
            self.port_apply_QPushButton.setStyleSheet('background-color: #39E547')
        except:
            self.log_QPlainTextEdit.appendPlainText('Error: port change unsuccessful; try changing port address')
    @pyqtSlot()
    def vol_apply_QPushButton_clicked(self):
        '''
        change bolus volume (mL)
        '''
        self.parameter['pump_vol'] = self.vol_QDoubleSpinBox.value()
        try:
            self.set_program()
            self.log_QPlainTextEdit.appendPlainText('Volume change to ' + '{:.3f}'.format(self.parameter['pump_vol']) + ' mL success!')
            self.save_parameter()
            self.vol_apply_QPushButton.setStyleSheet('background-color: #39E547')
        except: 
            self.log_QPlainTextEdit.appendPlainText('Volume change unsuccessful')
        
    @pyqtSlot()
    def pump_once_QPushButton_clicked(self):
        if self.is_pump_ready:
            self.thread_pool.start(self.pump)
            self.is_pump_ready = False
    
    @pyqtSlot()
    def vol_disp_QPushButton_clicked(self):
        self.pump.read_vol_dispensed()
    
    @pyqtSlot()
    def pump_thread_finished(self):
        # Set pump ready
        self.is_pump_ready = True
        
    @pyqtSlot(float)
    def pump_vol_dispensed_received(self, vol_disp):
        '''
        update/save total volume dispensed and make log
        '''        
        # self.parameter['vol_dispensed'] = self.parameter['vol_dispensed'] + vol_disp       
        
        # self.save_parameter()    
        self.log_QPlainTextEdit.appendPlainText('Volume dispensed: ' + '{:.3f}'.format(vol_disp) + ' mL')
    
    @pyqtSlot()
    def pump_error_received(self):
        self.log_QPlainTextEdit.appendPlainText('Serial comm. error; check port address.')
    @pyqtSlot()
    def reset_vol_QPushButton_clicked(self):
        
        self.pump.reset_vol_dispensed()
        self.log_QPlainTextEdit.appendPlainText('Resetting volume dispensed')
        
    @pyqtSlot()
    def port_QLineEdit_textEdited(self):
        self.port_apply_QPushButton.setStyleSheet('background-color: #FFCC00')
    @pyqtSlot()
    def vol_QDoubleSpinBox_valueChanged(self):
        self.vol_apply_QPushButton.setStyleSheet('background-color: #FFCC00')
    #%% FUNCTIONS
    def set_program(self):
        try:
            # Run phase
            self.serial_comm.write(str.encode('DIA ' +str(28) + '\r')) # syringe diameter in mm
            self.serial_comm.read_until(str.encode('ETX'))
            self.serial_comm.write(str.encode('PHN 1\r'))
            self.serial_comm.read_until(str.encode('ETX'))
            self.serial_comm.write(str.encode('FUN RAT\r'))
            self.serial_comm.read_until(str.encode('ETX'))
            self.serial_comm.write(str.encode('RAT ' + str(12) + ' MM\r')) # pump rate as ml/min; 12 default
            self.serial_comm.read_until(str.encode('ETX'))
            self.serial_comm.write(str.encode('VOL ML\r'))
            self.serial_comm.read_until(str.encode('ETX'))
            self.serial_comm.write(str.encode('VOL ' + str(self.parameter['pump_vol']) + '\r'))
            self.serial_comm.read_until(str.encode('ETX'))
            self.serial_comm.write(str.encode('DIR INF\r'))
            self.serial_comm.read_until(str.encode('ETX'))
            
            # Stop phase
            self.serial_comm.write(str.encode('PHN 2\r'))
            self.serial_comm.read_until(str.encode('ETX'))
            self.serial_comm.write(str.encode('FUN STP\r'))
            self.serial_comm.read_until(str.encode('ETX'))
            
            self.serial_comm.write(str.encode('PHN 1\r')) # set to phase 1 to rcv. the values of this setting later; otherwise, pump sends settings of the latest phase, which is 2
            self.serial_comm.read_until(str.encode('ETX')) 
        except:
           self.log_QPlainTextEdit.appendPlainText('Serial comm. error; check port address.') 
    def init_serial_comm(self):     
        try:
            # if serial comm. open, close
            if self.serial_comm.is_open: 
                self.serial_comm.close()
            
            # Serial comm. properties
            self.serial_comm.baudrate = 19200
            self.serial_comm.port = self.parameter['port_address']
            self.serial_comm.bytesize = serial.EIGHTBITS
            self.serial_comm.stopbits = serial.STOPBITS_ONE
            self.serial_comm.parity = serial.PARITY_NONE
            self.serial_comm.xonxoff = False # software flow control
            self.serial_comm.rtscts = False # hardware (RTS/CTS) flow control
            self.serial_comm.dsrdtr = False 
            self.serial_comm.timeout = 0.1
            self.serial_comm.open()
            # Check if port set OK by querying for bolus volume 
            self.serial_comm.write(str.encode('VOL\r'))
            message = self.serial_comm.read_until(str.encode('ETX'))
            message = message.decode()
            message = message.lower()
            vol_idx = message.rfind('s')
            try: # check if the return value can be converted to float
                float(message[vol_idx+1:vol_idx+6])
            except:
                self.log_QPlainTextEdit.appendPlainText('Serial comm. error; check port address.')
                return 1
        
        except:
                self.log_QPlainTextEdit.appendPlainText('Serial comm. error; check port address.')
        
    
    def clean_exit(self):
        '''
        to be called before exiting the app
        '''
        self.serial_comm.close()
        
    def save_parameter(self):
        try:
            with open(self.parameter_file_path,'r') as file:
                all_parameter = json.load(file)
            all_parameter[self.pump_name] = self.parameter    
            with open(self.parameter_file_path,'w') as file:
                json.dump(all_parameter, file, indent=4)
        except Exception as error:
            self.log_QPlainTextEdit.appendPlainText('Error in saving .json file:')
            self.log_QPlainTextEdit.appendPlainText(error)
            
    def set_default_parameter(self):
        self.parameter = {
                          'port_address': 'COM1',
                          'pump_rate': 12.0,
                          'pump_vol': 0.02,
                          'vol_dispensed': 0.0
                          }
        return self.parameter
    def update_parameter(self):
        '''
        use saved parameters to set GUI values
        '''
        self.port_QLineEdit.setText(self.parameter['port_address'])
        self.vol_QDoubleSpinBox.setValue(self.parameter['pump_vol'])
  
    