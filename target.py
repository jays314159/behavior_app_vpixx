from PyQt5.QtCore import QObject, Qt
from PyQt5.QtWidgets import QWidget,QVBoxLayout,QHBoxLayout,QLabel,QDoubleSpinBox,QComboBox,QPushButton
from PyQt5.QtGui import QColor

from pathlib import Path
import os, json

import app_lib as lib

class TargetWidget(QWidget,QObject):
    def __init__(self,tgt_name, fsm_to_screen_sndr,parent=None):
        super(TargetWidget,self).__init__(parent)
        self.fsm_to_screen_sndr = fsm_to_screen_sndr
        self.tgt_name = tgt_name
        self.init_gui()
        
        # Load parameter or set default values
        self.parameter, self.parameter_file_path = lib.load_parameter('','tgt_parameter.json',True,self.set_default_parameter,self.tgt_name)
        self.init_signals()
        self.update_parameter()
        self.save_QPushButton.setStyleSheet('background-color: #39E547')
        
          
    def init_signals(self):
        self.tgt_size_QDoubleSpinBox.valueChanged.connect(self.tgt_size_QDoubleSpinBox_valueChanged)
        self.line_width_QDoubleSpinBox.valueChanged.connect(self.line_width_QDoubleSpinBox_valueChanged)
        self.fill_color_QComboBox.currentTextChanged.connect(self.fill_color_QComboBox_currentTextChanged)
        self.line_color_QComboBox.currentTextChanged.connect(self.line_color_QComboBox_currentTextChanged)
        self.tgt_pos_x_QDoubleSpinBox.valueChanged.connect(self.tgt_pos_x_QDoubleSpinBox_valueChanged)
        self.tgt_pos_y_QDoubleSpinBox.valueChanged.connect(self.tgt_pos_y_QDoubleSpinBox_valueChanged)
        self.tgt_pos_QPushButton.clicked.connect(self.tgt_pos_QPushButton_clicked)
        self.save_QPushButton.clicked.connect(self.save_QPushButton_clicked)
        
    def init_gui(self):
        self.main_QVBoxLayout = QVBoxLayout()
        self.setLayout(self.main_QVBoxLayout)
        
        self.tgt_size_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.tgt_size_QHBoxLayout)
        self.tgt_size_QLabel = QLabel('Target size (deg):')
        self.tgt_size_QLabel.setAlignment(Qt.AlignRight)
        self.tgt_size_QHBoxLayout.addWidget(self.tgt_size_QLabel)
        self.tgt_size_QDoubleSpinBox = QDoubleSpinBox()
        self.tgt_size_QDoubleSpinBox.setRange(0.01, 20)
        self.tgt_size_QDoubleSpinBox.setDecimals(3)
        self.tgt_size_QDoubleSpinBox.setSingleStep(0.01)
        self.tgt_size_QDoubleSpinBox.setValue(0.2)
        self.tgt_size_QHBoxLayout.addWidget(self.tgt_size_QDoubleSpinBox)
        
        self.line_width_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.line_width_QHBoxLayout)
        self.line_width_QLabel = QLabel('Line width (px):')
        self.line_width_QLabel.setAlignment(Qt.AlignRight)
        self.line_width_QHBoxLayout.addWidget(self.line_width_QLabel)
        self.line_width_QDoubleSpinBox = QDoubleSpinBox()
        self.line_width_QDoubleSpinBox.setRange(0.05, 100)
        self.line_width_QDoubleSpinBox.setDecimals(3)
        self.line_width_QDoubleSpinBox.setSingleStep(0.01)
        self.line_width_QDoubleSpinBox.setValue(1.5)
        self.line_width_QHBoxLayout.addWidget(self.line_width_QDoubleSpinBox)

        self.fill_color_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.fill_color_QHBoxLayout)
        self.fill_color_QLabel = QLabel('Fill color:')
        self.fill_color_QLabel.setAlignment(Qt.AlignRight)
        self.fill_color_QHBoxLayout.addWidget(self.fill_color_QLabel)
        self.fill_color_QComboBox = QComboBox()
        self.fill_color_QHBoxLayout.addWidget(self.fill_color_QComboBox)
        self.fill_color_QComboBox.addItem('black')
        self.fill_color_QComboBox.setItemData(0,QColor(Qt.black),Qt.BackgroundRole)
        self.fill_color_QComboBox.addItem('cyan')
        self.fill_color_QComboBox.setItemData(1,QColor(Qt.cyan),Qt.BackgroundRole)
        self.fill_color_QComboBox.addItem('white')
        self.fill_color_QComboBox.setItemData(2,QColor(Qt.white),Qt.BackgroundRole)
        self.fill_color_QComboBox.addItem('blue')
        self.fill_color_QComboBox.setItemData(3,QColor(Qt.blue),Qt.BackgroundRole)
        self.fill_color_QComboBox.addItem('darkRed')
        self.fill_color_QComboBox.setItemData(4,QColor(Qt.darkRed),Qt.BackgroundRole)
        self.fill_color_QComboBox.addItem('red')
        self.fill_color_QComboBox.setItemData(5,QColor(Qt.red),Qt.BackgroundRole)
        self.fill_color_QComboBox.addItem('green')
        self.fill_color_QComboBox.setItemData(6,QColor(Qt.green),Qt.BackgroundRole)
        self.fill_color_QComboBox.addItem('magenta')
        self.fill_color_QComboBox.setItemData(7,QColor(Qt.magenta),Qt.BackgroundRole)
        self.fill_color_QComboBox.addItem('yellow')
        self.fill_color_QComboBox.setItemData(8,QColor(Qt.yellow),Qt.BackgroundRole)

        
        self.line_color_QHBoxLayout = QHBoxLayout()
        self.main_QVBoxLayout.addLayout(self.line_color_QHBoxLayout)
        self.line_color_QLabel = QLabel('Line color (R,G,B):')
        self.line_color_QLabel.setAlignment(Qt.AlignRight)
        self.line_color_QHBoxLayout.addWidget(self.line_color_QLabel)
        self.line_color_QComboBox = QComboBox()
        self.line_color_QHBoxLayout.addWidget(self.line_color_QComboBox)
        self.line_color_QComboBox.addItem('black')
        self.line_color_QComboBox.setItemData(0,QColor(Qt.black),Qt.BackgroundRole)
        self.line_color_QComboBox.addItem('cyan')
        self.line_color_QComboBox.setItemData(1,QColor(Qt.cyan),Qt.BackgroundRole)
        self.line_color_QComboBox.addItem('white')
        self.line_color_QComboBox.setItemData(2,QColor(Qt.white),Qt.BackgroundRole)
        self.line_color_QComboBox.addItem('blue')
        self.line_color_QComboBox.setItemData(3,QColor(Qt.blue),Qt.BackgroundRole)
        self.line_color_QComboBox.addItem('darkRed')
        self.line_color_QComboBox.setItemData(4,QColor(Qt.darkRed),Qt.BackgroundRole)
        self.line_color_QComboBox.addItem('red')
        self.line_color_QComboBox.setItemData(5,QColor(Qt.red),Qt.BackgroundRole)
        self.line_color_QComboBox.addItem('green')
        self.line_color_QComboBox.setItemData(6,QColor(Qt.green),Qt.BackgroundRole)
        self.line_color_QComboBox.addItem('magenta')
        self.line_color_QComboBox.setItemData(7,QColor(Qt.magenta),Qt.BackgroundRole)
        self.line_color_QComboBox.addItem('yellow')
        self.line_color_QComboBox.setItemData(8,QColor(Qt.yellow),Qt.BackgroundRole)
    
        self.tgt_pos_QHBoxLayout = QHBoxLayout()
        self.tgt_pos_QLabel = QLabel('Target Pos. (x,y) (deg): ')
        self.tgt_pos_QHBoxLayout.addWidget(self.tgt_pos_QLabel)
        self.tgt_pos_x_QDoubleSpinBox = QDoubleSpinBox()
        self.tgt_pos_x_QDoubleSpinBox.setValue(0)
        self.tgt_pos_x_QDoubleSpinBox.setDecimals(1)
        self.tgt_pos_x_QDoubleSpinBox.setSingleStep(0.1)
        self.tgt_pos_QHBoxLayout.addWidget(self.tgt_pos_x_QDoubleSpinBox)
        self.tgt_pos_y_QDoubleSpinBox = QDoubleSpinBox()
        self.tgt_pos_y_QDoubleSpinBox.setValue(0)
        self.tgt_pos_y_QDoubleSpinBox.setDecimals(1)
        self.tgt_pos_y_QDoubleSpinBox.setSingleStep(0.1)
        self.tgt_pos_QHBoxLayout.addWidget(self.tgt_pos_y_QDoubleSpinBox)
        self.tgt_pos_QPushButton = QPushButton('Apply')
        self.tgt_pos_QHBoxLayout.addWidget(self.tgt_pos_QPushButton)
        self.main_QVBoxLayout.addLayout(self.tgt_pos_QHBoxLayout)
    
        self.save_QPushButton = QPushButton('Save')
        self.main_QVBoxLayout.addWidget(self.save_QPushButton)
    #%% SLOTS
    def tgt_size_QDoubleSpinBox_valueChanged(self):
        self.fsm_to_screen_sndr.send((self.tgt_name,'change','size',self.tgt_size_QDoubleSpinBox.value()))
        self.parameter['size'] = self.tgt_size_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
    def line_width_QDoubleSpinBox_valueChanged(self):
        self.fsm_to_screen_sndr.send((self.tgt_name,'change','line_width',self.line_width_QDoubleSpinBox.value()))
        self.parameter['line_width'] = self.line_width_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
    def fill_color_QComboBox_currentTextChanged(self):
        self.fsm_to_screen_sndr.send((self.tgt_name,'change','fill_color',self.fill_color_QComboBox.currentText()))
        self.parameter['fill_color'] = self.fill_color_QComboBox.currentText()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
    def line_color_QComboBox_currentTextChanged(self):
        self.fsm_to_screen_sndr.send((self.tgt_name,'change','line_color',self.line_color_QComboBox.currentText()))
        self.parameter['line_color'] = self.line_color_QComboBox.currentText()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
    def tgt_pos_x_QDoubleSpinBox_valueChanged(self):
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
        self.parameter['pos'][0] = self.tgt_pos_x_QDoubleSpinBox.value()
    def tgt_pos_y_QDoubleSpinBox_valueChanged(self):
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
        self.parameter['pos'][1] = self.tgt_pos_y_QDoubleSpinBox.value()      
    def tgt_pos_QPushButton_clicked(self):
        self.fsm_to_screen_sndr.send((self.tgt_name,'change','pos',(self.tgt_pos_x_QDoubleSpinBox.value(),self.tgt_pos_y_QDoubleSpinBox.value())))
        self.parameter['pos'] = [self.tgt_pos_x_QDoubleSpinBox.value(),self.tgt_pos_y_QDoubleSpinBox.value()]
    def save_QPushButton_clicked(self):
        with open(self.parameter_file_path,'r') as file:
            all_parameter = json.load(file)
        all_parameter[self.tgt_name] = self.parameter    
        with open(self.parameter_file_path,'w') as file:
            json.dump(all_parameter, file, indent=4)
        self.save_QPushButton.setStyleSheet('background-color: #39E547')
    #%% FUNCTIONS
    def set_default_parameter(self):
        self.parameter = {'size':0.5,'line_width':1.5,'fill_color':'black','line_color':'blue','pos':[0,0]}
        return self.parameter

    def update_parameter(self):
        '''
        use saved parameters to set GUI values
        '''
        self.tgt_size_QDoubleSpinBox.setValue(self.parameter['size'])
        self.line_width_QDoubleSpinBox.setValue(self.parameter['line_width'])
        self.fill_color_QComboBox.setCurrentText(self.parameter['fill_color'])
        self.line_color_QComboBox.setCurrentText(self.parameter['line_color'])
        self.tgt_pos_x_QDoubleSpinBox.setValue(self.parameter['pos'][0])
        self.tgt_pos_y_QDoubleSpinBox.setValue(self.parameter['pos'][1])