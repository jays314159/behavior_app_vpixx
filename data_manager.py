"""
Laboratory for Computational Motor Control, Johns Hopkins School of Medicine
@author: Jay Pi <jay.s.314159@gmail.com>
"""
from PyQt5.QtCore import QRunnable, pyqtSignal, pyqtSlot, QObject

from pypixxlib._libdpx import DPxOpen, TPxEnableFreeRun, DPxSelectDevice, DPxUpdateRegCache, TPxDisableFreeRun, TPxSetupTPxSchedule
import h5py

import os
from datetime import date, datetime
from pathlib import Path
import numpy as np

class DataManagerSignals(QObject):
    to_main_thread = pyqtSignal(object)

class DataManager(QRunnable):
    def __init__(self):
        super().__init__()
        self.signals = DataManagerSignals()
        
        self.data_dir = Path(__file__).parent.resolve()
        print(self.data_dir)
        self.data_file_path = []
        self.trial_num = 0
        self.trial_data = {} # dictionary of 2000 Hz data
        
        self.setAutoDelete(False)
    @pyqtSlot()
    def run(self):
        '''
        save data to a specified HDF5 file
        '''

        data_file = h5py.File(self.data_file_path +'.hdf5','a',libver='latest')
        trial_grp = data_file.create_group("trial_"+str(self.trial_num))       
        for key,value in self.trial_data.items():
            trial_grp.create_dataset(key,data=self.trial_data[key])
        data_file.close() 
          
    def save_data(self):
        '''
        save data to a specified HDF5 file
        '''

        data_file = h5py.File(self.data_file_path +'.hdf5','a',libver='latest')
        group_name = "trial_"+str(self.trial_num)
        # If trial already exists, append data. Otherwise create new group
        try:
            trial_grp = data_file[group_name]
            appended_data = {}
            for trial_key,_ in trial_grp.items():
                self.trial_data[trial_key] = np.append(trial_grp[trial_key][:], self.trial_data[trial_key])
            del data_file[group_name]
        except:
            pass
        finally:
            trial_grp = data_file.create_group(group_name)       
            for key,value in self.trial_data.items():
                trial_grp.create_dataset(key,data=self.trial_data[key])
            data_file.close() 
    
    def init_data(self, exp_name, exp_parameter):
        '''
        initialize data file for the session and exp. parameters
        Arguments:
        exp_name - name of the experiment (str)
        exp_parameter - dictionary of parameters
        '''
        data_dir_full_path = Path(self.data_dir/'data'/date.today().strftime("%Y-%m-%d"))
        data_dir_full_path.mkdir(parents=True,exist_ok=True)
        data_file_name = exp_name + '_' + datetime.now().strftime("%H%M%S")
        self.data_file_path = os.path.join(data_dir_full_path, data_file_name)
        data_file = h5py.File(self.data_file_path+'.hdf5','a',libver='latest')
        for key,value in exp_parameter.items():
            data_file.attrs[key] = value
        data_file.close() 
        self.signals.to_main_thread.emit(('log','Saving data to "' + data_file_name+'.hdf5"'))
        
