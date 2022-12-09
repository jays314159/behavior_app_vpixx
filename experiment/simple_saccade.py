"""
Laboratory for Computational Motor Control, Johns Hopkins School of Medicine
@author: Jay Pi <jay.s.314159@gmail.com>
"""
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QComboBox, QPushButton, QLabel, QHBoxLayout, QDoubleSpinBox, QCheckBox, QPlainTextEdit,\
                            QDialog, QShortcut
from PyQt5.QtCore import QRunnable, QThreadPool, pyqtSignal, pyqtSlot, QObject, Qt, QTimer
import pyqtgraph as pg

from pypixxlib import tracker
from pypixxlib._libdpx import DPxOpen, TPxSetupTPxSchedule,TPxEnableFreeRun,DPxSelectDevice,DPxUpdateRegCache, DPxSetTPxAwake,\
                              TPxDisableFreeRun, DPxGetReg16,DPxGetTime,TPxBestPolyGetEyePosition, DPxSetDoutValue, TPxReadTPxData,\
                              DPxSetTPxSleep, DPxClose

import multiprocessing, sys, os, json, random, time, copy, ctypes, traceback, gc
sys.path.append('../app')
from pathlib import Path
import numpy as np
from collections import deque
from datetime import datetime

from fsm_gui import FsmGui
import app_lib as lib
from data_manager import DataManager

class SimpleSacFsmProcess(multiprocessing.Process):
    def __init__(self,exp_name, default_parameter_fnc, fsm_to_screen_sndr, fsm_to_gui_sndr, gui_to_fsm_Q, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array):
        super().__init__()
        self.exp_name = exp_name
        print(self.exp_name)
        self.default_parameter_fnc = default_parameter_fnc
        self.fsm_to_screen_sndr = fsm_to_screen_sndr
        self.fsm_to_gui_sndr = fsm_to_gui_sndr
        self.gui_to_fsm_Q = gui_to_fsm_Q
        self.stop_exp_Event = stop_exp_Event
        self.stop_fsm_process_Event = stop_fsm_process_Event
        self.real_time_data_Array = real_time_data_Array
        
        # Init var.
        self.eye_x = 0
        self.eye_y = 0
        self.tgt_x = 0
        self.tgt_y = 0
        self.start_x = 0
        self.start_y = 0
        self.cue_x = 0
        self.cue_y = 0
        # self.t = 0
        # self.pull_data_t = 0 # keep track of when data was pulled last from VPixx
    
    def run(self):
        # import faulthandler
        # faulthandler.disable()
        # faulthandler.enable()
        # gc.disable()

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
        
        # Remove all targets from screen
        self.fsm_to_screen_sndr.send(('all','rm'))
             
        run_exp = False
        # Process loop
        while not self.stop_fsm_process_Event.is_set():
            if not self.stop_exp_Event.is_set():
                # Load exp parameter
                fsm_parameter, parameter_file_path = lib.load_parameter('experiment','exp_parameter.json',True,self.default_parameter_fnc,self.exp_name)
                # Load calibration
                cal_parameter, _ = lib.load_parameter('calibration','cal_parameter.json',False,lib.set_default_cal_parameter)                  
                run_exp = True
                # Create target list
                target_pos_list = lib.make_prim_target(fsm_parameter)
                num_tgt_pos = len(target_pos_list)
                # Init. var
                self.t = DPxGetTime()
                self.pull_data_t = self.t
                trial_num = 1
                pump_to_use = 1 # which pump to use currently
                vel_samp_num = 3
                vel_t_data = deque(maxlen=vel_samp_num)
                eye_x_data = deque(maxlen=vel_samp_num)
                eye_y_data = deque(maxlen=vel_samp_num)
                eye_pos = [0,0]
                eye_vel = [0,0]
                eye_speed = 0.0
                bitMask = 0xffffff # for VPixx dout
                
            
            # Trial loop
            while not self.stop_fsm_process_Event.is_set() and run_exp: 
                if self.stop_exp_Event.is_set():
                    run_exp = False

                    # Remove all targets
                    self.fsm_to_screen_sndr.send(('all','rm'))

                    break
                # Init. trial variables; reset every trial
                self.init_trial_data()  
                self.trial_data['cal_matrix'] = cal_parameter['cal_matrix']
                state = 'INIT'   
                
                # FSM loop
                while not self.stop_fsm_process_Event.is_set() and run_exp:
                    if self.stop_exp_Event.is_set():
                        break
                    # Get time       
                    self.t = TPxBestPolyGetEyePosition(cal_data, raw_data) # this calls 'DPxUpdateRegCache' as well
                    sys_t = time.perf_counter()
                    while True:
                        if time.perf_counter() - sys_t >= 0.0001:
                            break
                    # Get eye status (blinking)
                    eye_status = DPxGetReg16(0x59A)
                    left_eye_blink = bool(eye_status & (1 << 1)) # right blink, left blink
                    if not left_eye_blink:
                        raw_data_left = [raw_data[2], raw_data[3],1] # [left x, left y, right x, right y]
                        eye_pos = lib.raw_to_deg(raw_data_left,cal_parameter['cal_matrix'])
                        self.eye_x = eye_pos[0]
                        self.eye_y = eye_pos[1]
                        # Compute eye velocity
                        vel_t_data.append(self.t)
                        eye_x_data.append(self.eye_x)
                        eye_y_data.append(self.eye_y)
                        if len(vel_t_data)==vel_samp_num:
                            eye_vel[0] = np.mean(np.diff(eye_x_data)/np.diff(vel_t_data))
                            eye_vel[1] = np.mean(np.diff(eye_y_data)/np.diff(vel_t_data))
                            eye_speed = np.sqrt(eye_vel[0]**2 + eye_vel[1]**2)
                    else:
                        self.eye_x = 9999 # invalid values; more stable than nan values for plotting purposes in pyqtgraph
                        self.eye_y = 9999 
                    # Send random signal
                    if random.random() > 0.5:
                        random_signal_value = 4
                    else:
                        random_signal_value = 0
                    DPxSetDoutValue(random_signal_value, bitMask)
                    if state == 'INIT':
                        print('state = INIT')
                        self.fsm_to_gui_sndr.send(('log',datetime.now().strftime("%H:%M:%S") + '; trial num: ' + str(trial_num)))
                        # Set trial parameters
                        tgt_idx = random.randint(0,num_tgt_pos-1) # Randomly pick target
                        start_pos = (fsm_parameter['horz_offset'], fsm_parameter['vert_offset'])
                        self.start_x = start_pos[0]
                        self.start_y = start_pos[1]
                        self.trial_data['start_x'].append(self.start_x)
                        self.trial_data['start_y'].append(self.start_y)
                        
                        cue_pos = np.array(target_pos_list[tgt_idx]['prim_tgt_pos']) + np.array(start_pos)
                        print(cue_pos)
                        self.cue_x = cue_pos[0]
                        self.cue_y = cue_pos[1]
                        self.trial_data['cue_x'].append(self.cue_x)
                        self.trial_data['cue_y'].append(self.cue_y)
                        # Send target data
                        self.fsm_to_gui_sndr.send(('tgt_data',(self.cue_x,self.cue_y)))
                        pursuit_angle = np.random.randint(0,360)
                        pursuit_start_x = np.cos(pursuit_angle*np.pi/180)*fsm_parameter['pursuit_amp']
                        pursuit_start_x += self.start_x
                        pursuit_v_x = (self.start_x - pursuit_start_x)/fsm_parameter['pursuit_dur']
                        pursuit_start_y = np.sin(pursuit_angle*np.pi/180)*fsm_parameter['pursuit_amp']
                        pursuit_start_y += self.start_y
                        pursuit_v_y = (self.start_y - pursuit_start_y)/fsm_parameter['pursuit_dur']
                        
                        state_start_time = self.t
                        state_inter_time = self.t
                        self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                        state = 'STR_TARGET_PURSUIT'
                        print('state = STR_TARGET_PURSUIT')
                        self.pd_state_start_time = self.t
                        self.pd_state = 'PD_ON'
                    if state == 'STR_TARGET_PURSUIT':
                        pursuit_x = pursuit_v_x*(self.t-state_start_time) + pursuit_start_x
                        pursuit_y = pursuit_v_y*(self.t-state_start_time) + pursuit_start_y  
                        self.tgt_x = pursuit_x
                        self.tgt_y = pursuit_y
                        self.fsm_to_screen_sndr.send(('tgt','draw',(self.tgt_x,self.tgt_y)))
                        if (self.t-state_start_time) > fsm_parameter['pursuit_dur']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_present'].append(self.t)
                            state = 'STR_TARGET_PRESENT'    
                            print('state = STR_TARGET_PRESENT')
                        if self.t - self.pull_data_t > 5:
                            self.pull_data_t = self.t
                            self.pull_data()
                            # Send trial data to GUI
                            self.fsm_to_gui_sndr.send(('trial_data',trial_num, self.trial_data))
                            self.init_trial_data()
                    if state == 'STR_TARGET_PRESENT':

                        if not left_eye_blink:
                            self.tgt_x = self.start_x
                            self.tgt_y = self.start_y
                            # self.present_tgt_and_pd_tgt()
                            self.fsm_to_screen_sndr.send(('tgt','draw',(self.tgt_x,self.tgt_y)))
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_fixation'].append(self.t)
                            state = 'STR_TARGET_FIXATION'
                            print('state = STR_TARGET_FIXATION')
                        elif (self.t-state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            self.fsm_to_screen_sndr.send(('all','rm')) # remove all targets
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            state = 'STR_TARGET_PURSUIT'
                            self.pd_state_start_time = self.t
                            self.pd_state = 'PD_ON'
                    if state == 'STR_TARGET_FIXATION':
                        
                        
                        eye_dist_from_tgt = np.sqrt((self.tgt_x-self.eye_x)**2 + (self.tgt_y-self.eye_y)**2)

                        # If eye not available or fixating at the start target, reset the timer
                        if eye_dist_from_tgt > fsm_parameter['rew_area']/2 or left_eye_blink:
                            state_inter_time = self.t
                        if (self.t-state_inter_time) >= fsm_parameter['min_fix_time']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_cue_tgt_present'].append(self.t)
                            print('state = CUE_TARGET_PRESENT')
                            state = 'CUE_TARGET_PRESENT'
                            self.pd_state_start_time = self.t
                            self.pd_state = 'PD_ON'
                        if (self.t-state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            self.fsm_to_screen_sndr.send(('all','rm')) # remove all targets
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            state = 'STR_TARGET_PURSUIT'
                            self.pd_state_start_time = self.t
                            self.pd_state = 'PD_ON'    
                    if state == 'CUE_TARGET_PRESENT':
                        self.tgt_x = self.cue_x
                        self.tgt_y = self.cue_y
                        self.fsm_to_screen_sndr.send(('tgt','draw',(self.tgt_x,self.tgt_y)))
                        # self.fsm_to_gui_sndr.send(('neutral_beep',0))
                        lib.playSound(1000,0.1) # neutral beep  
                        state_start_time = self.t
                        state_inter_time = self.t
                        self.trial_data['state_start_t_detect_sac_start'].append(self.t)
                        state = 'DETECT_SACCADE_START'
                        print('state = DETECT_SACCADE_START')
                    if state == 'DETECT_SACCADE_START':
                        # self.present_tgt_and_pd_tgt()
                        eye_dist_from_start_tgt = np.sqrt((self.start_x-self.eye_x)**2 + (self.start_y-self.eye_y)**2)
                        if eye_speed >= fsm_parameter['sac_detect_threshold']:         
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_saccade'].append(self.t)
                            state = 'SACCADE'                 
                        # If eye moves away from start target, reset trial after punishment period
                        elif eye_dist_from_start_tgt > fsm_parameter['rew_area']/2:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_incorrect_saccade'].append(self.t)
                            state = 'INCORRECT_SACCADE'
                            # self.fsm_to_gui_sndr.send(('pun_beep',0))
                            self.fsm_to_screen_sndr.send(('all','rm')) # remove all targets
                            
                        # If time runs out before saccade detected, play punishment sound and reset the trial
                        elif (self.t - state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            ######
                            # lib.playSound(200,0.1) # punishment beep
                            ######
                            self.fsm_to_screen_sndr.send(('all','rm'))
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            state = 'STR_TARGET_PURSUIT'
                            self.pd_state_start_time = self.t
                            self.pd_state = 'PD_ON'

                    if state == 'SACCADE':
                        # self.present_tgt_and_pd_tgt()
                        # Check to see if saccade is in the right direction
                        target_dir_vector = [self.cue_x-self.start_x,self.cue_y-self.start_y]
                        unit_target_dir_vector = target_dir_vector/np.linalg.norm(target_dir_vector)
                        ######
                        # saccade_dir_vector = [self.eye_x-self.start_x,self.eye_y-self.start_y]
                        saccade_dir_vector = eye_vel
                        ######
                        unit_saccade_dir_vector = saccade_dir_vector/np.linalg.norm(saccade_dir_vector)                    
                        angle_diff = np.arccos(np.dot(unit_target_dir_vector, unit_saccade_dir_vector))

                        if angle_diff >= np.pi/2:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_incorrect_saccade'].append(self.t)
                            state = 'INCORRECT_SACCADE'
                            # self.fsm_to_gui_sndr.send(('pun_beep',0))

                            # lib.playSound(200,0.1) # punishment beep
                        else:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_detect_sac_end'].append(self.t)
                            state = 'DETECT_SACCADE_END'
                            print('state = DETECT_SACCADE_END')
                    if state == 'DETECT_SACCADE_END':
                        # self.present_tgt_and_pd_tgt()

                        if (eye_speed < fsm_parameter['sac_on_off_threshold']) and (self.t-state_start_time > 0.005):#25):
                            # Check if saccade made to cue
                              eye_dist_from_tgt = np.sqrt((self.tgt_x-self.eye_x)**2 + (self.tgt_y-self.eye_y)**2)
                              print(eye_dist_from_tgt)
                              if eye_dist_from_tgt < fsm_parameter['rew_area']/2:
                                  state_start_time = self.t
                                  state_inter_time = self.t
                                  self.trial_data['state_start_t_deliver_rew'].append(self.t)
                                  state = 'DELIVER_REWARD'
                                  print('state = DELIVER_REWARD')
                              else:
                                  state_start_time = self.t
                                  state_inter_time = self.t
                                  self.trial_data['state_start_t_incorrect_saccade'].append(self.t)
                                  state = 'INCORRECT_SACCADE'
                                  # self.fsm_to_gui_sndr.send(('pun_beep',0))
                                  # lib.playSound(200,0.1) # punishment beep
                          # If time runs out before saccade detected, reset the trial
                        elif (self.t - state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            self.fsm_to_screen_sndr.send(('all','rm')) # remove all targets
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            state = 'STR_TARGET_PURSUIT'
                            self.pd_state_start_time = self.t
                            self.pd_state = 'PD_ON'                          
                    if state == 'DELIVER_REWARD':
                        # self.present_tgt_and_pd_tgt()
                        if (trial_num % fsm_parameter['pump_switch_interval']) == 0:
                            if pump_to_use == 1:
                                pump_to_use = 2
                            else:
                                pump_to_use = 1
                        self.fsm_to_gui_sndr.send(('pump_' + str(pump_to_use),0))
                                                
                        lib.playSound(2000,0.1) # reward beep
                        state_start_time = self.t
                        state_inter_time = self.t
                        self.trial_data['state_start_t_end_tgt_fixation'].append(self.t)
                        state = 'END_TARGET_FIXATION'     
                        print('state = END_TARGET_FIXATION')
                    if state == 'END_TARGET_FIXATION':
                        # self.present_tgt_and_pd_tgt()
                        eye_dist_from_tgt = np.sqrt((self.tgt_x-self.eye_x)**2 + (self.tgt_y-self.eye_y)**2)
    
                        if ((self.t - state_inter_time) >= fsm_parameter['min_fix_time']):
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_trial_success'].append(self.t)
                            state = 'TRIAL_SUCCESS'
                            print('state = TRIAL_SUCCESS')
                        # If time runs out before fixation finished, reset the trial
                        elif (self.t-state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            self.fsm_to_screen_sndr.send(('all','rm')) # remove all targets
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            state = 'STR_TARGET_PURSUIT'
                            pd_state_start_time = self.t
                            pd_state = 'PD_ON'                       
                    if state == 'INCORRECT_SACCADE':
                        if ((self.t - state_start_time) > fsm_parameter['pun_time']):
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            state = 'STR_TARGET_PURSUIT'
                            pd_state_start_time = self.t
                            pd_state = 'PD_ON'
                    if state == 'TRIAL_SUCCESS':
                        self.fsm_to_screen_sndr.send(('all','rm')) # remove all targets
                        if (self.t-state_start_time) > fsm_parameter['ITI']:
                            # Pull data
                            self.pull_data_t = self.t
                            self.pull_data()
                            # Send trial data to GUI
                            self.fsm_to_gui_sndr.send(('trial_data',trial_num, self.trial_data))
                            trial_num += 1
                            # while True:
                            #     if time.perf_counter() - sys_t >= 0.1:
                            #         break
                            # break # break the FSM of the current trial and move to the next trial
                            # Init. trial variables; reset every trial
                            self.init_trial_data()  
                            self.trial_data['cal_matrix'] = cal_parameter['cal_matrix']
                            state = 'INIT'   
                    # Append data 
                    self.trial_data['tgt_time_data'].append(self.t)
                    self.trial_data['tgt_x_data'].append(self.tgt_x)
                    self.trial_data['tgt_y_data'].append(self.tgt_y)
                    self.trial_data['eye_x_data'].append(self.eye_x)
                    self.trial_data['eye_y_data'].append(self.eye_y)
                    # Update shared real time data
                    with self.real_time_data_Array.get_lock():
                        self.real_time_data_Array[0] = self.t
                        self.real_time_data_Array[1] = self.eye_x
                        self.real_time_data_Array[2] = self.eye_y
                        self.real_time_data_Array[3] = self.tgt_x
                        self.real_time_data_Array[4] = self.tgt_y
       
        # # Save current trial data before finishing
        # self.fsm_to_gui_sndr.send(('trial_data',trial_num, self.trial_data))
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
        # Signal successful exit
        self.fsm_to_gui_sndr.send(1) 
        print('exiting process')
    def present_tgt_and_pd_tgt(self):
        '''
        call at each FSM state whenever target and photodiode target
        need to be displayed, usually after setting target positions.
        Photodiode target will erase itself after a specified period and 
        will be drawn again after 'self.pd_state' is set to 'PD_ON'
        '''
        if self.pd_state == 'PD_ON':
            self.fsm_to_screen_sndr.send(('all','draw',(self.tgt_x,self.tgt_y)))
            # Display PD target for 100 ms. If duration is too short, 
            # PD tgt may not become visible
            if self.t - self.pd_state_start_time > 0.1: 
                self.pd_state = 'PD_OFF'
                self.fsm_to_screen_sndr.send(('pd_tgt','rm'))
        else:
            self.fsm_to_screen_sndr.send(('tgt','draw',(self.tgt_x,self.tgt_y)))
 
    def pull_data(self):
        '''
        to be called every 10 s or when a trial finishes, whichever is earlier
        and flush the data from VPixx. Reason for this is to prevent the data
        from accumulating, which will incur a delay when getting data 
        '''
        print('pull data')
        tpxData = TPxReadTPxData(0)
        self.trial_data['vpixx_time_data'].extend(tpxData[0][0::22])
        self.trial_data['eye_lx_raw_data'].extend(tpxData[0][16::22])
        self.trial_data['eye_ly_raw_data'].extend(tpxData[0][17::22])
        self.trial_data['eye_l_pupil_data'].extend(tpxData[0][3::22])
        self.trial_data['eye_l_blink_data'].extend(tpxData[0][8::22])
        self.trial_data['eye_rx_raw_data'].extend(tpxData[0][18::22])
        self.trial_data['eye_ry_raw_data'].extend(tpxData[0][19::22])
        self.trial_data['eye_r_pupil_data'].extend(tpxData[0][6::22])
        self.trial_data['eye_r_blink_data'].extend(tpxData[0][9::22])
        self.trial_data['din_data'].extend(tpxData[0][7::22])
        self.trial_data['dout_data'].extend(tpxData[0][10::22])

        TPxSetupTPxSchedule() # flushes data
        print('finished pulling data')
    def init_trial_data(self):
        '''
        initializes a dict. of trial data;
        needs to be called at the start of every trial
        '''
        self.trial_data = {}
        self.trial_data['cal_matrix'] = [] # may be updated during exp.
        self.trial_data['state_start_t_str_tgt_pursuit'] = []
        self.trial_data['state_start_t_str_tgt_present'] = []
        self.trial_data['state_start_t_str_tgt_fixation'] = []
        self.trial_data['state_start_t_cue_tgt_present'] = []
        self.trial_data['state_start_t_detect_sac_start'] = []
        self.trial_data['state_start_t_saccade'] = []
        self.trial_data['state_start_t_detect_sac_end'] = []
        self.trial_data['state_start_t_deliver_rew'] = []
        self.trial_data['state_start_t_end_tgt_fixation'] = []
        self.trial_data['state_start_t_trial_success'] = []
        self.trial_data['state_start_t_incorrect_saccade'] = []
        self.trial_data['cue_x'] = []
        self.trial_data['cue_y'] = []       
        self.trial_data['start_x'] = []
        self.trial_data['start_y'] = []     
        self.trial_data['tgt_time_data'] = []
        self.trial_data['tgt_x_data'] = []
        self.trial_data['tgt_y_data'] = []
        self.trial_data['eye_x_data'] = []
        self.trial_data['eye_y_data'] = []
         # 2000 Hz data
        self.trial_data['eye_lx_raw_data'] = []
        self.trial_data['eye_ly_raw_data'] = []
        self.trial_data['eye_l_pupil_data'] = []
        self.trial_data['eye_l_blink_data'] = []
        self.trial_data['eye_rx_raw_data'] = []
        self.trial_data['eye_ry_raw_data'] = []
        self.trial_data['eye_r_pupil_data'] = []
        self.trial_data['eye_r_blink_data'] = []
        self.trial_data['vpixx_time_data'] = []
        self.trial_data['din_data'] = []
        self.trial_data['dout_data'] = []
        
class SimpleSacGui(FsmGui):
    def __init__(self,exp_name, fsm_to_screen_sndr, fsm_to_gui_rcvr, gui_to_fsm_sndr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array):        
        # import faulthandler
        # faulthandler.disable()
        # faulthandler.enable()
        
        self.fsm_to_screen_sndr = fsm_to_screen_sndr
        self.fsm_to_gui_rcvr = fsm_to_gui_rcvr
        self.gui_to_fsm_sndr = gui_to_fsm_sndr
        self.stop_exp_Event = stop_exp_Event
        self.stop_fsm_process_Event = stop_fsm_process_Event
        self.real_time_data_Array = real_time_data_Array
        self.exp_name = exp_name
        super(SimpleSacGui,self).__init__(self.fsm_to_screen_sndr, self.stop_fsm_process_Event)      
        self.init_gui()
        # Init. var
        self.eye_x = 0
        self.eye_y = 0
        self.tgt_x = 0
        self.tgt_y = 0
        
        # Load exp. parameter or set default values
        self.exp_parameter, self.parameter_file_path = lib.load_parameter('experiment','exp_parameter.json',True,self.set_default_exp_parameter,self.exp_name)
        self.update_exp_parameter()
        # Load calibration 
        self.cal_parameter, _ = lib.load_parameter('calibration','cal_parameter.json',False,lib.set_default_cal_parameter)
        if not self.cal_parameter['cal_status']:
            self.toolbar_run_QAction.setDisabled(True)
            self.log_QPlainTextEdit.appendPlainText('No calibration found. Please calibrate first.')
        
        self.data_manager = DataManager()
        self.init_signals()
    #%% SIGNALS
    def init_signals(self):
        self.data_QTimer.timeout.connect(self.data_QTimer_timeout)
        self.data_manager.signals.to_main_thread.connect(self.receive_signal)
        # Toolbar
        self.toolbar_run_QAction.triggered.connect(self.toolbar_run_QAction_triggered)
        self.toolbar_stop_QAction.triggered.connect(self.toolbar_stop_QAction_triggered)
        # Sidepanel
        self.horz_offset_QDoubleSpinBox.valueChanged.connect(self.horz_offset_QDoubleSpinBox_valueChanged)
        self.vert_offset_QDoubleSpinBox.valueChanged.connect(self.vert_offset_QDoubleSpinBox_valueChanged)
        self.max_allow_time_QDoubleSpinBox.valueChanged.connect(self.max_allow_time_QDoubleSpinBox_valueChanged)
        self.min_fix_time_QDoubleSpinBox.valueChanged.connect(self.min_fix_time_QDoubleSpinBox_valueChanged)
        self.max_wait_fixation_QDoubleSpinBox.valueChanged.connect(self.max_wait_fixation_QDoubleSpinBox_valueChanged)
        self.pun_time_QDoubleSpinBox.valueChanged.connect(self.pun_time_QDoubleSpinBox_valueChanged)
        self.time_to_reward_QDoubleSpinBox.valueChanged.connect(self.time_to_reward_QDoubleSpinBox_valueChanged)
        self.sac_detect_threshold_QDoubleSpinBox.valueChanged.connect(self.sac_detect_threshold_QDoubleSpinBox_valueChanged)
        self.sac_on_off_threshold_QDoubleSpinBox.valueChanged.connect(self.sac_on_off_threshold_QDoubleSpinBox_valueChanged)
        self.rew_area_QDoubleSpinBox.valueChanged.connect(self.rew_area_QDoubleSpinBox_valueChanged)
        self.pursuit_amp_QDoubleSpinBox.valueChanged.connect(self.pursuit_amp_QDoubleSpinBox_valueChanged)
        self.pursuit_dur_QDoubleSpinBox.valueChanged.connect(self.pursuit_dur_QDoubleSpinBox_valueChanged)
        self.prim_sac_amp_QDoubleSpinBox.valueChanged.connect(self.prim_sac_amp_QDoubleSpinBox_valueChanged)
        self.num_sac_dir_QDoubleSpinBox.valueChanged.connect(self.num_sac_dir_QDoubleSpinBox_valueChanged)
        self.first_dir_QDoubleSpinBox.valueChanged.connect(self.first_dir_QDoubleSpinBox_valueChanged)
        self.iti_QDoubleSpinBox.valueChanged.connect(self.iti_QDoubleSpinBox_valueChanged)
        self.pump_switch_QDoubleSpinBox.valueChanged.connect(self.pump_switch_QDoubleSpinBox_valueChanged)
        self.save_QPushButton.clicked.connect(self.save_QPushButton_clicked)
    
    #%% SLOTS
    @pyqtSlot()
    def toolbar_run_QAction_triggered(self):
        self.toolbar_run_QAction.setDisabled(True)
        self.toolbar_stop_QAction.setEnabled(True)
        # Start FSM
        self.stop_exp_Event.clear()
              
        # Disable some user functions
        self.sidepanel_parameter_QWidget.setDisabled(True)
        self.tgt.setDisabled(True)
        self.pd_tgt.setDisabled(True)
        # Save parameters
        self.save_QPushButton_clicked()

        # Reset data
        self.eye_x_data.clear()
        self.eye_y_data.clear()
        self.tgt_x_data.clear()
        self.tgt_y_data.clear()
        # print(self.t_data)
        self.t_data.clear()
        print(self.t_data)
        # Init. data       
        self.data_manager.init_data(self.exp_name, self.exp_parameter)
  
        # Start timer to get data from FSM
        self.data_QTimer.start(self.data_rate)
    @pyqtSlot()
    def toolbar_stop_QAction_triggered(self):
        self.toolbar_run_QAction.setEnabled(True)
        self.toolbar_stop_QAction.setDisabled(True)
        # Enable some user functions
        self.sidepanel_parameter_QWidget.setEnabled(True)
        self.tgt.setEnabled(True)
        self.pd_tgt.setEnabled(True)
        # Stop timer to stop getting data from fsm thread
        self.data_QTimer.stop()
        # Ask FSM to stop
        self.stop_exp_Event.set()
    @pyqtSlot()
    def data_QTimer_timeout(self):
        '''
        getting data from fsm thread
        '''
        if self.fsm_to_gui_rcvr.poll():
            message = self.fsm_to_gui_rcvr.recv()
            message_title = message[0]
            
            if message_title == 'tgt_data':
                cue_x = message[1][0]
                cue_y = message[1][1]
                self.plot_1_cue.setData([cue_x],[cue_y])
            if message_title == 'trial_data':
                self.data_manager.trial_num = message[1]
                self.data_manager.trial_data = message[2]
                self.data_manager.save_data()
            if message_title == 'pump_1':
                self.pump_1.pump_once_QPushButton_clicked()  
            if message_title == 'pump_2':
                self.pump_2.pump_once_QPushButton_clicked() 
            if message_title == 'log':
                self.log_QPlainTextEdit.appendPlainText(message[1])
            # if message_title == 'neutral_beep':
            #     # self.neutral_beep.play()
            #     self.neutral_beep_play()
            # if message_title == 'pun_beep':
            #     # self.pun_beep.play()
            #     self.pun_beep_play()
            # if message_title == 'reward_beep':
            #     # self.reward_beep.play()
            #     self.reward_beep_play()
            ######
            if message_title == 'error':
                print('error received')
                print(message[1])
        
        with self.real_time_data_Array.get_lock():
            t = self.real_time_data_Array[0]
            self.eye_x = self.real_time_data_Array[1]
            self.eye_y = self.real_time_data_Array[2]
            self.tgt_x = self.real_time_data_Array[3]
            self.tgt_y = self.real_time_data_Array[4]
        
        self.eye_x_data.append(self.eye_x)
        self.eye_y_data.append(self.eye_y)
        self.tgt_x_data.append(self.tgt_x)
        self.tgt_y_data.append(self.tgt_y)
        self.t_data.append(t)
        # Plot data
        self.plot_data()
        # self.plot_1_eye.setData([eye_x],[eye_y])
        # self.plot_1_tgt.setData([tgt_x],[tgt_y])
        # self.plot_2_data()
        # self.plot_2_eye_x.setData(self.t_data,self.eye_x_data)
        # self.plot_2_eye_y.setData(self.t_data,self.eye_y_data)
        # self.plot_2_tgt_x.setData(self.t_data,self.tgt_x_data)
        # self.plot_2_tgt_y.setData(self.t_data,self.tgt_y_data)
    
    def plot_data(self):
        self.plot_1_eye.setData([self.eye_x],[self.eye_y])
        self.plot_1_tgt.setData([self.tgt_x],[self.tgt_y])
        self.plot_2_eye_x.setData(self.t_data,self.eye_x_data)
        self.plot_2_eye_y.setData(self.t_data,self.eye_y_data)
        self.plot_2_tgt_x.setData(self.t_data,self.tgt_x_data)
        self.plot_2_tgt_y.setData(self.t_data,self.tgt_y_data)
        
    def neutral_beep_play(self):
        self.neutral_beep.play()
    def pun_beep_play(self):
        self.pun_beep.play()
    def reward_beep_play(self):
        self.reward_beep.play()
    
    @pyqtSlot(object)
    def receive_signal(self, signal):
        message = signal[0]
        if message == 'log':
            self.log_QPlainTextEdit.appendPlainText(signal[1])
    @pyqtSlot()
    def horz_offset_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['horz_offset'] = self.horz_offset_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')     
    @pyqtSlot()
    def vert_offset_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['vert_offset'] = self.vert_offset_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
    @pyqtSlot()
    def max_allow_time_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['max_allow_time'] = self.max_allow_time_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')     
    @pyqtSlot()
    def min_fix_time_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['min_fix_time'] = self.min_fix_time_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
    @pyqtSlot()
    def max_wait_fixation_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['max_wait_for_fixation'] = self.max_wait_fixation_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')     
    @pyqtSlot()
    def pun_time_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['pun_time'] = self.pun_time_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
    @pyqtSlot()
    def time_to_reward_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['time_to_reward'] = self.time_to_reward_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')     
    @pyqtSlot()
    def sac_detect_threshold_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['sac_detect_threshold'] = self.sac_detect_threshold_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
    @pyqtSlot()
    def sac_on_off_threshold_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['sac_on_off_threshold'] = self.sac_on_off_threshold_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')     
    @pyqtSlot()
    def rew_area_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['rew_area'] = self.rew_area_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
    @pyqtSlot()
    def pursuit_amp_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['pursuit_amp'] = self.pursuit_amp_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')     
    @pyqtSlot()
    def pursuit_dur_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['pursuit_dur'] = self.pursuit_dur_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
    @pyqtSlot()
    def prim_sac_amp_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['prim_sac_amp'] = self.prim_sac_amp_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
    @pyqtSlot()
    def num_sac_dir_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['num_prim_sac_dir'] = int(self.num_sac_dir_QDoubleSpinBox.value())
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
    @pyqtSlot()
    def first_dir_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['first_prim_sac_dir'] = int(self.first_dir_QDoubleSpinBox.value())
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
        
    @pyqtSlot()
    def iti_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['ITI'] = self.iti_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
        
    @pyqtSlot()
    def pump_switch_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['pump_switch_interval'] = self.pump_switch_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
    @pyqtSlot()
    def save_QPushButton_clicked(self):
        with open(self.parameter_file_path,'r') as file:
            all_parameter = json.load(file)
        all_parameter[self.exp_name] = self.exp_parameter    
        with open(self.parameter_file_path,'w') as file:
            json.dump(all_parameter, file, indent=4)
        self.save_QPushButton.setStyleSheet('background-color: #39E547')  
    #%% GUI
    def init_gui(self):
        # Customiaze plots
        self.plot_1_cue = self.plot_1_PlotWidget.plot(np.zeros((0)), np.zeros((0)), pen = None,\
            symbolBrush=None, symbolPen='g', symbol='+',symbolSize=14,name='cue') 
        self.plot_1_tgt = self.plot_1_PlotWidget.\
            plot(np.zeros((0)), np.zeros((0)), pen = None,\
            symbolBrush=None, symbolPen='b', symbol='+',symbolSize=14,name='tgt') 
        self.plot_1_eye = self.plot_1_PlotWidget.\
            plot(np.zeros((0)), np.zeros((0)), pen = None,\
            symbolBrush='k', symbolPen='k', symbol='o',symbolSize=10,name='eye',connect='finite')         
        # Side panel
        self.horz_offset_QHBoxLayout = QHBoxLayout()
        self.horz_offset_QLabel = QLabel('Horizontal offset (deg):')
        self.horz_offset_QHBoxLayout.addWidget(self.horz_offset_QLabel)
        self.horz_offset_QLabel.setAlignment(Qt.AlignRight)
        self.horz_offset_QDoubleSpinBox = QDoubleSpinBox()
        self.horz_offset_QDoubleSpinBox.setValue(0)
        self.horz_offset_QDoubleSpinBox.setMinimum(-50)
        self.horz_offset_QDoubleSpinBox.setMaximum(50)
        self.horz_offset_QDoubleSpinBox.setDecimals(1)
        self.horz_offset_QDoubleSpinBox.setSingleStep(0.1)
        self.horz_offset_QHBoxLayout.addWidget(self.horz_offset_QDoubleSpinBox)       
        self.sidepanel_custom_QVBoxLayout.addLayout(self.horz_offset_QHBoxLayout)
        
        self.vert_offset_QHBoxLayout = QHBoxLayout()
        self.vert_offset_QLabel = QLabel('Vertical offset (deg):')
        self.vert_offset_QLabel.setAlignment(Qt.AlignRight)
        self.vert_offset_QHBoxLayout.addWidget(self.vert_offset_QLabel)
        self.vert_offset_QDoubleSpinBox = QDoubleSpinBox()
        self.vert_offset_QDoubleSpinBox.setValue(0)
        self.vert_offset_QDoubleSpinBox.setMinimum(-50)
        self.vert_offset_QDoubleSpinBox.setMaximum(50)
        self.vert_offset_QDoubleSpinBox.setDecimals(1)
        self.vert_offset_QDoubleSpinBox.setSingleStep(0.1)
        self.vert_offset_QHBoxLayout.addWidget(self.vert_offset_QDoubleSpinBox)       
        self.sidepanel_custom_QVBoxLayout.addLayout(self.vert_offset_QHBoxLayout)
        
        self.max_allow_time_QHBoxLayout = QHBoxLayout()
        self.max_allow_time_QLabel = QLabel("Max. allowed time outside target (s):")
        self.max_allow_time_QLabel.setAlignment(Qt.AlignRight)
        self.max_allow_time_QHBoxLayout.addWidget(self.max_allow_time_QLabel)
        self.max_allow_time_QDoubleSpinBox = QDoubleSpinBox()
        self.max_allow_time_QDoubleSpinBox.setValue(0.7)
        self.max_allow_time_QDoubleSpinBox.setMaximum(10)
        self.max_allow_time_QDoubleSpinBox.setSingleStep(0.1)
        self.max_allow_time_QDoubleSpinBox.setDecimals(1)
        self.max_allow_time_QHBoxLayout.addWidget(self.max_allow_time_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.max_allow_time_QHBoxLayout)
        
        self.min_fix_time_QHBoxLayout = QHBoxLayout()
        self.min_fix_time_QLabel = QLabel("Minimum fixation time (s):")
        self.min_fix_time_QLabel.setAlignment(Qt.AlignRight)
        self.min_fix_time_QHBoxLayout.addWidget(self.min_fix_time_QLabel)
        self.min_fix_time_QDoubleSpinBox = QDoubleSpinBox()
        self.min_fix_time_QDoubleSpinBox.setValue(0.1)
        self.min_fix_time_QDoubleSpinBox.setMaximum(10)
        self.min_fix_time_QDoubleSpinBox.setSingleStep(0.1)
        self.min_fix_time_QDoubleSpinBox.setDecimals(1)
        self.min_fix_time_QHBoxLayout.addWidget(self.min_fix_time_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.min_fix_time_QHBoxLayout)
        
        self.max_wait_fixation_QHBoxLayout = QHBoxLayout()
        self.max_wait_fixation_QLabel = QLabel("Maximum wait for fixation (s):")
        self.max_wait_fixation_QLabel.setAlignment(Qt.AlignRight)
        self.max_wait_fixation_QHBoxLayout.addWidget(self.max_wait_fixation_QLabel)
        self.max_wait_fixation_QDoubleSpinBox = QDoubleSpinBox()
        self.max_wait_fixation_QDoubleSpinBox.setValue(1.5)
        self.max_wait_fixation_QDoubleSpinBox.setMaximum(10)
        self.max_wait_fixation_QDoubleSpinBox.setSingleStep(0.1)
        self.max_wait_fixation_QDoubleSpinBox.setDecimals(1)
        self.max_wait_fixation_QHBoxLayout.addWidget(self.max_wait_fixation_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.max_wait_fixation_QHBoxLayout)
        
        self.pun_time_QHBoxLayout = QHBoxLayout()
        self.pun_time_QLabel = QLabel("Punishment time (s):")
        self.pun_time_QLabel.setAlignment(Qt.AlignRight)
        self.pun_time_QHBoxLayout.addWidget(self.pun_time_QLabel)
        self.pun_time_QDoubleSpinBox = QDoubleSpinBox()
        self.pun_time_QDoubleSpinBox.setValue(0.1)
        self.pun_time_QDoubleSpinBox.setMaximum(10)
        self.pun_time_QDoubleSpinBox.setSingleStep(0.1)
        self.pun_time_QDoubleSpinBox.setDecimals(1)
        self.pun_time_QHBoxLayout.addWidget(self.pun_time_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.pun_time_QHBoxLayout)
        
        self.time_to_reward_QHBoxLayout = QHBoxLayout()
        self.time_to_reward_QLabel = QLabel("Time to reward (s):")
        self.time_to_reward_QLabel.setAlignment(Qt.AlignRight)
        self.time_to_reward_QHBoxLayout.addWidget(self.time_to_reward_QLabel)
        self.time_to_reward_QDoubleSpinBox = QDoubleSpinBox()
        self.time_to_reward_QDoubleSpinBox.setValue(0.1)
        self.time_to_reward_QDoubleSpinBox.setMaximum(10)
        self.time_to_reward_QDoubleSpinBox.setSingleStep(0.1)
        self.time_to_reward_QDoubleSpinBox.setDecimals(1)
        self.time_to_reward_QHBoxLayout.addWidget(self.time_to_reward_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.time_to_reward_QHBoxLayout)
        
        self.sac_detect_threshold_QHBoxLayout = QHBoxLayout()
        self.sac_detect_threshold_QLabel = QLabel("Saccade detection threshold (deg/s):")
        self.sac_detect_threshold_QLabel.setAlignment(Qt.AlignRight)
        self.sac_detect_threshold_QHBoxLayout.addWidget(self.sac_detect_threshold_QLabel)
        self.sac_detect_threshold_QDoubleSpinBox = QDoubleSpinBox()
        self.sac_detect_threshold_QDoubleSpinBox.setValue(150)
        self.sac_detect_threshold_QDoubleSpinBox.setMaximum(1000)
        self.sac_detect_threshold_QDoubleSpinBox.setSingleStep(5)
        self.sac_detect_threshold_QDoubleSpinBox.setDecimals(0)
        self.sac_detect_threshold_QHBoxLayout.addWidget(self.sac_detect_threshold_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.sac_detect_threshold_QHBoxLayout)
        
        self.sac_on_off_threshold_QHBoxLayout = QHBoxLayout()
        self.sac_on_off_threshold_QLabel = QLabel("Saccade onset/offset threshold (deg/s):")
        self.sac_on_off_threshold_QLabel.setAlignment(Qt.AlignRight)
        self.sac_on_off_threshold_QHBoxLayout.addWidget(self.sac_on_off_threshold_QLabel)
        self.sac_on_off_threshold_QDoubleSpinBox = QDoubleSpinBox()
        self.sac_on_off_threshold_QDoubleSpinBox.setValue(75)
        self.sac_on_off_threshold_QDoubleSpinBox.setMaximum(1000)
        self.sac_on_off_threshold_QDoubleSpinBox.setSingleStep(5)
        self.sac_on_off_threshold_QDoubleSpinBox.setDecimals(0)
        self.sac_on_off_threshold_QHBoxLayout.addWidget(self.sac_on_off_threshold_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.sac_on_off_threshold_QHBoxLayout)
        
        self.rew_area_QHBoxLayout = QHBoxLayout()
        self.rew_area_QLabel = QLabel("Reward area (deg):")
        self.rew_area_QLabel.setAlignment(Qt.AlignRight)
        self.rew_area_QHBoxLayout.addWidget(self.rew_area_QLabel)
        self.rew_area_QDoubleSpinBox = QDoubleSpinBox()
        self.rew_area_QDoubleSpinBox.setValue(3.0)
        self.rew_area_QDoubleSpinBox.setMaximum(20)
        self.rew_area_QDoubleSpinBox.setSingleStep(0.1)
        self.rew_area_QDoubleSpinBox.setDecimals(1)
        self.rew_area_QHBoxLayout.addWidget(self.rew_area_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.rew_area_QHBoxLayout)
        
        self.pursuit_amp_QHBoxLayout = QHBoxLayout()
        self.pursuit_amp_QLabel = QLabel("Pursuit amp. (deg):")
        self.pursuit_amp_QLabel.setAlignment(Qt.AlignRight)
        self.pursuit_amp_QHBoxLayout.addWidget(self.pursuit_amp_QLabel)
        self.pursuit_amp_QDoubleSpinBox = QDoubleSpinBox()
        self.pursuit_amp_QDoubleSpinBox.setValue(0.1)
        self.pursuit_amp_QDoubleSpinBox.setMaximum(20)
        self.pursuit_amp_QDoubleSpinBox.setSingleStep(0.1)
        self.pursuit_amp_QDoubleSpinBox.setDecimals(1)
        self.pursuit_amp_QHBoxLayout.addWidget(self.pursuit_amp_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.pursuit_amp_QHBoxLayout)
        
        self.pursuit_dur_QHBoxLayout = QHBoxLayout()
        self.pursuit_dur_QLabel = QLabel("Pursuit duration (s):")
        self.pursuit_dur_QLabel.setAlignment(Qt.AlignRight)
        self.pursuit_dur_QHBoxLayout.addWidget(self.pursuit_dur_QLabel)
        self.pursuit_dur_QDoubleSpinBox = QDoubleSpinBox()
        self.pursuit_dur_QDoubleSpinBox.setValue(0.1)
        self.pursuit_dur_QDoubleSpinBox.setMaximum(20)
        self.pursuit_dur_QDoubleSpinBox.setSingleStep(0.1)
        self.pursuit_dur_QDoubleSpinBox.setDecimals(1)
        self.pursuit_dur_QHBoxLayout.addWidget(self.pursuit_dur_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.pursuit_dur_QHBoxLayout)
        
        self.prim_sac_amp_QHBoxLayout = QHBoxLayout()
        self.prim_sac_amp_QLabel = QLabel("Primary saccade amp. (deg):")
        self.prim_sac_amp_QLabel.setAlignment(Qt.AlignRight)
        self.prim_sac_amp_QHBoxLayout.addWidget(self.prim_sac_amp_QLabel)
        self.prim_sac_amp_QDoubleSpinBox = QDoubleSpinBox()
        self.prim_sac_amp_QDoubleSpinBox.setValue(4.0)
        self.prim_sac_amp_QDoubleSpinBox.setMaximum(20)
        self.prim_sac_amp_QDoubleSpinBox.setSingleStep(0.1)
        self.prim_sac_amp_QDoubleSpinBox.setDecimals(1)
        self.prim_sac_amp_QHBoxLayout.addWidget(self.prim_sac_amp_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.prim_sac_amp_QHBoxLayout)
        
        self.num_sac_dir_QHBoxLayout = QHBoxLayout()
        self.num_sac_dir_QLabel = QLabel("Number of sac. direction:")
        self.num_sac_dir_QLabel.setAlignment(Qt.AlignRight)
        self.num_sac_dir_QHBoxLayout.addWidget(self.num_sac_dir_QLabel)
        self.num_sac_dir_QDoubleSpinBox = QDoubleSpinBox()
        self.num_sac_dir_QDoubleSpinBox.setValue(8)
        self.num_sac_dir_QDoubleSpinBox.setMaximum(20)
        self.num_sac_dir_QDoubleSpinBox.setMinimum(1)
        self.num_sac_dir_QDoubleSpinBox.setSingleStep(1)
        self.num_sac_dir_QDoubleSpinBox.setDecimals(0)
        self.num_sac_dir_QHBoxLayout.addWidget(self.num_sac_dir_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.num_sac_dir_QHBoxLayout)
        
        self.first_dir_QHBoxLayout = QHBoxLayout()
        self.first_dir_QLabel = QLabel("1st direction (deg):")
        self.first_dir_QLabel.setAlignment(Qt.AlignRight)
        self.first_dir_QHBoxLayout.addWidget(self.first_dir_QLabel)
        self.first_dir_QDoubleSpinBox = QDoubleSpinBox()
        self.first_dir_QDoubleSpinBox.setToolTip('Specifies starting direction of uniformly distributed target positions')
        self.first_dir_QDoubleSpinBox.setValue(0)
        self.first_dir_QDoubleSpinBox.setMaximum(359)
        self.first_dir_QDoubleSpinBox.setSingleStep(1)
        self.first_dir_QDoubleSpinBox.setDecimals(0)
        self.first_dir_QHBoxLayout.addWidget(self.first_dir_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.first_dir_QHBoxLayout)
        
        self.iti_QHBoxLayout = QHBoxLayout()
        self.iti_QLabel = QLabel("ITI (s):")
        self.iti_QLabel.setAlignment(Qt.AlignRight)
        self.iti_QHBoxLayout.addWidget(self.iti_QLabel)
        self.iti_QDoubleSpinBox = QDoubleSpinBox()
        self.iti_QDoubleSpinBox.setValue(0.1)
        self.iti_QDoubleSpinBox.setMaximum(20)
        self.iti_QDoubleSpinBox.setSingleStep(0.1)
        self.iti_QDoubleSpinBox.setDecimals(1)
        self.iti_QHBoxLayout.addWidget(self.iti_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.iti_QHBoxLayout)
        
        self.pump_switch_QHBoxLayout = QHBoxLayout()
        self.pump_switch_QLabel = QLabel("Pump switch interval (trials):")
        self.pump_switch_QLabel.setAlignment(Qt.AlignRight)
        self.pump_switch_QHBoxLayout.addWidget(self.pump_switch_QLabel)
        self.pump_switch_QDoubleSpinBox = QDoubleSpinBox()
        self.pump_switch_QDoubleSpinBox.setToolTip('After how many trials, pump should switch')
        self.pump_switch_QDoubleSpinBox.setValue(50)
        self.pump_switch_QDoubleSpinBox.setMaximum(9999)
        self.pump_switch_QDoubleSpinBox.setSingleStep(1)
        self.pump_switch_QDoubleSpinBox.setDecimals(0)
        self.pump_switch_QHBoxLayout.addWidget(self.pump_switch_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.pump_switch_QHBoxLayout)
        
        self.save_QPushButton = QPushButton('Save parameters')
        self.sidepanel_custom_QVBoxLayout.addWidget(self.save_QPushButton)
    #%% FUNCTIONS    
    def set_default_exp_parameter(self):
        exp_parameter = {'horz_offset':0.0,
                         'vert_offset':0.0,
                         'max_allow_time':0.7,
                         'min_fix_time':0.1,
                         'max_wait_for_fixation':1.5,
                         'pun_time':0.1,
                         'time_to_reward':0.1,
                         'sac_detect_threshold':150.0,
                         'sac_on_off_threshold':75.0,
                         'rew_area':3.0,
                         'pursuit_amp':0.1,
                         'pursuit_dur':0.1,
                         'prim_sac_amp':4.0,
                         'num_prim_sac_dir':8,
                         'first_prim_sac_dir': 0,
                         'ITI':0.1,
                         'pump_switch_interval':50}
        return exp_parameter
    
    def update_exp_parameter(self):
        '''
        update GUI parameters with the loaded parameters
        '''
        self.horz_offset_QDoubleSpinBox.setValue(self.exp_parameter['horz_offset'])
        self.vert_offset_QDoubleSpinBox.setValue(self.exp_parameter['vert_offset'])
        self.max_allow_time_QDoubleSpinBox.setValue(self.exp_parameter['max_allow_time'])
        self.min_fix_time_QDoubleSpinBox.setValue(self.exp_parameter['min_fix_time'])
        self.max_wait_fixation_QDoubleSpinBox.setValue(self.exp_parameter['max_wait_for_fixation'])
        self.pun_time_QDoubleSpinBox.setValue(self.exp_parameter['pun_time'])
        self.time_to_reward_QDoubleSpinBox.setValue(self.exp_parameter['time_to_reward'])
        self.sac_detect_threshold_QDoubleSpinBox.setValue(self.exp_parameter['sac_detect_threshold'])
        self.sac_on_off_threshold_QDoubleSpinBox.setValue(self.exp_parameter['sac_on_off_threshold'])
        self.rew_area_QDoubleSpinBox.setValue(self.exp_parameter['rew_area'])
        self.pursuit_amp_QDoubleSpinBox.setValue(self.exp_parameter['pursuit_amp'])
        self.pursuit_dur_QDoubleSpinBox.setValue(self.exp_parameter['pursuit_dur'])
        self.prim_sac_amp_QDoubleSpinBox.setValue(self.exp_parameter['prim_sac_amp'])
        self.num_sac_dir_QDoubleSpinBox.setValue(self.exp_parameter['num_prim_sac_dir'])
        self.first_dir_QDoubleSpinBox.setValue(self.exp_parameter['first_prim_sac_dir'])
        self.iti_QDoubleSpinBox.setValue(self.exp_parameter['ITI'])
        self.pump_switch_QDoubleSpinBox.setValue(self.exp_parameter['pump_switch_interval'])
        

        
class SimpleSacGuiProcess(multiprocessing.Process):
    def __init__(self, exp_name, fsm_to_screen_sndr, fsm_to_gui_rcvr, gui_to_fsm_sndr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array, parent=None):
        super(SimpleSacGuiProcess,self).__init__(parent)
        self.exp_name = exp_name
        self.fsm_to_screen_sndr = fsm_to_screen_sndr
        self.fsm_to_gui_rcvr = fsm_to_gui_rcvr
        self.gui_to_fsm_sndr = gui_to_fsm_sndr
        self.stop_exp_Event = stop_exp_Event
        self.real_time_data_Array = real_time_data_Array
        self.stop_fsm_process_Event = stop_fsm_process_Event
    def run(self):  
        try:
            app = QApplication(sys.argv)
            app_gui = SimpleSacGui(self.exp_name, self.fsm_to_screen_sndr, self.fsm_to_gui_rcvr, self.gui_to_fsm_sndr, self.stop_exp_Event, self.stop_fsm_process_Event, self.real_time_data_Array)
            app_gui.show()
            sys.exit(app.exec())
        except:
            traceback.print_exc()
        