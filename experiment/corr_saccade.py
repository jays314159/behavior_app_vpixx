"""
Laboratory for Computational Motor Control, Johns Hopkins School of Medicine
@author: Jay Pi <jay.s.314159@gmail.com>
"""
from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QApplication, QComboBox, QPushButton, QLabel, QHBoxLayout, QDoubleSpinBox, QCheckBox, QPlainTextEdit,\
                            QDialog, QShortcut, QTabWidget, QWidget, QVBoxLayout
from PyQt5.QtCore import QRunnable, QThreadPool, pyqtSignal, pyqtSlot, QObject, Qt, QTimer
from psychopy import monitors, visual, core

# VPixx related
from pypixxlib import tracker
from pypixxlib._libdpx import DPxOpen, TPxSetupTPxSchedule,TPxEnableFreeRun,DPxSelectDevice,DPxUpdateRegCache, DPxSetTPxAwake,\
                              TPxDisableFreeRun, DPxGetReg16,DPxGetTime,TPxBestPolyGetEyePosition, DPxSetDoutValue, TPxReadTPxData,\
                              DPxSetTPxSleep, DPxClose

from fsm_gui import FsmGui
from target import TargetWidget
import app_lib as lib
from data_manager import DataManager

import multiprocessing, sys, os, json, random, time, copy, ctypes, traceback, gc, zmq, math
sys.path.append('../app')
from pathlib import Path
import numpy as np
from collections import deque
from datetime import datetime

class CorrSacFsmProcess(multiprocessing.Process):
    def __init__(self,exp_name, fsm_to_gui_sndr, gui_to_fsm_Q, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array,main_parameter,mon_parameter):
        super().__init__()
        self.exp_name = exp_name
        self.fsm_to_gui_sndr = fsm_to_gui_sndr
        self.gui_to_fsm_Q = gui_to_fsm_Q
        self.stop_exp_Event = stop_exp_Event
        self.stop_fsm_process_Event = stop_fsm_process_Event
        self.real_time_data_Array = real_time_data_Array
        self.main_parameter = main_parameter
        self.mon_parameter = mon_parameter
        # Init var.
        self.eye_x = 0
        self.eye_y = 0
        self.tgt_x = 0
        self.tgt_y = 0
        self.start_x = 0
        self.start_y = 0
        self.cue_x = 0
        self.cue_y = 0
        self.t = math.nan
        self.pull_data_t = 0 # keep track of when data was pulled last from VPixx
    
    def run(self):
        # Set up exp. screen
        this_monitor = monitors.Monitor(self.mon_parameter['monitor_name'], width=self.mon_parameter['monitor_width'], distance=self.mon_parameter['monitor_distance'])
        this_monitor.save()
        this_monitor.setSizePix(self.mon_parameter['monitor_size'])
        self.window = visual.Window(size=self.mon_parameter['monitor_size'],screen=self.mon_parameter['monitor_num'], allowGUI=False, color='white', monitor=this_monitor,
                                units='deg', winType='pyglet', fullscr=True, checkTiming=False, waitBlanking=True)
        self.window.flip()
        
        # Make targets
        self.update_target()
        
        # Check if VPixx available; if so, open
        DPxOpen()
        tracker.TRACKPixx3().open() # this throws error if not device not open           
        DPxSetTPxAwake()
        DPxSelectDevice('DATAPIXX3')   
        DPxUpdateRegCache()
        
        # Get pointers to store data from device
        cal_data, raw_data = lib.VPixx_get_pointers_for_data()
             
        # Init. var.
        random_signal_flip_duration = 0.015 # in sec., how often to flip random signal
        bitMask = 0xffffff # for VPixx digital out, in hex bit
        DPxSetDoutValue(0, bitMask)
        DPxUpdateRegCache()
        
        run_exp = False
        # Process loop
        while not self.stop_fsm_process_Event.is_set():
            if not self.stop_exp_Event.is_set():
                # Turn on VPixx schedule; this needed to collect data
                lib.VPixx_turn_on_schedule()
                # Update targets
                self.update_target()
                # Load exp parameter
                fsm_parameter, parameter_file_path = lib.load_parameter('experiment','exp_parameter.json',True,True,CorrSacGui.set_default_parameter,self.exp_name,self.main_parameter['current_monkey'])
                cal_parameter, _ = lib.load_parameter('calibration','cal_parameter.json',True,True,lib.set_default_cal_parameter,'calibration',self.main_parameter['current_monkey'])  
                # Create target list
                target_pos_list = lib.make_corr_target(fsm_parameter)
                num_tgt_pos = len(target_pos_list)
                # Init. var
                DPxUpdateRegCache()
                self.t = DPxGetTime()
                self.pull_data_t = self.t
                random_signal_t = self.t
                trial_num = 1
                pump_to_use = 1 # which pump to use currently
                vel_samp_num = 3
                vel_t_data = deque(maxlen=vel_samp_num)
                eye_x_data = deque(maxlen=vel_samp_num)
                eye_y_data = deque(maxlen=vel_samp_num)
                eye_pos = [0,0]
                eye_vel = [0,0]
                eye_speed = 0.0
                right_eye_blink = True
                left_eye_blink = True
                # Reset digital out
                dout_ch_1 = 1 # nominal PD
                dout_ch_3 = 0 # random signal
                dout_ch_5 = 1 # LED
                DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                DPxUpdateRegCache()
                
                run_exp = True        
            # Trial loop
            while not self.stop_fsm_process_Event.is_set() and run_exp: 
                if self.stop_exp_Event.is_set():
                    run_exp = False
                    self.t = math.nan
                    # Turn off VPixx schedule
                    lib.VPixx_turn_off_schedule()
                    # Remove all targets
                    self.window.flip()
                    break
                # Init. trial variables; reset every trial
                self.init_trial_data()  
                self.trial_data['right_cal_matrix'] = cal_parameter['right_cal_matrix']
                self.trial_data['left_cal_matrix'] = cal_parameter['left_cal_matrix']
                state = 'INIT'   
                
                # FSM loop
                while not self.stop_fsm_process_Event.is_set() and run_exp:
                    if self.stop_exp_Event.is_set():
                        run_exp = False
                        self.t = math.nan
                        # Turn off VPixx schedule
                        lib.VPixx_turn_off_schedule()
                        # Remove all targets
                        self.window.flip()
                        break
                    # Send random signal for alignment
                    if (self.t - random_signal_t) > random_signal_flip_duration:
                        random_signal_t = self.t
                        if random.random() > 0.5:
                            dout_ch_3 = 1 
                        else:
                            dout_ch_3 = 0
                    DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                    # Get time       
                    self.t = TPxBestPolyGetEyePosition(cal_data, raw_data) # this calls 'DPxUpdateRegCache' as well

                    # Get eye status (blinking)
                    eye_status = DPxGetReg16(0x59A)
                    right_eye_blink = bool(eye_status & (1 << 0)) # << 0- (animal's) right blink (pink); << 1-left blink (cyan)
                    left_eye_blink = bool(eye_status & (1 << 1)) # << 0- (animal's) right blink (pink); << 1-left blink (cyan)
                    if cal_parameter['which_eye_tracked'] == 'Right':
                        if not right_eye_blink:
                            eye_blink = False
                            raw_data_right = [raw_data[0], raw_data[1],1] # [(animal's) right x, right y (pink), left x, left y (cyan)]
                            eye_pos = lib.raw_to_deg(raw_data_right,cal_parameter['right_cal_matrix'])
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
                            eye_blink = True
                            self.eye_x = 9999 # invalid values; more stable than nan values for plotting purposes in pyqtgraph
                            self.eye_y = 9999 
                    else:
                        if not left_eye_blink:
                            eye_blink = False
                            raw_data_left = [raw_data[2], raw_data[3],1] # [(animal's) right x, right y (pink), left x, left y (cyan)]
                            eye_pos = lib.raw_to_deg(raw_data_left,cal_parameter['left_cal_matrix'])
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
                            eye_blink = True
                            self.eye_x = 9999 # invalid values; more stable than nan values for plotting purposes in pyqtgraph
                            self.eye_y = 9999 
                    
                    if state == 'INIT':                     
                        # print('state = INIT')               
                        # Set trial parameters
                        tgt_idx = random.randint(0,num_tgt_pos-1) # Randomly pick target
                        start_pos = (fsm_parameter['horz_offset'], fsm_parameter['vert_offset'])
                        self.start_x = start_pos[0]
                        self.start_y = start_pos[1]
                        self.trial_data['start_x'].append(self.start_x)
                        self.trial_data['start_y'].append(self.start_y)
                        
                        cue_pos = np.array(target_pos_list[tgt_idx]['prim_tgt_pos']) + np.array(start_pos)
                        self.cue_x = cue_pos[0]
                        self.cue_y = cue_pos[1]
                        self.trial_data['cue_x'].append(self.cue_x)
                        self.trial_data['cue_y'].append(self.cue_y)
                        end_pos = np.array(target_pos_list[tgt_idx]['corr_tgt_pos']) + np.array(start_pos)
                        self.end_x = end_pos[0]
                        self.end_y = end_pos[1]
                        self.trial_data['end_x'].append(self.end_x)
                        self.trial_data['end_y'].append(self.end_y)
                        # Send target data
                        self.fsm_to_gui_sndr.send(('tgt_data',(self.cue_x,self.cue_y,self.end_x,self.end_y)))
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
                        dout_ch_1 = 0
                        dout_ch_5 = 0
                        self.pd_tgt.draw()
                        DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                        DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                        self.window.flip() 
                        state = 'STR_TARGET_PURSUIT'
                    
                    if state == 'STR_TARGET_PURSUIT':
                        pursuit_x = pursuit_v_x*(self.t-state_start_time) + pursuit_start_x
                        pursuit_y = pursuit_v_y*(self.t-state_start_time) + pursuit_start_y  
                        self.tgt_x = pursuit_x
                        self.tgt_y = pursuit_y
                        self.tgt.pos = (self.tgt_x,self.tgt_y)
                        self.tgt.draw()
                        self.pd_tgt.draw()
                        self.window.flip()
                        if (self.t-state_start_time) > fsm_parameter['pursuit_dur']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_present'].append(self.t)
                            state = 'STR_TARGET_PRESENT'  
                            dout_ch_1 = 1
                            dout_ch_5 = 1
                            self.tgt.draw()
                            DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                            DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                            self.window.flip()                     
                        if self.t - self.pull_data_t > 5:
                            self.pull_data_t = self.t
                            self.pull_data()    
                            # Send trial data to GUI
                            self.fsm_to_gui_sndr.send(('trial_data',trial_num, self.trial_data))
                            self.init_trial_data()
                    
                    if state == 'STR_TARGET_PRESENT':
                        if not eye_blink:
                            self.tgt_x = self.start_x
                            self.tgt_y = self.start_y
                            self.tgt.pos = (self.tgt_x,self.tgt_y)
                            self.tgt.draw()
                            self.window.flip()
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_fixation'].append(self.t)
                            state = 'STR_TARGET_FIXATION'
                        elif (self.t-state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            dout_ch_1 = 0
                            dout_ch_5 = 0
                            self.pd_tgt.draw()
                            DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                            DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                            self.window.flip() 
                            state = 'STR_TARGET_PURSUIT'

                    if state == 'STR_TARGET_FIXATION':
                        eye_dist_from_tgt = np.sqrt((self.tgt_x-self.eye_x)**2 + (self.tgt_y-self.eye_y)**2)
                        # If eye not available or fixating at the start target, reset the timer
                        if eye_dist_from_tgt > fsm_parameter['rew_area']/2 or eye_blink:
                            state_inter_time = self.t
                        if (self.t-state_inter_time) >= fsm_parameter['min_fix_time']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_cue_tgt_present'].append(self.t)
                            self.tgt_x = self.cue_x
                            self.tgt_y = self.cue_y
                            self.tgt.pos = (self.tgt_x,self.tgt_y)                   
                            self.tgt.draw()
                            self.pd_tgt.draw()
                            dout_ch_1 = 0
                            dout_ch_5 = 0
                            DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                            DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                            self.window.flip() 
                            lib.playSound(1000,0.1) # neutral beep  
                            state = 'CUE_TARGET_PRESENT'
                        if (self.t-state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            dout_ch_1 = 0
                            dout_ch_5 = 0
                            self.pd_tgt.draw()
                            DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                            DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                            self.window.flip() 
                            state = 'STR_TARGET_PURSUIT'  
                    
                    if state == 'CUE_TARGET_PRESENT':
                        state_start_time = self.t
                        state_inter_time = self.t
                        self.trial_data['state_start_t_detect_sac_start'].append(self.t)
                        state = 'DETECT_SACCADE_START'
                    
                    if state == 'DETECT_SACCADE_START':
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
                            dout_ch_1 = 1
                            dout_ch_5 = 1
                            DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                            DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                            self.window.flip() 
                            state = 'INCORRECT_SACCADE'
                        # If time runs out before saccade detected, play punishment sound and reset the trial
                        elif (self.t - state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            ######
                            # lib.playSound(200,0.1) # punishment beep
                            ######
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            dout_ch_1 = 0
                            dout_ch_5 = 0
                            self.pd_tgt.draw()
                            DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                            DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                            self.window.flip() 
                            state = 'STR_TARGET_PURSUIT'
                    
                    if state == 'SACCADE':
                        # Check to see if saccade is in the right direction
                        target_dir_vector = [self.cue_x-self.start_x,self.cue_y-self.start_y]
                        unit_target_dir_vector = target_dir_vector/np.linalg.norm(target_dir_vector)
                        saccade_dir_vector = eye_vel
                        unit_saccade_dir_vector = saccade_dir_vector/np.linalg.norm(saccade_dir_vector)                    
                        angle_diff = np.arccos(np.dot(unit_target_dir_vector, unit_saccade_dir_vector))

                        if angle_diff >= np.pi/2:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_incorrect_saccade'].append(self.t)
                            dout_ch_1 = 1
                            dout_ch_5 = 1
                            DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                            DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                            self.window.flip() 
                            state = 'INCORRECT_SACCADE'
                        else:
                            # Move the target to secondary pos.
                            self.tgt_x = self.end_x
                            self.tgt_y = self.end_y
                            self.tgt.pos = (self.tgt_x,self.tgt_y)
                            self.tgt.draw()
                            self.pd_tgt.draw()
                            self.window.flip() 
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_detect_sac_end'].append(self.t)
                            state = 'DETECT_SACCADE_END'
                    
                    if state == 'DETECT_SACCADE_END':
                        if (eye_speed < fsm_parameter['sac_on_off_threshold']) and (self.t-state_start_time > 0.005):#25):
                            # Check if saccade made to cue or end tgt.
                            eye_dist_from_cue_tgt = np.sqrt((self.cue_x-self.eye_x)**2 + (self.cue_y-self.eye_y)**2)
                            eye_dist_from_end_tgt = np.sqrt((self.end_x-self.eye_x)**2 + (self.end_y-self.eye_y)**2)
                            if ((eye_dist_from_cue_tgt < fsm_parameter['rew_area']/2) or (eye_dist_from_end_tgt < fsm_parameter['rew_area']/2)):
                                state_start_time = self.t
                                state_inter_time = self.t
                                self.trial_data['state_start_t_deliver_rew'].append(self.t)
                                state = 'DELIVER_REWARD'
                            else:
                                state_start_time = self.t
                                state_inter_time = self.t
                                self.trial_data['state_start_t_incorrect_saccade'].append(self.t)
                                dout_ch_1 = 1
                                dout_ch_5 = 1
                                DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                                DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                                self.window.flip() 
                                state = 'INCORRECT_SACCADE'
                        # If time runs out before saccade detected, reset the trial
                        elif (self.t - state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            dout_ch_1 = 0
                            dout_ch_5 = 0
                            self.pd_tgt.draw()
                            DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                            DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                            self.window.flip() 
                            state = 'STR_TARGET_PURSUIT'
                    
                    if state == 'DELIVER_REWARD':
                        eye_dist_from_cue_tgt = np.sqrt((self.cue_x-self.eye_x)**2 + (self.cue_y-self.eye_y)**2)
                        eye_dist_from_end_tgt = np.sqrt((self.end_x-self.eye_x)**2 + (self.end_y-self.eye_y)**2)
                        if eye_dist_from_end_tgt < fsm_parameter['rew_area']/2:
                            if (trial_num % fsm_parameter['pump_switch_interval']) == 0:
                                if pump_to_use == 1:
                                    pump_to_use = 2
                                else:
                                    pump_to_use = 1
                                self.fsm_to_gui_sndr.send(('log','Pump switchd to '+str(pump_to_use)))
                            self.fsm_to_gui_sndr.send(('pump_' + str(pump_to_use),0))
                                                    
                            lib.playSound(2000,0.1) # reward beep
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_end_tgt_fixation'].append(self.t)
                            self.tgt.draw()
                            dout_ch_1 = 1
                            dout_ch_5 = 1
                            DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                            DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                            self.window.flip()
                            state = 'END_TARGET_FIXATION'  
                        # If animal makes random saccade instead of corrective one, reset trial
                        elif (eye_dist_from_cue_tgt > fsm_parameter['rew_area']/2) and (eye_dist_from_end_tgt > fsm_parameter['rew_area']/2):
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            dout_ch_1 = 0
                            dout_ch_5 = 0
                            self.pd_tgt.draw()
                            DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                            DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                            self.window.flip() 
                            state = 'STR_TARGET_PURSUIT'
                        elif (self.t - state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            dout_ch_1 = 0
                            dout_ch_5 = 0
                            self.pd_tgt.draw()
                            DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                            DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                            self.window.flip() 
                            state = 'STR_TARGET_PURSUIT'
                    
                    if state == 'END_TARGET_FIXATION':
                        eye_dist_from_tgt = np.sqrt((self.tgt_x-self.eye_x)**2 + (self.tgt_y-self.eye_y)**2)   
                        if ((self.t - state_inter_time) >= fsm_parameter['min_fix_time']):
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_trial_success'].append(self.t)
                            self.window.flip() # remove all targets
                            state = 'TRIAL_SUCCESS'
                        # If time runs out before fixation finished, reset the trial
                        # No explicit fixation required
                        elif (self.t-state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            dout_ch_1 = 0
                            dout_ch_5 = 0
                            self.pd_tgt.draw()
                            DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                            DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                            self.window.flip() 
                            state = 'STR_TARGET_PURSUIT'                     
                    
                    if state == 'INCORRECT_SACCADE':
                        if ((self.t - state_start_time) > fsm_parameter['pun_time']):
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            dout_ch_1 = 0
                            dout_ch_5 = 0
                            self.pd_tgt.draw()
                            DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
                            DPxUpdateRegCache() # calling this delays fsm by ~0.25 ms
                            self.window.flip() 
                            state = 'STR_TARGET_PURSUIT'
                    
                    if state == 'TRIAL_SUCCESS':
                        if (self.t-state_start_time) > fsm_parameter['ITI']:
                            # Pull data
                            self.pull_data_t = self.t
                            self.pull_data()
                            # Send trial data to GUI
                            self.fsm_to_gui_sndr.send(('log',datetime.now().strftime("%H:%M:%S") + '; trial num: ' + str(trial_num) + ' -> completed'))
                            self.fsm_to_gui_sndr.send(('trial_data',trial_num, self.trial_data))
                            trial_num += 1
                            self.init_trial_data()  
                            self.trial_data['right_cal_matrix'] = cal_parameter['right_cal_matrix']
                            self.trial_data['left_cal_matrix'] = cal_parameter['left_cal_matrix']
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
                        
        # Close PsychoPy
        core.quit()
        # Turn off VPixx schedule
        lib.VPixx_turn_off_schedule()
        # Close VPixx devices
        DPxSetTPxSleep()
        DPxSelectDevice('DATAPIXX3')
        DPxUpdateRegCache()  
        DPxClose()        
        tracker.TRACKPixx3().close()  
        # Reset digital out
        dout_ch_1 = 1 # nominal PD
        dout_ch_3 = 0 # random signal
        dout_ch_5 = 1 # LED
        DPxSetDoutValue(dout_ch_1 + (2**2)*dout_ch_3 + (2**4)*dout_ch_5, bitMask)
        DPxUpdateRegCache()
        # Reset time
        self.t = math.nan
        
    def pull_data(self):
        '''
        to be called every 10 s or when a trial finishes, whichever is earlier
        and flush the data from VPixx. Reason for this is to prevent the data
        from accumulating, which will incur a delay when getting data 
        '''
        tpxData = TPxReadTPxData(0)
        self.trial_data['device_time_data'].extend(tpxData[0][0::22])
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

        TPxSetupTPxSchedule() # flushes data in DATAPixx buffer
    
    def update_target(self):
        tgt_parameter, _ = lib.load_parameter('','tgt_parameter.json',True,False,lib.set_default_tgt_parameter,'tgt')
        pd_tgt_parameter,_ = lib.load_parameter('','tgt_parameter.json',True,False,lib.set_default_tgt_parameter,'pd_tgt')
        self.tgt = visual.Rect(win=self.window, width=tgt_parameter['size'],height=tgt_parameter['size'], units='deg', 
                      lineColor=tgt_parameter['line_color'],fillColor=tgt_parameter['fill_color'],
                      lineWidth=tgt_parameter['line_width'])
        self.tgt.draw() # draw once already, because the first draw may be slower - Poth, 2018   
        self.pd_tgt = visual.Rect(win=self.window, width=pd_tgt_parameter['size'],height=pd_tgt_parameter['size'], units='deg', 
                      lineColor=pd_tgt_parameter['line_color'],fillColor=pd_tgt_parameter['fill_color'],
                      lineWidth=pd_tgt_parameter['line_width'])
        self.pd_tgt.pos = pd_tgt_parameter['pos']
        self.pd_tgt.draw()
        self.window.clearBuffer() # clear the back buffer of previously drawn stimuli - Poth, 2018
        
    def init_trial_data(self):
        '''
        initializes a dict. of trial data;
        needs to be called at the start of every trial
        '''
        self.trial_data = {}
        self.trial_data['right_cal_matrix'] = [] # may be updated during exp.
        self.trial_data['left_cal_matrix'] = []
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
        self.trial_data['end_x'] = []
        self.trial_data['end_y'] = []  
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
        self.trial_data['device_time_data'] = []
        self.trial_data['din_data'] = []
        self.trial_data['dout_data'] = []
        
    def set_default_parameter(self):
        parameter = {
                    'horz_offset':0.0,
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
                    'corr_sac_amp':2.0,
                    'num_corr_sac_dir':8,
                    'first_corr_sac_dir': 0,
                    'ITI':0.1,
                    'pump_switch_interval':50}
        return parameter  
    
class CorrSacGui(FsmGui):
    def __init__(self,exp_name, fsm_to_gui_rcvr, gui_to_fsm_sndr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array,main_parameter):        
        self.exp_name = exp_name
        self.fsm_to_gui_rcvr = fsm_to_gui_rcvr
        self.gui_to_fsm_sndr = gui_to_fsm_sndr
        self.stop_exp_Event = stop_exp_Event
        self.stop_fsm_process_Event = stop_fsm_process_Event
        self.real_time_data_Array = real_time_data_Array 
        self.main_parameter = main_parameter
        super(CorrSacGui,self).__init__(self.stop_fsm_process_Event)      
        self.init_gui()
        
        # Create socket for ZMQ
        try:
            context = zmq.Context()
            self.fsm_to_plot_socket = context.socket(zmq.PUB)
            self.fsm_to_plot_socket.bind("tcp://192.168.0.2:5556")
            
            self.fsm_to_plot_priority_socket = context.socket(zmq.PUB)
            self.fsm_to_plot_priority_socket.bind("tcp://192.168.0.2:5557")
            
            self.plot_to_fsm_socket = context.socket(zmq.SUB)
            self.plot_to_fsm_socket.connect("tcp://192.168.0.1:5558")
            self.plot_to_fsm_socket.subscribe("")
            self.plot_to_fsm_poller = zmq.Poller()
            self.plot_to_fsm_poller.register(self.plot_to_fsm_socket, zmq.POLLIN)
        except Exception as error:
            self.log_QPlainTextEdit.appendPlainText('Error in starting zmq sockets:')
            self.log_QPlainTextEdit.appendPlainText(str(error) + '.')
            self.toolbar_run_QAction.setDisabled(True)
            self.toolbar_connect_QAction.setDisabled(True)
            
        # Load exp. parameter or set default values
        self.exp_parameter, self.parameter_file_path = lib.load_parameter('experiment','exp_parameter.json',True,True,self.set_default_parameter,self.exp_name,self.main_parameter['current_monkey'])
        self.cal_parameter, _ = lib.load_parameter('calibration','cal_parameter.json',True,True,lib.set_default_cal_parameter,'calibration',self.main_parameter['current_monkey'])
        self.update_parameter()
        
        which_eye_tracked = self.cal_parameter['which_eye_tracked'].lower()
        if not self.cal_parameter[which_eye_tracked + '_cal_status']:
            self.toolbar_run_QAction.setDisabled(True)
            self.log_QPlainTextEdit.appendPlainText('No calibration found. Please calibrate first.')
        
        self.data_manager = DataManager()
        self.init_signals()
    #%% SIGNALS
    def init_signals(self):
        self.data_QTimer.timeout.connect(self.data_QTimer_timeout)
        self.receiver_QTimer.timeout.connect(self.receiver_QTimer_timeout)
        # Toolbar
        self.toolbar_connect_QAction.triggered.connect(self.toolbar_connect_QAction_triggered)
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
        self.num_prim_sac_dir_QDoubleSpinBox.valueChanged.connect(self.num_prim_sac_dir_QDoubleSpinBox_valueChanged)
        self.first_prim_sac_dir_QDoubleSpinBox.valueChanged.connect(self.first_prim_sac_dir_QDoubleSpinBox_valueChanged)
        self.corr_sac_amp_QDoubleSpinBox.valueChanged.connect(self.corr_sac_amp_QDoubleSpinBox_valueChanged)
        self.num_corr_sac_dir_QDoubleSpinBox.valueChanged.connect(self.num_corr_sac_dir_QDoubleSpinBox_valueChanged)
        self.first_corr_sac_dir_QDoubleSpinBox.valueChanged.connect(self.first_corr_sac_dir_QDoubleSpinBox_valueChanged)
        self.iti_QDoubleSpinBox.valueChanged.connect(self.iti_QDoubleSpinBox_valueChanged)
        self.pump_switch_QDoubleSpinBox.valueChanged.connect(self.pump_switch_QDoubleSpinBox_valueChanged)
        self.save_QPushButton.clicked.connect(self.save_QPushButton_clicked)
    
    #%% SLOTS
    @pyqtSlot()
    def toolbar_run_QAction_triggered(self):
        # Check to see if plot process ready
        self.fsm_to_plot_priority_socket.send_pyobj(('confirm_connection',0))
        # Wait for confirmation for 5 sec.
        if self.plot_to_fsm_poller.poll(5000):
            msg = self.plot_to_fsm_socket.recv_pyobj(flags=zmq.NOBLOCK)
            if msg[0] == 0:
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
                # Init. data    
                if self.cal_parameter['which_eye_tracked'] == 'Left':
                    self.exp_parameter['right_eye_tracked'] = 0
                    self.exp_parameter['left_eye_tracked'] = 1
                else:
                    self.exp_parameter['right_eye_tracked'] = 1
                    self.exp_parameter['left_eye_tracked'] = 0
                self.exp_parameter['version'] = 1.0
                self.fsm_to_plot_priority_socket.send_pyobj(('init_data',self.exp_name, self.exp_parameter))
                # Start timer to get data from FSM
                self.data_QTimer.start(self.data_rate)
                # Tell plot GUI we are starting
                self.fsm_to_plot_priority_socket.send_pyobj(('run',0))
        else:
            self.log_QPlainTextEdit.appendPlainText('No connection with plotting computer.')
    
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
        # Tell plot GUI we are stopping
        self.fsm_to_plot_priority_socket.send_pyobj(('stop',0))
    
    @pyqtSlot()
    def toolbar_connect_QAction_triggered(self):
        '''
        when triggered, start to receive messages from another computer
        '''
        self.receiver_QTimer.start(10)
        self.toolbar_connect_QAction.setDisabled(True)
    @pyqtSlot()
    def data_QTimer_timeout(self):
        '''
        getting data from fsm process and send them to another computer
        '''
        if self.fsm_to_gui_rcvr.poll():
            msg = self.fsm_to_gui_rcvr.recv()
            msg_title = msg[0]
            self.fsm_to_plot_priority_socket.send_pyobj(msg)
            if msg_title == 'log':
                self.log_QPlainTextEdit.appendPlainText(msg[1])

        with self.real_time_data_Array.get_lock():
            t = self.real_time_data_Array[0]
            eye_x = self.real_time_data_Array[1]
            eye_y = self.real_time_data_Array[2]
            tgt_x = self.real_time_data_Array[3]
            tgt_y = self.real_time_data_Array[4]        
        self.fsm_to_plot_socket.send_pyobj((t,eye_x,eye_y,tgt_x,tgt_y))
        
    @pyqtSlot()
    def receiver_QTimer_timeout(self):
        '''
        receive command from another computer
        '''
        if self.plot_to_fsm_poller.poll(1):
            msg = self.plot_to_fsm_socket.recv_pyobj(flags=zmq.NOBLOCK)
            msg_title = msg[0]
            if msg_title == 'run':
                self.toolbar_run_QAction_triggered()
            if msg_title == 'stop':
                self.toolbar_stop_QAction_triggered()
            if msg_title == 'confirm_connection':
                self.fsm_to_plot_priority_socket.send_pyobj((0,0))

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
    def num_prim_sac_dir_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['num_prim_sac_dir'] = int(self.num_prim_sac_dir_QDoubleSpinBox.value())
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
    @pyqtSlot()
    def first_prim_sac_dir_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['first_prim_sac_dir'] = int(self.first_prim_sac_dir_QDoubleSpinBox.value())
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
    @pyqtSlot()
    def corr_sac_amp_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['corr_sac_amp'] = self.corr_sac_amp_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
    @pyqtSlot()
    def num_corr_sac_dir_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['num_corr_sac_dir'] = int(self.num_corr_sac_dir_QDoubleSpinBox.value())
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
    @pyqtSlot()
    def first_corr_sac_dir_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['first_corr_sac_dir'] = int(self.first_corr_sac_dir_QDoubleSpinBox.value())
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
        all_parameter[self.main_parameter['current_monkey']][self.exp_name] = self.exp_parameter    
        with open(self.parameter_file_path,'w') as file:
            json.dump(all_parameter, file, indent=4)
        self.save_QPushButton.setStyleSheet('background-color: #39E547')  
    #%% GUI
    def init_gui(self):
        # Disable plots
        self.plot_1_PlotWidget.deleteLater()
        self.plot_2_PlotWidget.deleteLater()
        # Disable pumps
        self.pump_1.deleteLater()
        self.pump_2.deleteLater()
        # Side panel tabs for extra params.
        self.sidepanel_params_TabWidget= QTabWidget()
        self.sidepanel_params_1_tab_QWidget = QWidget()
        self.sidepanel_params_1_tab_QVBoxLayout = QVBoxLayout()
        self.sidepanel_params_1_tab_QWidget.setLayout(self.sidepanel_params_1_tab_QVBoxLayout)
        self.sidepanel_params_2_tab_QWidget = QWidget()
        self.sidepanel_params_2_tab_QVBoxLayout = QVBoxLayout()
        self.sidepanel_params_2_tab_QWidget.setLayout(self.sidepanel_params_2_tab_QVBoxLayout)
        self.sidepanel_params_TabWidget.addTab(self.sidepanel_params_1_tab_QWidget, '1')
        self.sidepanel_params_TabWidget.addTab(self.sidepanel_params_2_tab_QWidget, '2')
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
        
        self.corr_sac_amp_QHBoxLayout = QHBoxLayout()
        self.corr_sac_amp_QLabel = QLabel("Corrective saccade amp. (deg):")
        self.corr_sac_amp_QLabel.setAlignment(Qt.AlignRight)
        self.corr_sac_amp_QHBoxLayout.addWidget(self.corr_sac_amp_QLabel)
        self.corr_sac_amp_QDoubleSpinBox = QDoubleSpinBox()
        self.corr_sac_amp_QDoubleSpinBox.setValue(2.0)
        self.corr_sac_amp_QDoubleSpinBox.setMaximum(20)
        self.corr_sac_amp_QDoubleSpinBox.setSingleStep(0.1)
        self.corr_sac_amp_QDoubleSpinBox.setDecimals(1)
        self.corr_sac_amp_QHBoxLayout.addWidget(self.corr_sac_amp_QDoubleSpinBox)
        self.sidepanel_custom_QVBoxLayout.addLayout(self.corr_sac_amp_QHBoxLayout)
        
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
        
        self.sidepanel_custom_QVBoxLayout.addWidget(self.sidepanel_params_TabWidget)
        
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
        self.sidepanel_params_1_tab_QVBoxLayout.addLayout(self.max_allow_time_QHBoxLayout)
        
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
        self.sidepanel_params_1_tab_QVBoxLayout.addLayout(self.max_wait_fixation_QHBoxLayout)
        
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
        self.sidepanel_params_1_tab_QVBoxLayout.addLayout(self.pun_time_QHBoxLayout)
        
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
        self.sidepanel_params_1_tab_QVBoxLayout.addLayout(self.time_to_reward_QHBoxLayout)
        
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
        self.sidepanel_params_1_tab_QVBoxLayout.addLayout(self.sac_detect_threshold_QHBoxLayout)
        
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
        self.sidepanel_params_1_tab_QVBoxLayout.addLayout(self.sac_on_off_threshold_QHBoxLayout)
        
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
        self.sidepanel_params_2_tab_QVBoxLayout.addLayout(self.pursuit_amp_QHBoxLayout)
        
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
        self.sidepanel_params_2_tab_QVBoxLayout.addLayout(self.pursuit_dur_QHBoxLayout)
        
        self.num_prim_sac_dir_QHBoxLayout = QHBoxLayout()
        self.num_prim_sac_dir_QLabel = QLabel("Number of prim. sac. direction:")
        self.num_prim_sac_dir_QLabel.setAlignment(Qt.AlignRight)
        self.num_prim_sac_dir_QHBoxLayout.addWidget(self.num_prim_sac_dir_QLabel)
        self.num_prim_sac_dir_QDoubleSpinBox = QDoubleSpinBox()
        self.num_prim_sac_dir_QDoubleSpinBox.setValue(8)
        self.num_prim_sac_dir_QDoubleSpinBox.setMaximum(20)
        self.num_prim_sac_dir_QDoubleSpinBox.setMinimum(1)
        self.num_prim_sac_dir_QDoubleSpinBox.setSingleStep(1)
        self.num_prim_sac_dir_QDoubleSpinBox.setDecimals(0)
        self.num_prim_sac_dir_QHBoxLayout.addWidget(self.num_prim_sac_dir_QDoubleSpinBox)
        self.sidepanel_params_2_tab_QVBoxLayout.addLayout(self.num_prim_sac_dir_QHBoxLayout)
        
        self.first_prim_sac_dir_QHBoxLayout = QHBoxLayout()
        self.first_prim_sac_dir_QLabel = QLabel("1st prim. sac. direction (deg):")
        self.first_prim_sac_dir_QLabel.setAlignment(Qt.AlignRight)
        self.first_prim_sac_dir_QHBoxLayout.addWidget(self.first_prim_sac_dir_QLabel)
        self.first_prim_sac_dir_QDoubleSpinBox = QDoubleSpinBox()
        self.first_prim_sac_dir_QDoubleSpinBox.setToolTip('Specifies starting direction of uniformly distributed target positions for prim. sac.')
        self.first_prim_sac_dir_QDoubleSpinBox.setValue(0)
        self.first_prim_sac_dir_QDoubleSpinBox.setMaximum(359)
        self.first_prim_sac_dir_QDoubleSpinBox.setSingleStep(1)
        self.first_prim_sac_dir_QDoubleSpinBox.setDecimals(0)
        self.first_prim_sac_dir_QHBoxLayout.addWidget(self.first_prim_sac_dir_QDoubleSpinBox)
        self.sidepanel_params_2_tab_QVBoxLayout.addLayout(self.first_prim_sac_dir_QHBoxLayout)
        
        self.num_corr_sac_dir_QHBoxLayout = QHBoxLayout()
        self.num_corr_sac_dir_QLabel = QLabel("Number of corr. sac. direction:")
        self.num_corr_sac_dir_QLabel.setAlignment(Qt.AlignRight)
        self.num_corr_sac_dir_QHBoxLayout.addWidget(self.num_corr_sac_dir_QLabel)
        self.num_corr_sac_dir_QDoubleSpinBox = QDoubleSpinBox()
        self.num_corr_sac_dir_QDoubleSpinBox.setValue(8)
        self.num_corr_sac_dir_QDoubleSpinBox.setMaximum(20)
        self.num_corr_sac_dir_QDoubleSpinBox.setMinimum(1)
        self.num_corr_sac_dir_QDoubleSpinBox.setSingleStep(1)
        self.num_corr_sac_dir_QDoubleSpinBox.setDecimals(0)
        self.num_corr_sac_dir_QHBoxLayout.addWidget(self.num_corr_sac_dir_QDoubleSpinBox)
        self.sidepanel_params_2_tab_QVBoxLayout.addLayout(self.num_corr_sac_dir_QHBoxLayout)
        
        self.first_corr_sac_dir_QHBoxLayout = QHBoxLayout()
        self.first_corr_sac_dir_QLabel = QLabel("1st corr. sac. direction (deg):")
        self.first_corr_sac_dir_QLabel.setAlignment(Qt.AlignRight)
        self.first_corr_sac_dir_QHBoxLayout.addWidget(self.first_corr_sac_dir_QLabel)
        self.first_corr_sac_dir_QDoubleSpinBox = QDoubleSpinBox()
        self.first_corr_sac_dir_QDoubleSpinBox.setToolTip('Specifies starting direction of uniformly distributed target positions for corr. sac.')
        self.first_corr_sac_dir_QDoubleSpinBox.setValue(0)
        self.first_corr_sac_dir_QDoubleSpinBox.setMaximum(359)
        self.first_corr_sac_dir_QDoubleSpinBox.setSingleStep(1)
        self.first_corr_sac_dir_QDoubleSpinBox.setDecimals(0)
        self.first_corr_sac_dir_QHBoxLayout.addWidget(self.first_corr_sac_dir_QDoubleSpinBox)
        self.sidepanel_params_2_tab_QVBoxLayout.addLayout(self.first_corr_sac_dir_QHBoxLayout)
        
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
        self.sidepanel_params_2_tab_QVBoxLayout.addLayout(self.iti_QHBoxLayout)
        
        self.save_QPushButton = QPushButton('Save parameters')
        self.sidepanel_custom_QVBoxLayout.addWidget(self.save_QPushButton)
    #%% FUNCTIONS    
    def set_default_parameter(self):
        parameter = {
                    'horz_offset':0.0,
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
                    'corr_sac_amp':2.0,
                    'num_corr_sac_dir':8,
                    'first_corr_sac_dir': 0,
                    'ITI':0.1,
                    'pump_switch_interval':50
                    }
        return parameter
    
    def update_parameter(self):
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
        self.num_prim_sac_dir_QDoubleSpinBox.setValue(self.exp_parameter['num_prim_sac_dir'])
        self.first_prim_sac_dir_QDoubleSpinBox.setValue(self.exp_parameter['first_prim_sac_dir'])
        self.corr_sac_amp_QDoubleSpinBox.setValue(self.exp_parameter['corr_sac_amp'])
        self.num_corr_sac_dir_QDoubleSpinBox.setValue(self.exp_parameter['num_corr_sac_dir'])
        self.first_corr_sac_dir_QDoubleSpinBox.setValue(self.exp_parameter['first_corr_sac_dir'])
        self.iti_QDoubleSpinBox.setValue(self.exp_parameter['ITI'])
        self.pump_switch_QDoubleSpinBox.setValue(self.exp_parameter['pump_switch_interval'])

        
class CorrSacGuiProcess(multiprocessing.Process):
    def __init__(self, exp_name, fsm_to_gui_rcvr, gui_to_fsm_sndr, stop_exp_Event, stop_fsm_process_Event, real_time_data_Array, main_parameter, parent=None):
        super(CorrSacGuiProcess,self).__init__(parent)
        self.exp_name = exp_name
        self.fsm_to_gui_rcvr = fsm_to_gui_rcvr
        self.gui_to_fsm_sndr = gui_to_fsm_sndr
        self.stop_exp_Event = stop_exp_Event
        self.real_time_data_Array = real_time_data_Array
        self.stop_fsm_process_Event = stop_fsm_process_Event
        self.main_parameter = main_parameter
    def run(self):  
        app = QApplication(sys.argv)
        app_gui = CorrSacGui(self.exp_name, self.fsm_to_gui_rcvr, self.gui_to_fsm_sndr, self.stop_exp_Event, self.stop_fsm_process_Event, self.real_time_data_Array, self.main_parameter)
        app_gui.setWindowIcon(QtGui.QIcon(os.path.join('.', 'icon', 'experiment_window.png')))
        app_gui.show()
        sys.exit(app.exec())
