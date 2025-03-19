"""
Laboratory for Computational Motor Control, Johns Hopkins School of Medicine
@author: Jay Pi <jay.s.314159@gmail.com>
"""
from PyQt5 import QtCore, QtGui
from PyQt5.QtWidgets import QApplication, QComboBox, QPushButton, QLabel, QHBoxLayout, QDoubleSpinBox, QCheckBox, QPlainTextEdit,\
                            QDialog, QShortcut, QTabWidget, QWidget, QVBoxLayout
from PyQt5.QtCore import QRunnable, QThreadPool, pyqtSignal, pyqtSlot, QObject, Qt, QTimer
from psychopy import monitors, visual, core
from psychopy import event as psychopy_event

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
import time

class DelaySacEyeProcess(multiprocessing.Process):
    def __init__(self,exp_name,data_sndr,stop_exp_Event,stop_fsm_process_Event,data_change_Event,end_trial_Event,real_time_data_Array, data_ch_1,data_ch_5,data_ch_change, main_parameter,mon_parameter):
        super().__init__()
        self.exp_name = exp_name
        self.data_sndr = data_sndr
        self.stop_exp_Event = stop_exp_Event
        self.stop_fsm_process_Event = stop_fsm_process_Event
        self.real_time_data_Array = real_time_data_Array
        self.main_parameter = main_parameter
        self.mon_parameter = mon_parameter
        self.data_change_Event = data_change_Event
        self.end_trial_Event = end_trial_Event
        #self.no_tracker_Event = no_tracker_Event
        #self.mouse_disable_Event = mouse_disable_Event
        self.dout_ch_1 = data_ch_1
        self.dout_ch_5 = data_ch_5
        self.data_ch_change = data_ch_change
        
        self.mouse_mode = False
        #self.no_tracker = False
        #self.no_tracker_Event.clear()
        
        # Init var.
        self.eye_x = 0
        self.eye_y = 0
        self.t = math.nan
        
    def run(self):
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
            
            
        new_data_received = False
        cal_parameter, _ = lib.load_parameter('calibration','cal_parameter.json',True,True,lib.set_default_cal_parameter,'calibration',self.main_parameter['current_monkey'])


        run_exp = False
        self.init_trial_data()
        self.trial_data['right_cal_matrix'] = cal_parameter['right_cal_matrix']
        self.trial_data['left_cal_matrix'] = cal_parameter['left_cal_matrix']
        
        while not self.stop_fsm_process_Event.is_set():
            if not run_exp:
                lib.VPixx_turn_on_schedule()
                vel_samp_num = 3
                vel_t_data = deque(maxlen=vel_samp_num)
                eye_x_data = deque(maxlen=vel_samp_num)
                eye_y_data = deque(maxlen=vel_samp_num)
                eye_pos = [0,0]
                eye_vel = [0,0]
                eye_speed = 0.0
                right_eye_blink = True
                left_eye_blink = True
                run_exp = True
                old_t = 0;
                random_signal_t = self.t
                
                dout_ch_3 = 0 # random signal
                with self.dout_ch_1.get_lock(), self.dout_ch_5.get_lock():
                    self.dout_ch_1.value = 1
                    self.dout_ch_5.value = 1
                    DPxSetDoutValue(self.dout_ch_1.value + (2**2)*dout_ch_3 + (2**4)*self.dout_ch_5.value, bitMask)
                    DPxUpdateRegCache()
                    
            if (self.t - random_signal_t) > random_signal_flip_duration:
                random_signal_t = self.t
                if random.random() > 0.5:
                    dout_ch_3 = 1 
                else:
                    dout_ch_3 = 0
                    
            self.t = TPxBestPolyGetEyePosition(cal_data, raw_data)
               
            # Fix indentation
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
                            
            with self.real_time_data_Array.get_lock():
                self.real_time_data_Array[0] = self.t
                self.real_time_data_Array[1] = self.eye_x
                self.real_time_data_Array[2] = self.eye_y
                self.real_time_data_Array[3] = eye_speed
                self.real_time_data_Array[4] = eye_blink
                self.data_change_Event.set()

            if self.end_trial_Event.is_set():
                self.data_sndr.send(self.trial_data)
                self.end_trial_Event.clear()
                self.init_trial_data()
                
            if self.data_ch_change.is_set():
                with self.dout_ch_1.get_lock(), self.dout_ch_5.get_lock():
                    DPxSetDoutValue(self.dout_ch_1.value + (2**2)*dout_ch_3 + (2**4)*self.dout_ch_5.value, bitMask)
                    DPxUpdateRegCache()
                self.data_ch_change.clear()
                         
                        
                        
    def init_trial_data(self):
        '''
        initializes a dict. of trial data;
        needs to be called at the start of every trial
        '''
        self.trial_data = {}
        self.trial_data['cal_matrix'] = [] # may be updated during exp.
        self.trial_data['eye_x_data'] = []
        self.trial_data['eye_y_data'] = []
        self.trial_data['eye_time_data'] = []
        
        
            
        

class DelaySacFsmProcess(multiprocessing.Process):
    def __init__(self,exp_name, fsm_to_gui_sndr, gui_to_fsm_Q, data_rcvr, stop_exp_Event, stop_fsm_process_Event,data_change_Event,end_trial_Event, mouse_enable_Event, real_time_data_Array, eye_data_Array, data_ch_1,data_ch_5,data_ch_change, main_parameter, mon_parameter):
        super().__init__()
        self.exp_name = exp_name
        self.fsm_to_gui_sndr = fsm_to_gui_sndr
        self.gui_to_fsm_Q = gui_to_fsm_Q
        self.data_rcvr = data_rcvr
        self.stop_exp_Event = stop_exp_Event
        self.stop_fsm_process_Event = stop_fsm_process_Event
        self.real_time_data_Array = real_time_data_Array
        self.eye_data_Array = eye_data_Array
        self.main_parameter = main_parameter
        self.mon_parameter = mon_parameter
        self.data_change_Event = data_change_Event
        self.end_trial_Event = end_trial_Event
        self.mouse_enable_Event = mouse_enable_Event
        self.mouse_mode = False
        self.dout_ch_1 = data_ch_1
        self.dout_ch_5 = data_ch_5
        self.data_ch_change = data_ch_change
        
        # Init var.
        self.eye_x = 9999
        self.eye_y = 9999
        self.tgt_x = 0
        self.tgt_y = 0
        self.start_x = 0
        self.start_y = 0
        self.cue_x = 0
        self.cue_y = 0
        self.t = math.nan
        #self.pull_data_t = 0 # keep track of when data was pulled last from VPixx
        self.eye_blink = True;
    
    def run(self):
        # import faulthandler
        # faulthandler.disable()
        # faulthandler.enable()
        # gc.disable()
        print("FSM Run started")
        print(self.mouse_mode)
        
        # Set up exp. screen
        this_monitor = monitors.Monitor(self.mon_parameter['monitor_name'], width=self.mon_parameter['monitor_width'], distance=self.mon_parameter['monitor_distance'])
        this_monitor.save()
        this_monitor.setSizePix(self.mon_parameter['monitor_size'])
        self.window = visual.Window(size=self.mon_parameter['monitor_size'],screen=self.mon_parameter['monitor_num'], allowGUI=False, color='white', monitor=this_monitor,
                                units='deg', winType='pyglet', fullscr=True, checkTiming=False, waitBlanking=True)
        self.mouse_tracker = psychopy_event.Mouse(win=self.window)
        self.window.flip()
        
        # Make targets
        #self.update_target()
        
        # Init. var.
        random_signal_flip_duration = 0.015 # in sec., how often to flip random signal
        
                 
        run_exp = False
        # Process loop
        while not self.stop_fsm_process_Event.is_set():
            if not self.stop_exp_Event.is_set():
                # Update targets
                # Load exp parameter
                
                if self.mouse_enable_Event.is_set():
                    print("Mouse mode")
                    self.mouse_mode = True
                    exp_parameter_filename = 'mouse_exp_parameter.json'
                else:
                    print("Eye mode")
                    self.mouse_mode = False
                    exp_parameter_filename = 'exp_parameter.json'
                
                fsm_parameter, _ = lib.load_parameter('experiment',exp_parameter_filename,True,True,self.set_default_parameter,self.exp_name, self.main_parameter['current_monkey'])
                cal_parameter, _ = lib.load_parameter('calibration','cal_parameter.json',True,True,lib.set_default_cal_parameter,'calibration',self.main_parameter['current_monkey'])
                arrow_param,_ = lib.load_parameter('','cue_parameter.json',True,False,lib.set_default_tgt_parameter,'arrow')
                landolt_param,_ = lib.load_parameter('','cue_parameter.json',True,False,lib.set_default_tgt_parameter,'landolt')
                p_a = fsm_parameter['ambiguity_prob']
                
                coherence_pairs = []
                coherence_prob = [1 - p_a,p_a/4,p_a/4,p_a/4,0,0,p_a/4,0,0]
                for counter_c in range(3):
                    for counter_c_dir in range(3):
                        coherence_pairs.append((counter_c,landolt_param['coherence'][counter_c_dir]))
                        
                
                self.fsm_parameter = fsm_parameter
                self.update_target()
                
                # Create target list
                #target_pos_list = lib.make_corr_target(fsm_parameter)
                target_pos_list,corr_pos_list = lib.make_corr_targets_independent(fsm_parameter)
                num_tgt_pos = len(target_pos_list)
                num_corr_pos = len(corr_pos_list)
                print(num_tgt_pos)
                #num_tgt_display = fsm_parameter['num_tgt_display']-1
                #fsm_parameter['max_wait_for_corrective'] = 0.5 # Add a menu option for this
                fsm_parameter['max_wait_for_corrective'] = fsm_parameter['max_wait_for_fixation']
                # Init. var
                self.t = 0
                old_t = self.t
                new_t = old_t
                self.send_data_t = self.t
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
                new_data_received = False
                if num_tgt_pos % fsm_parameter['num_tgt_display'] == 0:
                    tgt_step_size = int(num_tgt_pos/fsm_parameter['num_tgt_display'])
                else:
                    fsm_parameter['randomize_targets'] = True #Evenly spaced targets are not possible if an odd number are displayed
                    print("Randomized target locations")
                
                # Reset digital out
                print(f'Mouse mode: {self.mouse_mode}')
                run_exp = True
            # Trial loop
            while not self.stop_fsm_process_Event.is_set() and run_exp: 
                if self.stop_exp_Event.is_set():
                    run_exp = False

                    self.window.flip()
                    self.t = math.nan
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
                        self.window.flip()
                        self.t = math.nan
                        break
                    # Send random signal for alignment
                    self.write_Dout(0,0)
                
                    if self.mouse_mode:
                        t = time.time()
                        time_diff = t - self.t
                        if time_diff > 1e-2:
                            self.t = t

                            self.window.winHandle.dispatch_events()
                            mouse_pos = self.mouse_tracker.getPos()
                            eye_vel_x = (mouse_pos[0] - self.eye_x)/time_diff
                            eye_vel_y = (mouse_pos[1] - self.eye_y)/time_diff
                            
                            self.eye_speed = np.sqrt(eye_vel_x**2 + eye_vel_y**2);
                            self.eye_x = mouse_pos[0]
                            self.eye_y = mouse_pos[1]
                            #print(mouse_pos)
                            self.eye_blink = False
                            
                    else:                    
                        if self.data_change_Event.is_set(): # Get the latest EyeLink data
                            with self.eye_data_Array.get_lock():
                                self.t = self.eye_data_Array[0]
                                self.eye_x = self.eye_data_Array[1]
                                self.eye_y = self.eye_data_Array[2]
                                self.eye_speed = self.eye_data_Array[3]
                                self.eye_blink = self.eye_data_Array[4]
                                self.data_change_Event.clear()
                        
                        
                    if state == 'INIT':
                        print('state = INIT')
                        
                        # Set trial parameters
                        tgt_idx = random.randint(0,num_tgt_pos-1) # Randomly pick target
                        corr_idx = random.randint(0,num_corr_pos-1)
                            
                        start_pos = (fsm_parameter['horz_offset'], fsm_parameter['vert_offset'])
                        self.start_x = start_pos[0]
                        self.start_y = start_pos[1]
                        self.trial_data['start_x'].append(self.start_x)
                        self.trial_data['start_y'].append(self.start_y)
                        
                        cues = ['arrow','landolt']
                        cue_type = fsm_parameter['cue_type']
                        if cue_type == 'both':
                            cue_type = cues[random.randint(0,1)]
                        
                        
                        cue_pos = np.array(target_pos_list[tgt_idx]['prim_tgt_pos']) + np.array(start_pos)
                        self.cue_x = cue_pos[0]
                        self.cue_y = cue_pos[1]
                        self.trial_data['cue_x'].append(self.cue_x)
                        self.trial_data['cue_y'].append(self.cue_y)
                        
                        end_pos = np.array(corr_pos_list[corr_idx]['corr_tgt_pos'])
                        self.end_x = end_pos[0] + self.cue_x
                        self.end_y = end_pos[1] + self.cue_y
                        self.trial_data['end_x'].append(self.end_x)
                        self.trial_data['end_y'].append(self.end_y)
                        
                        self.cue_end_vector = np.array([self.end_x-self.cue_x, self.end_y-self.cue_y])
                            
                        # Send target data
                        self.fsm_to_gui_sndr.send(('tgt_data',(self.cue_x,self.cue_y)))
                        pursuit_angle = np.random.randint(0,360)
                        pursuit_start_x = np.cos(pursuit_angle*np.pi/180)*fsm_parameter['pursuit_amp']
                        pursuit_start_x += self.start_x
                        pursuit_v_x = (self.start_x - pursuit_start_x)/fsm_parameter['pursuit_dur']
                        pursuit_start_y = np.sin(pursuit_angle*np.pi/180)*fsm_parameter['pursuit_amp']
                        pursuit_start_y += self.start_y
                        pursuit_v_y = (self.start_y - pursuit_start_y)/fsm_parameter['pursuit_dur']
                        
                        delay_time = np.random.rand()*(fsm_parameter['max_delay'] - fsm_parameter['min_delay']) + fsm_parameter['min_delay']
                        self.trial_data['delay_time'].append(delay_time)
                        # Pick a random delay time
                        
                        cue_duration = fsm_parameter['cue_duration']
                        if np.random.rand() > fsm_parameter['cue_probability']:
                            cue_duration = 0
                        self.trial_data['cue_duration'].append(cue_duration)
                        
                        # Choose set of targets to display
                        if np.random.rand() < fsm_parameter['choice_prob']:
                            num_tgt_display = fsm_parameter['num_tgt_display'] - 1
                        else:
                            num_tgt_display = 0
                            
                        
                        tgt_display_list = []
                        if fsm_parameter['randomize_targets']:
                            tgt_choices = list(np.arange(0,num_tgt_pos))
                            tgt_choices.remove(tgt_idx)
                            for counter_tgt in range(num_tgt_display):
                                curr_choice = random.choice(tgt_choices)
                                tgt_choices.remove(curr_choice)
                                tgt_display_list.append(curr_choice)
                               
                        else:
                            for counter_tgt in range(tgt_step_size,num_tgt_pos,tgt_step_size):
                                curr_idx = (tgt_idx + counter_tgt) % num_tgt_pos
                                tgt_display_list.append(curr_idx)
                                
                        tgt_display_byte = 0    
                        tgt_display_coords = []    
                        for counter_tgt in range(num_tgt_display):
                            curr_pos = np.array(target_pos_list[tgt_display_list[counter_tgt]]['prim_tgt_pos']) + np.array(start_pos)
                            tgt_display_coords.append(curr_pos)
                            tgt_display_byte = tgt_display_byte + 2**tgt_display_list[counter_tgt]
                        self.trial_data['tgt_display_byte'].append(tgt_display_byte)
                        
                        if cue_type == 'arrow':
                            coherence_choice_idx = np.random.choice(range(len(coherence_pairs)),p=coherence_prob)
                            coherence_choices = coherence_pairs[coherence_choice_idx]
                            arrow_coherence_choice = coherence_choices[0]
                            ori_coherence = coherence_choices[1]
                            bias_direction = (-1)**(np.random.choice([0,1])) # This will be either +1 or -1
                            
                            self.cue = self.cue_shapes[arrow_coherence_choice]
                            ori = np.rad2deg(np.arctan2(self.start_y - self.cue_y,self.cue_x - self.start_x))
                            ori = ori + (1 - ori_coherence)*bias_direction*22.5 # If coherence is 0 (completely ambiguous), the arrow will be placed halfway between the target and an adjacent target
                            self.cue.ori = ori
                            self.trial_data['arrow_coherence'].append(arrow_param['coherence'][arrow_coherence_choice])
                            
                        else:
                            ori_coherence = np.random.choice(landolt_param['coherence'],p=[1-p_a,p_a/2,p_a/2])
                            bias_direction = (-1)**(np.random.choice([0,1]))
                            
                            self.cue = self.landolt_c
                            ori =  np.rad2deg(np.arctan2(self.start_y - self.cue_y,self.cue_x - self.start_x))
                            ori = ori + (1 - ori_coherence)*bias_direction*22.5
                            self.landolt_rect.ori = ori
                            self.landolt_rect_pos = 0.5*np.array([np.cos(self.landolt_rect.ori*np.pi/180), -np.sin(self.landolt_rect.ori*np.pi/180)])
                            
                        self.trial_data['cue_type'].append(cues.index(cue_type))
                        self.trial_data['cue_orientation'].append(ori)
                        
                        if fsm_parameter['center_cue']:
                            self.cue.pos = (self.start_x, self.start_y)
                        elif fsm_parameter['fixed_cue_pos']:
                            ang = fsm_parameter['first_cue_dir']*np.pi/180
                            self.cue.pos = fsm_parameter['prim_sac_amp']*np.array([np.cos(ang), np.sin(ang)]) + np.array(start_pos)
                        else:
                            num_cue_dir = int(fsm_parameter['num_cue_dir'])
                            prim_sac_amp = fsm_parameter['prim_sac_amp']
                            first_dir = fsm_parameter['first_cue_dir'] + 360*(tgt_idx/num_tgt_pos)
                            cue_pos_list = lib.make_prim_target_arg(num_cue_dir,prim_sac_amp,first_dir)
                            
                            cue_idx = random.randint(0,len(cue_pos_list)-1)
                            self.cue.pos = cue_pos_list[cue_idx]
                            
                               
                        
                        state_start_time = self.t
                        state_inter_time = self.t
                        self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                        self.write_Dout(0,0)
                        self.pd_tgt.draw()

                        self.window.flip()
                        state = 'STR_TARGET_PURSUIT'
                        print('state = STR_TARGET_PURSUIT')
                        
                        
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
                            print('state = STR_TARGET_PRESENT')
                            self.tgt.draw()
                            self.write_Dout(1,1)
                            
                            self.window.flip()
                        # Every few seconds, send data to be saved
                        if self.t - self.send_data_t > 5:
                            self.send_data_t = self.t
                            # Send trial data to GUI
                            self.get_trial_eye_data()
                            self.fsm_to_gui_sndr.send(('trial_data',trial_num, self.trial_data))
                            self.init_trial_data()
                            
                    if state == 'STR_TARGET_PRESENT':
                        if not self.eye_blink:
                            self.tgt_x = self.start_x
                            self.tgt_y = self.start_y
                            self.tgt.pos = (self.tgt_x,self.tgt_y) 
                            self.tgt.draw()
                            self.window.flip()
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_fixation'].append(self.t)
                            state = 'STR_TARGET_FIXATION'
                            print('state = STR_TARGET_FIXATION')

                        elif (self.t-state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            self.write_Dout(0,0)
                            self.pd_tgt.draw()
                            self.window.flip()
                            state = 'STR_TARGET_PURSUIT'
                            
                    if state == 'STR_TARGET_FIXATION':
                        eye_dist_from_tgt = np.sqrt((self.tgt_x-self.eye_x)**2 + (self.tgt_y-self.eye_y)**2)
                        # If eye not available or fixating at the start target, reset the timer
                        if eye_dist_from_tgt > fsm_parameter['rew_area']/2 or self.eye_blink:
                            state_inter_time = self.t
                        if (self.t-state_inter_time) >= fsm_parameter['min_fix_time']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            #self.tgt_x = self.cue_x
                            #self.tgt_y = self.cue_y
                                 
                            if cue_duration == 0:
                                self.trial_data['state_start_t_delay_fixation'].append(self.t)
                                state = 'DELAY_FIXATION'
                                print('state = DELAY_FIXATION')
                            else:
                                self.trial_data['state_start_t_cue_display'].append(self.t)
                                state = 'DISPLAY_CUE'
                                print('state = DISPLAY_CUE')
                        if (self.t-state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            
                            self.pd_tgt.draw()
                            self.write_Dout(0,0)
                            self.window.flip()
                            state = 'STR_TARGET_PURSUIT'  
                            
                    if state == 'DISPLAY_CUE': # This is the state where you wait while the cue is displayed
                        if not self.eye_blink:
                            self.draw_cue(cue_type,fsm_parameter['center_cue'],False)
                            self.tgt.draw()
                            self.pd_tgt.draw()
                            self.write_Dout(0,0)
                            self.window.flip()
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_cue_fixation'].append(self.t)
                            state = 'CUE_FIXATION'
                            print('state = CUE_FIXATION')
                            
                    if state == 'CUE_FIXATION':
                        # The animal should not be able to view the cue twice by breaking fixation, so we display the cue for a fixed amount of time,
                        # regardless of whether the animal maintains fixation

                        if (self.t-state_inter_time) >= fsm_parameter['cue_duration']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            
                            print(fsm_parameter['mask_duration'])
                            print(fsm_parameter['mask_duration'] > 0)
                            if fsm_parameter['mask_duration'] > 1e-3:     
                                self.draw_cue(cue_type,fsm_parameter['center_cue'],True)
                                self.trial_data['state_start_t_mask_cue'].append(self.t)
                                state = 'MASK_CUE'
                                print('state = MASK_CUE')
                            else:
                                state = 'DELAY_FIXATION'
                                self.trial_data['state_start_t_delay_fixation'].append(self.t)
                                print('state = DELAY_FIXATION')
                                  
                            self.tgt.draw()
                            self.pd_tgt.draw()
                            self.window.flip()
                            
                    if state == 'MASK_CUE':
                        if (self.t-state_inter_time) >= fsm_parameter['mask_duration']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_delay_fixation'].append(self.t)
                                 
                            self.pd_tgt.draw()
                            self.tgt.draw()
                            self.window.flip() # Clears the mask
                            state = 'DELAY_FIXATION'
                            print('state = DELAY_FIXATION')
                   
                            
                    if state == 'DELAY_FIXATION':
                        eye_dist_from_tgt = np.sqrt((self.tgt_x-self.eye_x)**2 + (self.tgt_y-self.eye_y)**2)
                        # If not fixating, restart the delay state. This ensures that the animal must fixate for the full delay,
                        # while also making sure that it can't get the cue to show multiple times.
                        if eye_dist_from_tgt > fsm_parameter['rew_area']/2 or self.eye_blink:
                            state_inter_time = self.t
                            
                        if (self.t-state_inter_time) >= delay_time:
                            state_start_time = self.t
                            state_inter_time = self.t   
                            self.trial_data['state_start_t_ecc_tgt_present'].append(self.t)
                            
                            lib.playSound(1000,0.1) # Neutral beep
                            for counter_tgt in range(len(tgt_display_coords)):
                                distractor_pos = tgt_display_coords[counter_tgt]
                                self.tgt.pos = (distractor_pos[0], distractor_pos[1])
                                self.tgt.draw()
                                
                            self.tgt_x = self.cue_x
                            self.tgt_y = self.cue_y
                            self.tgt.pos = (self.tgt_x,self.tgt_y)                   
                            self.tgt.draw()
                            self.pd_tgt.draw()
                            self.write_Dout(0,0)
                            self.window.flip()
                            state = 'ECCENTRIC_TGT_PRESENT'
                            print('state = ECCENTRIC_TGT_PRESENT')
                            
                        if (self.t-state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            self.write_Dout(0,0)
                            self.pd_tgt.draw()
            
                            self.window.flip()
                            state = 'STR_TARGET_PURSUIT'
                            
                            
                    if state == 'ECCENTRIC_TGT_PRESENT':
                        state_start_time = self.t
                        state_inter_time = self.t
                        self.trial_data['state_start_t_detect_sac_start'].append(self.t)
                        state = 'DETECT_SACCADE_START'
                        print('state = DETECT_SACCADE_START')
                        
                    if state == 'DETECT_SACCADE_START':
                        eye_dist_from_start_tgt = np.sqrt((self.start_x-self.eye_x)**2 + (self.start_y-self.eye_y)**2)
                        #print(f'Dist: {eye_dist_from_tgt}, Speed: {self.eye_speed}')
                        if self.eye_speed >= fsm_parameter['sac_detect_threshold']:         
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.window.flip()
                            self.trial_data['state_start_t_saccade'].append(self.t)
                            state = 'SACCADE'                 
                        # If eye moves away from start target, reset trial after punishment period
                        elif eye_dist_from_start_tgt > fsm_parameter['rew_area']/2:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_incorrect_saccade'].append(self.t)
                            self.write_Dout(1,1)
                            self.window.flip()
                            print('state = INCORRECT_SACCADE')
                            state = 'INCORRECT_SACCADE'                         
                        # If time runs out before saccade detected, play punishment sound and reset the trial
                        elif (self.t - state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            ######
                            # lib.playSound(200,0.1) # punishment beep
                            ######
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            self.write_Dout(0,0)
                            self.pd_tgt.draw()
                            self.window.flip()
                            print('state = STR_TARGET_PURSUIT')
                            state = 'STR_TARGET_PURSUIT'
                    
                    if state == 'SACCADE':
                        eye_dist_from_start_tgt = np.sqrt((self.start_x-self.eye_x)**2 + (self.start_y-self.eye_y)**2)
                        if eye_dist_from_start_tgt > fsm_parameter['rew_area']/2:
                            # Check to see if saccade is in the right direction
                            target_dir_vector = [self.cue_x-self.start_x,self.cue_y-self.start_y]
                            unit_target_dir_vector = target_dir_vector/np.linalg.norm(target_dir_vector)
                        
                            saccade_dir_vector = [self.eye_x-self.start_x,self.eye_y-self.start_y]

                            unit_saccade_dir_vector = saccade_dir_vector/np.linalg.norm(saccade_dir_vector)                    
                            angle_diff = np.arccos(np.dot(unit_target_dir_vector, unit_saccade_dir_vector))

                            if angle_diff < np.pi/2:
                                self.tgt.pos = (self.end_x,self.end_y)              
                                self.tgt.draw()
                                
                            self.pd_tgt.draw()
                            self.window.flip()
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_detect_sac_end'].append(self.t)
                            state = 'DETECT_SACCADE_END'
                            print('state = DETECT_SACCADE_END')
                    
                    if state == 'DETECT_SACCADE_END':
                        if (self.eye_speed < fsm_parameter['sac_on_off_threshold']) and (self.t-state_start_time > 0.005):#25):
                        
                            # Check if saccade made to cue
                            eye_dist_from_cue_tgt = np.sqrt((self.cue_x-self.eye_x)**2 + (self.cue_y-self.eye_y)**2)

                            if (eye_dist_from_cue_tgt < fsm_parameter['rew_area']/2):
                                state_start_time = self.t
                                state_inter_time = self.t
                                  
                                self.trial_data['state_start_t_corr_sac'].append(self.t)
                                state = 'CORR_SACCADE'
                                print('state = CORR_SACCADE')
                              
                            else:
                                state_start_time = self.t
                                state_inter_time = self.t
                                
                                wrong_tgt_bool = False
                                for counter_tgt in range(len(tgt_display_coords)):
                                    distractor_pos = tgt_display_coords[counter_tgt]
                                    eye_dist_from_distractor = np.sqrt((self.eye_x-distractor_pos[0])**2 + (self.eye_y-distractor_pos[1])**2)
                                    if eye_dist_from_distractor < fsm_parameter['rew_area']/2:
                                        remove_tgt_ind = counter_tgt
                                        self.trial_data['distractor_x'].append(distractor_pos[0])
                                        self.trial_data['distractor_y'].append(distractor_pos[1])
                                        wrong_tgt_bool = True

                                self.write_Dout(1,1)
                                self.window.flip()
                                if wrong_tgt_bool:
                                    tgt_display_coords.pop(remove_tgt_ind)
                                    self.trial_data['state_start_t_wrong_target'].append(self.t)
                                    state = 'WRONG_TARGET'
                                else: 
                                    self.trial_data['state_start_t_incorrect_saccade'].append(self.t)
                                    state = 'INCORRECT_SACCADE'
                             
                          # If time runs out before saccade detected, reset the trial
                        elif (self.t - state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            self.pd_tgt.draw()
                            self.write_Dout(0,0)
                            self.window.flip() 
                            state = 'STR_TARGET_PURSUIT'
                            
                        
                    if state == 'CORR_SACCADE':
                        eye_dist_from_end_tgt = np.sqrt((self.end_x-self.eye_x)**2 + (self.end_y-self.eye_y)**2)
                        eye_dist_from_cue_tgt = np.sqrt((self.tgt_x-self.eye_x)**2 + (self.tgt_y-self.eye_y)**2)
                        cue_eye_vector = np.array([self.eye_x-self.cue_x, self.eye_y-self.cue_y]) 
                        
                        if eye_dist_from_end_tgt < fsm_parameter['rew_area']/2:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_deliver_rew'].append(self.t)
                            state = 'DELIVER_REWARD'
                            print('state = DELIVER_REWARD')
                                
                        elif (self.t - state_start_time) >= fsm_parameter['max_wait_for_corrective']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            self.pd_tgt.draw()
                            self.write_Dout(0,0)
                            self.window.flip() 
                            state = 'STR_TARGET_PURSUIT'
                            
                        
                        # If animal makes random saccade instead of corrective one, reset trial
                        # The line_dist formula ensures that the animal is not penalized as long as it stays close to the line between the end target and the cue target
                        elif eye_dist_from_cue_tgt > fsm_parameter['rew_area']/2:
                            line_dist = abs((self.end_y-self.cue_y)*self.eye_x - (self.end_x-self.cue_x)*self.eye_y + self.end_x*self.cue_y - self.end_y*self.cue_x)/np.linalg.norm(self.cue_end_vector)
                            if (np.dot(self.cue_end_vector,cue_eye_vector)) < 0 or (line_dist > fsm_parameter['rew_area']/2):
                            
                                state_start_time = self.t
                                state_inter_time = self.t
                                self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                                self.pd_tgt.draw()
                                self.write_Dout(0,0)
                                self.window.flip() 
                                state = 'STR_TARGET_PURSUIT'
                        
                            
                    if state == 'DELIVER_REWARD':
                        if (trial_num % fsm_parameter['pump_switch_interval']) == 0:
                            if pump_to_use == 1:
                                pump_to_use = 1
                            else:
                                pump_to_use = 1
                            self.fsm_to_gui_sndr.send(('log','Pump switchd to '+str(pump_to_use)))
                        self.fsm_to_gui_sndr.send(('pump',pump_to_use,'pump',0))
                        print("Sent to GUI")
                                                
                        lib.playSound(2000,0.1) # reward beep
                        state_start_time = self.t
                        state_inter_time = self.t
                        self.trial_data['state_start_t_end_tgt_fixation'].append(self.t)
                        self.tgt.draw()
                        
                        self.write_Dout(1,1)
                        self.window.flip()
                        state = 'END_TARGET_FIXATION'  
                        print('state = END_TARGET_FIXATION')
                        
                    if state == 'END_TARGET_FIXATION':
                        eye_dist_from_tgt = np.sqrt((self.tgt_x-self.eye_x)**2 + (self.tgt_y-self.eye_y)**2)   
                        if ((self.t - state_inter_time) >= fsm_parameter['min_fix_time']):
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_trial_success'].append(self.t)
                            self.window.flip() # remove all targets
                            state = 'TRIAL_SUCCESS'
                            print('state = TRIAL_SUCCESS')
                        # If time runs out before fixation finished, reset the trial
                        # No explicit fixation required
                        elif (self.t-state_start_time) >= fsm_parameter['max_wait_for_fixation']:
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            self.pd_tgt.draw()
                            self.write_Dout(0,0)
                            
                            self.window.flip() 
                            state = 'STR_TARGET_PURSUIT'                       
                    
                    if state == 'INCORRECT_SACCADE':
                        # self.fsm_to_gui_sndr.send(('pun_beep',0))
                        if ((self.t - state_start_time) > fsm_parameter['pun_time']):
                            state_start_time = self.t
                            state_inter_time = self.t
                            self.trial_data['state_start_t_str_tgt_pursuit'].append(self.t)
                            self.pd_tgt.draw()
                            self.write_Dout(0,0)
                            self.window.flip()
                            state = 'STR_TARGET_PURSUIT'
                            
                    if state == 'WRONG_TARGET':
                        lib.playSound(500,0.1)
                        state_start_time = self.t
                        state_inter_time = self.t
                        self.trial_data['state_start_t_incorrect_saccade'].append(self.t)
                        
                        self.window.flip()
                        state = 'INCORRECT_SACCADE'
                            
                    if state == 'TRIAL_SUCCESS':
                        self.window.flip() # remove all targets
                        if (self.t-state_start_time) > fsm_parameter['ITI']:
                            self.send_data_t = self.t
                            
                            # Get trial eye data from eye process
                            self.get_trial_eye_data()
                            
                            # Send trial data to GUI
                            self.fsm_to_gui_sndr.send(('log',datetime.now().strftime("%H:%M:%S") + '; trial num: ' + str(trial_num) + ' -> completed'))
                            self.fsm_to_gui_sndr.send(('trial_data',trial_num, self.trial_data))
                            trial_num += 1
                            self.init_trial_data()  
                            self.trial_data['right_cal_matrix'] = cal_parameter['right_cal_matrix']
                            self.trial_data['left_cal_matrix'] = cal_parameter['left_cal_matrix']
                            state = 'INIT'  
                            
                    # Update shared real time data
                    with self.real_time_data_Array.get_lock():
                        self.real_time_data_Array[0] = self.t
                        self.real_time_data_Array[1] = self.eye_x
                        self.real_time_data_Array[2] = self.eye_y
                        self.real_time_data_Array[3] = self.tgt_x
                        self.real_time_data_Array[4] = self.tgt_y
                    

        # Turn off EyeLink
        #eyelink_worker.close(eye_tracker)
        
        # Close PsychoPy
        core.quit()
        
        # Reset digital out
        self.write_Dout(1,1)
        
        # Reset time
        self.t = math.nan
    
    def get_trial_eye_data(self):
        self.end_trial_Event.set()
        eye_data = self.data_rcvr.recv() # This will wait until the eye data arrives
        self.trial_data['eye_x_data'] = eye_data['eye_x_data']
        self.trial_data['eye_y_data'] = eye_data['eye_y_data']
        self.trial_data['cal_matrix'] = eye_data['cal_matrix']
        self.trial_data['eye_time_data'] = eye_data['eye_time_data']
    
    def update_target(self):
        tgt_parameter, _ = lib.load_parameter('','tgt_parameter.json',True,False,lib.set_default_tgt_parameter,'tgt')
        pd_tgt_parameter,_ = lib.load_parameter('','tgt_parameter.json',True,False,lib.set_default_tgt_parameter,'pd_tgt')
        arrow_param,_ = lib.load_parameter('','cue_parameter.json',True,False,lib.set_default_tgt_parameter,'arrow')
        landolt_param,_ = lib.load_parameter('','cue_parameter.json',True,False,lib.set_default_tgt_parameter,'landolt')
        
        self.tgt = visual.Rect(win=self.window, width=tgt_parameter['size'],height=tgt_parameter['size'], units='deg', 
                      lineColor=tgt_parameter['line_color'],fillColor=tgt_parameter['fill_color'],
                      lineWidth=tgt_parameter['line_width'])
        self.tgt.draw() # draw once already, because the first draw may be slower - Poth, 2018   
        self.pd_tgt = visual.Rect(win=self.window, width=pd_tgt_parameter['size'],height=pd_tgt_parameter['size'], units='deg', 
                      lineColor=pd_tgt_parameter['line_color'],fillColor=pd_tgt_parameter['fill_color'],
                      lineWidth=pd_tgt_parameter['line_width'])
        self.pd_tgt.pos = pd_tgt_parameter['pos']
        
        self.cue_shapes = []
        for counter_coh in range(len(arrow_param['coherence'])):
            shape = ((arrow_param['coherence'][counter_coh]-1, 0),(0,1),(1,0),(0,-1))
            curr_cue = visual.shape.ShapeStim(win=self.window,lineColor=arrow_param['line_color'],fillColor=arrow_param['fill_color'],
                      lineWidth=arrow_param['line_width'],vertices=shape,
                      size = (arrow_param['cue_length'],arrow_param['cue_width']),units='deg')
            self.cue_shapes.append(curr_cue)
            
        self.cue_circle = visual.Circle(win=self.window,size=arrow_param['circle_size'],fillColor=arrow_param['circle_color'])
        self.arrow_mask = visual.Circle(win=self.window,size=2*arrow_param['cue_length'],fillColor=arrow_param['fill_color'])
        self.cue = self.cue_shapes[0]
        
        self.landolt_c = visual.Circle(win=self.window,size=landolt_param['size'], lineColor=landolt_param['line_color'],fillColor=landolt_param['line_color'],lineWidth=0, units='deg')
        self.landolt_circ_inner = visual.Circle(win=self.window,size=landolt_param['size'] - landolt_param['line_width'], fillColor="white", lineWidth=0, units="deg")
        self.landolt_rect = visual.Rect(win=self.window,width=landolt_param['rect_width'],height=landolt_param['rect_height'],fillColor=landolt_param['rect_color'], lineColor=landolt_param['rect_color'])
        print(tgt_parameter['line_width'])
        #self.pd_tgt.draw()
        self.window.clearBuffer() # clear the back buffer of previously drawn stimuli - Poth, 2018
        
    def draw_cue(self,cue_type,center_cue,mask):
        if cue_type == 'arrow':
            if mask:
                self.arrow_mask.pos = self.cue.pos
                self.arrow_mask.draw()
            else:
                self.cue.draw() 
            if not center_cue: #This draws the arrow underneath the circle
                self.cue_circle.pos = self.cue.pos 
                self.cue_circle.draw()
        else:
            self.landolt_rect.pos = self.cue.pos + self.landolt_rect_pos
            #self.landolt_rect.pos = self.cue.pos
            self.landolt_circ_inner.pos = self.cue.pos
            self.cue.draw()
            self.landolt_circ_inner.draw()
            if not mask:
                self.landolt_rect.draw()
    
    def write_Dout(self,dout_ch1_value,dout_ch5_value):
        with self.dout_ch_1.get_lock(), self.dout_ch_5.get_lock():
            self.dout_ch_1.value = dout_ch1_value
            self.dout_ch_5.value = dout_ch5_value
        self.data_ch_change.set()        
    
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
        self.trial_data['state_start_t_cue_display'] = []
        self.trial_data['state_start_t_cue_fixation'] = []
        self.trial_data['state_start_t_mask_cue'] = []
        self.trial_data['state_start_t_delay_fixation'] = []
        self.trial_data['state_start_t_ecc_tgt_present'] = []
        self.trial_data['state_start_t_detect_sac_start'] = []
        self.trial_data['state_start_t_saccade'] = []
        self.trial_data['state_start_t_detect_sac_end'] = []
        self.trial_data['state_start_t_corr_sac'] = []
        self.trial_data['state_start_t_deliver_rew'] = []
        self.trial_data['state_start_t_end_tgt_fixation'] = []
        self.trial_data['state_start_t_trial_success'] = []
        self.trial_data['state_start_t_incorrect_saccade'] = []
        self.trial_data['state_start_t_wrong_target'] = []
        self.trial_data['cue_x'] = []
        self.trial_data['cue_y'] = []
        self.trial_data['end_x'] = []
        self.trial_data['end_y'] = []   
        self.trial_data['start_x'] = []
        self.trial_data['start_y'] = []
        self.trial_data['tgt_display_byte'] = []
        self.trial_data['distractor_x'] = []
        self.trial_data['distractor_y'] = []
        self.trial_data['cue_type'] = []
        self.trial_data['arrow_coherence'] = []
        self.trial_data['cue_orientation'] = []
        self.trial_data['delay_time'] = []
        self.trial_data['cue_duration'] = []
        
        
        
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
                     'ITI':0.1,
                     'pump_switch_interval':50,
                     'cue_duration':0.2,
                     'mask_duration':0.1,
                     'cue_probability':1.0,
                     'min_delay':0,
                     'max_delay':0,
                     'cue_type':'arrow',
                     'num_tgt_display':1,
                     'randomize_targets':False,
                     'ambiguity_prob':0.0,
                     'choice_prob':0.0,
                     'fixed_cue_pos':True,
                     'first_cue_dir':90,
                     'num_cue_dir':1,
                     'center_cue':True
                     }
        return parameter
        
class DelaySacGui(FsmGui):
    def __init__(self,exp_name, fsm_to_gui_rcvr, gui_to_fsm_sndr, stop_exp_Event, stop_fsm_process_Event,mouse_toggle_Event, real_time_data_Array, main_parameter):        
        # import faulthandler
        # faulthandler.disable()
        # faulthandler.enable()
        
        self.fsm_to_gui_rcvr = fsm_to_gui_rcvr
        self.gui_to_fsm_sndr = gui_to_fsm_sndr
        self.stop_exp_Event = stop_exp_Event
        self.stop_fsm_process_Event = stop_fsm_process_Event
        self.mouse_toggle_Event = mouse_toggle_Event
        self.real_time_data_Array = real_time_data_Array
        self.exp_name = exp_name
        self.main_parameter = main_parameter
        super(DelaySacGui,self).__init__(self.stop_fsm_process_Event)      
        self.init_gui()
        self.thread_pool = QThreadPool()
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
        _,self.mouse_file_path = lib.load_parameter('experiment','mouse_exp_parameter.json',True,True,self.set_default_parameter,self.exp_name,self.main_parameter['current_monkey'])
        
        self.mouse_mode = False
        
        self.cue_types = ['arrow','landolt','both']

        which_eye_tracked = self.cal_parameter['which_eye_tracked'].lower()
        if not self.cal_parameter[which_eye_tracked + '_cal_status']:
            self.toolbar_run_QAction.setDisabled(True)
            self.log_QPlainTextEdit.appendPlainText('No calibration found. Please calibrate first.')
        
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
        #self.max_allow_time_QDoubleSpinBox.valueChanged.connect(self.max_allow_time_QDoubleSpinBox_valueChanged)
        self.min_fix_time_QDoubleSpinBox.valueChanged.connect(self.min_fix_time_QDoubleSpinBox_valueChanged)
        self.max_wait_fixation_QDoubleSpinBox.valueChanged.connect(self.max_wait_fixation_QDoubleSpinBox_valueChanged)
        #self.pun_time_QDoubleSpinBox.valueChanged.connect(self.pun_time_QDoubleSpinBox_valueChanged)
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
        
        self.iti_QDoubleSpinBox.valueChanged.connect(self.iti_QDoubleSpinBox_valueChanged)
        self.pump_switch_QDoubleSpinBox.valueChanged.connect(self.pump_switch_QDoubleSpinBox_valueChanged)
        
        self.cue_duration_QDoubleSpinBox.valueChanged.connect(self.cue_duration_QDoubleSpinBox_valueChanged)
        self.mask_duration_QDoubleSpinBox.valueChanged.connect(self.mask_duration_QDoubleSpinBox_valueChanged)
        self.cue_probability_QDoubleSpinBox.valueChanged.connect(self.cue_probability_QDoubleSpinBox_valueChanged)
        self.min_delay_QDoubleSpinBox.valueChanged.connect(self.min_delay_QDoubleSpinBox_valueChanged)
        self.max_delay_QDoubleSpinBox.valueChanged.connect(self.max_delay_QDoubleSpinBox_valueChanged)
        self.cue_type_QComboBox.currentIndexChanged.connect(self.cue_type_QComboBox_indexChanged)
        self.num_tgt_display_QDoubleSpinBox.valueChanged.connect(self.num_tgt_display_QDoubleSpinBox_valueChanged)
        self.random_tgt_QCheckBox.stateChanged.connect(self.random_tgt_QCheckBox_stateChanged)
        self.ambiguity_prob_QDoubleSpinBox.valueChanged.connect(self.ambiguity_prob_QDoubleSpinBox_valueChanged)
        self.choice_prob_QDoubleSpinBox.valueChanged.connect(self.choice_prob_QDoubleSpinBox_valueChanged)
        
        self.center_cue_QCheckBox.stateChanged.connect(self.center_cue_QCheckBox_stateChanged)
        self.fixed_cue_pos_QCheckBox.stateChanged.connect(self.fixed_cue_pos_QCheckBox_stateChanged)
        self.first_cue_dir_QDoubleSpinBox.valueChanged.connect(self.first_cue_dir_QDoubleSpinBox_valueChanged)
        self.num_cue_dir_QDoubleSpinBox.valueChanged.connect(self.num_cue_dir_QDoubleSpinBox_valueChanged)
        
        self.save_QPushButton.clicked.connect(self.save_QPushButton_clicked)
        
        self.mouse_enable_QPushButton.clicked.connect(self.mouse_enable_clicked)
        self.mouse_disable_QPushButton.clicked.connect(self.mouse_disable_clicked)
        
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
                print("Cleared stop_exp_Event")     
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
        if self.plot_to_fsm_poller.poll(1):
            msg = self.plot_to_fsm_socket.recv_pyobj(flags=zmq.NOBLOCK)
            msg_title = msg[0]
            if msg_title == 'run':
                self.toolbar_run_QAction_triggered()
            if msg_title == 'stop':
                self.toolbar_stop_QAction_triggered()
            if msg_title == 'confirm_connection':
                self.fsm_to_plot_priority_socket.send_pyobj((0,0))
                
    @pyqtSlot(object)
    def send_processed_data(self, msg):
        msg_title = msg[0]
        if msg_title == 'log':
            self.log_QPlainTextEdit.appendPlainText(msg[1])
            self.fsm_to_plot_priority_socket.send_pyobj(msg)
        elif msg_title == 'processed_data':
            self.fsm_to_plot_priority_socket.send_pyobj(('processed_eyelink_data', msg[1]))
            
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
    def iti_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['ITI'] = self.iti_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')  
        
    @pyqtSlot()
    def pump_switch_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['pump_switch_interval'] = self.pump_switch_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
        
    @pyqtSlot()
    def cue_duration_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['cue_duration'] = self.cue_duration_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
    @pyqtSlot()
    def mask_duration_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['mask_duration'] = self.mask_duration_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00') 
    @pyqtSlot()
    def cue_probability_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['cue_probability'] = self.cue_probability_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')        
    @pyqtSlot()
    def min_delay_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['min_delay'] = self.min_delay_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')    
    @pyqtSlot()
    def max_delay_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['max_delay'] = self.max_delay_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
    @pyqtSlot()
    def cue_type_QComboBox_indexChanged(self):
        self.exp_parameter['cue_type'] = self.cue_types[self.cue_type_QComboBox.currentIndex()]
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
    @pyqtSlot()
    def num_tgt_display_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['num_tgt_display'] = int(self.num_tgt_display_QDoubleSpinBox.value())
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
    @pyqtSlot()
    def random_tgt_QCheckBox_stateChanged(self):
        self.exp_parameter['randomize_targets'] = self.random_tgt_QCheckBox.isChecked()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
    @pyqtSlot()
    def ambiguity_prob_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['ambiguity_prob'] = self.ambiguity_prob_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
    @pyqtSlot()
    def choice_prob_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['choice_prob'] = self.choice_prob_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
        
    @pyqtSlot()
    def center_cue_QCheckBox_stateChanged(self):
        print("Center cue state changed")
        self.exp_parameter['center_cue'] = self.center_cue_QCheckBox.isChecked()
        if self.center_cue_QCheckBox.isChecked():
            self.fixed_cue_pos_QCheckBox.setDisabled(True)
            self.fixed_cue_pos_QCheckBox.setChecked(False)
            self.exp_parameter['fixed_cue_pos'] = False
            self.num_cue_dir_QDoubleSpinBox.setDisabled(True)
            self.num_cue_dir_QDoubleSpinBox.setValue(1)
            self.exp_parameter['num_cue_dir'] = 1
            self.first_cue_dir_QDoubleSpinBox.setDisabled(True)
        else:
            self.fixed_cue_pos_QCheckBox.setEnabled(True)
            self.num_cue_dir_QDoubleSpinBox.setEnabled(True)
            self.first_cue_dir_QDoubleSpinBox.setEnabled(True)
    @pyqtSlot()
    def fixed_cue_pos_QCheckBox_stateChanged(self):
        self.exp_parameter['fixed_cue_pos'] = self.fixed_cue_pos_QCheckBox.isChecked()
        if self.fixed_cue_pos_QCheckBox.isChecked():
            self.num_cue_dir_QDoubleSpinBox.setDisabled(True)
            self.num_cue_dir_QDoubleSpinBox.setValue(1)
            self.exp_parameter['num_cue_dir'] = 1
        else:
            self.num_cue_dir_QDoubleSpinBox.setEnabled(True)
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
    @pyqtSlot()
    def first_cue_dir_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['first_cue_dir'] = self.first_cue_dir_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
    @pyqtSlot()
    def num_cue_dir_QDoubleSpinBox_valueChanged(self):
        self.exp_parameter['num_cue_dir'] = self.num_cue_dir_QDoubleSpinBox.value()
        self.save_QPushButton.setStyleSheet('background-color: #FFCC00')
        
        
            
    @pyqtSlot()
    def save_QPushButton_clicked(self):
        parameter_file_path = self.parameter_file_path
        if self.mouse_mode:
            parameter_file_path = self.mouse_file_path
            
        with open(parameter_file_path,'r') as file:
            all_parameter = json.load(file)
        all_parameter[self.main_parameter['current_monkey']][self.exp_name] = self.exp_parameter 
        with open(parameter_file_path,'w') as file:
            json.dump(all_parameter, file, indent=4)
        self.save_QPushButton.setStyleSheet('background-color: #39E547')
        
    @pyqtSlot()
    def mouse_enable_clicked(self):
        print("Set mouse mode")
        self.mouse_toggle_Event.set()
        self.mouse_mode = True
        self.mouse_disable_QPushButton.setEnabled(True)
        self.mouse_enable_QPushButton.setDisabled(True)
        self.exp_parameter, _ = lib.load_parameter('experiment','mouse_exp_parameter.json',True,True,self.set_default_parameter,self.exp_name,self.main_parameter['current_monkey'])
        self.update_parameter()
    
    @pyqtSlot()
    def mouse_disable_clicked(self):
        self.mouse_disable_QPushButton.setDisabled(True)

        self.mouse_toggle_Event.clear()
        self.mouse_mode = False
        self.mouse_enable_QPushButton.setEnabled(True)
        self.exp_parameter, _ = lib.load_parameter('experiment','exp_parameter.json',True,True,self.set_default_parameter,self.exp_name,self.main_parameter['current_monkey'])
        self.update_parameter()
        
    #%% GUI
    def init_gui(self):
        # Disable plots
        self.plot_1_PlotWidget.deleteLater()
        self.plot_2_PlotWidget.deleteLater()
        # Disable pumps
        #self.pump_1.deleteLater()
        #self.pump_2.deleteLater()
        
        # Side panel tabs for extra params
        self.sidepanel_params_TabWidget= QTabWidget()
        self.sidepanel_params_1_tab_QWidget = QWidget()
        self.sidepanel_params_1_tab_QVBoxLayout = QVBoxLayout()
        self.sidepanel_params_1_tab_QWidget.setLayout(self.sidepanel_params_1_tab_QVBoxLayout)
        self.sidepanel_params_2_tab_QWidget = QWidget()
        self.sidepanel_params_2_tab_QVBoxLayout = QVBoxLayout()
        self.sidepanel_params_2_tab_QWidget.setLayout(self.sidepanel_params_2_tab_QVBoxLayout)
        self.sidepanel_params_3_tab_QWidget = QWidget()
        self.sidepanel_params_3_tab_QVBoxLayout = QVBoxLayout()
        self.sidepanel_params_3_tab_QWidget.setLayout(self.sidepanel_params_3_tab_QVBoxLayout)
        self.sidepanel_params_4_tab_QWidget = QWidget()
        self.sidepanel_params_4_tab_QVBoxLayout = QVBoxLayout()
        self.sidepanel_params_4_tab_QWidget.setLayout(self.sidepanel_params_4_tab_QVBoxLayout)
        self.sidepanel_params_TabWidget.addTab(self.sidepanel_params_1_tab_QWidget, 'General')
        self.sidepanel_params_TabWidget.addTab(self.sidepanel_params_2_tab_QWidget, 'Corr Sac.')
        self.sidepanel_params_TabWidget.addTab(self.sidepanel_params_3_tab_QWidget, 'Delay Task')
        self.sidepanel_params_TabWidget.addTab(self.sidepanel_params_4_tab_QWidget, 'Extra')
        
        self.sidepanel_custom_QVBoxLayout.addWidget(self.sidepanel_params_TabWidget)
        
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
        self.sidepanel_params_1_tab_QVBoxLayout.addLayout(self.horz_offset_QHBoxLayout)
        
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
        self.sidepanel_params_1_tab_QVBoxLayout.addLayout(self.vert_offset_QHBoxLayout)
        
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
        self.sidepanel_params_1_tab_QVBoxLayout.addLayout(self.min_fix_time_QHBoxLayout)
        
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
        self.sidepanel_params_1_tab_QVBoxLayout.addLayout(self.rew_area_QHBoxLayout)
        
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
        self.sidepanel_params_1_tab_QVBoxLayout.addLayout(self.iti_QHBoxLayout)
        
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
        self.sidepanel_params_1_tab_QVBoxLayout.addLayout(self.pump_switch_QHBoxLayout)
        
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
        self.sidepanel_params_2_tab_QVBoxLayout.addLayout(self.prim_sac_amp_QHBoxLayout)
        
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
        self.sidepanel_params_2_tab_QVBoxLayout.addLayout(self.corr_sac_amp_QHBoxLayout)
        
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
        
        '''
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
        '''
        
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
        self.sidepanel_params_2_tab_QVBoxLayout.addLayout(self.sac_detect_threshold_QHBoxLayout)
        
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
        self.sidepanel_params_2_tab_QVBoxLayout.addLayout(self.sac_on_off_threshold_QHBoxLayout)
        
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
        
        self.cue_duration_QHBoxLayout = QHBoxLayout()
        self.cue_duration_QLabel = QLabel("Cue Duration (s):")
        self.cue_duration_QLabel.setAlignment(Qt.AlignRight)
        self.cue_duration_QHBoxLayout.addWidget(self.cue_duration_QLabel)
        self.cue_duration_QDoubleSpinBox = QDoubleSpinBox()
        self.cue_duration_QDoubleSpinBox.setValue(0.1)
        self.cue_duration_QDoubleSpinBox.setMaximum(1)
        self.cue_duration_QDoubleSpinBox.setSingleStep(0.01)
        self.cue_duration_QDoubleSpinBox.setDecimals(2)
        self.cue_duration_QHBoxLayout.addWidget(self.cue_duration_QDoubleSpinBox)
        self.sidepanel_params_3_tab_QVBoxLayout.addLayout(self.cue_duration_QHBoxLayout)
        
        self.mask_duration_QHBoxLayout = QHBoxLayout()
        self.mask_duration_QLabel = QLabel("Mask Duration (s):")
        self.mask_duration_QLabel.setAlignment(Qt.AlignRight)
        self.mask_duration_QHBoxLayout.addWidget(self.mask_duration_QLabel)
        self.mask_duration_QDoubleSpinBox = QDoubleSpinBox()
        self.mask_duration_QDoubleSpinBox.setValue(0.1)
        self.mask_duration_QDoubleSpinBox.setMaximum(1)
        self.mask_duration_QDoubleSpinBox.setSingleStep(0.01)
        self.mask_duration_QDoubleSpinBox.setDecimals(2)
        self.mask_duration_QHBoxLayout.addWidget(self.mask_duration_QDoubleSpinBox)
        self.sidepanel_params_3_tab_QVBoxLayout.addLayout(self.mask_duration_QHBoxLayout)
        
        self.cue_probability_QHBoxLayout = QHBoxLayout()
        self.cue_probability_QLabel = QLabel("Cue Probability (s):")
        self.cue_probability_QLabel.setAlignment(Qt.AlignRight)
        self.cue_probability_QHBoxLayout.addWidget(self.cue_probability_QLabel)
        self.cue_probability_QDoubleSpinBox = QDoubleSpinBox()
        self.cue_probability_QDoubleSpinBox.setValue(1)
        self.cue_probability_QDoubleSpinBox.setMaximum(1)
        self.cue_probability_QDoubleSpinBox.setSingleStep(0.1)
        self.cue_probability_QDoubleSpinBox.setDecimals(1)
        self.cue_probability_QHBoxLayout.addWidget(self.cue_probability_QDoubleSpinBox)
        self.sidepanel_params_3_tab_QVBoxLayout.addLayout(self.cue_probability_QHBoxLayout)
        
        self.min_delay_QHBoxLayout = QHBoxLayout()
        self.min_delay_QLabel = QLabel("Minimum Delay (s):")
        self.min_delay_QLabel.setAlignment(Qt.AlignRight)
        self.min_delay_QHBoxLayout.addWidget(self.min_delay_QLabel)
        self.min_delay_QDoubleSpinBox = QDoubleSpinBox()
        self.min_delay_QDoubleSpinBox.setValue(0)
        self.min_delay_QDoubleSpinBox.setMaximum(5)
        self.min_delay_QDoubleSpinBox.setSingleStep(0.01)
        self.min_delay_QDoubleSpinBox.setDecimals(2)
        self.min_delay_QHBoxLayout.addWidget(self.min_delay_QDoubleSpinBox)
        self.sidepanel_params_3_tab_QVBoxLayout.addLayout(self.min_delay_QHBoxLayout)
        
        self.max_delay_QHBoxLayout = QHBoxLayout()
        self.max_delay_QLabel = QLabel("Maximum Delay (s):")
        self.max_delay_QLabel.setAlignment(Qt.AlignRight)
        self.max_delay_QHBoxLayout.addWidget(self.max_delay_QLabel)
        self.max_delay_QDoubleSpinBox = QDoubleSpinBox()
        self.max_delay_QDoubleSpinBox.setValue(0)
        self.max_delay_QDoubleSpinBox.setMaximum(5)
        self.max_delay_QDoubleSpinBox.setSingleStep(0.01)
        self.max_delay_QDoubleSpinBox.setDecimals(2)
        self.max_delay_QHBoxLayout.addWidget(self.max_delay_QDoubleSpinBox)
        self.sidepanel_params_3_tab_QVBoxLayout.addLayout(self.max_delay_QHBoxLayout)
        
        self.cue_type_QHBoxLayout = QHBoxLayout()
        self.cue_type_QLabel = QLabel("Cue Type:")
        self.cue_type_QLabel.setAlignment(Qt.AlignRight)
        self.cue_type_QHBoxLayout.addWidget(self.cue_type_QLabel)
        self.cue_type_QComboBox = QComboBox()
        self.cue_type_QComboBox.addItems(['Arrow', 'Landolt C', 'Both'])
        self.cue_type_QHBoxLayout.addWidget(self.cue_type_QComboBox)
        self.sidepanel_params_3_tab_QVBoxLayout.addLayout(self.cue_type_QHBoxLayout)
        
        self.ambiguity_prob_QHBoxLayout = QHBoxLayout()
        self.ambiguity_prob_QLabel = QLabel("Probability of Ambiguous Cue:")
        self.ambiguity_prob_QLabel.setAlignment(Qt.AlignRight)
        self.ambiguity_prob_QHBoxLayout.addWidget(self.ambiguity_prob_QLabel)
        self.ambiguity_prob_QDoubleSpinBox = QDoubleSpinBox()
        self.ambiguity_prob_QDoubleSpinBox.setValue(0.0)
        self.ambiguity_prob_QDoubleSpinBox.setMaximum(1)
        self.ambiguity_prob_QDoubleSpinBox.setSingleStep(0.01)
        self.ambiguity_prob_QDoubleSpinBox.setDecimals(2)
        self.ambiguity_prob_QHBoxLayout.addWidget(self.ambiguity_prob_QDoubleSpinBox)
        self.sidepanel_params_3_tab_QVBoxLayout.addLayout(self.ambiguity_prob_QHBoxLayout)
        
        self.choice_prob_QHBoxLayout = QHBoxLayout()
        self.choice_prob_QLabel = QLabel("Choice Probability:")
        self.choice_prob_QLabel.setAlignment(Qt.AlignRight)
        self.choice_prob_QHBoxLayout.addWidget(self.choice_prob_QLabel)
        self.choice_prob_QDoubleSpinBox = QDoubleSpinBox()
        self.choice_prob_QDoubleSpinBox.setValue(0.0)
        self.choice_prob_QDoubleSpinBox.setMaximum(1)
        self.choice_prob_QDoubleSpinBox.setSingleStep(0.01)
        self.choice_prob_QDoubleSpinBox.setDecimals(2)
        self.choice_prob_QHBoxLayout.addWidget(self.choice_prob_QDoubleSpinBox)
        self.sidepanel_params_3_tab_QVBoxLayout.addLayout(self.choice_prob_QHBoxLayout)
        
        self.num_tgt_display_QHBoxLayout = QHBoxLayout()
        self.num_tgt_display_QLabel = QLabel("Number of Targets Displayed:")
        self.num_tgt_display_QLabel.setAlignment(Qt.AlignRight)
        self.num_tgt_display_QHBoxLayout.addWidget(self.num_tgt_display_QLabel)
        self.num_tgt_display_QDoubleSpinBox = QDoubleSpinBox()
        self.num_tgt_display_QDoubleSpinBox.setValue(8)
        self.num_tgt_display_QDoubleSpinBox.setMaximum(8)
        self.num_tgt_display_QDoubleSpinBox.setSingleStep(1)
        self.num_tgt_display_QDoubleSpinBox.setDecimals(0)
        self.num_tgt_display_QHBoxLayout.addWidget(self.num_tgt_display_QDoubleSpinBox)
        self.sidepanel_params_3_tab_QVBoxLayout.addLayout(self.num_tgt_display_QHBoxLayout)
        
        self.random_tgt_QCheckBox = QCheckBox('Randomize Targets')
        self.sidepanel_params_3_tab_QVBoxLayout.addWidget(self.random_tgt_QCheckBox)
        
        self.fixed_cue_pos_QCheckBox = QCheckBox('Fixed Cue Position')
        self.sidepanel_params_4_tab_QVBoxLayout.addWidget(self.fixed_cue_pos_QCheckBox)
        
        self.center_cue_QCheckBox = QCheckBox('Cue at Center Fixation')
        self.sidepanel_params_4_tab_QVBoxLayout.addWidget(self.center_cue_QCheckBox)
        
        self.first_cue_dir_QHBoxLayout = QHBoxLayout()
        self.first_cue_dir_QLabel = QLabel("Cue Direction:")
        self.first_cue_dir_QLabel.setAlignment(Qt.AlignRight)
        self.first_cue_dir_QHBoxLayout.addWidget(self.first_cue_dir_QLabel)
        self.first_cue_dir_QDoubleSpinBox = QDoubleSpinBox()
        self.first_cue_dir_QDoubleSpinBox.setValue(90)
        self.first_cue_dir_QDoubleSpinBox.setMaximum(359)
        self.first_cue_dir_QDoubleSpinBox.setSingleStep(1)
        self.first_cue_dir_QDoubleSpinBox.setDecimals(0)
        self.first_cue_dir_QHBoxLayout.addWidget(self.first_cue_dir_QDoubleSpinBox)
        self.sidepanel_params_4_tab_QVBoxLayout.addLayout(self.first_cue_dir_QHBoxLayout)
        
        self.num_cue_dir_QHBoxLayout = QHBoxLayout()
        self.num_cue_dir_QLabel = QLabel("Number of Cue Directions:")
        self.num_cue_dir_QLabel.setAlignment(Qt.AlignRight)
        self.num_cue_dir_QHBoxLayout.addWidget(self.num_cue_dir_QLabel)
        self.num_cue_dir_QDoubleSpinBox = QDoubleSpinBox()
        self.num_cue_dir_QDoubleSpinBox.setValue(1)
        self.num_cue_dir_QDoubleSpinBox.setMaximum(8)
        self.num_cue_dir_QDoubleSpinBox.setSingleStep(1)
        self.num_cue_dir_QDoubleSpinBox.setDecimals(0)
        self.num_cue_dir_QHBoxLayout.addWidget(self.num_cue_dir_QDoubleSpinBox)
        self.sidepanel_params_4_tab_QVBoxLayout.addLayout(self.num_cue_dir_QHBoxLayout)
        
        self.reverse_prob_QHBoxLayout = QHBoxLayout()
        self.reverse_prob_QLabel = QLabel("Probability of Reversed Cue:")
        self.reverse_prob_QLabel.setAlignment(Qt.AlignRight)
        self.reverse_prob_QHBoxLayout.addWidget(self.reverse_prob_QLabel)
        self.reverse_prob_QDoubleSpinBox = QDoubleSpinBox()
        self.reverse_prob_QDoubleSpinBox.setValue(0)
        self.reverse_prob_QDoubleSpinBox.setMaximum(1)
        self.reverse_prob_QDoubleSpinBox.setSingleStep(0.1)
        self.reverse_prob_QDoubleSpinBox.setDecimals(2)
        self.reverse_prob_QHBoxLayout.addWidget(self.reverse_prob_QDoubleSpinBox)
        self.sidepanel_params_4_tab_QVBoxLayout.addLayout(self.reverse_prob_QHBoxLayout)
        
        self.save_QPushButton = QPushButton('Save parameters')
        self.sidepanel_custom_QVBoxLayout.addWidget(self.save_QPushButton)
        
        self.mouse_enable_QPushButton = QPushButton('Switch to Mouse Mode');
        self.mouse_disable_QPushButton = QPushButton('Switch to Eye Tracker Mode');
        self.sidepanel_custom_QVBoxLayout.addWidget(self.mouse_enable_QPushButton)
        self.sidepanel_custom_QVBoxLayout.addWidget(self.mouse_disable_QPushButton)
        self.mouse_enable_QPushButton.setEnabled(True)
        self.mouse_disable_QPushButton.setDisabled(True)
        
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
                         'ITI':0.1,
                         'pump_switch_interval':50,
                         'cue_duration':0.2,
                         'mask_duration':0.1,
                         'cue_probability':1.0,
                         'min_delay':0,
                         'max_delay':0,
                         'cue_type':'arrow',
                         'num_tgt_display':1,
                         'randomize_targets':False,
                         'ambiguity_prob':0.0,
                         'choice_prob':0.0,
                         'fixed_cue_pos':True,
                         'first_cue_dir':90,
                         'num_cue_dir':1,
                         'center_cue':False
                         }
        return parameter
    
    def update_parameter(self):
        '''
        update GUI parameters with the loaded parameters
        '''
        self.horz_offset_QDoubleSpinBox.setValue(self.exp_parameter['horz_offset'])
        self.vert_offset_QDoubleSpinBox.setValue(self.exp_parameter['vert_offset'])
        #self.max_allow_time_QDoubleSpinBox.setValue(self.exp_parameter['max_allow_time'])
        self.min_fix_time_QDoubleSpinBox.setValue(self.exp_parameter['min_fix_time'])
        self.max_wait_fixation_QDoubleSpinBox.setValue(self.exp_parameter['max_wait_for_fixation'])
        #self.pun_time_QDoubleSpinBox.setValue(self.exp_parameter['pun_time'])
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
        
        self.iti_QDoubleSpinBox.setValue(self.exp_parameter['ITI'])
        self.pump_switch_QDoubleSpinBox.setValue(self.exp_parameter['pump_switch_interval'])
        self.cue_duration_QDoubleSpinBox.setValue(self.exp_parameter['cue_duration'])
        self.mask_duration_QDoubleSpinBox.setValue(self.exp_parameter['mask_duration'])
        self.cue_probability_QDoubleSpinBox.setValue(self.exp_parameter['cue_probability'])
        cue_types = ['arrow','landolt','both']
        self.cue_type_QComboBox.setCurrentIndex(cue_types.index(self.exp_parameter['cue_type']))
        self.min_delay_QDoubleSpinBox.setValue(self.exp_parameter['min_delay'])
        self.max_delay_QDoubleSpinBox.setValue(self.exp_parameter['max_delay'])
        self.num_tgt_display_QDoubleSpinBox.setValue(self.exp_parameter['num_tgt_display'])
        self.random_tgt_QCheckBox.setChecked(self.exp_parameter['randomize_targets'])
        self.ambiguity_prob_QDoubleSpinBox.setValue(self.exp_parameter['ambiguity_prob'])
        self.choice_prob_QDoubleSpinBox.setValue(self.exp_parameter['choice_prob'])
        
        self.fixed_cue_pos_QCheckBox.setChecked(self.exp_parameter['fixed_cue_pos'])
        self.first_cue_dir_QDoubleSpinBox.setValue(self.exp_parameter['first_cue_dir'])
        self.num_cue_dir_QDoubleSpinBox.setValue(self.exp_parameter['num_cue_dir'])
        self.center_cue_QCheckBox.setChecked(self.exp_parameter['center_cue'])
        
        if self.center_cue_QCheckBox.isChecked():
            self.fixed_cue_pos_QCheckBox.setDisabled(True)
            self.fixed_cue_pos_QCheckBox.setChecked(False)
            self.num_cue_dir_QDoubleSpinBox.setDisabled(True)
            self.num_cue_dir_QDoubleSpinBox.setValue(1)
            self.exp_parameter['num_cue_dir'] = 1
            self.first_cue_dir_QDoubleSpinBox.setDisabled(True)
        
        elif self.fixed_cue_pos_QCheckBox.isChecked():
            self.num_cue_dir_QDoubleSpinBox.setDisabled(True)
            self.num_cue_dir_QDoubleSpinBox.setValue(1)
            self.exp_parameter['num_cue_dir'] = 1
        else:
            self.num_cue_dir_QDoubleSpinBox.setEnabled(True)
        


        
class DelaySacGuiProcess(multiprocessing.Process):
    def __init__(self, exp_name, fsm_to_gui_rcvr, gui_to_fsm_sndr, stop_exp_Event, stop_fsm_process_Event,mouse_toggle_Event, real_time_data_Array, main_parameter, parent=None):
        super(DelaySacGuiProcess,self).__init__(parent)
        self.exp_name = exp_name
        self.fsm_to_gui_rcvr = fsm_to_gui_rcvr
        self.gui_to_fsm_sndr = gui_to_fsm_sndr
        self.stop_exp_Event = stop_exp_Event
        self.real_time_data_Array = real_time_data_Array
        self.stop_fsm_process_Event = stop_fsm_process_Event
        self.mouse_toggle_Event = mouse_toggle_Event
        self.main_parameter = main_parameter
    def run(self):  
        app = QApplication(sys.argv)
        app_gui = DelaySacGui(self.exp_name, self.fsm_to_gui_rcvr, self.gui_to_fsm_sndr, self.stop_exp_Event, self.stop_fsm_process_Event,self.mouse_toggle_Event, self.real_time_data_Array, self.main_parameter)
        app_gui.setWindowIcon(QtGui.QIcon(os.path.join('.', 'icon', 'experiment_window.png')))
        app_gui.show()
        sys.exit(app.exec())

        
