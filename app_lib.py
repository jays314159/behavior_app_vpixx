"""
Laboratory for Computational Motor Control, Johns Hopkins School of Medicine
@author: Jay Pi <jay.s.314159@gmail.com>
"""
from pypixxlib._libdpx import DPxSelectDevice, TPxSetupTPxSchedule, TPxEnableFreeRun, DPxUpdateRegCache, TPxDisableFreeRun

import math, simpleaudio, ctypes, json, os
import numpy as np
from matplotlib import path
from pathlib import Path

def playSound(freq, duration):
    '''
    plays the beep. Use only for imprecise sound timing
    Arguments:
    freq - frequency in Hz (int)
    duration - duration of beep in s (float). 
    '''
    fs = 44100  # 44100 samples per second
    # Generate array with duration*sample_rate steps, ranging between 0 and duration
    t = np.linspace(0, duration, int(duration * fs), False)
    
    # Generate a sine wave
    note = np.sin(freq * t * 2 * np.pi)
    
    # Ensure that highest value is in 16-bit range
    audio = note * (2**15 - 1) / np.max(np.abs(note))
    # Convert to 16-bit data
    audio = audio.astype(np.int16)
    
    # Start playback
    simpleaudio.play_buffer(audio, 1, 2, fs)
    
    return 0
def set_default_exp_parameter(exp_name):
    if exp_name == 'simple_saccade':
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
                         'first_dir': 0,
                         'ITI':0.1,
                         'pump_switch_interval':50}
    return exp_parameter
def load_parameter(folder_name,file_name,multi_instance,default_parameter_fnc, instance_name=None):
    '''
    Arguments:
        folder_name - name of the folder containing parameter file (str)
                    - if file in parent folder, input empty string ''
        file_name - name of the file containing parameter (str)
        multi_instance - specifies if there are different instances of parameters
                         in a single file (boolean)
        default_parameter_fnc - function/method that returns default parameters 
                                if no saved parameters found                 
        instance_name - specifies instance name if 'multi_instance' is true (optional, str)
        
    Returns:
        parameter - dictionary of loaded parameters or default parameters in case
                    no saved parameters found
        parameter_file_path - full path of parameter file
                            - e.g., 'C\\Users\\a\\app\\exp_parameter.json'
    '''
    print(instance_name)
    parameter_file_path = os.path.join(str(Path().absolute()),folder_name,file_name)
    if os.path.exists(parameter_file_path):
        with open(parameter_file_path,'r+') as file:
            if multi_instance == True:
                all_parameter = json.load(file)
                if instance_name in all_parameter:
                    parameter = all_parameter[instance_name]
                # If specific instance of parameters not exist
                else:
                    parameter = default_parameter_fnc(instance_name)
                    all_parameter[instance_name] = parameter
                    file.seek(0)
                    json.dump(all_parameter,file,indent=4)
            else:
                parameter = json.load(file)
    # If no file exists
    else:
        parameter = default_parameter_fnc(instance_name)
        with open(parameter_file_path,'w') as file:
            if multi_instance == True:
                json.dump({instance_name: parameter}, file, indent=4)
            else:
                json.dump(parameter, file, indent=4)
                    
    return parameter, parameter_file_path    



def set_default_cal_parameter(cal_name):
    '''
    # Arguments:
    #     parameter - empty or partially filled dictionary of default calibration parameters
    Returns:
        parameter - dictionary of default calibration parameters 
    '''
    if cal_name == 'calibration':
        parameter = {'start_x': 0,
                     'start_y': 0,
                     'pursuit_amp': 0.7,
                     'pursuit_dur': 0.7,
                     'is_pursuit_tgt': True,
                     'cal_dur': 1,
                     'ITI': 1,
                     'tgt_1': [-5,5,True],
                     'tgt_2': [0,5,True],
                     'tgt_3': [5,5,True],
                     'tgt_4': [-5,0,True],
                     'tgt_5': [0,0,True],
                     'tgt_6': [5,0,True],
                     'tgt_7': [-5,-5,True],
                     'tgt_8': [0,-5,True],
                     'tgt_9': [5,-5,True],
                     'left_cal_matrix': [[0,0,0],[0,0,0],[0,0,0]],
                     'left_cal_status': False,
                     'RMSE': 0.0
                     }
    elif cal_name == 'refinement':
        parameter = {'mode': 'Auto',
                     'tgt_pos': [0,0],
                     'tgt_dur': 1,
                     'center_tgt_pos': [0,0],
                     'dist_from_center': 4.0,
                     'tgt_manual_list': [0,0],
                     'tgt_auto_list': [[0,0]],
                     'rew_area': 4,
                     'auto_pump': True
                    }
    return parameter
    
def inpolygon(xq, yq, xv, yv):
    """
    checks which points are inside of the specified polygon
    Arguments:
        xv - np.array([xv1, xv2, xv3, ..., xvN, xv1])
        yv - np.array([yv1, yv2, yv3, ..., yvN, yv1])
        xq - np.array([xq1, xq2, xq3, ..., xqN])
        yq - np.array([yq1, yq2, yq3, ..., yqN])
    Returns:
        contained_p - np.bool array indicating if the query points specified by xq and yq
                      are inside of the polygon area defined by xv and yv
    """
    shape = xq.shape
    xq = xq.reshape(-1)
    yq = yq.reshape(-1)
    xv = xv.reshape(-1)
    yv = yv.reshape(-1)
    q = [(xq[i], yq[i]) for i in range(xq.shape[0])]
    p = path.Path([(xv[i], yv[i]) for i in range(xv.shape[0])])
    contained_p = p.contains_points(q).reshape(shape)
    return contained_p

def VPixx_turn_on_schedule():
    '''
    ready VPixx to get data.
    Run once before finite state machine starts
    '''
    DPxSelectDevice('DATAPIXX3')
    TPxSetupTPxSchedule()
    TPxEnableFreeRun()
    DPxUpdateRegCache()

def VPixx_turn_off_schedule():
    '''
    disable VPixx data collection.
    Run once after finite state machine finishes
    '''
    DPxSelectDevice('DATAPIXX3')
    TPxSetupTPxSchedule() # resets data
    TPxDisableFreeRun()
    DPxUpdateRegCache()

def VPixx_get_pointers_for_data():
    '''
    need 'ctypes' pointers to pass into 'TPxBestPolyGetEyePosition' 
    to get eye data. See examples use.
    Returns:
        cal_data - dummy var., as we don't use VPixx device calibration
        raw_data - list. After passing into 'TPxBestPolyGetEyePosition' gets populated with
                   left x, left y, right x, right y raw eye data
    '''
    length = 4
    int_list = [0]*length
    item_count = len(int_list)
    int_list2 = [0]*length
    item_count2 = len(int_list2)
    cal_data = (ctypes.c_double * item_count)(*int_list) # dummy var., won't be using values calibrated from device. Arg. for 'TPxBestPolyGetEyePosition'
    raw_data = (ctypes.c_double * item_count2)(*int_list2)
    return cal_data, raw_data
    
def make_prim_target(parameter):
    '''
    Arguments:
    parameter - dictionary of parameters
    Returns:
    tgt_list - a list of target positions uniformly distributed around a circle according to a
                  number of targets and their amplitude
             - each element is a dictionary with all parameters needed for a target
    '''
    tgt_list = []
    num_prim_sac_dir = parameter['num_prim_sac_dir']
    prim_sac_amp = parameter['prim_sac_amp']
    first_dir = parameter['first_prim_sac_dir']
    
    for prim_tgt_idx in range(num_prim_sac_dir):
        prim_tgt_dir = 2*math.pi/num_prim_sac_dir*prim_tgt_idx + first_dir*math.pi/180
        prim_tgt_x = prim_sac_amp*math.cos(prim_tgt_dir)
        prim_tgt_y = prim_sac_amp*math.sin(prim_tgt_dir)
        
        tgt_list.append({'prim_tgt_pos': [prim_tgt_x, prim_tgt_y]})
    return tgt_list

def make_corr_target(parameter):
    '''
    Arguments:
    parameter - dictionary of parameters
    Returns:
    tgt_list - a list of target positions uniformly distributed around a circle according to a
                  number of targets and their amplitude
             - each element is a dictionary with all parameters needed for a set of 
               primary and secondary targets
    '''
    tgt_list = []
    num_prim_sac_dir = parameter['num_prim_sac_dir']
    prim_sac_amp = parameter['prim_sac_amp']
    first_prim_sac_dir = parameter['first_prim_sac_dir']
    num_corr_sac_dir = parameter['num_corr_sac_dir']
    corr_sac_amp = parameter['corr_sac_amp']
    first_corr_sac_dir = parameter['first_corr_sac_dir']
    
    for prim_tgt_idx in range(num_prim_sac_dir):
        prim_tgt_dir = 2*math.pi/num_prim_sac_dir*prim_tgt_idx + first_prim_sac_dir*math.pi/180
        prim_tgt_x = prim_sac_amp*math.cos(prim_tgt_dir)
        prim_tgt_y = prim_sac_amp*math.sin(prim_tgt_dir)
        for corr_tgt_idx in range (num_corr_sac_dir):
            corr_tgt_dir = 2*math.pi/num_corr_sac_dir*corr_tgt_idx + first_corr_sac_dir*math.pi/180
            corr_tgt_x = corr_sac_amp*math.cos(corr_tgt_dir) + prim_tgt_x
            corr_tgt_y = corr_sac_amp*math.sin(corr_tgt_dir) + prim_tgt_y
            tgt_list.append({'prim_tgt_pos': [prim_tgt_x, prim_tgt_y],
                             'corr_tgt_pos': [corr_tgt_x, corr_tgt_y]})
    
    return tgt_list

def raw_to_deg(raw_data, cal_matrix):
    '''
    converts raw VPixx data into screen values in degrees
    Arguments:
        raw_data - [raw x, raw y, 1]
        cal_matrix - [[c1,c2,0],[c3,c4,0],[c5,c6,1]] 
                     where c5 is x bias and c6 is y bias
    Returns:
        deg_data[0:2] - x and y in degrees (list) where
                        x is c1*x_raw + c3*y_raw + c5 and
                        y is c2*x_raw + c4*y_raw + c6 
    '''
    deg_data = np.array(raw_data) @ cal_matrix 
    return deg_data[0:2]

