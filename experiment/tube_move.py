"""
Laboratory for Computational Motor Control, Johns Hopkins School of Medicine
@author: Mohammad Reza Heydari <drmrheydari@gmail.com>
"""

import math
import multiprocessing
import os
import sys
from functools import partial
import random

import zmq
from PyQt5 import QtGui
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QHBoxLayout, QPushButton, QApplication
from psychopy import monitors, visual, core
from pypixxlib import tracker
from pypixxlib._libdpx import (  # NoQA
    DPxOpen,
    DPxSetTPxAwake,
    DPxSelectDevice,
    DPxUpdateRegCache,
    DPxSetDoutValue,
    DPxGetTime,
    TPxBestPolyGetEyePosition,
    DPxGetReg16,
    DPxSetTPxSleep,
    DPxClose,
    TPxReadTPxData,
    TPxSetupTPxSchedule,
)

import app_lib as lib
from data_manager import DataManager
from fsm_gui import FsmGui


class TubeMoveFsmProcess(multiprocessing.Process):
    def __init__(
        self,
        exp_name,
        fsm_to_gui_sndr,
        gui_to_fsm_q,
        stop_exp_event,
        stop_fsm_process_event,
        real_time_data_array,
        main_parameter,
        tube_move_left_Event,
        tube_move_right_Event,
        tube_move_center_Event,
        mon_parameter,
    ):
        super().__init__()
        self.exp_name = exp_name
        self.fsm_to_gui_sndr = fsm_to_gui_sndr
        self.gui_to_fsm_q = gui_to_fsm_q
        self.stop_exp_event = stop_exp_event
        self.stop_fsm_process_event = stop_fsm_process_event
        self.real_time_data_array = real_time_data_array
        self.main_parameter = main_parameter
        self.tube_move_left_Event = tube_move_left_Event
        self.tube_move_right_Event = tube_move_right_Event
        self.tube_move_center_Event = tube_move_center_Event
        self.mon_parameter = mon_parameter

        self.window = None
        self.t = 0

    def run(self):
        # Set up exp. screen
        this_monitor = monitors.Monitor(
            self.mon_parameter["monitor_name"],
            width=self.mon_parameter["monitor_width"],
            distance=self.mon_parameter["monitor_distance"],
        )
        this_monitor.save()
        this_monitor.setSizePix(self.mon_parameter["monitor_size"])
        self.window = visual.Window(
            size=self.mon_parameter["monitor_size"],
            screen=self.mon_parameter["monitor_num"],
            allowGUI=False,
            color="white",
            monitor=this_monitor,
            units="deg",
            winType="pyglet",
            fullscr=True,
            checkTiming=False,
            waitBlanking=True,
        )
        self.window.flip()

        # Check if VPixx available; if so, open
        DPxOpen()
        tracker.TRACKPixx3().open()  # this throws error if not device not open
        DPxSetTPxAwake()
        DPxSelectDevice("DATAPIXX3")
        DPxUpdateRegCache()

        # Get pointers to store data from device
        cal_data, raw_data = lib.VPixx_get_pointers_for_data()

        # Init. var.
        random_signal_flip_duration = 0.015  # in sec., how often to flip random signal
        dout_ch_1 = 1  # nominal PD
        dout_ch_3 = 0  # random signal
        dout_ch_5 = 1  # LED
        DPxSetDoutValue(dout_ch_1 + (2 ** 2) * dout_ch_3 + (2 ** 4) * dout_ch_5, 0xFFFFFF)
        DPxUpdateRegCache()

        run_exp = False
        random_signal_t = math.nan
        bit_mask = 1 << 2 | 1 << 4
        # Process loop
        while not self.stop_fsm_process_event.is_set():
            if not self.stop_exp_event.is_set():
                # Turn on VPixx schedule; this needed to collect data
                lib.VPixx_turn_on_schedule()
                # Load exp parameter
                fsm_parameter, _ = lib.load_parameter(
                    "experiment",
                    "exp_parameter.json",
                    True,
                    True,
                    self.set_default_parameter,
                    self.exp_name,
                    self.main_parameter["current_monkey"],
                )
                # Init. var
                DPxUpdateRegCache()
                self.t = DPxGetTime()
                random_signal_t = self.t

                run_exp = True

            if self.tube_move_left_Event.is_set():
                DPxSetDoutValue(1 << 2 | 0 << 4, bit_mask)
            elif self.tube_move_center_Event.is_set():
                DPxSetDoutValue(0, bit_mask)
            elif self.tube_move_right_Event.is_set():
                DPxSetDoutValue(0 << 2 | 1 << 4, bit_mask)
            DPxUpdateRegCache()

            # Trial loop
            while not self.stop_fsm_process_event.is_set() and run_exp:
                if self.stop_exp_event.is_set():
                    run_exp = False
                    self.t = math.nan
                    # Turn off VPixx schedule
                    lib.VPixx_turn_off_schedule()
                    # Remove all targets
                    self.window.flip()
                    break

                # FSM loop
                while not self.stop_fsm_process_event.is_set() and run_exp:
                    if self.stop_exp_event.is_set():
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
                    DPxSetDoutValue(dout_ch_1 + (2 ** 2) * dout_ch_3 + (2 ** 4) * dout_ch_5, 0xFFFFFF)
                    # Get time
                    self.t = TPxBestPolyGetEyePosition(
                        cal_data, raw_data
                    )  # this calls 'DPxUpdateRegCache' as well

        # Close PsychoPy
        core.quit()
        # Turn off VPixx schedule
        lib.VPixx_turn_off_schedule()
        # Close VPixx devices
        DPxSetTPxSleep()
        DPxSelectDevice("DATAPIXX3")
        DPxUpdateRegCache()
        DPxClose()
        tracker.TRACKPixx3().close()
        # Reset digital out
        dout_ch_1 = 1  # nominal PD
        dout_ch_3 = 0  # random signal
        dout_ch_5 = 1  # LED
        DPxSetDoutValue(dout_ch_1 + (2**2) * dout_ch_3 + (2**4) * dout_ch_5, 0xFFFFFF)
        DPxUpdateRegCache()

    def set_default_parameter(self):
        parameter = {
            "horz_offset": 0.0,
            "vert_offset": 0.0,
            "max_allow_time": 0.7,
            "min_fix_time": 0.1,
            "max_wait_for_fixation": 1.5,
            "pun_time": 0.1,
            "time_to_reward": 0.1,
            "sac_detect_threshold": 150.0,
            "sac_on_off_threshold": 75.0,
            "rew_area": 3.0,
            "pursuit_amp": 0.1,
            "pursuit_dur": 0.1,
            "prim_sac_amp": 4.0,
            "num_prim_sac_dir": 8,
            "first_prim_sac_dir": 0,
            "ITI": 0.1,
            "pump_switch_interval": 50,
        }
        return parameter


class TubeMoveGui(FsmGui):
    def __init__(
        self,
        exp_name,
        fsm_to_gui_rcvr,
        gui_to_fsm_sndr,
        stop_exp_event,
        stop_fsm_process_event,
        real_time_data_array,
        tube_move_left_Event,
        tube_move_right_Event,
        tube_move_center_Event,
        main_parameter,
    ):
        self.exp_name = exp_name
        self.fsm_to_gui_rcvr = fsm_to_gui_rcvr
        self.gui_to_fsm_sndr = gui_to_fsm_sndr
        self.stop_exp_event = stop_exp_event
        self.stop_fsm_process_event = stop_fsm_process_event
        self.real_time_data_array = real_time_data_array
        self.tube_move_left_Event = tube_move_left_Event
        self.tube_move_right_Event = tube_move_right_Event
        self.tube_move_center_Event = tube_move_center_Event
        self.main_parameter = main_parameter
        super().__init__(self.stop_fsm_process_event)
        self.__init_gui__()

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
            self.log_QPlainTextEdit.appendPlainText("Error in starting zmq sockets:")
            self.log_QPlainTextEdit.appendPlainText(str(error) + ".")
            self.toolbar_run_QAction.setDisabled(True)
            self.toolbar_connect_QAction.setDisabled(True)

        # Load exp. parameter or set default values
        self.exp_parameter, self.parameter_file_path = lib.load_parameter(
            "experiment",
            "exp_parameter.json",
            True,
            True,
            self.set_default_parameter,
            self.exp_name,
            self.main_parameter["current_monkey"],
        )
        self.update_parameter()

        self.data_manager = DataManager()
        self.__init_signals__()

    def __init_signals__(self):
        self.move_left_QPushButton.clicked.connect(  # NoQA
            partial(self.move_q_push_button_clicked, "left")  # NoQA
        )
        self.move_center_QPushButton.clicked.connect(  # NoQA
            partial(self.move_q_push_button_clicked, "center")  # NoQA
        )
        self.move_right_QPushButton.clicked.connect(  # NoQA
            partial(self.move_q_push_button_clicked, "right")  # NoQA
        )

    @pyqtSlot()
    def move_q_push_button_clicked(self, direction: str):
        """

        Args:
            direction: must be 'left', 'center', 'right'

        Returns:

        """
        if direction == "left":
            self.tube_move_right_Event.clear()
            self.tube_move_center_Event.clear()
            self.tube_move_left_Event.set()
        elif direction == "center":
            self.tube_move_right_Event.clear()
            self.tube_move_left_Event.clear()
            self.tube_move_center_Event.set()
        elif direction == "right":
            self.tube_move_left_Event.clear()
            self.tube_move_center_Event.clear()
            self.tube_move_right_Event.set()

    def __init_gui__(self):
        # Disable plots
        self.plot_1_PlotWidget.deleteLater()
        self.plot_2_PlotWidget.deleteLater()
        # Disable pumps
        self.pump_1.deleteLater()
        self.pump_2.deleteLater()
        # Disable target
        self.tgt.deleteLater()
        self.pd_tgt.deleteLater()
        # Side panel
        self.port_QHBoxLayout = QHBoxLayout()
        self.sidepanel_custom_QVBoxLayout.addLayout(self.port_QHBoxLayout)
        self.move_left_QPushButton = QPushButton("Left")
        self.port_QHBoxLayout.addWidget(self.move_left_QPushButton)
        self.move_center_QPushButton = QPushButton("Center")
        self.port_QHBoxLayout.addWidget(self.move_center_QPushButton)
        self.move_right_QPushButton = QPushButton("Right")
        self.port_QHBoxLayout.addWidget(self.move_right_QPushButton)

    @staticmethod
    def set_default_parameter():
        return {}

    def update_parameter(self):
        """
        update GUI parameters with the loaded parameters
        """
        pass


class TubeMoveGuiProcess(multiprocessing.Process):
    def __init__(
        self,
        exp_name,
        fsm_to_gui_rcvr,
        gui_to_fsm_sndr,
        stop_exp_event,
        stop_fsm_process_event,
        real_time_data_array,
        tube_move_left_Event,
        tube_move_right_Event,
        tube_move_center_Event,
        main_parameter,
        parent=None,
    ):
        super(TubeMoveGuiProcess, self).__init__(parent)
        self.exp_name = exp_name
        self.fsm_to_gui_rcvr = fsm_to_gui_rcvr
        self.gui_to_fsm_sndr = gui_to_fsm_sndr
        self.stop_exp_event = stop_exp_event
        self.real_time_data_array = real_time_data_array
        self.stop_fsm_process_event = stop_fsm_process_event
        self.main_parameter = main_parameter
        self.tube_move_left_Event = tube_move_left_Event
        self.tube_move_right_Event = tube_move_right_Event
        self.tube_move_center_Event = tube_move_center_Event

    def run(self):
        app = QApplication(sys.argv)
        app_gui = TubeMoveGui(
            self.exp_name,
            self.fsm_to_gui_rcvr,
            self.gui_to_fsm_sndr,
            self.stop_exp_event,
            self.stop_fsm_process_event,
            self.real_time_data_array,
            self.tube_move_left_Event,
            self.tube_move_right_Event,
            self.tube_move_center_Event,
            self.main_parameter,
        )
        app_gui.setWindowIcon(
            QtGui.QIcon(os.path.join(".", "icon", "experiment_window.png"))
        )
        app_gui.show()
        sys.exit(app.exec())
