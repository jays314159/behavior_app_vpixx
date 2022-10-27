from PyQt5.QtCore import QRunnable, pyqtSlot, QThreadPool

import numpy as np
import simpleaudio

class SoundWorker(QRunnable):
    def __init__(self, freq, duration):
        super().__init__()
        self.setAutoDelete(False)
        
        self.fs = 44100  # 44100 samples per second
        # Generate array with duration*sample_rate steps, ranging between 0 and duration
        t = np.linspace(0, duration, int(duration * self.fs), False)
        # Generate a sine wave
        note = np.sin(freq * t * 2 * np.pi)
        
        # Ensure that highest value is in 16-bit range
        self.audio = note * (2**15 - 1) / np.max(np.abs(note))
        # Convert to 16-bit data
        self.audio = self.audio.astype(np.int16)
    @pyqtSlot()
    def run(self):
        simpleaudio.play_buffer(self.audio,1,2,self.fs)
        
class Sound():
    def __init__(self, freq, duration):
        self.sound_worker = SoundWorker(freq, duration)
        self.thread_pool = QThreadPool() 
    def play(self):
        # self.thread_pool.start(self.sound_worker)
        self.sound_worker.run()