import sys
import meta_features as mf
import numpy as np
import os
from sklearn.externals import joblib

class MotionPrimitive():

    SIGNALS = {'STARTED': 1, 'CLOSED': 0, 'EMPTY': -1}

    def __init__(self, init_time, threshold=0.03):
        self.buffer = []
        self.threshold = threshold
        self.current_state = MotionPrimitive.SIGNALS['EMPTY']
        self.initial_reference_time = init_time
        self.clf = joblib.load(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'classifier.pkl'))
    def expand(self,value, value_time):
        if value > self.threshold:
            if self.current_state == MotionPrimitive.SIGNALS['CLOSED']:
                raise RuntimeError('You should empty the buffer upon calling emptyBuffer() or getFeatures()!')
                sys.exit(-1)

            self.buffer.append((value, (value_time - self.initial_reference_time).to_sec()))
            self.current_state = MotionPrimitive.SIGNALS['STARTED']
            return True
        elif (value <= self.threshold) and self.buffer:
            self.current_state =  MotionPrimitive.SIGNALS['CLOSED']
            return False
        else:
            self.current_state =  MotionPrimitive.SIGNALS['EMPTY']
            return False
        
    def getFeatures(self):
    
        buffer = self.emptyBuffer()
        data = [i[0] for i in buffer]                            # data
        duration = (buffer[-1][-1] - buffer[0][-1])              # activity duration

        if not data:
            raise ValueError("Buffer is empty")

        else:
            # calculating features
            std = mf.std(data)
            max_min = mf.max_min(data)
            sma = mf.sma(data)
            rms = mf.rms(data)
            fft_energy = mf.fft_energy(data)
            max_value = mf.max_value(data)

            #NOTE self.clf.predict_proba returns a list of prediction. Since we have a simple sample
            # per time, we return the zero-index.
            return {"prediction": self.clf.predict_proba(np.array([std,max_min,sma,rms,
                                                                   fft_energy,max_value]).reshape(1, -1))[0],
                    "duration" : duration}

    def emptyBuffer(self):
        data = self.buffer
        self.clear()
        self.current_state =  MotionPrimitive.SIGNALS['EMPTY']
        return data

    def clear(self):
        self.buffer = []
