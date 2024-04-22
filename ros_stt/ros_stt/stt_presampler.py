import webrtcvad as vad
import time
import os

class SttPresampler:
    """
    This class is to presample the /audio topic from NAO
    It filters audio samples in which people are speaking and not. 
    At the end it outputs a single .wav file in which the spoken promt to NAO is contained.
    This can then furter besent to a speak-to-text node which processes the file to text.
    """
    def __init__(self):
        pass