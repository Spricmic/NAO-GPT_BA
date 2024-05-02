import webrtcvad as vad
import azure.cognitiveservices.speech as speechsdk
import time
from dotenv import load_dotenv


class SttAzureInterface:
    """
    This class takes a .wav file and transcribes it to a string text.
    For this it uses the azure speech API 
    """
    def __init__(self, wav_file):
        pass