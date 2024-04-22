import webrtcvad as vad
import speech_recognition as sr
from dotenv import load_dotenv

class SttAzureInterface:
    """
    This class takes a .wav file and transcribes it to a string text.
    For this it uses the azure speech API 
    """
    def __init__(self):
        pass