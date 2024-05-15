import webrtcvad
import time
import os
import wave


class SttPresampler:
    """
    This class is to presample the /audio topic from NAO
    It filters audio samples in which people are speaking and not. 
    At the end it outputs a single .wav file in which the spoken promt to NAO is contained.
    It also downsamples the /audiotopic to be accepted by the speak-to-text modul.
    This can then further be sent to a speak-to-text node which processes the file to text.
    """
    def __init__(self):
        self.FRAME_DURATION_MS = 30  # duration of one sample passed to VAD to determain if somebody is speaking. 
        self.SAMPLE_RATE = 16000  # rate at which the sudio is sampled.
        self.SAMPLE_WITH = 2  #  
        self.CHANNELS = 1  # number of channels from where audio is streamed.
        self.CHUNK_SIZE = int(self.SAMPLE_RATE * self.FRAME_DURATION_MS / 1000) * self.SAMPLE_WITH
        self.SILENCE_DURATION = 1
        self.audio_buffer = []  # buffer used to by VAD to determain if somebody is speaking
        self.DELEAT_OLD_WAV = True
        self.wav_file = []  # all audio gets saved here unitl it is terminated.
        self.vad = webrtcvad.Vad(1)


    def is_spooken(self, buffer):
        return self.vad.is_speech(buffer, self.SAMPLE_RATE)


    def add_buffer_to_wav(self, buffer):
        self.wav_file.append(buffer)


    def reset_wav(self):
        self.wav_file = []


def main():
    # this section is only for testing purposes. The call to this class needs to be made from ros_stt.py
    pass


if __name__ == "__main__":
    main()