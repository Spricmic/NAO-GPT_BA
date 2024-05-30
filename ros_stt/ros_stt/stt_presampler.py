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
        self.SAMPLE_WITH = 2  # sample with in bytes
        self.CHANNELS = 1  # number of channels from where audio is streamed.
        self.CHUNK_SIZE = int(self.SAMPLE_RATE * self.FRAME_DURATION_MS / 1000) * self.SAMPLE_WITH
        self.SILENCE_DURATION = 1
        self.audio_buffer = []  # buffer used to by VAD to determain if somebody is speaking
        self.DELEAT_OLD_WAV = True
        self.audio_data = []  # all audio gets saved here unitl it is terminated.
        self.is_recording = False
        self.vad = webrtcvad.Vad(1)


    def start_recording(self):
        self.audio_data = []
        self.is_recording = True

    def stop_recording(self, filename):
        self.is_recording = False
        self.save_to_wav(filename)

    def extend_audio(self, data):
        if self.is_recording:
            self.audio_data.extend(data)

    def save_to_wav(self, filename):
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(self.SAMPLE_WITH)
            wf.setframerate(self.SAMPLE_RATE)
            print(f"trying to save: {b''.join([int(x).to_bytes(self.SAMPLE_WITH, byteorder='little', signed=True) for x in self.audio_data])}")
            wf.writeframes(b''.join([int(x).to_bytes(self.SAMPLE_WITH, byteorder='little', signed=True) for x in self.audio_data]))
            print("stt_presampler: saved .wav file")

    def is_spooken(self, buffer):
        return self.vad.is_speech(buffer, self.SAMPLE_RATE)


    def add_buffer_to_wav(self, buffer):
        if self.is_recording:
            self.audio_data.append(buffer)


def main():
    # this section is only for testing purposes. The call to this class needs to be made from ros_stt.py
    pass


if __name__ == "__main__":
    main()