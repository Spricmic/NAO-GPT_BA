import azure.cognitiveservices.speech as speechsdk
import os
import csv
import time
from dotenv import load_dotenv


class SttAzureInterface:
    """
    This class takes a .wav file and transcribes it to a string text.
    For this it uses the azure speech API 
    """

    def __init__(self):
        load_dotenv()
        self.speechsdk_key = os.environ["SPEECH_KEY"]
        self.speechsdk_region = os.environ["SPEECH_REGION"]
        if not self.speechsdk_key or not self.speechsdk_region:
            raise ValueError("Azure speech key or region not found in environment variables")
        self.speech_config = speechsdk.SpeechConfig(subscription=self.speechsdk_key,
                                                    region=self.speechsdk_region)

    def delete_wav_file(self, wav_file):
        """
        deletes a file

        :param wav_file: path to the file that should be deleted.
        """
        os.remove(wav_file)

    def evaluate_wav(self, wav_file):
        """
        Sends the call to the API which interacts with the Azure speechservices. Returns the transcribed audio as string.

        :param wav_file: Name or Path to the wav_file to transcribe.
        :return: The response from SpeechServices.
        """
        # print("evl wav was called.")  #debugging statement
        self.audio_config = speechsdk.AudioConfig(filename=str(wav_file))
        self.speech_recognizer = speechsdk.SpeechRecognizer(speech_config=self.speech_config,
                                                            audio_config=self.audio_config)
        self.result = self.speech_recognizer.recognize_once_async().get()

        if self.result.reason == speechsdk.ResultReason.RecognizedSpeech:
            print("Recognized: {}".format(self.result.text))
            return str(self.result.text)
        elif self.result.reason == speechsdk.ResultReason.NoMatch:
            print("No speech could be recognized: {}".format(self.result.no_match_details))
            return None
        elif self.result.reason == speechsdk.ResultReason.Canceled:
            cancellation_details = self.result.cancellation_details
            print("Speech Recognition canceled: {}".format(cancellation_details.reason))
            return None
        elif self.result.cancellation_details.reason == speechsdk.CancellationReason.Error:
            print("Error details: {}".format(self.result.cancellation_details.error_details))
            print("Did you set the speech resource key and region values?")
            return None

        # self.delete_wav_file(wav_file)


def main():
    # this section is only for testing purposes. The call to this class needs to be made from ros_stt.py
    stt = SttAzureInterface()
    log_file = 'transcription_log2.csv'

    with open(log_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Filename", "Response Duration (seconds)", "Transcribed Text"])

        i = 1
        while i <= 78 :
            wav_file = "test" + str(i)
            full_path = str("C:/Users/michi/Documents/BALocal/BA_GIT/NAO-GPT_BA/temp_storage/") + wav_file + str(".wav")
            if wav_file.lower() == 'q':
                break

            if not os.path.isfile(full_path):
                print(f"The file {wav_file} does not exist.")
                print(f"path tried: {full_path}")
                continue

            start_time = time.time()
            transcribed_text = stt.evaluate_wav(full_path)
            end_time = time.time()
            response_duration = end_time - start_time

            writer.writerow([wav_file, response_duration, transcribed_text])
            print(f"Logged transcription for {wav_file}")
            i = i+1

    print(f"Transcriptions logged in {log_file}")


if __name__ == "__main__":
    main()
