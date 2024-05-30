import azure.cognitiveservices.speech as speechsdk
import os
from dotenv import load_dotenv



class SttAzureInterface:
    """
    This class takes a .wav file and transcribes it to a string text.
    For this it uses the azure speech API 
    """
    def __init__(self):
        load_dotenv()
        self.speechsdk_key = os.environ["SPEECH_KEY"]
        self.speechsdk_region =  os.environ["SPEECH_REGION"]
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
        #print("evl wav was called.")  #debugging statement
        #print(f"{self.speechsdk_key} {self.speechsdk_region}")
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
        elif cancellation_details.reason == speechsdk.CancellationReason.Error:
            print("Error details: {}".format(cancellation_details.error_details))
            print("Did you set the speech resource key and region values?")
            return None
        
        self.delete_wav_file(wav_file)

def main():
    # this section is only for testing purposes. The call to this class needs to be made from ros_stt.py
    stt = SttAzureInterface()
    print(stt.evaluate_wav("/home/nao/NAO_WS/src/ros_stt/ros_stt/test1.wav"))
    input("Press Enter to continue...")
    print(stt.evaluate_wav("/home/nao/NAO_WS/src/ros_stt/ros_stt/test2.wav"))
    


if __name__ == "__main__":
    main()