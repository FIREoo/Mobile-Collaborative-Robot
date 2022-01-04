#!/usr/bin/env python3

from __future__ import print_function
from typing import Text

from nlp_pkg.srv import AddTwoInts, AddTwoIntsResponse
from std_srvs.srv import Empty, EmptyResponse
from std_srvs.srv import SetBool, SetBoolResponse
import rospy

import pyaudio
import wave
import threading
import time
import speech_recognition
import jieba


class Record:

    def __init__(self):
        self.CHUNK = 2**10
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 44100
        self.RECORD_SECONDS = 1
        self.WAVE_OUTPUT_FILENAME = "output.wav"

        self.loop = True
        self.frames = []
        self.mic = pyaudio.PyAudio()

    def recording(self):
        self.stream = self.mic.open(format=self.FORMAT, channels=self.CHANNELS, rate=self.RATE, input=True, frames_per_buffer=self.CHUNK)
        while (self.loop):
            # time.sleep(self.RECORD_SECONDS)
            data = self.stream.read(self.CHUNK)
            self.frames.append(data)

    def start(self):
        # t = threading.Thread(target=self.recording, args=(self, ))
        self.loop = True
        self.frames = []
        t = threading.Thread(target=self.recording)
        t.start()

    def stop(self):
        self.loop = False
        self.stream.stop_stream()
        self.stream.close()
        # self.mic.terminate()
        outputFile = wave.open(self.WAVE_OUTPUT_FILENAME, 'wb')
        outputFile.setnchannels(self.CHANNELS)
        outputFile.setsampwidth(self.mic.get_sample_size(self.FORMAT))
        outputFile.setframerate(self.RATE)
        outputFile.writeframes(b''.join(self.frames))
        outputFile.close()
        rospy.loginfo("save done")

    def Voice_To_Text(self, file):
        r = speech_recognition.Recognizer()
        with speech_recognition.WavFile(file) as source:
            audio = r.record(source)  # read the entire audio file

        try:
            Text = r.recognize_google(audio, language="zh-TW")
            # Text = r.recognize_sphinx(audio, language="zh-CH")

        except r.UnknowValueError:
            Text = "無法翻譯"
        except r.RequestError as e:
            Text = "無法翻譯{0}".format(e)

        return Text


def server_record():
    rospy.init_node('record_server_node')
    s = rospy.Service('record_service', SetBool, handle_call)
    rospy.loginfo("\033[0;42mRecord server is on!\033[0m")
    rospy.spin()


def handle_call(req):
    # print("call server by " + str(req.data))
    try:
        if (req.data == True):
            rospy.loginfo("\033[0;33mStart record\033[0m")
            rc.start()
            return SetBoolResponse(True, str(req.data))
        else:
            rospy.loginfo("\033[0;33mStop record\033[0m")
            rc.stop()
            test = rc.Voice_To_Text(rc.WAVE_OUTPUT_FILENAME)
            rospy.loginfo("Text:\033[0;95m" + test + "\033[0m")
            return SetBoolResponse(True, test)

    except:
        return SetBoolResponse(False, str(req.data))


rc = Record()
if __name__ == "__main__":
    server_record()

    # text = rc.Voice_To_Text("nono.wav")
    # print(text)
