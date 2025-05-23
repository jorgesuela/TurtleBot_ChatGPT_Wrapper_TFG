#!/usr/bin/env python3

import rospy
from sound_play.libsoundplay import SoundClient
from gtts import gTTS
import tempfile
import os
import subprocess

class RobotSpeaker:
    def __init__(self):
        self.soundhandle = SoundClient()
        rospy.sleep(1)

    def say(self, text):
        tts = gTTS(text, lang='es')
        with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
            tts.save(f.name)
            subprocess.run(['mpg123', f.name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            os.unlink(f.name)

    def play_beep(self):
        self.soundhandle.play(SoundClient.BACKINGUP)

    def play_custom_sound(self, path_to_wav):
        self.soundhandle.playWave(path_to_wav)

