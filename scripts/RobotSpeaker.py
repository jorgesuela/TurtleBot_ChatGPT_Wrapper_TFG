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
        rospy.sleep(0.5)

    def say(self, text):

        tts = gTTS(text, lang='es')

        with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
            tts.save(f.name)
            temp_mp3 = f.name

        out_file = "/tmp/out.mp3"

        # 🔇 silenciar ffmpeg
        subprocess.run([
            'ffmpeg', '-y',
            '-i', temp_mp3,
            '-filter:a', 'atempo=1.25', # regula la velocidad del audio de respuesta del robot
            out_file
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # 🔇 silenciar mpg123
        subprocess.run([
            'mpg123', out_file
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        os.unlink(temp_mp3)
        os.remove(out_file)

    def play_beep(self):
        self.soundhandle.play(SoundClient.BACKINGUP)

    def play_custom_sound(self, path_to_wav):
        self.soundhandle.playWave(path_to_wav)