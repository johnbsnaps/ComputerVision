import edge_tts
import asyncio
import os
import threading
import subprocess

class Talker:
    def __init__(self, voice="en-US-ChristopherNeural"):
        self.voice = voice
        self.wav_file = "tts_output.wav"
        self.mp3_file = "tts_output.mp3"

    async def _speak_async(self, text):
        # Generate MP3 with edge-tts
        communicate = edge_tts.Communicate(text, self.voice)
        await communicate.save(self.mp3_file)

        # Convert to proper WAV using ffmpeg
        subprocess.run([
            "ffmpeg", "-y", "-i", self.mp3_file,
            "-ar", "24000", "-ac", "1", "-sample_fmt", "s16",
            self.wav_file
        ])

        # Play using aplay
        self._play_audio(self.wav_file)

    def _play_audio(self, file_path):
        subprocess.run(["aplay", file_path])

    def say(self, text):
        threading.Thread(target=self._run_async, args=(text,), daemon=True).start()

    def _run_async(self, text):
        asyncio.run(self._speak_async(text))

# Create a default instance for easy access
_talker_instance = Talker()

def say(text):
    _talker_instance.say(text)

def set_voice(voice_name):
    _talker_instance.voice = voice_name
