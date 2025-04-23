import edge_tts
import asyncio
import os
import threading
import subprocess

class Talker:
    def __init__(self, voice="en-US-ChristopherNeural"):
        self.voice = voice
        self.temp_file = "tts_output.wav"

    async def _speak_async(self, text):
        communicate = edge_tts.Communicate(text, self.voice)
        await communicate.save(self.temp_file)
        self._play_audio(self.temp_file)

    def _play_audio(self, file_path):
        # Use aplay to play the audio file on Raspberry Pi
        subprocess.run(["aplay", file_path])

    def say(self, text):
        threading.Thread(target=self._run_async, args=(text,), daemon=True).start()

    def _run_async(self, text):
        asyncio.run(self._speak_async(text))

# Create a default instance for convenient use
_talker_instance = Talker()

# Expose a simple interface like: ttsCode.say("Hello!")
def say(text):
    _talker_instance.say(text)

# Optional: switch voice dynamically if needed
def set_voice(voice_name):
    _talker_instance.voice = voice_name
