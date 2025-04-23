import edge_tts
import asyncio
import os
import threading
import wave
import pyaudio

class Talker:
    def __init__(self, voice="en-US-ChristopherNeural"):
        self.voice = voice
        self.temp_file = "tts_output.wav"

    async def _speak_async(self, text):
        communicate = edge_tts.Communicate(text, self.voice)
        await communicate.save(self.temp_file)
        self._play_audio(self.temp_file)

    def _play_audio(self, file_path):
        wf = wave.open(file_path, 'rb')

        p = pyaudio.PyAudio()
        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True)

        data = wf.readframes(1024)
        while data:
            stream.write(data)
            data = wf.readframes(1024)

        stream.stop_stream()
        stream.close()
        p.terminate()
        wf.close()

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
