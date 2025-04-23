import edge_tts
import asyncio
#import tempfile
import os
import threading
import playsound  # or your own playback method

############ This is a talk file that creates a .wav file and saves it temp then
############ plays the audio file back.

##    To import the imports you might need to do this:
##        pip install edge-tts playsound4

##    These were sample voices I was trying out
##    "en-US-ChristopherNeural" (default human)
##    "en-US-GuyNeural" (robotic tone)
##    "en-US-SaraNeural" (teen girl)
##    "en-US-MichelleNeural" (elderly tone)
##    "en-US-AriaNeural" (snarky-sounding female)
##    "en-US-DavisNeural" as the wrestler voice

class Talk:
    def __init__(self, voice="en-US-ChristopherNeural"):
        self.voice = voice

    async def _speak_async(self, text):
        temp_file = "tts.wav"
        communicate = edge_tts.Communicate(text, self.voice)
        await communicate.save(temp_file)  ##this will save to the same directory, but I did create a tmp directory 
        playsound.playsound(temp_file)

    def say(self, text):
        threading.Thread(target=self._run_async, args=(text,), daemon=True).start()

    def _run_async(self, text):
        asyncio.run(self._speak_async(text))
def main():
    speaker = Talk(voice="en-US-GuyNeural")  # Try other voices too
    speaker.say("Hello! I am your robot.")
    
    # Prevent script from exiting too fast
    import time
    time.sleep(3)
main()