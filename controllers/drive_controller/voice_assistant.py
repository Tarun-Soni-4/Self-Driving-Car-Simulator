import os
import sys
import playsound
import speech_recognition as SR
from gtts import gTTS

recognizer = SR.Recognizer()
microphone = SR.Microphone()
Log = dict()
error_Log = list()
def VA_speaks(respond: dict) -> None:
    file = "VA.mp3"
    try:
        respond = gTTS(text=respond, lang="en", slow=False)
        respond.save(file)
        playsound.playsound(file)
        if os.path.isfile(file):
            os.remove(file)
    except Exception as e:
        error_Log.append("[VA] in va speak: %s" % e)

def VA_recognise():
    if not isinstance(recognizer, SR.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")
    if not isinstance(microphone, SR.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")
    with microphone as source:
        audio = recognizer.listen(source, phrase_time_limit=4)
    response = {
        "success": True,
        "error": None,
        "transcription": None
    }
    try:
        response["transcription"] = recognizer.recognize_google(audio)
        print("You:", response["transcription"])
        sys.stdout.flush()
    except SR.RequestError:
        response["success"] = False
        response["error"] = "API unavailable"
    except SR.UnknownValueError:
        response["error"] = "Unable to recognize speech"
    return response

def process_order(input_text):
    order = None
    if input_text["transcription"]:
        input_text = input_text["transcription"].lower()
        if "change lane" in input_text:
            VA_speaks("Lane changing has been activating.")
            Log[input_text] = "Lane changing has been activating."
            order = 1
        elif "speed up" in input_text:
            VA_speaks("Car has been speeding up.")
            Log[input_text] = "Car has been speeding up."
            order = 2
        elif "speed of the car" in input_text:
            order = 3
        elif "slow down" in input_text:
            VA_speaks("Car has been slowing down.")
            Log[input_text] = "Car has been slowing down."
            order = 4
        elif "exit" in input_text:
            VA_speaks("Have a good day Sir.")
            Log[input_text] = "CHave a good day Sir.."
            order = 5
        else:
            VA_speaks(" I couldn't understand your order, Please try again.")
        if order is not None:
            return order

def greeting():
    speak = "Hello! I'm Tarun I have 5 basic commands: ""Change lane"" for changing the current lane, " \
            " ""Speed up"" for speeding up the car, ""speed of the car""for current speed information, " \
            " ""slow down"" for slowing down the car. ""exit"" for closing the voice assistant." \
            "For activating me just say ""Tarun""."
    VA_speaks(speak)

def VA_response_fun(respond_dict):
    for key, value in respond_dict.items():
        VA_speaks(value)
        Log[key] = value
    respond_dict.clear()

def main(m_Log, m_error_Log):
    global Log, error_Log
    Log = m_Log
    error_Log = m_error_Log
    order = None
    text = VA_recognise()
    if not text["success"]:
        VA_speaks("I didn't catch that Sir. What did you say? Please say again!")
        Log["un success input"] = "I didn't catch that Sir. What did you say? Please say again!"
    if text["transcription"]:
        text = text["transcription"].lower()
        if "tarun" in text:
            VA_speaks("I'm listening Sir.")
            Log[text] = "I'm listening Sir."
            text = VA_recognise()
            order = process_order(text)
    return order, Log, error_Log
greeting()
main()
