"""
sudo pip uninstall dronekit
sudo pip uninstall pymavlink
cd dronekit-python
git pull
sudo python setup.py build
sudo python setup.py install
¬¬¬¬¬¬¬¬¬ UNDER CONSTRUCTION ¬¬¬¬¬¬¬¬
"""

import speech_recognition as sr
from time import ctime
import time
import os
from gtts import gTTS
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import Tkinter as tk
import argparse



#-- Connect to the vehicle
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default = '')
args = parser.parse_args()

print('Connecting...')

vehicle = connect('udp:127.0.0.1:14551')

#-- Setup the commanded flying speed
gnd_speed = 5 # [m/s]

#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)

 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

    Bitmask to indicate which dimensions should be ignored by the vehicle
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that
    none of the setpoint dimensions should be ignored). Mapping:
    bit 1: x,  bit 2: y,  bit 3: z,
    bit 4: vx, bit 5: vy, bit 6: vz,
    bit 7: ax, bit 8: ay, bit 9:


    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


#-- Voice Controlfunction

def voice():
	  if event.keysym == 'r':
           print("r pressed >> Set the vehicle to RTL")
   	        vehicle.mode = VehicleMode("RTL")

    else: #-- non standard keys
        if event.keysym == 'Up':
            set_velocity_body(vehicle, gnd_speed, 0, 0)
        elif event.keysym == 'Down':
            set_velocity_body(vehicle,-gnd_speed, 0, 0)
        elif event.keysym == 'Left':
            set_velocity_body(vehicle, 0, -gnd_speed, 0)
        elif event.keysym == 'Right':
            set_velocity_body(vehicle, 0, gnd_speed, 0)

#---- MAIN FUNCTION
#- Takeoff
arm_and_takeoff(10)

#- Read the keyboard with tkinter
root = tk.Tk()
print(">> Control the drone with the arrow keys. Press r for RTL mode")
root.bind_all('<Key>', key)
root.mainloop()

def speak(audioString):
    print(audioString)
tts = gTTS(text=audioString, lang='en')
tts.save("audio.mp3")
os.system("mpg321 audio.mp3")

def recordAudio():
# Record Audio
    r = sr.Recognizer()
with sr.Microphone() as source:
    print("Say something!")
audio = r.listen(source)

# Speech recognition using Google Speech Recognition
data = ""
try:
# Uses the default API key
# To use another API key: `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
    data = r.recognize_google(audio)
    print("You said: " + data)
except sr.UnknownValueError:
    print("Google Speech Recognition could not understand audio")
except sr.RequestError as e:
    print("Could not request results from Google Speech Recognition service; {0}".format(e))

return data

def initialization(data):
    if "start" in data:
        return arm_and_takeoff(altitude)

if "turn right" in data:
    speak(ctime())

if "" in data:


# initialization
time.sleep(2)
speak("")
while 1:
    data = recordAudio()
initializationS(data)
