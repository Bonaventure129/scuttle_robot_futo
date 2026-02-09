#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import speech_recognition as sr
import pyttsx3
import threading
import json
import os
import pyaudio
from vosk import Model, KaldiRecognizer

# ANSI Colors
GREEN = '\033[92m'
BOLD = '\033[1m'
RESET = '\033[0m'

class VoiceManager(Node):
    def __init__(self):
        super().__init__('voice_manager')
        
        # --- Config ---
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel_voice', 10)
        self.sub_name = self.create_subscription(String, '/object_name', self.narrator_callback, 10)
        
        # --- Voice Engine ---
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)
        self.engine.setProperty('volume', 0.9)
        
        # --- Offline Recognition Setup (Vosk) ---
        model_path = "model" # Folder name you extracted
        if not os.path.exists(model_path):
            self.get_logger().error("Model not found! Please download vosk-model-small-en-us and name it 'model'")
            return

        self.model = Model(model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000)
        self.mic = pyaudio.PyAudio()
        self.stream = self.mic.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, input_device_index=4, frames_per_buffer=8192)     
           
        self.listen_thread = threading.Thread(target=self.listen_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()
        
        self.last_spoken_object = ""
        print(f"{BOLD}{GREEN}>>> OFFLINE VOICE SYSTEM READY. Say 'Forward', 'Back', 'Stop' <<< {RESET}")

    def narrator_callback(self, msg):
        obj_name = msg.data
        if obj_name != self.last_spoken_object:
            self.speak(f"I see a {obj_name}")
            self.last_spoken_object = obj_name

    def speak(self, text):
        self.engine.say(text)
        self.engine.runAndWait()

    def listen_loop(self):
        self.stream.start_stream()
        
        while rclpy.ok():
            data = self.stream.read(4096, exception_on_overflow=False)
            
            if self.recognizer.AcceptWaveform(data):
                result = json.loads(self.recognizer.Result())
                command = result.get('text', '')
                
                if command:
                    print(f"{GREEN}{BOLD}>>> VOICE COMMAND RECEIVED: '{command}' <<<{RESET}")
                    self.process_command(command)

    def process_command(self, cmd):
        twist = Twist()
        valid_cmd = False
        
        if "forward" in cmd or "go" in cmd:
            twist.linear.x = 0.2
            self.speak("Moving Forward")
            valid_cmd = True
        elif "back" in cmd or "reverse" in cmd:
            twist.linear.x = -0.2
            self.speak("Reversing")
            valid_cmd = True
        elif "left" in cmd:
            twist.angular.z = 0.5
            self.speak("Turning Left")
            valid_cmd = True
        elif "right" in cmd:
            twist.angular.z = -0.5
            self.speak("Turning Right")
            valid_cmd = True
        elif "stop" in cmd or "halt" in cmd:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.speak("Stopping")
            valid_cmd = True
        
        if valid_cmd:
            self.pub_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()