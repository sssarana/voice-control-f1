import os
import subprocess
import threading
import queue
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        self.publisher_ = self.create_publisher(String, 'voice_commands', 10)
        self.audio_queue = queue.Queue()
        self.recognizer = sr.Recognizer()
        self.command_list = ["forward", "backward", "left", "right", "stop"]

        self.gstreamer_thread = threading.Thread(target=self.start_gstreamer_pipeline)
        self.gstreamer_thread.daemon = True
        self.gstreamer_thread.start()

        self.get_logger().info('Voice Control Node has been started.')
        self.recognize_speech()

    def start_gstreamer_pipeline(self):
        gst_command = [
            'gst-launch-1.0',
            'alsasrc', '!', 
            'audioconvert', '!', 
            'audioresample', '!', 
            'audio/x-raw,format=S16LE,channels=1,rate=48000', '!',
            'queue', '!',
            'fdsink', 'fd=1'
        ]

        gst_process = subprocess.Popen(gst_command, stdout=subprocess.PIPE, bufsize=1024)
        
        while True:
            data = gst_process.stdout.read(1024)
            if data:
                self.audio_queue.put(data)
            else:
                break

    def recognize_speech(self):
        with sr.Microphone(sample_rate=48000) as source:
            self.recognizer.adjust_for_ambient_noise(source)
            self.get_logger().info("Listening...")

            while True:
                try:
                    audio_data = b''.join([self.audio_queue.get() for _ in range(10)])
                    if not audio_data:
                        continue

                    audio = sr.AudioData(audio_data, sample_rate=48000, sample_width=2)
                    command = self.recognizer.recognize_google(audio).lower()
                    self.get_logger().info(f"Transcript: {command}")

                    for cmd in self.command_list:
                        if cmd in command:
                            self.publish_command(cmd)

                except sr.UnknownValueError:
                    self.get_logger().info("Google Speech Recognition could not understand audio")
                except sr.RequestError as e:
                    self.get_logger().info(f"Could not request results from Google Speech Recognition service; {e}")

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
