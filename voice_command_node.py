import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import subprocess
import io

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.recognizer = sr.Recognizer()
        self.pub = self.create_publisher(String, '/voice_commands', 10)
        self.device_config()
        self.listen()

    def device_config(self):
        # Configure audio settings using TinyALSA
        subprocess.run(['tinymix', 'set', 'TX DMIC MUX2', 'DMIC3'])
        subprocess.run(['tinymix', 'set', 'TX_CDC_DMA_TX_3 Channels', 'One'])
        subprocess.run(['tinymix', 'set', 'TX_AIF1_CAP Mixer DEC2', '1'])
        subprocess.run(['tinymix', 'set', 'MultiMedia1 Mixer TX_CDC_DMA_TX_3', '1'])

    def listen(self):
        # Start recording using tinycap with stdout piping
        record_command = ['tinycap', '-', '-d', '0', '-c', '1', '-r', '48000', '-b', '16']
        self.get_logger().info("Starting audio capture...")

        # Use subprocess to capture audio in real-time
        with subprocess.Popen(record_command, stdout=subprocess.PIPE, bufsize=1024) as proc:
            while rclpy.ok():
                audio_data = proc.stdout.read(1024)  # Read 1024 bytes (1KB) chunks
                if not audio_data:
                    break
                self.process_audio(audio_data)

    def process_audio(self, audio_data):
        # Convert raw PCM data to AudioData format for speech recognition
        audio_stream = io.BytesIO(audio_data)
        audio = sr.AudioData(audio_stream.read(), 48000, 2)

        try:
            command = self.recognizer.recognize_google(audio)
            self.get_logger().info(f"Recognized command: {command}")
            msg = String()
            msg.data = command
            self.pub.publish(msg)
        except sr.UnknownValueError:
            self.get_logger().info("Could not understand the command.")
        except sr.RequestError as e:
            self.get_logger().info(f"Request error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
