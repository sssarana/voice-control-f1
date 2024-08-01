import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
import speech_recognition as sr
import subprocess
import io

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        self.recognizer = sr.Recognizer()
        self.pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
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
        record_command = ['tinycap', '-', '-c', '1', '-r', '48000', '-b', '16']
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
        audio = sr.AudioData(audio_stream.read(), 48000, 1)

        try:
            command = self.recognizer.recognize_google(audio)
            self.get_logger().info(f"Recognized command: {command}")
            self.handle_command(command.lower())
        except sr.UnknownValueError:
            self.get_logger().info("Could not understand the command.")
        except sr.RequestError as e:
            self.get_logger().info(f"Request error: {e}")

    def handle_command(self, command):
        drive_msg = AckermannDriveStamped()
        if command == "forward":
            drive_msg.drive.speed = 1.0
        elif command == "backward":
            drive_msg.drive.speed = -1.0
        elif command == "left":
            drive_msg.drive.steering_angle = 0.5
        elif command == "right":
            drive_msg.drive.steering_angle = -0.5
        elif command == "stop":
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
        else:
            self.get_logger().info(f"Unknown command: {command}")
            return

        self.pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
