"""
FILE LOCATION: fuad_ws/src/integration_pkg/integration_pkg/voice_node.py

Voice Controller Node
----------------------
Maps spoken keywords to Twist messages and publishes on /voice_cmd.

DEPENDENCIES:
  pip install SpeechRecognition pyaudio
  OR for offline: pip install vosk
  OR for accuracy: pip install openai-whisper

This example uses the `speech_recognition` library (Google Web API).
Replace the recognizer with Vosk/Whisper if you need offline operation.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time

# Try importing speech_recognition; if missing, the node logs a warning.
try:
    import speech_recognition as sr
    HAS_SR = True
except ImportError:
    HAS_SR = False


class VoiceNode(Node):
    def __init__(self):
        super().__init__("voice_controller")

        # ------------------------------------------------------------------
        # Parameters (flat names from params.yaml)
        # ------------------------------------------------------------------
        self.declare_parameter("voice_cmd_topic", "/voice_cmd")
        voice_cmd_topic = self.get_parameter("voice_cmd_topic").value

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self.voice_pub = self.create_publisher(Twist, voice_cmd_topic, 10)

        # ------------------------------------------------------------------
        # Command dictionary (loaded from params.yaml in a real setup)
        # ------------------------------------------------------------------
        self.command_map = {
            "forward":  self._twist(1.0, 0.0, 0.0, 0.0),
            "backward": self._twist(-1.0, 0.0, 0.0, 0.0),
            "left":     self._twist(0.0, 0.0, 0.0, 1.0),
            "right":    self._twist(0.0, 0.0, 0.0, -1.0),
            "up":       self._twist(0.0, 0.0, 1.0, 0.0),
            "down":     self._twist(0.0, 0.0, -1.0, 0.0),
            "takeoff":  self._twist(0.0, 0.0, 1.0, 0.0),
            "land":     self._twist(0.0, 0.0, -1.0, 0.0),
            "hover":    self._twist(0.0, 0.0, 0.0, 0.0),
            "stop":     self._twist(0.0, 0.0, 0.0, 0.0),
        }

        # ------------------------------------------------------------------
        # Speech recognition thread
        # ------------------------------------------------------------------
        if HAS_SR:
            self.recognizer = sr.Recognizer()
            self.microphone = sr.Microphone()
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source)
            self.listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.listen_thread.start()
            self.get_logger().info("VoiceNode started. Listening for commands...")
        else:
            self.get_logger().warn(
                "speech_recognition not installed. Voice commands disabled.\n"
                "Install with: pip install SpeechRecognition pyaudio"
            )

    def _twist(self, lx, ly, lz, az):
        t = Twist()
        t.linear.x = lx
        t.linear.y = ly
        t.linear.z = lz
        t.angular.z = az
        return t

    def _listen_loop(self):
        while rclpy.ok():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=2)
                text = self.recognizer.recognize_google(audio).lower()
                self.get_logger().info(f"Heard: '{text}'")
                for keyword, twist in self.command_map.items():
                    if keyword in text:
                        self.voice_pub.publish(twist)
                        self.get_logger().info(f"Voice command matched: {keyword}")
                        break
            except sr.WaitTimeoutError:
                pass
            except sr.UnknownValueError:
                pass
            except sr.RequestError as e:
                self.get_logger().error(f"Speech recognition error: {e}")
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
