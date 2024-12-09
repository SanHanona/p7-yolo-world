import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import whisper
import numpy as np
import torch
import argparse
import os
import sys
from datetime import datetime, timedelta, timezone
from queue import Queue
import speech_recognition as sr
from time import sleep

class WhisperNode(Node):
    def __init__(self, args):
        super().__init__('whisper_node')

        # Parameters
        self.model_name = args.model
        self.energy_threshold = args.energy_threshold
        self.record_timeout = args.record_timeout
        self.phrase_timeout = args.phrase_timeout

        # Topics
        self.subscription = self.create_subscription(Bool, '/attention', self.attention_callback, 0)
        self.publisher = self.create_publisher(String, '/speech', 10)

        # Whisper model and SpeechRecognition
        self.phrase_time = None
        self.data_queue = Queue()
        self.audio_model = whisper.load_model(self.model_name, device='cpu')
        self.transcription = ['']

        # Microphone setup
        self.recorder = sr.Recognizer()
        self.recorder.energy_threshold = self.energy_threshold
        self.recorder.dynamic_energy_threshold = False
        mic_name = args.default_microphone
        self.source = None
        self.source = self.initialize_microphone(mic_name)
        


        # Start listening in the background
        if self.source:
            with self.source as source:  # Wrap the source initialization in a 'with' block
                self.recorder.adjust_for_ambient_noise(source)  # Adjust for ambient noise
            self.recorder.listen_in_background(self.source, self.record_callback, phrase_time_limit=self.record_timeout)
            self.get_logger().info("Microphone initialized and listening.")
        else:
            self.get_logger().error("Microphone not found. Exiting.")
            if rclpy.ok():
                rclpy.shutdown()

    def initialize_microphone(self, mic_name):
        if mic_name == 'list':
            self.get_logger().info("Available microphone devices are:")
            for index, name in enumerate(sr.Microphone.list_microphone_names()):
                self.get_logger().info(f"Microphone with name \"{name}\" found")
            return None
        else:
            for index, name in enumerate(sr.Microphone.list_microphone_names()):
                if mic_name in name:
                    return sr.Microphone(sample_rate=16000, device_index=index)

        self.get_logger().warn(f"Microphone '{mic_name}' not found. Using default microphone.")
        return sr.Microphone(sample_rate=16000)

    def record_callback(self, _, audio: sr.AudioData) -> None:
        """
        Threaded callback to receive audio data when recordings finish.
        """
        data = audio.get_raw_data()
        self.data_queue.put(data)

    def attention_callback(self, msg: Bool):
        """
        Callback for /attention topic.
        If a message is received, process the transcription.
        """
        self.get_logger().info(f"Attention triggered: {msg.data}")
        if msg.data:
            self.process_transcription()

    def process_transcription(self):
        """
        Process the audio data queue and publish transcriptions.
        """
        self.get_logger().info("process_transcription")
        now = datetime.now(timezone.utc).timestamp()
        self.get_logger().info(str(now))
        # if not self.data_queue.empty(): I HAVE REMOVED THIS !!!!!!!!!!!!!!!!
        phrase_complete = False
        self.data_queue.queue.clear()
        while phrase_complete == False:
            if self.phrase_time and self.phrase_time - now > float(self.phrase_timeout):
                phrase_complete = True
            self.phrase_time = datetime.now(timezone.utc).timestamp()
            deltataime=self.phrase_time - now
            self.get_logger().info(str(deltataime))

            audio_data = b''.join(self.data_queue.queue)
            self.data_queue.queue.clear()

            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

            result = self.audio_model.transcribe(audio_np, fp16=torch.cuda.is_available())
            text = result['text'].strip()

            if phrase_complete:
                self.transcription.append(text)
                self.get_logger().info("Stop listening !!!")
                self.publisher.publish(String(data=self.transcription[0]))
                self.get_logger().info(f"Published transcription: {self.transcription[0]}")
                self.transcription = ['']
                self.phrase_time = False
                break
            else:
                self.transcription[-1] += text
                self.get_logger().info(str(self.transcription))
                
                
            

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="tiny.en", help="Model to use",
                        choices=['tiny.en', 'tiny', 'base.en', 'base', 'small.en', 'small', 'medium.en', 'medium', 'large-v1', 'large-v2', 'large'])
    parser.add_argument("--energy_threshold", default=1000,
                        help="Energy level for mic to detect.", type=int)
    parser.add_argument("--record_timeout", default=5,
                        help="How real time the recording is in seconds.", type=float)
    parser.add_argument("--phrase_timeout", default=5,
                        help="How much empty space between recordings before we "
                             "consider it a new line in the transcription.", type=float)
    parser.add_argument("--default_microphone", default='hw:2,0',
                            help="Default microphone name for SpeechRecognition. "
                                 "Run this with 'list' to view available Microphones.", type=str)
    parsed_args = parser.parse_args()

    # Convert Namespace to sys.argv-like list
    rclpy_args = None
    if args is None:
        rclpy_args = sys.argv
    else:
        rclpy_args = args

    rclpy.init(args=rclpy_args)
    node = WhisperNode(parsed_args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
