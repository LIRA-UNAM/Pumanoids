import rclpy
from rclpy.node import Node
import os
import torch
import soundfile as sf
from qwen_tts import Qwen3TTSModel
from std_msgs.msg import String

INSTR = "Energético, acento mexicano"
AUDIO_BASH = "aplay -D plughw:2,0 \"tts_output.wav\""

class TTSSubscriber(Node):

    def __init__(self):
        super().__init__('text_to_speech_subscriber')
        self.model = Qwen3TTSModel.from_pretrained(
        "Qwen/Qwen3-TTS-12Hz-0.6B-CustomVoice",
        device_map="cuda:0",
        dtype=torch.bfloat16,
        attn_implementation="flash_attention_2",
        )
        self.subscription = self.create_subscription(
            String,
            '/tts_query',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def generate_speech(self,txt):
        # Generate speech with specific instructions
        wavs, sr = self.model.generate_custom_voice(
            text=txt,
            language="Spanish", 
            speaker="Ryan",
            instruct=INSTR, 
        )

        # Save the generated audio
        sf.write("tts_output.wav", wavs[0], sr)

    def listener_callback(self, msg):
        self.get_logger().info('Processing txt: "%s"' % msg.data)
        self.generate_speech(msg.data)
        os.system(AUDIO_BASH)


def main(args=None):
    rclpy.init(args=args)

    tts_processor = TTSSubscriber()

    rclpy.spin(tts_processor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tts_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()