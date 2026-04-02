import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import os
import wave
from piper.voice import PiperVoice
from piper.config import SynthesisConfig

TEST_TEXT = ". .  Amo el canto del cenzontle pájaro de 400 voces, amo el color del jade, y el enervante perfume de las flores, pero amo más a mi hermano, el hombre"
AUDIO_BASH = "aplay \"/dev/shm/tts_output.wav\""

class TTSSubscriber(Node):

    def __init__(self):
        super().__init__('text_to_speech_subscriber')
        
        # Obtener ruta dinámica a la carpeta 'models' relativa a este script
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.model = os.path.join(current_dir, "models", "es_MX-claude-high.onnx")
        self.config = os.path.join(current_dir, "models", "es_MX-claude-high.onnx.json")
        
        self.voice = PiperVoice.load(model_path=self.model,config_path=self.config)
        
        self.subscription = self.create_subscription(
            String,
            '/tts_query',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def generate_speech(self,txt):
        # Generate speech with specific instructions
        with wave.open("/dev/shm/tts_output.wav", "wb") as wav_file:
            self.voice.synthesize_wav(
                txt, 
                wav_file,
                length_scale=1.0,
                noise_scale=0.667,
                noise_w_scale=0.8
            )

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
