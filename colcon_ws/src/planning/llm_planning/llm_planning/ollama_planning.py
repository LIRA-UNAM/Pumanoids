import os
from openai import OpenAI
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

SM_INIT = 0
SM_LOAD_INITIAL_PROMPTS = 10
SM_LOOK_FOR_PERSON = 20
SM_RANDOM_MOVEMENT = 30
SM_APPROACH_TO_PERSON = 40
SM_INITIAL_INTERACTION = 50
SM_INTERACTION = 60


class OllamaPlanningNode(Node):
    def load_prompts(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as f:
                return f.read()
        except Exception as e:
            self.get_logger().warn(f"No se pudo cargar el archivo {path}: {e}")
            return ""

    def send_prompt(self, msg):
        self.msg_history.append({"role": "user", "content": msg})
        response = self.client.chat.completions.create(
            model=self.model_name,
            messages=self.msg_history
        )
        self.msg_history.append({"role": "assistant", "content": response.choices[0].message.content})

    def callback_prompt(self, msg):
        if self.new_prompt:
            self.get_logger().info("Ignoring received prompt...")
            return
        self.prompt = msg.data
        self.new_prompt = True
    
    def __init__(self):
        super().__init__("ollama_planning_node")
        self.get_logger().info("INITIALIZING OLLAMA PLANNING NODE")
        self.msg_history = []
        self.url_api = "http://localhost:11434/api/chat"
        self.client = OpenAI(base_url="http://localhost:8000/v1", api_key="Empty")
        self.model_name = "RedHatAI/Qwen3-4B-quantized.w4a16"
        self.prompt = ""
        self.new_prompt = False
        self.sub_query = self.create_subscription(String, '/sp_rec/recognized', self.callback_prompt, 1)
        self.pub_tts = self.create_publisher(String, '/tts_query', 1)

    def spin(self):
        # Obtener la ruta del archivo desde el share directory de ROS 2
        package_share_directory = get_package_share_directory('llm_planning')
        prompt_path = os.path.join(package_share_directory, 'config', 'SystemPrompt.txt')
        
        system_prompt = self.load_prompts(prompt_path)
        
        self.msg_history.append({"role": "system", "content": system_prompt})
        self.get_logger().info(f"System prompt loaded from {prompt_path}")
        
        self.get_logger().info("Waiting for new prompt...")
        while rclpy.ok():
            if(self.new_prompt):
                self.get_logger().info("Sending prompt: " + self.prompt)
                self.send_prompt(self.prompt)
                self.get_logger().info("Response received: " + self.msg_history[-1]["content"])
                self.pub_tts.publish(String(data=self.msg_history[-1]["content"]))
                delay_counter = 1.9*len(self.msg_history[-1]["content"])+20
                while delay_counter > 0 and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0)
                    self.get_clock().sleep_for(Duration(seconds=0.05))
                    delay_counter -= 1
                self.get_logger().info("Waiting for new prompt")
                self.new_prompt = False
            rclpy.spin_once(self, timeout_sec=0)
            self.get_clock().sleep_for(Duration(seconds=0.05))

def main(args=None):
    rclpy.init(args=args)
    ollama_planning_node= OllamaPlanningNode()
    ollama_planning_node.spin()
    ollama_planning_node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
