import requests
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String

SM_INIT = 0
SM_LOAD_INITIAL_PROMPTS = 10
SM_LOOK_FOR_PERSON = 20
SM_RANDOM_MOVEMENT = 30
SM_APPROACH_TO_PERSON = 40
SM_INITIAL_INTERACTION = 50
SM_INTERACTION = 60


class OllamaPlanningNode(Node):
    def load_prompts(self, path):
        lines = open(path).readlines()
        prompts = []
        for l in lines:
            s = l.rstrip()
            if(len(s)>3):
                prompts.append(s)
        return prompts

    def send_prompt(self, msg):
        self.msg_history.append({"role": "user", "content": msg})
        resp = requests.post(self.url_api, json={"model": "llama3", "messages": self.msg_history, "stream":False, "options":{"num_ctx":8192}})
        self.msg_history.append(resp.json()["message"])

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
        self.prompt = ""
        self.new_prompt = False
        self.sub_query = self.create_subscription(String, '/sp_rec/recognized', self.callback_prompt, 1)
        self.pub_tts = self.create_publisher(String, '/tts_query', 1)

    def spin(self):
        prompts = self.load_prompts("/home/thedoctor/InfoSaludDigna.txt")
        self.send_prompt("Genera respuestas de máximo veinte palabras")
        for p in prompts:
            self.get_logger().info("Sending info about salud digna: " + self.prompt[:50])
            self.send_prompt(p)
            self.get_logger().info("Response received: " + self.msg_history[-1]["content"])
        self.send_prompt("Da respuestas muy sintetizadas y concisas")
        
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
