from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Nodo del Oído (ASR - Reconocimiento de voz)
    asr_node = Node(
        package='speech2text',
        executable='faster_whisper_asr',  # Asegúrate de que este sea el nombre del entry_point en el setup.py de speech2text
        name='faster_whisper_node',
        output='screen'
    )
    
    # Nodo del Cerebro (LLM - Planificación y respuesta)
    llm_node = Node(
        package='llm_planning',
        executable='ollama_planning',
        name='ollama_planning_node',
        output='screen'
    )
    
    # Nodo de la Boca (TTS - Texto a Voz)
    tts_node = Node(
        package='text2speech',
        executable='pipertts',
        name='text_to_speech_subscriber',
        output='screen'
    )

    return LaunchDescription([
        asr_node,
        llm_node,
        tts_node
    ])