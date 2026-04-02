import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    # Directorio local para guardar el modelo y no descargarlo de nuevo
    hf_cache_dir = os.path.expanduser('~/.cache/huggingface')
    
    # Ejecutar el servidor vLLM en Docker
    vllm_server = ExecuteProcess(
        cmd=[
            'docker', 'run', '--rm', '--runtime=nvidia', '--network', 'host',
            '-v', f'{hf_cache_dir}:/root/.cache/huggingface',
            'ghcr.io/nvidia-ai-iot/vllm:latest-jetson-orin',
            'vllm', 'serve', 'RedHatAI/Qwen3-4B-quantized.w4a16',
            '--gpu-memory-utilization', '0.4', '--max-model-len', '32678'
        ],
        output='screen'
    )
    
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
        vllm_server,
        asr_node,
        llm_node,
        tts_node
    ])