import os
from openai import OpenAI

client = OpenAI(
    # This is the default and can be omitted
    base_url="http://localhost:8000/v1",
    api_key="Empty"
)

response = client.responses.create(
    model="RedHatAI/Qwen3-4B-quantized.w4a16",
    instructions="You are a coding assistant that talks like a pirate. Your responses CANNOT be longer than 3 sentences.",
    input="Who was Lawrence of Arabia?",
)

print(response.output_text)
