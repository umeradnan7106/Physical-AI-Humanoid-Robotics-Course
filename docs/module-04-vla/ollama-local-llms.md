---
id: ollama-local-llms
title: Ollama & Local LLMs
sidebar_position: 4
description: Run Llama 3, Mistral, and other LLMs locally with Ollama for cost-free robot planning.
keywords: [ollama, llama 3, mistral, local llm, offline ai]
---

# Ollama & Local LLMs

**Zero API costs. Full privacy. Offline operation.**

## Why Run LLMs Locally?

**Cloud APIs** (GPT-4, Claude):
- ✅ State-of-the-art performance
- ❌ $0.01-0.10 per request
- ❌ Requires internet
- ❌ Data sent to third parties

**Local LLMs** (Ollama):
- ✅ Free (GPU compute only)
- ✅ Works offline
- ✅ Data stays on-device
- ❌ Requires GPU (8GB+ VRAM)
- ❌ Slightly lower quality than GPT-4

**Best of both worlds**: Prototype with GPT-4, deploy with Ollama.

## Install Ollama

**Linux**:
```bash
curl -fsSL https://ollama.com/install.sh | sh
```

**macOS** (Apple Silicon):
```bash
brew install ollama
```

**Windows** (WSL2):
```bash
curl -fsSL https://ollama.com/install.sh | sh
```

**Start Ollama server**:
```bash
ollama serve
# Runs on http://localhost:11434
```

## Pull Models

**Llama 3.1 8B** (recommended for robotics):
```bash
ollama pull llama3.1:8b
```

**Llama 3.1 70B** (best quality, requires 48GB+ VRAM):
```bash
ollama pull llama3.1:70b
```

**Mistral 7B** (alternative):
```bash
ollama pull mistral:7b
```

**Phi-3 Mini** (fastest, 4GB VRAM):
```bash
ollama pull phi3:mini
```

**List installed models**:
```bash
ollama list
```

## Test Ollama

**Command line**:
```bash
ollama run llama3.1:8b "Generate a JSON plan to navigate to the kitchen"
```

**Python API**:
```bash
pip install ollama
```

```python
import ollama

response = ollama.chat(
    model='llama3.1:8b',
    messages=[
        {'role': 'system', 'content': 'You are a robot task planner. Output JSON only.'},
        {'role': 'user', 'content': 'Go to the kitchen'}
    ]
)

print(response['message']['content'])
```

## Ollama Planner for ROS 2

**ollama_planner.py**:
```python
import ollama
import json

class OllamaPlanner:
    def __init__(self, model_name="llama3.1:8b"):
        self.model_name = model_name

    def plan_action(self, user_command: str, robot_state: dict = None) -> dict:
        system_prompt = """You are a robot task planner. Generate JSON action plans.

Available actions:
- navigate: Move to location (params: location_name)
- pick: Grasp object (params: object_id)
- place: Release object (params: location_name)
- wait: Pause execution (params: duration_seconds)

Available locations: kitchen, living_room, bedroom, table, shelf

Output ONLY valid JSON in this format:
{"steps": [{"action": "navigate", "params": {"location_name": "kitchen"}}]}

If unclear: {"error": "Clarification needed: <question>"}
"""

        if robot_state:
            system_prompt += f"\n\nCurrent robot state: {json.dumps(robot_state)}"

        response = ollama.chat(
            model=self.model_name,
            messages=[
                {'role': 'system', 'content': system_prompt},
                {'role': 'user', 'content': user_command}
            ],
            format='json',  # Force JSON output
            options={'temperature': 0.0}  # Deterministic
        )

        # Parse response
        text = response['message']['content']
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            # Fallback: extract JSON from markdown code blocks
            if '```json' in text:
                json_str = text.split('```json')[1].split('```')[0].strip()
                return json.loads(json_str)
            raise

# Usage
planner = OllamaPlanner(model_name="llama3.1:8b")
plan = planner.plan_action("Bring me a cup from the kitchen")
print(plan)
```

## Performance Comparison

**Test**: "Bring me the red cup from the kitchen table"

| Model | Latency | Quality | VRAM |
|-------|---------|---------|------|
| **GPT-4o** | 1.2s | ⭐⭐⭐⭐⭐ | N/A (cloud) |
| **Claude Sonnet** | 1.0s | ⭐⭐⭐⭐⭐ | N/A (cloud) |
| **Llama 3.1 70B** | 3.5s | ⭐⭐⭐⭐ | 48GB |
| **Llama 3.1 8B** | 0.8s | ⭐⭐⭐ | 8GB |
| **Mistral 7B** | 0.7s | ⭐⭐⭐ | 6GB |
| **Phi-3 Mini** | 0.3s | ⭐⭐ | 4GB |

**Recommendation**: Llama 3.1 8B (best balance of speed/quality/VRAM).

## Multi-Provider Adapter

**Unified interface for GPT-4/Claude/Ollama**:
```python
from abc import ABC, abstractmethod

class LLMAdapter(ABC):
    @abstractmethod
    def plan_action(self, user_command: str) -> dict:
        pass

class OllamaAdapter(LLMAdapter):
    def __init__(self, model_name="llama3.1:8b"):
        self.planner = OllamaPlanner(model_name)

    def plan_action(self, user_command: str) -> dict:
        return self.planner.plan_action(user_command)

class GPT4Adapter(LLMAdapter):
    def __init__(self, api_key):
        self.planner = GPT4Planner(api_key)

    def plan_action(self, user_command: str) -> dict:
        return self.planner.plan_action(user_command)

# Swap providers via environment variable
def get_planner() -> LLMAdapter:
    provider = os.getenv("LLM_PROVIDER", "ollama")

    if provider == "ollama":
        return OllamaAdapter(model_name="llama3.1:8b")
    elif provider == "gpt4":
        return GPT4Adapter(api_key=os.getenv("OPENAI_API_KEY"))
    elif provider == "claude":
        return ClaudeAdapter(api_key=os.getenv("ANTHROPIC_API_KEY"))
    else:
        raise ValueError(f"Unknown provider: {provider}")

# Usage
planner = get_planner()
plan = planner.plan_action("Go to the bedroom")
```

**Switch providers**:
```bash
# Use Ollama
export LLM_PROVIDER=ollama
ros2 run llm_planner planner_node

# Use GPT-4 (for testing)
export LLM_PROVIDER=gpt4
export OPENAI_API_KEY=sk-...
ros2 run llm_planner planner_node
```

## Optimize Ollama Performance

**Use quantized models** (smaller, faster):
```bash
# 4-bit quantization (2× faster, 4× less VRAM)
ollama pull llama3.1:8b-q4_K_M

# 8-bit quantization (balanced)
ollama pull llama3.1:8b-q8_0
```

**Reduce context length**:
```python
response = ollama.chat(
    model='llama3.1:8b',
    messages=messages,
    options={
        'num_ctx': 2048,  # Default 4096, reduce for speed
        'temperature': 0.0
    }
)
```

**GPU acceleration** (NVIDIA only):
```bash
# Verify GPU detected
ollama list
# Should show "CUDA" in capabilities

# Check GPU usage
nvidia-smi
```

## Troubleshooting

**"Connection refused"**:
```bash
# Ensure Ollama server is running
ollama serve
```

**"Model not found"**:
```bash
ollama pull llama3.1:8b
```

**"Out of memory"**:
- Use smaller model: `phi3:mini` (4GB VRAM)
- Use quantized model: `llama3.1:8b-q4_K_M`
- Reduce context: `num_ctx: 2048`

**"JSON parsing error"**:
- Enable `format='json'` in ollama.chat()
- Add "Output ONLY valid JSON" to system prompt
- Use try/except with fallback extraction

## Benchmark Your Setup

```python
import time

planner = OllamaPlanner(model_name="llama3.1:8b")

commands = [
    "Go to the kitchen",
    "Pick up the red box from the shelf",
    "Bring me a cup from the living room"
]

for cmd in commands:
    start = time.time()
    plan = planner.plan_action(cmd)
    latency = time.time() - start
    print(f"{cmd}: {latency:.2f}s - {plan}")
```

**Expected latency** (Llama 3.1 8B on RTX 3080):
- Simple commands: 0.5-1.0s
- Complex chains: 1.0-2.0s

**Next**: [Action Execution Pipeline](/docs/module-04-vla/action-execution)
