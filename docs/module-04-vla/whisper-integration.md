---
id: whisper-integration
title: Whisper Integration
sidebar_position: 2
description: Integrate OpenAI Whisper for real-time speech-to-text transcription.
keywords: [whisper, speech to text, openai whisper, voice input, pyaudio]
---

# Whisper Integration

**Speech → Text → Robot Commands.**

## What is Whisper?

**OpenAI Whisper**: State-of-the-art speech-to-text model
- **Multilingual**: Supports 99 languages
- **Robust**: Handles background noise, accents
- **Open-source**: Run locally without API costs
- **Fast**: GPU inference at 20× real-time speed

**Model sizes**:
- `tiny`: 39M params, fast (CPU-friendly)
- `base`: 74M params
- `small`: 244M params
- `medium`: 769M params
- `large`: 1550M params, most accurate

## Install Whisper

**Python package**:
```bash
pip install openai-whisper
```

**With GPU acceleration** (faster):
```bash
pip install openai-whisper[gpu]
# Requires CUDA toolkit
```

**For ROS 2 integration**:
```bash
pip install pyaudio sounddevice numpy
```

## Basic Whisper Usage

**Transcribe audio file**:
```python
import whisper

model = whisper.load_model("base")
result = model.transcribe("audio.mp3")
print(result["text"])
# Output: "Go to the kitchen and bring me a cup"
```

**With language hint** (faster):
```python
result = model.transcribe("audio.mp3", language="en")
```

## Real-Time Microphone Transcription

**whisper_node.py** (ROS 2 integration):
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import pyaudio
import numpy as np
import threading
import queue

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.publisher = self.create_publisher(String, '/voice/transcript', 10)

        # Load Whisper model
        self.get_logger().info('Loading Whisper model...')
        self.model = whisper.load_model("base")
        self.get_logger().info('Whisper ready')

        # Audio settings
        self.sample_rate = 16000
        self.chunk_duration = 3  # seconds
        self.audio_queue = queue.Queue()

        # Start audio capture thread
        self.audio_thread = threading.Thread(target=self.capture_audio, daemon=True)
        self.audio_thread.start()

        # Start transcription timer
        self.timer = self.create_timer(0.1, self.process_audio)

    def capture_audio(self):
        p = pyaudio.PyAudio()
        stream = p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=1024
        )

        self.get_logger().info('Microphone active. Speak now!')

        buffer = []
        chunk_size = self.sample_rate * self.chunk_duration

        while rclpy.ok():
            data = stream.read(1024)
            audio_data = np.frombuffer(data, dtype=np.int16)
            buffer.extend(audio_data)

            if len(buffer) >= chunk_size:
                self.audio_queue.put(np.array(buffer[:chunk_size]))
                buffer = buffer[chunk_size:]

    def process_audio(self):
        if not self.audio_queue.empty():
            audio_chunk = self.audio_queue.get()

            # Normalize audio
            audio_float = audio_chunk.astype(np.float32) / 32768.0

            # Transcribe
            result = self.model.transcribe(audio_float, language="en", fp16=False)
            text = result["text"].strip()

            if text:
                self.get_logger().info(f'Transcribed: "{text}"')
                msg = String()
                msg.data = text
                self.publisher.publish(msg)

def main():
    rclpy.init()
    node = WhisperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Run**:
```bash
ros2 run whisper_ros whisper_node
```

**Listen for transcripts**:
```bash
ros2 topic echo /voice/transcript
```

## Optimize for Latency

**Use faster-whisper** (CTranslate2 backend):
```bash
pip install faster-whisper
```

**Modified code**:
```python
from faster_whisper import WhisperModel

# Load model (4× faster than original Whisper)
model = WhisperModel("base", device="cuda", compute_type="float16")

# Transcribe
segments, info = model.transcribe("audio.mp3", language="en")
for segment in segments:
    print(segment.text)
```

**Performance comparison**:
| Implementation | GPU Inference Time (3s audio) |
|----------------|-------------------------------|
| **openai-whisper** | 1.2s |
| **faster-whisper** | 0.3s |
| **whisper.cpp** | 0.5s (CPU-only) |

## Voice Activity Detection (VAD)

**Problem**: Transcribing silence wastes compute.

**Solution**: Use VAD to detect speech before transcribing.

**With Silero VAD**:
```bash
pip install silero
```

```python
import torch
from silero_vad import load_silero_vad

vad_model = load_silero_vad()

def is_speech(audio_chunk):
    speech_prob = vad_model(torch.from_numpy(audio_chunk), 16000).item()
    return speech_prob > 0.5

# Only transcribe if speech detected
if is_speech(audio_chunk):
    result = whisper_model.transcribe(audio_chunk)
```

## Integrate with LLM Pipeline

**Complete voice → LLM flow**:
```python
class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.whisper_sub = self.create_subscription(
            String, '/voice/transcript', self.transcript_callback, 10
        )
        self.command_pub = self.create_publisher(String, '/robot/command', 10)

    def transcript_callback(self, msg):
        user_text = msg.data
        self.get_logger().info(f'User said: "{user_text}"')

        # Send to LLM planner (next chapter)
        # For now, simple keyword matching
        if "kitchen" in user_text.lower():
            cmd = String()
            cmd.data = '{"action": "navigate", "location": "kitchen"}'
            self.command_pub.publish(cmd)
```

## Troubleshooting

**"No module named 'whisper'"**:
```bash
pip install --upgrade openai-whisper
```

**"ALSA/PortAudio errors" (Linux)**:
```bash
sudo apt install portaudio19-dev python3-pyaudio
```

**"Slow transcription"**:
- Use `tiny` or `base` model instead of `large`
- Enable GPU: `pip install openai-whisper[gpu]`
- Use `faster-whisper` library

**"Transcription is gibberish"**:
- Check microphone input: `arecord -l` (Linux) or System Preferences (macOS)
- Ensure `language="en"` matches spoken language
- Reduce background noise

## Testing

**Validate transcription accuracy**:
```bash
# Record 10-second test
arecord -d 10 -f S16_LE -r 16000 test.wav

# Transcribe offline
python -c "import whisper; print(whisper.load_model('base').transcribe('test.wav')['text'])"
```

**Benchmark latency**:
```python
import time

start = time.time()
result = model.transcribe(audio_chunk)
latency = time.time() - start
print(f"Latency: {latency:.2f}s for {len(audio_chunk)/16000:.1f}s audio")
```

**Next**: [LLM Planning with GPT-4/Claude](/docs/module-04-vla/llm-planning)
