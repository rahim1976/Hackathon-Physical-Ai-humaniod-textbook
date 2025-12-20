# Voice-to-Action with OpenAI Whisper

## Introduction

This chapter introduces OpenAI Whisper as the foundation for voice-to-action systems in humanoid robotics. Whisper is a state-of-the-art speech recognition model that can convert spoken language into text with high accuracy. In the context of humanoid robots, Whisper enables natural human-robot interaction by allowing users to communicate with robots using everyday speech.

The chapter will cover the installation and configuration of Whisper, integration with audio input systems, and processing of voice commands for robotic action execution. You'll learn how to set up real-time voice processing pipelines and validate the accuracy of voice-to-text conversion in various environments.

## Learning Objectives

By the end of this chapter, you will be able to:
- Install and configure OpenAI Whisper for voice processing applications
- Set up audio input systems for real-time voice command processing
- Process voice commands with Whisper and validate output accuracy
- Handle different audio conditions and optimize Whisper performance
- Integrate Whisper with robotic control systems for action execution

## Prerequisites

Before starting this chapter, you should have:
- Basic understanding of audio processing concepts
- Familiarity with Python programming
- Access to OpenAI API keys for Whisper usage
- Basic knowledge of ROS 2 for later integration

## Understanding OpenAI Whisper

### Overview of Whisper Technology

OpenAI Whisper is a robust speech-to-text model that demonstrates strong performance across multiple languages and domains. The model is designed to handle various audio conditions including:

- Different accents and speaking styles
- Background noise and environmental conditions
- Various recording qualities and equipment
- Multi-language support (98+ languages)

### Whisper Model Variants

Whisper comes in several model sizes with different performance characteristics:

| Model | Size | Required VRAM | Relative Speed | English-only | Multilingual |
|-------|------|---------------|----------------|--------------|--------------|
| tiny  | 75 MB | ~1 GB | 32x | 3.0% | 4.0% |
| base  | 145 MB | ~1 GB | 16x | 2.9% | 3.5% |
| small | 445 MB | ~2 GB | 6x | 2.1% | 2.6% |
| medium | 830 MB | ~5 GB | 2x | 1.9% | 2.7% |
| large | 1.5 GB | ~10 GB | 1x | 1.8% | 2.9% |

*Based on LibriSpeech dev-clean dataset. Speed measured on Intel i9-12900K CPU.

### Key Features

1. **Multilingual Support**: Can transcribe and translate between languages
2. **Robustness**: Handles various audio conditions and accents
3. **Timestamps**: Provides precise timing information for speech segments
4. **Punctuation**: Automatically adds punctuation to transcriptions
5. **Speaker Diarization**: Can distinguish between different speakers

## Installing and Setting Up Whisper

### Installation Options

Whisper can be used in multiple ways depending on your requirements:

#### Option 1: OpenAI API (Recommended for beginners)

```bash
pip install openai
```

#### Option 2: OpenAI's Whisper Library (For local processing)

```bash
pip install openai-whisper
```

#### Option 3: Hugging Face Transformers (Alternative implementation)

```bash
pip install transformers torch
```

### Basic Setup and Configuration

```python
# Example: Basic Whisper setup with OpenAI API
import openai
import os

# Set your OpenAI API key
openai.api_key = os.getenv("OPENAI_API_KEY")

def transcribe_audio_file(audio_file_path):
    """
    Transcribe an audio file using OpenAI Whisper API
    """
    with open(audio_file_path, "rb") as audio_file:
        transcript = openai.Audio.transcribe(
            model="whisper-1",
            file=audio_file,
            response_format="text"
        )
    return transcript

# Example usage
transcription = transcribe_audio_file("command.wav")
print(f"Transcription: {transcription}")
```

### Local Whisper Installation (For advanced users)

For local processing without API calls:

```python
# Example: Local Whisper setup
import whisper

# Load different model sizes
model_tiny = whisper.load_model("tiny")
model_base = whisper.load_model("base")
model_small = whisper.load_model("small")
model_medium = whisper.load_model("medium")
model_large = whisper.load_model("large")

def transcribe_with_local_model(audio_path, model_size="small"):
    """
    Transcribe audio using local Whisper model
    """
    model = whisper.load_model(model_size)

    # Load audio and pad/trim it to fit 30-second context
    audio = whisper.load_audio(audio_path)
    audio = whisper.pad_or_trim(audio)

    # Make log-Mel spectrogram and move to the same device as the model
    mel = whisper.log_mel_spectrogram(audio).to(model.device)

    # Detect the spoken language with the largest probability
    _, probs = model.detect_language(mel)
    detected_lang = max(probs, key=probs.get)

    # Decode the audio
    options = whisper.DecodingOptions()
    result = whisper.decode(model, mel, options)

    return {
        "text": result.text,
        "language": detected_lang,
        "segments": result.segments if hasattr(result, 'segments') else []
    }
```

## Audio Input and Preprocessing

### Audio Capture Setup

For real-time voice command processing, you'll need to set up audio capture:

```python
# Example: Real-time audio capture for Whisper
import pyaudio
import wave
import numpy as np
import threading
import time
from datetime import datetime

class AudioRecorder:
    def __init__(self, chunk=1024, format=pyaudio.paInt16, channels=1, rate=16000):
        self.chunk = chunk
        self.format = format
        self.channels = channels
        self.rate = rate
        self.p = pyaudio.PyAudio()

        # Initialize recording parameters
        self.frames = []
        self.is_recording = False
        self.recording_thread = None

    def start_recording(self, filename=None):
        """
        Start recording audio from microphone
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recording_{timestamp}.wav"

        self.is_recording = True
        self.frames = []

        # Open stream
        self.stream = self.p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        # Start recording in a separate thread
        self.recording_thread = threading.Thread(target=self._record_audio)
        self.recording_thread.start()

        return filename

    def stop_recording(self, filename):
        """
        Stop recording and save to file
        """
        self.is_recording = False

        if self.recording_thread:
            self.recording_thread.join()

        # Save the recorded data as a WAV file
        wf = wave.open(filename, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(self.p.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(self.frames))
        wf.close()

        self.stream.stop_stream()
        self.stream.close()

        return filename

    def _record_audio(self):
        """
        Internal method to record audio in a loop
        """
        while self.is_recording:
            data = self.stream.read(self.chunk)
            self.frames.append(data)

# Example usage
recorder = AudioRecorder()
filename = recorder.start_recording()
time.sleep(5)  # Record for 5 seconds
recorder.stop_recording(filename)
```

### Audio Preprocessing for Whisper

Whisper works best with specific audio formats. Here's how to preprocess audio:

```python
# Example: Audio preprocessing for optimal Whisper performance
import librosa
import soundfile as sf
import numpy as np

def preprocess_audio_for_whisper(input_file, output_file=None):
    """
    Preprocess audio file to optimize for Whisper transcription
    """
    if output_file is None:
        output_file = input_file.replace('.wav', '_processed.wav')

    # Load audio file
    audio, sr = librosa.load(input_file, sr=16000, mono=True)

    # Normalize audio
    audio = audio / np.max(np.abs(audio))

    # Apply slight noise reduction (optional)
    # This can help with background noise
    from scipy import signal
    b, a = signal.butter(8, 0.04, 'highpass')
    audio = signal.filtfilt(b, a, audio)

    # Save processed audio
    sf.write(output_file, audio, 16000)

    return output_file

def validate_audio_quality(audio_file):
    """
    Validate audio quality for Whisper processing
    """
    audio, sr = librosa.load(audio_file)

    # Check if audio meets minimum requirements
    duration = len(audio) / sr
    rms = np.sqrt(np.mean(audio**2))

    quality_metrics = {
        'duration': duration,
        'sample_rate': sr,
        'rms_amplitude': rms,
        'is_silent': rms < 0.001,
        'is_too_long': duration > 25  # Whisper is optimized for shorter clips
    }

    return quality_metrics
```

## Real-time Voice Processing Pipeline

### Voice Activity Detection

Implementing voice activity detection to trigger processing only when speech is detected:

```python
# Example: Voice Activity Detection (VAD) for Whisper
import webrtcvad
import collections

class VoiceActivityDetector:
    def __init__(self, aggressiveness=3, sample_rate=16000, frame_duration_ms=30):
        self.vad = webrtcvad.Vad(aggressiveness)
        self.sample_rate = sample_rate
        self.frame_duration_ms = frame_duration_ms
        self.frame_size = int(sample_rate * frame_duration_ms / 1000) * 2  # 2 bytes per sample

        # For voice activity detection
        self.ring_buffer = collections.deque(maxlen=30)
        self.triggered = False
        self.vad_frames = []
        self.temp_end = 0
        self.max_silence_for_end = 30  # frames

    def is_speech(self, audio_frame):
        """
        Check if the audio frame contains speech
        """
        try:
            return self.vad.is_speech(audio_frame, self.sample_rate)
        except:
            return False

    def process_audio_chunk(self, audio_chunk):
        """
        Process audio chunk for voice activity detection
        """
        speech_detected = self.is_speech(audio_chunk)

        if not self.triggered:
            self.ring_buffer.append((audio_chunk, speech_detected))
            num_voiced = len([f for f, speech in self.ring_buffer if speech])

            # If we're getting more voice than silence
            if num_voiced > 0.8 * self.ring_buffer.maxlen:
                self.triggered = True
                self.vad_frames = [f for f, s in self.ring_buffer if s]
                self.ring_buffer.clear()
        else:
            self.vad_frames.append(audio_chunk)
            self.ring_buffer.append((audio_chunk, speech_detected))
            num_unvoiced = len([f for f, speech in self.ring_buffer if not speech])

            # If more than 90% of the frames in the ring buffer are unvoiced
            if num_unvoiced > 0.9 * self.ring_buffer.maxlen:
                self.temp_end = len(self.vad_frames)
            if self.temp_end != 0 and num_unvoiced > self.max_silence_for_end:
                self.triggered = False
                temp_vad_frames = self.vad_frames
                self.vad_frames = []
                self.temp_end = 0
                return True, temp_vad_frames  # Return the speech segment

        return False, []
```

### Complete Voice Processing Pipeline

```python
# Example: Complete voice-to-text processing pipeline
import asyncio
import tempfile
import os
from pydub import AudioSegment

class WhisperVoiceProcessor:
    def __init__(self, model_size="small", use_api=True):
        self.use_api = use_api
        self.model_size = model_size

        if not use_api:
            import whisper
            self.model = whisper.load_model(model_size)

        self.audio_recorder = AudioRecorder()
        self.vad_detector = VoiceActivityDetector()

    async def process_voice_command(self, timeout=10):
        """
        Process a voice command from start to finish
        """
        # Start recording
        temp_file = tempfile.mktemp(suffix='.wav')
        recording_file = self.audio_recorder.start_recording(temp_file)

        # Wait for voice activity or timeout
        start_time = time.time()
        speech_segments = []

        while time.time() - start_time < timeout:
            # In a real implementation, you'd continuously check for speech
            # This is simplified for the example
            pass

        # Stop recording
        self.audio_recorder.stop_recording(recording_file)

        # Process the recorded audio
        processed_file = preprocess_audio_for_whisper(recording_file)

        # Transcribe with Whisper
        if self.use_api:
            transcription = await self.transcribe_with_api(processed_file)
        else:
            transcription = await self.transcribe_locally(processed_file)

        # Clean up temporary files
        os.remove(recording_file)
        os.remove(processed_file)

        return transcription

    async def transcribe_with_api(self, audio_file):
        """
        Transcribe audio using OpenAI API
        """
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(
            None,
            transcribe_audio_file,
            audio_file
        )

    async def transcribe_locally(self, audio_file):
        """
        Transcribe audio using local Whisper model
        """
        result = transcribe_with_local_model(audio_file, self.model_size)
        return result["text"]
```

## Integration with Robotic Systems

### Voice Command Validation

Validating voice commands before passing them to the robotic system:

```python
# Example: Voice command validation and filtering
import re
from typing import List, Dict

class VoiceCommandValidator:
    def __init__(self):
        # Define safe commands that the robot can execute
        self.allowed_commands = {
            'navigation': [
                'go forward', 'move forward', 'go back', 'move back',
                'turn left', 'turn right', 'go to', 'move to', 'stop',
                'halt', 'pause', 'continue'
            ],
            'manipulation': [
                'pick up', 'grasp', 'grab', 'release', 'drop',
                'lift', 'lower', 'open', 'close'
            ],
            'communication': [
                'speak', 'say', 'hello', 'goodbye', 'thank you'
            ]
        }

        # Define potentially dangerous commands to filter
        self.dangerous_keywords = [
            'self-destruct', 'shutdown', 'power off', 'emergency stop',
            'kill', 'destroy', 'harm', 'danger'
        ]

    def validate_command(self, transcription: str) -> Dict:
        """
        Validate a voice command for safety and executability
        """
        command = transcription.lower().strip()
        validation_result = {
            'is_valid': False,
            'is_safe': True,
            'command_type': None,
            'parsed_command': None,
            'confidence': 0.0
        }

        # Check for dangerous keywords
        for keyword in self.dangerous_keywords:
            if keyword in command:
                validation_result['is_safe'] = False
                return validation_result

        # Try to match against allowed commands
        for cmd_type, commands in self.allowed_commands.items():
            for allowed_cmd in commands:
                if allowed_cmd in command:
                    validation_result['is_valid'] = True
                    validation_result['command_type'] = cmd_type
                    validation_result['parsed_command'] = allowed_cmd
                    validation_result['confidence'] = 0.8  # Base confidence

                    # Calculate more specific confidence based on match quality
                    if command == allowed_cmd:
                        validation_result['confidence'] = 1.0
                    elif allowed_cmd in command and len(command.split()) <= len(allowed_cmd.split()) + 2:
                        validation_result['confidence'] = 0.9

                    break
            if validation_result['is_valid']:
                break

        return validation_result

    def extract_parameters(self, command: str, command_type: str) -> Dict:
        """
        Extract parameters from voice command
        """
        params = {}

        if command_type == 'navigation':
            # Extract location or distance from navigation commands
            distance_match = re.search(r'(\d+(?:\.\d+)?)\s*(meters?|m|feet|ft)', command)
            if distance_match:
                params['distance'] = float(distance_match.group(1))
                params['unit'] = distance_match.group(2)

            # Extract destination from "go to" commands
            to_match = re.search(r'go to\s+(.+)|move to\s+(.+)', command)
            if to_match:
                destination = to_match.group(1) or to_match.group(2)
                params['destination'] = destination.strip()

        elif command_type == 'manipulation':
            # Extract object from manipulation commands
            object_match = re.search(r'(?:pick up|grasp|grab|lift)\s+(.+)', command)
            if object_match:
                params['object'] = object_match.group(1).strip()

        return params
```

### ROS 2 Integration

Integrating voice processing with ROS 2 for robotic action execution:

```python
# Example: ROS 2 integration for voice-controlled robot actions
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import AudioData
import json

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')

        # Publishers for different robot actions
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speech_publisher = self.create_publisher(String, '/robot_speech', 10)

        # Subscriber for voice commands
        self.voice_subscriber = self.create_subscription(
            String, '/voice_commands', self.voice_command_callback, 10
        )

        # Initialize command validator
        self.validator = VoiceCommandValidator()

        self.get_logger().info('Voice control node initialized')

    def voice_command_callback(self, msg):
        """
        Process incoming voice command
        """
        command_text = msg.data

        # Validate the command
        validation = self.validator.validate_command(command_text)

        if not validation['is_valid']:
            self.get_logger().warn(f'Invalid command: {command_text}')
            return

        if not validation['is_safe']:
            self.get_logger().error(f'Dangerous command detected: {command_text}')
            return

        # Extract parameters
        params = self.validator.extract_parameters(command_text, validation['command_type'])

        # Execute appropriate action based on command type
        if validation['command_type'] == 'navigation':
            self.execute_navigation_command(command_text, params)
        elif validation['command_type'] == 'manipulation':
            self.execute_manipulation_command(command_text, params)
        elif validation['command_type'] == 'communication':
            self.execute_communication_command(command_text, params)

    def execute_navigation_command(self, command, params):
        """
        Execute navigation-related voice commands
        """
        twist = Twist()

        if 'forward' in command or 'move' in command:
            twist.linear.x = 0.5  # Move forward at 0.5 m/s
        elif 'back' in command:
            twist.linear.x = -0.5  # Move backward
        elif 'left' in command:
            twist.angular.z = 0.5  # Turn left
        elif 'right' in command:
            twist.angular.z = -0.5  # Turn right
        elif 'stop' in command or 'halt' in command:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Executed navigation command: {command}')

    def execute_communication_command(self, command, params):
        """
        Execute communication-related voice commands
        """
        response_msg = String()

        if 'hello' in command:
            response_msg.data = "Hello! How can I assist you today?"
        elif 'thank you' in command:
            response_msg.data = "You're welcome! Happy to help."
        elif 'goodbye' in command:
            response_msg.data = "Goodbye! Have a great day."
        else:
            response_msg.data = f"I heard you say: {command}"

        self.speech_publisher.publish(response_msg)
```

## Performance Optimization and Tuning

### Whisper Model Selection

Choosing the right Whisper model based on your requirements:

```python
# Example: Model selection based on performance requirements
class WhisperModelSelector:
    def __init__(self):
        self.models = {
            'tiny': {'size': 75, 'vram': 1, 'speed': 32, 'accuracy': 0.96},
            'base': {'size': 145, 'vram': 1, 'speed': 16, 'accuracy': 0.97},
            'small': {'size': 445, 'vram': 2, 'speed': 6, 'accuracy': 0.979},
            'medium': {'size': 830, 'vram': 5, 'speed': 2, 'accuracy': 0.981},
            'large': {'size': 1500, 'vram': 10, 'speed': 1, 'accuracy': 0.971}
        }

    def recommend_model(self, requirements):
        """
        Recommend the best Whisper model based on requirements
        """
        vram_available = requirements.get('vram_gb', 8)
        speed_requirement = requirements.get('min_speed_x', 10)  # Minimum speed multiplier
        accuracy_requirement = requirements.get('min_accuracy', 0.95)
        max_size_mb = requirements.get('max_model_size_mb', 1000)

        best_model = None
        for model_name, specs in self.models.items():
            if (specs['vram'] <= vram_available and
                specs['speed'] >= speed_requirement and
                specs['accuracy'] >= accuracy_requirement and
                specs['size'] <= max_size_mb):

                if best_model is None or self.models[best_model]['accuracy'] < specs['accuracy']:
                    best_model = model_name

        return best_model or 'base'  # Default fallback

# Example usage
selector = WhisperModelSelector()
requirements = {
    'vram_gb': 6,
    'min_speed_x': 5,
    'min_accuracy': 0.97,
    'max_model_size_mb': 500
}
recommended_model = selector.recommend_model(requirements)
print(f"Recommended model: {recommended_model}")
```

### Audio Quality Optimization

Optimizing audio input for better Whisper performance:

```python
# Example: Audio quality optimization techniques
class AudioQualityOptimizer:
    def __init__(self):
        self.noise_threshold = 0.01
        self.silence_threshold = 0.001
        self.min_speech_duration = 0.2  # seconds

    def optimize_audio_input(self, audio_data, sample_rate=16000):
        """
        Optimize audio input for Whisper processing
        """
        import librosa
        import numpy as np

        # Normalize audio
        audio_data = audio_data / np.max(np.abs(audio_data))

        # Apply noise reduction if needed
        if np.std(audio_data) > self.noise_threshold:
            # Simple spectral gating for noise reduction
            from scipy import signal
            b, a = signal.butter(8, 0.04, 'highpass')
            audio_data = signal.filtfilt(b, a, audio_data)

        # Trim silence from beginning and end
        trimmed_audio, _ = librosa.effects.trim(
            audio_data,
            top_db=20,  # Adjust threshold as needed
            frame_length=2048,
            hop_length=512
        )

        # Ensure minimum length for Whisper
        min_samples = sample_rate * 0.1  # 0.1 seconds minimum
        if len(trimmed_audio) < min_samples:
            # Pad with zeros if too short
            padding = min_samples - len(trimmed_audio)
            trimmed_audio = np.pad(trimmed_audio, (0, padding), mode='constant')

        return trimmed_audio

    def detect_audio_quality_issues(self, audio_data, sample_rate=16000):
        """
        Detect potential audio quality issues
        """
        import numpy as np

        metrics = {}

        # Calculate RMS amplitude
        rms = np.sqrt(np.mean(audio_data**2))
        metrics['rms_amplitude'] = float(rms)

        # Check for clipping
        max_val = np.max(np.abs(audio_data))
        metrics['clipping_ratio'] = float(max_val / 0.99) if max_val > 0 else 0.0

        # Check for silence
        metrics['is_silent'] = rms < self.silence_threshold

        # Calculate signal-to-noise ratio (approximate)
        if len(audio_data) > 1000:  # Need enough samples
            # Estimate noise as the minimum amplitude in quiet segments
            window_size = 1024
            windows = len(audio_data) // window_size
            window_amps = []

            for i in range(windows):
                window = audio_data[i*window_size:(i+1)*window_size]
                window_amp = np.sqrt(np.mean(window**2))
                window_amps.append(window_amp)

            if window_amps:
                noise_floor = np.percentile(window_amps, 10)  # 10th percentile as noise estimate
                signal_level = np.percentile(window_amps, 90)  # 90th percentile as signal estimate
                if noise_floor > 0:
                    metrics['estimated_snr_db'] = 20 * np.log10(signal_level / noise_floor)
                else:
                    metrics['estimated_snr_db'] = float('inf')

        return metrics
```

## Practical Examples

### Example 1: Basic Voice Command System

```python
# Example: Complete basic voice command system
import asyncio
import threading
import time

class BasicVoiceCommandSystem:
    def __init__(self):
        self.processor = WhisperVoiceProcessor(model_size="base")
        self.validator = VoiceCommandValidator()
        self.robot_control = VoiceControlNode()  # ROS 2 node

        self.listening = False
        self.command_queue = []

    def start_listening(self):
        """
        Start the voice command listening system
        """
        self.listening = True

        while self.listening:
            try:
                # Process a voice command
                transcription = asyncio.run(
                    self.processor.process_voice_command(timeout=5)
                )

                if transcription.strip():
                    # Validate the command
                    validation = self.validator.validate_command(transcription)

                    if validation['is_valid'] and validation['is_safe']:
                        # Add to command queue for processing
                        self.command_queue.append({
                            'command': transcription,
                            'validation': validation,
                            'timestamp': time.time()
                        })

                        # Execute immediately for this example
                        self.execute_command(transcription, validation)
                    else:
                        print(f"Command rejected: {transcription}")

            except Exception as e:
                print(f"Error processing voice command: {e}")
                time.sleep(0.1)  # Brief pause before retrying

    def execute_command(self, command, validation):
        """
        Execute a validated voice command
        """
        print(f"Executing command: {command}")

        # In a real system, you would send this to your robot control system
        # For now, we'll just print what would happen
        params = self.validator.extract_parameters(command, validation['command_type'])
        print(f"Command type: {validation['command_type']}, Parameters: {params}")

    def stop_listening(self):
        """
        Stop the voice command system
        """
        self.listening = False

# Example usage
if __name__ == "__main__":
    system = BasicVoiceCommandSystem()

    # Start listening in a separate thread
    listen_thread = threading.Thread(target=system.start_listening)
    listen_thread.start()

    # Let it run for a while
    time.sleep(30)  # Listen for 30 seconds

    system.stop_listening()
    listen_thread.join()
```

### Example 2: Voice Command with Context Awareness

Implementing voice commands that consider the robot's current state and environment.

## Troubleshooting

### Common Issues and Solutions

1. **Poor Transcription Accuracy**
   - Ensure audio is recorded at 16kHz sample rate
   - Use a good quality microphone positioned properly
   - Minimize background noise during recording
   - Consider using a larger Whisper model for better accuracy

2. **High Latency**
   - Use smaller Whisper models for faster processing
   - Optimize audio preprocessing steps
   - Consider using GPU acceleration if available
   - Use streaming approaches for real-time processing

3. **Memory Issues**
   - Use smaller models that fit in available memory
   - Process audio in smaller chunks
   - Clear model cache when switching models
   - Consider using the API for memory-intensive operations

### Performance Monitoring

```python
# Example: Performance monitoring for voice processing
class VoiceProcessingMonitor:
    def __init__(self):
        self.transcription_times = []
        self.accuracy_metrics = []
        self.error_count = 0

    def log_transcription(self, transcription_time, accuracy=None):
        """
        Log transcription performance metrics
        """
        self.transcription_times.append(transcription_time)
        if accuracy is not None:
            self.accuracy_metrics.append(accuracy)

    def get_performance_summary(self):
        """
        Get summary of voice processing performance
        """
        if not self.transcription_times:
            return "No data collected yet"

        avg_time = sum(self.transcription_times) / len(self.transcription_times)
        min_time = min(self.transcription_times)
        max_time = max(self.transcription_times)

        summary = {
            'average_transcription_time': avg_time,
            'min_transcription_time': min_time,
            'max_transcription_time': max_time,
            'total_transcriptions': len(self.transcription_times),
            'error_count': self.error_count
        }

        if self.accuracy_metrics:
            avg_accuracy = sum(self.accuracy_metrics) / len(self.accuracy_metrics)
            summary['average_accuracy'] = avg_accuracy

        return summary
```

## Exercises

1. **Whisper Setup**: Install and configure Whisper with different model sizes, comparing performance and accuracy
2. **Audio Processing**: Set up real-time audio capture and preprocessing pipeline for optimal Whisper performance
3. **Command Validation**: Implement a more sophisticated command validation system with custom command vocabularies
4. **Integration Challenge**: Integrate voice processing with a simple robot simulation to execute basic commands

## Summary

This chapter has introduced you to OpenAI Whisper for voice-to-action systems in humanoid robotics. You've learned how to set up and configure Whisper, process real-time audio, validate commands for safety, and integrate with robotic systems. The foundation laid here will be essential for the cognitive planning and complete VLA integration covered in subsequent chapters.

## Next Steps

In the next chapter, we'll explore cognitive planning using LLMs to convert natural language commands into ROS 2 actions, building upon the voice processing foundation established here.