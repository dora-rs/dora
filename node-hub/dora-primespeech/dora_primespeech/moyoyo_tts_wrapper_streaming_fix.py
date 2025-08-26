#!/usr/bin/env python3
"""Fixed MoYoYo TTS wrapper with real streaming support via audio chunking."""

import sys
import os
from pathlib import Path
import numpy as np
import soundfile as sf
import importlib.util
import logging
from typing import Generator, Tuple, Optional
import re

logger = logging.getLogger(__name__)

# Setup paths for VoiceDialogue MoYoYo TTS
# Try environment variable first, then fallback to default
VOICEDIALOGUE_PATH = Path(os.environ.get("VOICEDIALOGUE_PATH", 
                                         os.path.expanduser("~/VoiceDialogue")))
# Also check common installation locations
if not VOICEDIALOGUE_PATH.exists():
    # Try the original hardcoded path as last resort
    if Path("/Users/yuechen/home/VoiceDialogue").exists():
        VOICEDIALOGUE_PATH = Path("/Users/yuechen/home/VoiceDialogue")
        
THIRD_PARTY_PATH = VOICEDIALOGUE_PATH / "third_party"
MOYOYO_TTS_PATH = THIRD_PARTY_PATH / "moyoyo_tts"

# Check if MoYoYo TTS is available
MOYOYO_AVAILABLE = False
if MOYOYO_TTS_PATH.exists():
    # Add to Python path
    sys.path.insert(0, str(THIRD_PARTY_PATH))
    sys.path.insert(0, str(VOICEDIALOGUE_PATH / "src"))
    
    # Monkey-patch LangSegment before any imports
    try:
        spec = importlib.util.spec_from_file_location(
            "LangSegment.LangSegment",
            str(MOYOYO_TTS_PATH / "LangSegment_fix.py")
        )
        langseg_module = importlib.util.module_from_spec(spec)
        sys.modules['LangSegment'] = langseg_module
        sys.modules['LangSegment.LangSegment'] = langseg_module
        spec.loader.exec_module(langseg_module)
        
        # Now import MoYoYo TTS
        from moyoyo_tts.TTS_infer_pack.TTS import TTS_Config, TTS
        from moyoyo_tts.TTS_infer_pack.text_segmentation_method import get_method as get_seg_method
        MOYOYO_AVAILABLE = True
        logger.info("MoYoYo TTS successfully imported")
    except ImportError as e:
        logger.warning(f"Failed to import MoYoYo TTS: {e}")
        MOYOYO_AVAILABLE = False


class StreamingMoYoYoTTSWrapper:
    """Fixed wrapper for MoYoYo TTS with real streaming via audio chunking."""
    
    def __init__(self, voice="doubao", device="cpu", enable_streaming=True, chunk_duration=0.5):
        """Initialize streaming MoYoYo TTS wrapper.
        
        Args:
            voice: Voice name (doubao, luoxiang, yangmi, etc.)
            device: Device to use (cpu or cuda)
            enable_streaming: Enable streaming output
            chunk_duration: Duration of each audio chunk in seconds for streaming
        """
        self.voice = voice
        self.device = device
        self.enable_streaming = enable_streaming
        self.chunk_duration = chunk_duration  # Duration of each streamed chunk
        self.tts = None
        self.models_path = VOICEDIALOGUE_PATH / "assets/models/tts/moyoyo"
        
        # Abort flag for interrupting synthesis
        self._abort_synthesis = False
        
        # Optimization parameters - disable MoYoYo's broken "streaming"
        self.optimization_config = {
            "batch_size": 10,  # Smaller batches for faster first output
            "text_split_method": "cut5",  # Automatic segmentation
            "split_bucket": True,  # Enable bucketing
            "return_fragment": False,  # DISABLE MoYoYo's broken streaming
            "fragment_interval": 0.03,
            "parallel_infer": True,
            "top_k": 3,
            "top_p": 0.95,
            "temperature": 0.8,
            "repetition_penalty": 1.35,
            "seed": 233333,
        }
        
        if MOYOYO_AVAILABLE:
            self._init_tts()
    
    def _init_tts(self):
        """Initialize the TTS engine."""
        if not MOYOYO_AVAILABLE:
            logger.warning("MoYoYo TTS not available")
            return
        
        # Configuration based on voice
        voice_configs = {
            "doubao": {
                "t2s_weights": "GPT_weights/doubao_best_gpt.ckpt",
                "vits_weights": "SoVITS_weights/doubao_best_sovits.pth",
                "ref_audio": "ref_audios/doubao_ref.wav",
                "prompt_text": "我叫豆包呀，能陪你聊天解闷，不管是聊生活趣事，知识科普还是帮你出主意，我都在行哦。",
            },
            "luoxiang": {
                "t2s_weights": "GPT_weights/luoxiang_best_gpt.ckpt",
                "vits_weights": "SoVITS_weights/luoxiang_best_sovits.pth",
                "ref_audio": "ref_audios/luoxiang_ref.wav",
                "prompt_text": "我觉得你的逻辑非常混乱，这已经涉及法外狂徒了。希望大家提高法律意识，培养法制观念，千万不要有侥幸的心理。",
            },
            "yangmi": {
                "t2s_weights": "GPT_weights/yangmi_best_gpt.ckpt",
                "vits_weights": "SoVITS_weights/yangmi_best_sovits.pth",
                "ref_audio": "ref_audios/yangmi_ref.wav",
                "prompt_text": "让梦想，照进现实。不管在什么情况下，只要是坚定的，永远相信美好的事情，即将发生。",
            },
        }
        
        voice_config = voice_configs.get(self.voice, voice_configs["doubao"])
        
        # Create TTS configuration
        custom_config = {
            "device": self.device,
            "is_half": self.device != "cpu",
            "version": "v2",
            "t2s_weights_path": str(self.models_path / voice_config["t2s_weights"]),
            "vits_weights_path": str(self.models_path / voice_config["vits_weights"]),
            "cnhuhbert_base_path": str(self.models_path / "chinese-hubert-base"),
            "bert_base_path": str(self.models_path / "chinese-roberta-wwm-ext-large"),
        }
        
        config_dict = {
            "version": "v2",
            "custom": custom_config
        }
        
        try:
            logger.info(f"Initializing MoYoYo TTS with voice: {self.voice}")
            self.tts = TTS(config_dict)
            
            # Store reference audio info
            self.ref_audio_path = str(self.models_path / voice_config["ref_audio"])
            self.prompt_text = voice_config["prompt_text"]
            
            # Pre-cache reference audio
            self.tts.set_ref_audio(self.ref_audio_path)
            
            logger.info("MoYoYo TTS initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize MoYoYo TTS: {e}")
            self.tts = None
    
    def _split_text_smartly(self, text, max_chunk_chars=50):
        """Split text into smaller chunks for progressive synthesis.
        
        Args:
            text: Input text
            max_chunk_chars: Maximum characters per chunk
            
        Returns:
            list: Text chunks
        """
        # First split by sentences
        sentence_delimiters = r'[。！？.!?；;]'
        sentences = re.split(f'({sentence_delimiters})', text)
        
        # Reconstruct sentences with their delimiters
        reconstructed = []
        for i in range(0, len(sentences), 2):
            if i + 1 < len(sentences):
                reconstructed.append(sentences[i] + sentences[i + 1])
            else:
                reconstructed.append(sentences[i])
        
        # Group sentences into chunks
        chunks = []
        current_chunk = ""
        
        for sentence in reconstructed:
            sentence = sentence.strip()
            if not sentence:
                continue
                
            if len(current_chunk) + len(sentence) <= max_chunk_chars:
                current_chunk += sentence
            else:
                if current_chunk:
                    chunks.append(current_chunk)
                
                # If single sentence is too long, split by commas
                if len(sentence) > max_chunk_chars:
                    comma_parts = re.split(r'([，,])', sentence)
                    sub_chunk = ""
                    for part in comma_parts:
                        if len(sub_chunk) + len(part) <= max_chunk_chars:
                            sub_chunk += part
                        else:
                            if sub_chunk:
                                chunks.append(sub_chunk)
                            sub_chunk = part
                    if sub_chunk:
                        chunks.append(sub_chunk)
                else:
                    current_chunk = sentence
        
        if current_chunk:
            chunks.append(current_chunk)
        
        return chunks
    
    def _chunk_audio(self, audio_data, sample_rate):
        """Split audio into smaller chunks for streaming.
        
        Args:
            audio_data: Audio numpy array
            sample_rate: Sample rate
            
        Yields:
            Audio chunks
        """
        chunk_samples = int(sample_rate * self.chunk_duration)
        total_samples = len(audio_data)
        
        for start_idx in range(0, total_samples, chunk_samples):
            end_idx = min(start_idx + chunk_samples, total_samples)
            chunk = audio_data[start_idx:end_idx]
            
            # Apply fade in/out to avoid clicks
            if len(chunk) > 100:
                # Fade in for first 10 samples
                if start_idx == 0:
                    fade_len = min(10, len(chunk) // 2)
                    fade_in = np.linspace(0, 1, fade_len)
                    chunk[:fade_len] *= fade_in
                
                # Fade out for last 10 samples
                if end_idx == total_samples:
                    fade_len = min(10, len(chunk) // 2)
                    fade_out = np.linspace(1, 0, fade_len)
                    chunk[-fade_len:] *= fade_out
            
            yield chunk
    
    def abort_synthesis(self):
        """Abort any ongoing synthesis."""
        self._abort_synthesis = True
        logger.info("Synthesis abort requested")
    
    def synthesize_streaming(self, text, language="zh", speed=1.0) -> Generator[Tuple[int, np.ndarray], None, None]:
        """Synthesize speech with real streaming output.
        
        Args:
            text: Text to synthesize
            language: Language code
            speed: Speed factor
        
        Yields:
            tuple: (sample_rate, audio_fragment) for each fragment
        """
        if not MOYOYO_AVAILABLE or self.tts is None:
            logger.warning("MoYoYo TTS not available")
            # Return placeholder
            sample_rate = 16000
            duration = min(len(text) * 0.15, 10.0)
            t = np.linspace(0, duration, int(sample_rate * duration))
            audio = np.sin(2 * np.pi * 440 * t) * 0.3
            yield sample_rate, audio.astype(np.float32)
            return
        
        # Reset abort flag at start of new synthesis (safe timing)
        self._abort_synthesis = False
        
        try:
            logger.info(f"Starting streaming synthesis for {len(text)} chars")
            
            # Split text into smaller chunks for progressive synthesis
            text_chunks = self._split_text_smartly(text, max_chunk_chars=40)
            logger.info(f"Split into {len(text_chunks)} text chunks")
            
            fragment_count = 0
            
            # Process each text chunk
            for chunk_idx, text_chunk in enumerate(text_chunks):
                # Check abort flag
                if self._abort_synthesis:
                    logger.info(f"Synthesis aborted at text chunk {chunk_idx + 1}/{len(text_chunks)}")
                    break
                if not text_chunk.strip():
                    continue
                
                logger.debug(f"Processing chunk {chunk_idx + 1}: {text_chunk[:30]}...")
                
                # Prepare inputs without MoYoYo's broken streaming
                inputs = {
                    "text": text_chunk,
                    "text_lang": language,
                    "ref_audio_path": self.ref_audio_path,
                    "prompt_text": self.prompt_text,
                    "prompt_lang": "zh",
                    "speed_factor": speed,
                    "return_fragment": False,  # Don't use MoYoYo's broken streaming
                    **self.optimization_config
                }
                
                # Generate audio for this text chunk
                for result in self.tts.run(inputs):
                    sample_rate, chunk_audio = result
                    break  # Only yields once when return_fragment=False
                
                # Convert to float32 if needed
                if chunk_audio.dtype == np.int16:
                    chunk_audio = chunk_audio.astype(np.float32) / 32768.0
                
                # Stream this chunk's audio in smaller pieces
                for audio_fragment in self._chunk_audio(chunk_audio, sample_rate):
                    # Check abort flag before yielding each fragment
                    if self._abort_synthesis:
                        logger.info(f"Synthesis aborted at audio fragment {fragment_count + 1}")
                        return  # Exit generator completely
                    
                    fragment_count += 1
                    logger.debug(f"Yielding fragment {fragment_count}: {len(audio_fragment)/sample_rate:.3f}s")
                    yield sample_rate, audio_fragment
            
            logger.info(f"Streaming complete: {fragment_count} fragments")
            
        except Exception as e:
            logger.error(f"Streaming synthesis failed: {e}")
            # Yield placeholder on error
            sample_rate = 16000
            duration = min(len(text) * 0.15, 10.0)
            t = np.linspace(0, duration, int(sample_rate * duration))
            audio = np.sin(2 * np.pi * 440 * t) * 0.3
            yield sample_rate, audio.astype(np.float32)
    
    def synthesize(self, text, language="zh", speed=1.0):
        """Synthesize speech from text (non-streaming).
        
        Args:
            text: Text to synthesize
            language: Language code
            speed: Speed factor
        
        Returns:
            tuple: (sample_rate, audio_data) or (None, None) if failed
        """
        # Reset abort flag at start of new synthesis (safe timing)
        self._abort_synthesis = False
        
        if not MOYOYO_AVAILABLE or self.tts is None:
            logger.warning("MoYoYo TTS not available")
            # Return placeholder
            sample_rate = 16000
            duration = min(len(text) * 0.15, 10.0)
            t = np.linspace(0, duration, int(sample_rate * duration))
            audio = np.sin(2 * np.pi * 440 * t) * 0.3
            return sample_rate, audio.astype(np.float32)
        
        try:
            # Prepare inputs
            inputs = {
                "text": text,
                "text_lang": language,
                "ref_audio_path": self.ref_audio_path,
                "prompt_text": self.prompt_text,
                "prompt_lang": "zh",
                "speed_factor": speed,
                "return_fragment": False,
                **self.optimization_config
            }
            
            logger.info(f"Synthesizing {len(text)} chars")
            
            # Generate audio
            for result in self.tts.run(inputs):
                sample_rate, audio_data = result
                break
            
            # Convert to float32 if needed
            if audio_data.dtype == np.int16:
                audio_data = audio_data.astype(np.float32) / 32768.0
            
            logger.info(f"Synthesized {len(audio_data)/sample_rate:.2f}s audio")
            return sample_rate, audio_data
            
        except Exception as e:
            logger.error(f"TTS synthesis failed: {e}")
            # Return placeholder
            sample_rate = 16000
            duration = min(len(text) * 0.15, 10.0)
            t = np.linspace(0, duration, int(sample_rate * duration))
            audio = np.sin(2 * np.pi * 440 * t) * 0.3
            return sample_rate, audio.astype(np.float32)


# Test function
def test_streaming_fix():
    """Test the fixed streaming wrapper."""
    import time
    import sounddevice as sd
    
    print("\n" + "=" * 60)
    print("Testing Fixed Streaming MoYoYo TTS Wrapper")
    print("=" * 60)
    
    # Initialize wrapper
    tts = StreamingMoYoYoTTSWrapper(
        voice="doubao", 
        device="cpu", 
        enable_streaming=True,
        chunk_duration=0.3  # 300ms chunks
    )
    
    # Test text
    test_text = "你好，这是真正的流式语音合成测试。人工智能技术正在改变我们的世界，让机器能够理解和生成人类语言。"
    
    print(f"\nTest text: {test_text}")
    print("\nStarting streaming synthesis...")
    
    start_time = time.time()
    fragment_count = 0
    first_fragment_time = None
    
    for sample_rate, audio_fragment in tts.synthesize_streaming(test_text, language="zh", speed=1.1):
        fragment_count += 1
        elapsed = time.time() - start_time
        
        if fragment_count == 1:
            first_fragment_time = elapsed
            print(f"\n🎯 First fragment at {elapsed:.3f}s")
            if elapsed < 1.0:
                print("✅ EXCELLENT - Under 1 second!")
            elif elapsed < 2.0:
                print("👍 GOOD - Under 2 seconds")
            else:
                print("⚠️  SLOW - Over 2 seconds")
        
        duration = len(audio_fragment) / sample_rate
        print(f"  Fragment {fragment_count}: {duration:.3f}s audio at {elapsed:.2f}s")
        
        # Play the audio
        sd.play(audio_fragment, sample_rate)
        
        if fragment_count >= 10:
            print("  ... (stopping after 10 fragments)")
            break
    
    total_time = time.time() - start_time
    
    print("\n" + "=" * 60)
    print("Results:")
    print(f"  Total fragments: {fragment_count}")
    print(f"  First fragment latency: {first_fragment_time:.3f}s" if first_fragment_time else "  No fragments")
    print(f"  Total time: {total_time:.2f}s")
    
    if fragment_count > 1:
        print("\n✅ REAL STREAMING IS WORKING!")
    else:
        print("\n❌ Still not streaming properly")
    
    print("=" * 60)
    
    # Wait for audio to finish
    sd.wait()


if __name__ == "__main__":
    test_streaming_fix()