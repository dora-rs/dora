#!/usr/bin/env python3
"""Final test of MoYoYo TTS using VoiceDialogue models."""

import sys
import os
from pathlib import Path

# Set up paths
voicedialogue_path = Path("/Users/yuechen/home/VoiceDialogue")
third_party_path = voicedialogue_path / "third_party"

# Add to Python path
sys.path.insert(0, str(third_party_path))
sys.path.insert(0, str(voicedialogue_path / "src"))

# Monkey-patch LangSegment before any imports
import importlib.util
spec = importlib.util.spec_from_file_location(
    "LangSegment.LangSegment",
    str(third_party_path / "moyoyo_tts" / "LangSegment_fix.py")
)
langseg_module = importlib.util.module_from_spec(spec)
sys.modules['LangSegment'] = langseg_module
sys.modules['LangSegment.LangSegment'] = langseg_module
spec.loader.exec_module(langseg_module)

# Now import MoYoYo TTS
try:
    from moyoyo_tts.TTS_infer_pack.TTS import TTS_Config, TTS
    print("✓ Successfully imported MoYoYo TTS")
    MOYOYO_AVAILABLE = True
except ImportError as e:
    print(f"✗ Failed to import MoYoYo TTS: {e}")
    MOYOYO_AVAILABLE = False

def test_real_tts():
    """Test real TTS synthesis."""
    if not MOYOYO_AVAILABLE:
        print("MoYoYo TTS not available")
        return None
    
    print("\n" + "=" * 60)
    print("Testing Real MoYoYo TTS")
    print("=" * 60)
    
    # Configuration
    models_path = voicedialogue_path / "assets/models/tts/moyoyo"
    
    custom_config = {
        "device": "cpu",
        "is_half": False,
        "version": "v2",
        "t2s_weights_path": str(models_path / "GPT_weights/doubao_best_gpt.ckpt"),
        "vits_weights_path": str(models_path / "SoVITS_weights/doubao_best_sovits.pth"),
        "cnhuhbert_base_path": str(models_path / "chinese-hubert-base"),
        "bert_base_path": str(models_path / "chinese-roberta-wwm-ext-large"),
    }
    
    # Check files exist
    for key, path in custom_config.items():
        if key in ["device", "is_half", "version"]:
            continue
        p = Path(path)
        if p.is_file():
            if p.exists():
                print(f"✓ {key}: {p.stat().st_size / 1024 / 1024:.2f} MB")
            else:
                print(f"✗ Missing: {path}")
                return None
        elif p.is_dir():
            if p.exists():
                total_size = sum(f.stat().st_size for f in p.rglob("*") if f.is_file())
                print(f"✓ {key}: {total_size / 1024 / 1024:.2f} MB")
            else:
                print(f"✗ Missing directory: {path}")
                return None
    
    # Create config dict with proper structure
    config_dict = {
        "version": "v2",
        "custom": custom_config
    }
    
    # Initialize TTS directly
    print("\nInitializing TTS...")
    tts = TTS(config_dict)
    
    print("✓ TTS initialized")
    
    # Get reference audio
    ref_audio_path = str(models_path / "ref_audios/doubao_ref.wav")
    prompt_text = "我叫豆包呀，能陪你聊天解闷，不管是聊生活趣事，知识科普还是帮你出主意，我都在行哦。"
    
    # Test synthesis
    test_text = "你好，我是豆包。终于成功使用真实的MoYoYo语音合成引擎生成了这段测试音频。语音效果应该非常自然流畅。"
    print(f"\nSynthesizing: {test_text}")
    
    # Generate audio using run method (it's a generator)
    inputs = {
        "text": test_text,
        "text_lang": "zh",
        "ref_audio_path": ref_audio_path,
        "prompt_text": prompt_text,
        "prompt_lang": "zh",
        "top_k": 5,
        "top_p": 1.0,
        "temperature": 1.0,
        "speed_factor": 1.1,
    }
    
    # Get the first (and only) result from the generator
    for result in tts.run(inputs):
        sample_rate, audio_data = result
        break
    
    # Save audio
    import soundfile as sf
    import numpy as np
    
    # Convert to float32 if needed
    if audio_data.dtype != np.float32:
        if audio_data.dtype == np.int16:
            audio_data = audio_data.astype(np.float32) / 32768.0
        else:
            audio_data = audio_data.astype(np.float32)
    
    output_path = "test_real_moyoyo.wav"
    sf.write(output_path, audio_data, sample_rate)
    
    print(f"\n✓ Audio saved to: {output_path}")
    print(f"  Sample rate: {sample_rate} Hz")
    print(f"  Duration: {len(audio_data) / sample_rate:.2f} seconds")
    print(f"  Audio shape: {audio_data.shape}")
    print(f"  Audio dtype: {audio_data.dtype}")
    
    return output_path

if __name__ == "__main__":
    try:
        output = test_real_tts()
        if output:
            print(f"\n{'='*60}")
            print("✓ SUCCESS! Real voice synthesis completed!")
            print(f"Play the audio with: afplay {output}")
            print(f"{'='*60}")
        else:
            print("\n✗ Test failed")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()