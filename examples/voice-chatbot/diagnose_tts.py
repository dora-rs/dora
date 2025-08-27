#!/usr/bin/env python3
"""
Diagnostic script for PrimeSpeech TTS tone generation issue.
This script helps identify why TTS generates tones instead of real voice.
"""

import os
import sys
from pathlib import Path
import subprocess

def check_model_files():
    """Check if PrimeSpeech models are real or placeholders."""
    models_dir = Path.home() / ".dora" / "models" / "primespeech" / "moyoyo"
    
    print("=" * 60)
    print("PRIMESPEECH MODEL FILE CHECK")
    print("=" * 60)
    
    required_models = {
        "GPT_weights/doubao_best_gpt.ckpt": 140_000_000,  # ~140MB
        "SoVITS_weights/doubao_best_sovits.pth": 80_000_000,  # ~80MB
        "ref_audios/doubao_ref.wav": 50_000,  # >50KB
        "chinese-hubert-base/pytorch_model.bin": 180_000_000,  # ~180MB
        "chinese-roberta-wwm-ext-large/pytorch_model.bin": 610_000_000,  # ~610MB
    }
    
    issues = []
    
    for model_path, min_size in required_models.items():
        full_path = models_dir / model_path
        
        if not full_path.exists():
            print(f"❌ MISSING: {model_path}")
            issues.append(f"Missing {model_path}")
        else:
            size = full_path.stat().st_size
            if size < min_size:
                print(f"⚠️  PLACEHOLDER: {model_path} (size: {size/1024/1024:.1f}MB, expected: >{min_size/1024/1024:.0f}MB)")
                issues.append(f"Placeholder file: {model_path}")
                
                # Check if it's a text placeholder
                try:
                    with open(full_path, 'r') as f:
                        content = f.read(100)
                        if "placeholder" in content.lower() or "download" in content.lower():
                            print(f"   → Contains placeholder text: '{content[:50]}...'")
                except:
                    pass
            else:
                print(f"✅ OK: {model_path} (size: {size/1024/1024:.1f}MB)")
    
    return issues

def check_environment():
    """Check environment variables and configuration."""
    print("\n" + "=" * 60)
    print("ENVIRONMENT CONFIGURATION")
    print("=" * 60)
    
    env_vars = {
        "USE_HUGGINGFACE_MODELS": os.environ.get("USE_HUGGINGFACE_MODELS", "not set"),
        "VOICEDIALOGUE_PATH": os.environ.get("VOICEDIALOGUE_PATH", "not set"),
        "PRIMESPEECH_MODEL_DIR": os.environ.get("PRIMESPEECH_MODEL_DIR", "not set"),
        "VOICE_NAME": os.environ.get("VOICE_NAME", "not set"),
    }
    
    issues = []
    
    for var, value in env_vars.items():
        if var == "USE_HUGGINGFACE_MODELS" and value != "true":
            print(f"⚠️  {var}: {value} (should be 'true' to use downloaded models)")
            issues.append(f"Set {var}=true")
        else:
            status = "✅" if value != "not set" else "⚠️ "
            print(f"{status} {var}: {value}")
    
    return issues

def check_imports():
    """Check if MoYoYo TTS can be imported."""
    print("\n" + "=" * 60)
    print("PYTHON MODULE IMPORT CHECK")
    print("=" * 60)
    
    issues = []
    
    # Add PrimeSpeech to path
    primespeech_path = Path("/Users/yuechen/home/arios/dora/node-hub/dora-primespeech")
    sys.path.insert(0, str(primespeech_path))
    
    try:
        # Try to import the TTS wrapper
        from dora_primespeech import moyoyo_tts_wrapper_streaming_fix
        print("✅ Can import moyoyo_tts_wrapper_streaming_fix")
        
        # Check if MoYoYo is available
        if hasattr(moyoyo_tts_wrapper_streaming_fix, 'MOYOYO_AVAILABLE'):
            if moyoyo_tts_wrapper_streaming_fix.MOYOYO_AVAILABLE:
                print("✅ MOYOYO_AVAILABLE = True")
            else:
                print("❌ MOYOYO_AVAILABLE = False (TTS will generate tones!)")
                issues.append("MoYoYo TTS module not available")
        
    except ImportError as e:
        print(f"❌ Cannot import TTS wrapper: {e}")
        issues.append(f"Import error: {e}")
    
    # Check for MoYoYo dependencies
    try:
        import VoiceDialogue
        print("✅ VoiceDialogue module available")
    except ImportError:
        print("⚠️  VoiceDialogue module not found (OK if using HuggingFace models)")
    
    return issues

def suggest_fixes(all_issues):
    """Suggest fixes based on detected issues."""
    print("\n" + "=" * 60)
    print("SUGGESTED FIXES")
    print("=" * 60)
    
    if not all_issues:
        print("✅ No issues detected! TTS should work correctly.")
        return
    
    print("⚠️  Issues detected that will cause tone generation:\n")
    
    fixes = []
    
    # Check for placeholder files
    if any("Placeholder" in issue or "Missing" in issue for issue in all_issues):
        fixes.append("""
1. Download real PrimeSpeech models:
   cd /Users/yuechen/home/arios/dora/examples/model-manager
   python download_models.py --download primespeech-base
   python download_models.py --voice Doubao
        """)
    
    # Check for environment issues
    if any("USE_HUGGINGFACE_MODELS" in issue for issue in all_issues):
        fixes.append("""
2. Enable HuggingFace models in your YAML config:
   Edit examples/voice-chatbot/voice-chat-no-aec.yml
   Set: USE_HUGGINGFACE_MODELS: "true"
        """)
    
    # Check for import issues
    if any("Import error" in issue or "not available" in issue for issue in all_issues):
        fixes.append("""
3. Install missing dependencies:
   cd /Users/yuechen/home/arios/dora/node-hub/dora-primespeech
   pip install -e .
        """)
    
    for fix in fixes:
        print(fix)
    
    print("\n4. Test TTS after fixes:")
    print("   python examples/voice-chatbot/test_primespeech.py")

def main():
    print("\n🔍 DIAGNOSING PRIMESPEECH TTS TONE GENERATION ISSUE\n")
    
    all_issues = []
    
    # Run all checks
    model_issues = check_model_files()
    all_issues.extend(model_issues)
    
    env_issues = check_environment()
    all_issues.extend(env_issues)
    
    import_issues = check_imports()
    all_issues.extend(import_issues)
    
    # Provide diagnosis
    print("\n" + "=" * 60)
    print("DIAGNOSIS SUMMARY")
    print("=" * 60)
    
    if all_issues:
        print("\n❌ TONE GENERATION WILL OCCUR because:")
        for issue in all_issues:
            print(f"   • {issue}")
        
        print("\n📝 Root Cause:")
        print("   The TTS fallback to 440Hz sine wave tone when:")
        print("   1. Model files are missing or placeholders")
        print("   2. MoYoYo TTS module fails to import")
        print("   3. TTS initialization fails")
        
        suggest_fixes(all_issues)
    else:
        print("\n✅ All checks passed! TTS should generate real voice.")
        print("\nIf you still hear tones, check:")
        print("   1. The pipeline is using the correct config file")
        print("   2. No exceptions during runtime (check logs)")
        print("   3. Audio player is receiving correct format")
    
    return 0 if not all_issues else 1

if __name__ == "__main__":
    sys.exit(main())