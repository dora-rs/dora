"""
Configuration for PrimeSpeech TTS node.
"""

import os
from pathlib import Path
from typing import Optional, Dict, Any
from dataclasses import dataclass, field


@dataclass
class PrimeSpeechConfig:
    """PrimeSpeech configuration class"""
    
    # Model selection
    VOICE_NAME = os.getenv("VOICE_NAME", "Doubao")  # Voice character name
    REPOSITORY = os.getenv("REPOSITORY", "MoYoYoTech/tone-models")  # HuggingFace repo
    
    # Model paths
    MODEL_DIR = os.getenv("PRIMESPEECH_MODEL_DIR", "")  # Override model directory
    GPT_MODEL_PATH = os.getenv("GPT_MODEL_PATH", "")  # Override GPT model path
    SOVITS_MODEL_PATH = os.getenv("SOVITS_MODEL_PATH", "")  # Override SoVITS model path
    REFERENCE_AUDIO_PATH = os.getenv("REFERENCE_AUDIO_PATH", "")  # Override reference audio
    
    # Language settings
    TEXT_LANG = os.getenv("TEXT_LANG", "auto")  # auto, zh, en, ja
    PROMPT_LANG = os.getenv("PROMPT_LANG", "auto")  # auto, zh, en, ja
    PROMPT_TEXT = os.getenv("PROMPT_TEXT", "")  # Optional custom prompt text
    
    # Inference parameters (optimized for streaming)
    TOP_K = int(os.getenv("TOP_K", "3"))  # Lower for faster inference
    TOP_P = float(os.getenv("TOP_P", "0.95"))
    TEMPERATURE = float(os.getenv("TEMPERATURE", "0.8"))  # Lower for consistency
    SPEED_FACTOR = float(os.getenv("SPEED_FACTOR", "1.0"))
    BATCH_SIZE = int(os.getenv("BATCH_SIZE", "100"))
    SEED = int(os.getenv("SEED", "233333"))
    
    # Processing options
    TEXT_SPLIT_METHOD = os.getenv("TEXT_SPLIT_METHOD", "cut5")  # cut0, cut1, cut2, cut3, cut4, cut5
    SPLIT_BUCKET = os.getenv("SPLIT_BUCKET", "true").lower() == "true"
    RETURN_FRAGMENT = os.getenv("RETURN_FRAGMENT", "false").lower() == "true"  # Respect env variable
    FRAGMENT_INTERVAL = float(os.getenv("FRAGMENT_INTERVAL", "0.3"))
    
    # Performance
    USE_GPU = os.getenv("USE_GPU", "false").lower() == "true"
    DEVICE = os.getenv("DEVICE", "cuda" if USE_GPU else "cpu")
    NUM_THREADS = int(os.getenv("NUM_THREADS", "4"))
    
    # Audio settings
    SAMPLE_RATE = int(os.getenv("SAMPLE_RATE", "32000"))
    
    # Logging
    LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO").upper()  # DEBUG, INFO, WARNING, ERROR
    
    @staticmethod
    def get_models_dir() -> Path:
        """Get models directory path"""
        # Check environment variable first
        models_dir = os.getenv("PRIMESPEECH_MODEL_DIR")
        if models_dir:
            return Path(models_dir)
        
        # Default to home directory
        home = Path.home()
        default_dir = home / ".dora" / "models" / "primespeech"
        default_dir.mkdir(parents=True, exist_ok=True)
        return default_dir


# Voice configurations with default prompts and settings
VOICE_CONFIGS = {
    "Doubao": {
        "repository": "MoYoYoTech/tone-models",
        "gpt_weights": "GPT_weights/doubao_best_gpt.ckpt",
        "sovits_weights": "SoVITS_weights/doubao_best_sovits.pth",
        "reference_audio": "ref_audios/doubao_ref.wav",
        "prompt_text": "我叫豆包呀，能陪你聊天解闷，不管是聊生活趣事，知识科普还是帮你出主意，我都在行哦。",
        "text_lang": "zh",
        "prompt_lang": "zh",
        "speed_factor": 1.1,
    },
    "Luo Xiang": {
        "repository": "MoYoYoTech/tone-models",
        "gpt_weights": "GPT_weights/luoxiang_best_gpt.ckpt",
        "sovits_weights": "SoVITS_weights/luoxiang_best_sovits.pth",
        "reference_audio": "ref_audios/luoxiang_ref.wav",
        "prompt_text": "复杂的问题背后也许没有统一的答案，选择站在正方还是反方，其实取决于你对一系列价值判断的回答。",
        "text_lang": "zh",
        "prompt_lang": "zh",
        "speed_factor": 1.1,
    },
    "Yang Mi": {
        "repository": "MoYoYoTech/tone-models",
        "gpt_weights": "GPT_weights/yangmi_best_gpt.ckpt",
        "sovits_weights": "SoVITS_weights/yangmi_best_sovits.pth",
        "reference_audio": "ref_audios/yangmi_ref.wav",
        "prompt_text": "你谁知道, 人生只有一次啊. 你怎么知道那样选, 你当下来说, 应该那样选.",
        "text_lang": "zh",
        "prompt_lang": "zh",
        "speed_factor": 1.1,
    },
    "Zhou Jielun": {
        "repository": "MoYoYoTech/tone-models",
        "gpt_weights": "GPT_weights/zhoujielun_best_gpt.ckpt",
        "sovits_weights": "SoVITS_weights/zhoujielun_best_sovits.pth",
        "reference_audio": "ref_audios/zhoujielun_ref.wav",
        "prompt_text": "其实我我现在讲的这些奥，都是我未来成功的一些关键。",
        "text_lang": "zh",
        "prompt_lang": "zh",
        "speed_factor": 1.1,
    },
    "Ma Yun": {
        "repository": "MoYoYoTech/tone-models",
        "gpt_weights": "GPT_weights/mayun_best_gpt.ckpt",
        "sovits_weights": "SoVITS_weights/mayun_best_sovits.pth",
        "reference_audio": "ref_audios/mayun_ref.wav",
        "prompt_text": "这是我们最大的希望能招聘的到人。所以今天阿里巴巴公司内部，我自己这么觉得，人才梯队的建设非常之好。",
        "text_lang": "zh",
        "prompt_lang": "zh",
        "speed_factor": 1.1,
    },
    "Maple": {
        "repository": "MoYoYoTech/tone-models",
        "gpt_weights": "GPT_weights/maple_best_gpt.ckpt",
        "sovits_weights": "SoVITS_weights/maple_best_sovits.pth",
        "reference_audio": "ref_audios/maple_ref.wav",
        "prompt_text": "There was a little tea shop in a bustling village. Every morning, the owner, an elderly woman, would wake up early.",
        "text_lang": "en",
        "prompt_lang": "en",
        "speed_factor": 1.0,
    },
    "Cove": {
        "repository": "MoYoYoTech/tone-models",
        "gpt_weights": "GPT_weights/cove_best_gpt.ckpt",
        "sovits_weights": "SoVITS_weights/cove_best_sovits.pth",
        "reference_audio": "ref_audios/cove_ref.wav",
        "prompt_text": "and he has a long career in the Senate representing Delaware. So both have had significant impacts on American politics and policies.",
        "text_lang": "en",
        "prompt_lang": "en",
        "speed_factor": 1.0,
    },
    "BYS": {
        "repository": "MoYoYoTech/tone-models",
        "gpt_weights": "GPT_weights/bys_best_gpt.ckpt",
        "sovits_weights": "SoVITS_weights/bys_best_sovits.pth",
        "reference_audio": "ref_audios/bys_ref.wav",
        "prompt_text": "今天天气不错，适合出去走走。",
        "text_lang": "zh",
        "prompt_lang": "zh",
        "speed_factor": 1.1,
    },
    "Ellen": {
        "repository": "MoYoYoTech/tone-models",
        "gpt_weights": "GPT_weights/ellen_best_gpt.ckpt",
        "sovits_weights": "SoVITS_weights/ellen_best_sovits.pth",
        "reference_audio": "ref_audios/ellen_ref.wav",
        "prompt_text": "Welcome to the show! Today we have some amazing guests.",
        "text_lang": "en",
        "prompt_lang": "en",
        "speed_factor": 1.1,
    },
    "Juniper": {
        "repository": "MoYoYoTech/tone-models",
        "gpt_weights": "GPT_weights/juniper_best_gpt.ckpt",
        "sovits_weights": "SoVITS_weights/juniper_best_sovits.pth",
        "reference_audio": "ref_audios/juniper_ref.wav",
        "prompt_text": "The forest was quiet, with only the sound of leaves rustling in the wind.",
        "text_lang": "en",
        "prompt_lang": "en",
        "speed_factor": 1.1,
    },
    "Ma Baoguo": {
        "repository": "MoYoYoTech/tone-models",
        "gpt_weights": "GPT_weights/mabaoguo_best_gpt.ckpt",
        "sovits_weights": "SoVITS_weights/mabaoguo_best_sovits.pth",
        "reference_audio": "ref_audios/mabaoguo_ref.wav",
        "prompt_text": "年轻人不讲武德，偷袭我这个六十九岁的老同志。",
        "text_lang": "zh",
        "prompt_lang": "zh",
        "speed_factor": 1.1,
    },
    "Shen Yi": {
        "repository": "MoYoYoTech/tone-models",
        "gpt_weights": "GPT_weights/shenyi_best_gpt.ckpt",
        "sovits_weights": "SoVITS_weights/shenyi_best_sovits.pth",
        "reference_audio": "ref_audios/shenyi_ref.wav",
        "prompt_text": "今天我们来分析一下这个案例的关键点。",
        "text_lang": "zh",
        "prompt_lang": "zh",
        "speed_factor": 1.1,
    },
    "Trump": {
        "repository": "MoYoYoTech/tone-models",
        "gpt_weights": "GPT_weights/trump_best_gpt.ckpt",
        "sovits_weights": "SoVITS_weights/trump_best_sovits.pth",
        "reference_audio": "ref_audios/trump_ref.wav",
        "prompt_text": "We're going to make America great again, and it's going to be tremendous.",
        "text_lang": "en",
        "prompt_lang": "en",
        "speed_factor": 1.1,
    },
}