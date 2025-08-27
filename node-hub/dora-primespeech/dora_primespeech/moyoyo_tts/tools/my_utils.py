import os
import traceback

# import ffmpeg
import numpy as np
import soundfile as sf
import librosa

from moyoyo_tts.tools.i18n.i18n import I18nAuto

i18n = I18nAuto(language=os.environ.get('language', 'Auto'))

def load_audio(file, sr):
    try:
        file = clean_path(file)  # 防止小白拷路径头尾带了空格和"和回车
        if os.path.exists(file) == False:
            raise RuntimeError(
                "You input a wrong audio path that does not exists, please fix it!"
            )
        
        # 使用soundfile读取音频文件
        data, original_sr = sf.read(file, dtype='float32')
        
        # 如果是多声道，转换为单声道（取平均值）
        if len(data.shape) > 1:
            data = np.mean(data, axis=1)
        
        # 如果采样率不匹配，进行重采样
        if original_sr != sr:
            data = librosa.resample(data, orig_sr=original_sr, target_sr=sr)
        
        return data.flatten()
        
    except Exception as e:
        traceback.print_exc()
        raise RuntimeError(i18n("音频加载失败"))
    
# def load_audio(file, sr):
#     try:
#         # https://github.com/openai/whisper/blob/main/whisper/audio.py#L26
#         # This launches a subprocess to decode audio while down-mixing and resampling as necessary.
#         # Requires the ffmpeg CLI and `ffmpeg-python` package to be installed.
#         file = clean_path(file)  # 防止小白拷路径头尾带了空格和"和回车
#         if os.path.exists(file) == False:
#             raise RuntimeError(
#                 "You input a wrong audio path that does not exists, please fix it!"
#             )
#         out, _ = (
#             ffmpeg.input(file, threads=0)
#             .output("-", format="f32le", acodec="pcm_f32le", ac=1, ar=sr)
#             .run(cmd=["ffmpeg", "-nostdin"], capture_stdout=True, capture_stderr=True)
#         )
#     except Exception as e:
#         traceback.print_exc()
#         raise RuntimeError(i18n("音频加载失败"))

#     return np.frombuffer(out, np.float32).flatten()


def clean_path(path_str: str):
    if path_str.endswith(('\\', '/')):
        return clean_path(path_str[0:-1])
    path_str = path_str.replace('/', os.sep).replace('\\', os.sep)
    return path_str.strip(" ").strip('\'').strip("\n").strip('"').strip(" ").strip("\u202a")
