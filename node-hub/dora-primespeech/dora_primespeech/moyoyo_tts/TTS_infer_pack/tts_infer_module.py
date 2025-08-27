import os
import random
import typing
from pathlib import Path

from .TTS import TTS, TTS_Config

# from pydub import AudioSegment
# from pydub.playback import play
# from setting import settings
current_dir = Path(__file__).parent
parent_dir = current_dir.parent


class TTSModule:
    def __init__(self, tts_config: TTS_Config):
        self.tts_pipeline = TTS(tts_config)

    def setup_inference_params(
            self,
            text_lang: typing.Literal["en", "all_zh", "all_ja"] = 'en',
            ref_audio: str = '',
            prompt_text: str = '',
            prompt_lang: str = '',
            top_k: int = 60,
            top_p: float = 1.0,
            temperature: float = 1.0,
            text_split_method: typing.Literal['cut0', 'cut1', 'cut2', 'cut3', 'cut4', 'cut5'] = 'cut5',
            batch_size: int = 100,
            speed_factor: float = 1.1,
            split_bucket: bool = True,
            return_fragment: bool = False,
            fragment_interval: float = 0.1,
            seed: int = 233333,
            parallel_infer: bool = True,
    ):
        """
        Set up inference parameters for the model.

        :param text_lang: Language of the input text. Choose from "en" (English), "all_zh" (Chinese), or "all_ja" (Japanese).
        :param ref_audio: Path to the reference audio file. This audio is used as a voice reference for the output.
        :param prompt_text: Text of the reference speech. This can be used to guide the model's output style or content.
        :param prompt_lang: Language of the reference speech. Should correspond to the language of prompt_text.
        :param top_k: Top-K sampling parameter for GPT. Limits the next token selection to the K most likely tokens.
        :param top_p: Top-P (nucleus) sampling parameter for GPT. Sets a cumulative probability cutoff for token selection.
        :param temperature: Temperature for GPT sampling. Higher values increase randomness, lower values make output more deterministic.
        :param text_split_method: Method to split the input text:
            - "cut0": No splitting (process whole text at once)
            - "cut1": Split every 4 sentences
            - "cut2": Split every 50 words
            - "cut3": Split at Chinese full stops '。'
            - "cut4": Split at English periods '.'
            - "cut5": Automatic splitting (recommended)
        :param batch_size: Batch size for inference. Larger values may speed up processing but require more memory.
        :param speed_factor: Factor to control the speed of the output audio. Values > 1 speed up, < 1 slow down.
        :param split_bucket: Whether to use bucket splitting for more efficient processing. Recommended to keep on.
        :param return_fragment: If True, return individual fragments of the generated audio.
        :param fragment_interval: Time interval (in seconds) between each sentence or fragment in the output audio.
        :param seed: Random seed for reproducibility. Set a fixed value for consistent results across runs.
        :param parallel_infer: whether to use parallel inference.
        """
        self.text_lang = text_lang
        self.ref_audio = str(ref_audio)
        self.prompt_text = prompt_text
        self.prompt_lang = prompt_lang
        self.top_k = max(1, int(top_k))  # 确保top_k至少为1
        self.top_p = max(0.0, min(1.0, float(top_p)))  # 确保top_p在0到1之间
        self.temperature = max(0.0, float(temperature))  # 确保temperature非负
        self.text_split_method = text_split_method
        self.batch_size = max(1, int(batch_size))  # 确保batch_size至少为1
        self.speed_factor = max(0.1, float(speed_factor))  # 确保speed_factor至少为0.1
        self.split_bucket = bool(split_bucket)
        self.return_fragment = bool(return_fragment)
        self.fragment_interval = max(0.0, float(fragment_interval))  # 确保fragment_interval非负
        self.seed = int(seed)
        self.parallel_infer = parallel_infer

        # Validate that the reference audio file exists
        if self.ref_audio and not os.path.exists(self.ref_audio):
            raise FileNotFoundError(f"Reference audio file not found: {self.ref_audio}")

    def inference(self, text, text_lang, ref_audio_path, prompt_text, prompt_lang, top_k,
                  top_p, temperature, text_split_method, batch_size, speed_factor,
                  ref_text_free, split_bucket, fragment_interval, seed, parallel_infer: bool):
        actual_seed = seed if seed not in [-1, "", None] else random.randrange(1 << 32)
        inputs = {
            "text": text,
            "text_lang": text_lang,
            "ref_audio_path": ref_audio_path,
            "prompt_text": prompt_text if not ref_text_free else "",
            "prompt_lang": prompt_lang,
            "top_k": top_k,
            "top_p": top_p,
            "temperature": temperature,
            "text_split_method": text_split_method,
            "batch_size": int(batch_size),
            "speed_factor": float(speed_factor),
            "split_bucket": split_bucket,
            "return_fragment": False,
            "fragment_interval": fragment_interval,
            "seed": actual_seed,
            "parallel_infer": parallel_infer
        }
        # print(inputs)
        for item in self.tts_pipeline.run(inputs):
            yield item, actual_seed

    def generate_audio(self, text, warmup: bool = False):

        [output] = self.inference(
            text,
            self.text_lang,
            self.ref_audio,
            self.prompt_text,
            self.prompt_lang,
            self.top_k,
            self.top_p,
            self.temperature,
            self.text_split_method,
            self.batch_size,
            self.speed_factor,
            self.return_fragment,
            self.split_bucket,
            self.fragment_interval,
            self.seed,
            self.parallel_infer,
        )

        if warmup:
            return None

        return output

        # solution v1
        # print(f"Audio generated path : '{parent_dir}'")
        # # output_folder = "./output"  # 指定输出目录
        # output_folder = str(settings.AUDIO_OUTPUT_FOLDER)
        # if not os.path.exists(output_folder):
        #     os.makedirs(output_folder)  # 如果目录不存在，则创建它
        # now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S%f")  # 获取当前时间的时间戳
        # file_name = os.path.join(output_folder, f'output_{now}_{count}.wav')
        # sf.write(file_name, output[0][1], samplerate=output[0][0], subtype='PCM_16')
        #
        # print(f"Audio file saved to: {file_name}")
        # return file_name

# 使用示例
# tts_module = TTSModule("./moyoyo_tts/configs/tts_infer.yaml")
# text = "Well, you know what the say, Families are like fudge, mostly sweet, but sometimes nuts. My family is doing great, thanks for asking! My son is growing up to be a smart and handsome young man, just like his mom. He's currently working on his own talker show, which I'm sure will be even more hilarious than mine."
# tts_module.generate_audio(text)
