import os
import sys

import pyarrow as pa
import torch
from dora import DoraStatus
from sentence_transformers import SentenceTransformer, util

SHOULD_BE_INCLUDED = [
    "webcam.py",
    "object_detection.py",
    "plot.py",
]


## Get all python files path in given directory
def get_all_functions(path):
    raw = []
    paths = []
    for root, dirs, files in os.walk(path):
        for file in files:
            if file.endswith(".py"):
                if file not in SHOULD_BE_INCLUDED:
                    continue
                path = os.path.join(root, file)
                with open(path, encoding="utf8") as f:
                    ## add file folder to system path
                    sys.path.append(root)
                    ## import module from path
                    raw.append(f.read())
                    paths.append(path)

    return raw, paths


def search(query_embedding, corpus_embeddings, paths, raw, k=5, file_extension=None):
    cos_scores = util.cos_sim(query_embedding, corpus_embeddings)[0]
    top_results = torch.topk(cos_scores, k=min(k, len(cos_scores)), sorted=True)
    out = []
    for score, idx in zip(top_results[0], top_results[1]):
        out.extend([raw[idx], paths[idx], score])
    return out


class Operator:
    """ """

    def __init__(self):
        ## TODO: Add a initialisation step
        self.model = SentenceTransformer("BAAI/bge-large-en-v1.5")
        self.encoding = []
        # file directory
        path = os.path.dirname(os.path.abspath(__file__))

        self.raw, self.path = get_all_functions(path)
        # Encode all files
        self.encoding = self.model.encode(self.raw)

    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            if dora_event["id"] == "query":
                values = dora_event["value"].to_pylist()

                query_embeddings = self.model.encode(values)
                output = search(
                    query_embeddings,
                    self.encoding,
                    self.path,
                    self.raw,
                )
                [raw, path, score] = output[0:3]
                send_output(
                    "raw_file",
                    pa.array([{"raw": raw, "path": path, "user_message": values[0]}]),
                    dora_event["metadata"],
                )
            else:
                input = dora_event["value"][0].as_py()
                index = self.path.index(input["path"])
                self.raw[index] = input["raw"]
                self.encoding[index] = self.model.encode([input["raw"]])[0]

        return DoraStatus.CONTINUE


if __name__ == "__main__":
    operator = Operator()
