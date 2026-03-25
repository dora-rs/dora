"""Sentence Transformers operator for dora-rs dataflow.

This operator uses the Sentence Transformers library to index and search
Python code files within a directory. It allows for semantic search over
the codebase by encoding files and queries into a vector space.
"""

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
    """Recursively discover and read Python files in a directory.

    Only includes files specified in the SHOULD_BE_INCLUDED list.

    Args:
        path (str): The root directory to search.

    Returns:
        tuple: A pair containing (list of file contents, list of file paths).
    """
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
    """Perform a semantic search using cosine similarity.

    Args:
        query_embedding (Tensor): The embedding of the search query.
        corpus_embeddings (Tensor): Executable embeddings of the codebase.
        paths (List[str]): List of file paths corresponding to embeddings.
        raw (List[str]): List of raw file contents.
        k (int): Number of top results to return.
        file_extension (str, optional): Not currently used.

    Returns:
        List: A flattened list of [content, path, score] for the top K matches.
    """
    cos_scores = util.cos_sim(query_embedding, corpus_embeddings)[0]
    top_results = torch.topk(cos_scores, k=min(k, len(cos_scores)), sorted=True)
    out = []
    for score, idx in zip(top_results[0], top_results[1]):
        out.extend([raw[idx], paths[idx], score])
    return out


class Operator:
    """Dora operator for semantic search over Python files."""

    def __init__(self):
        ## TODO: Add a initialisation step
        """Initializes the operator, discovers files, and generates initial embeddings."""
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
        """Handle search queries or file updates.

        If the input is "query", it performs a semantic search. Otherwise,
        it updates the internal index with the provided file content.

        Args:
            dora_event (dict): The event from dora-rs.
            send_output (Callable): Callback to emit search results.

        Returns:
            DoraStatus: CONTINUE to allow further interactions.
        """
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
