import pickle

MAX_SECONDS_LATENCY = 10


def load(inputs, key):
    data_wrapper = pickle.loads(inputs[key])
    (data, timestamps) = data_wrapper
    return data, timestamps


def dump(data, timestamps):
    data_wrapper = (data, timestamps)
    pickled_data = pickle.dumps(data_wrapper)
    return pickled_data
