import asyncio

counter = 0

import logging


def return_1(x):
    global counter
    counter += 1
    print(counter)
    logging.info(x)
    return {"b": "b", "c": "c"}
