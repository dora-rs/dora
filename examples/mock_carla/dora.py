import json
import time

import zenoh
from zenoh import QueryTarget, Target, config, queryable

ZENOH_HOST = ["tcp/192.168.1.15:7447"]
config = zenoh.Config()
config.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(ZENOH_HOST))
z = zenoh.open(config)


def register(source):
    state = {}
    messages = {}

    def decorator(function):
        def wrapper(change):

            key_expr = change.key_expr
            keys = str(key_expr).split("/")
            *key, input, output = keys[1].split("%")
            residual_key_expr = "/".join(keys[1:])
            messages[input] = change.value.decode()
            result = function(state, messages)

            z.put(residual_key_expr, result[output])

        return z.subscribe("*/" + source + "*/**", wrapper)

    return decorator


def register_sink(source):
    state = {}

    def decorator(function):
        def wrapper(change):
            result = function(state, change)
            return result

        return z.subscribe("*/" + source + "**", wrapper)

    return decorator
