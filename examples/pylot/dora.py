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

    def decorator(function):
        def wrapper(change):

            key_expr = change.key_expr
            keys = str(key_expr).split("/")
            current_index = keys.index(source)
            residual_key_expr = "/".join(keys[current_index + 1 :])

            result = function(state, change)

            z.put(residual_key_expr, result)

        return z.subscribe("**/" + source + "/**", wrapper)

    return decorator


def register_sink(source):
    state = {}

    def decorator(function):
        def wrapper(change):
            result = function(state, change)
            return result

        return z.subscribe(source + "/**", wrapper)

    return decorator


def fuse(src_left, src_right):
    def decorator(function):
        while True:
            replies_left = z.get(
                src_left, target=QueryTarget(queryable.ALL_KINDS, Target.All())
            )
            replies_right = z.get(
                src_right,
                target=QueryTarget(queryable.ALL_KINDS, Target.All()),
            )

            time.sleep(1)

    return decorator
