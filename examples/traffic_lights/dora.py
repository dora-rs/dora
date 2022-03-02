import zenoh

z = zenoh.open()


def register(source, destination=""):
    state = {}

    def decorator(function):
        def wrapper(change):
            info = z.info()["info_pid"]
            result = function(state, change)
            if destination:
                z.put(destination + "/" + info, result)

        return z.subscribe(source + "/*", wrapper)

    return decorator
