import asyncio

counter = 0


async def return_1(x):
    global counter
    counter += 1
    print(counter)
    print(x)
    return {"b": "b", "c": "c"}
