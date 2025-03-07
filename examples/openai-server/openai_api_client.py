import contextlib

from openai import OpenAI

client = OpenAI(base_url="http://localhost:8000/v1", api_key="dummy_api_key")


def test_list_models() -> None:
    try:
        models = client.models.list()
        for _model in models.data:
            pass
    except Exception:
        pass


def test_chat_completion(user_input) -> None:
    with contextlib.suppress(Exception):
        client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": user_input},
            ],
        )


if __name__ == "__main__":
    test_list_models()

    chat_input = input("Enter a message for chat completion: ")
    test_chat_completion(chat_input)
