"""TODO: Add docstring."""
from openai import OpenAI
import httpx

transport = httpx.HTTPTransport(proxy=None) 
http_client = httpx.Client(transport=transport)
client = OpenAI(base_url="http://127.0.0.1:8118/v1", api_key="dummy_api_key", http_client=http_client)

def test_list_models():
    """TODO: Add docstring."""
    try:
        models = client.models.list()
        print("Available models:")
        for model in models.data:
            print(f"- {model.id}")
    except Exception as e:
        print(f"Error listing models: {e}")


def test_chat_completion(user_input):
    """TODO: Add docstring."""
    try:
        response = client.chat.completions.create(
            model="kimi-latest",
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": user_input},
            ],
        )
        print("Chat Completion Response:")
        print(response.choices[0].message.content)
    except Exception as e:
        print(f"Error in chat completion: {e}")


if __name__ == "__main__":
    print("Testing API endpoints...")
    # test_list_models()
    print("\n" + "=" * 50 + "\n")

    while True:
        chat_input = input("Enter a message for chat completion: ")
        test_chat_completion(chat_input)

        print("\n" + "=" * 50 + "\n")

    # test_chat_completion_image_url(chat_input)
    # print("\n" + "=" * 50 + "\n")
    # test_chat_completion_image_base64(chat_input)
    # print("\n" + "=" * 50 + "\n")
