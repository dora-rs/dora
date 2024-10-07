from openai import OpenAI
import os

client = OpenAI(base_url="http://localhost:8000/v1", api_key="dummy_api_key")


def test_list_models():
    try:
        models = client.models.list()
        print("Available models:")
        for model in models.data:
            print(f"- {model.id}")
    except Exception as e:
        print(f"Error listing models: {e}")


def test_chat_completion(user_input):
    try:
        response = client.chat.completions.create(
            model="gpt-3.5-turbo",
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
    test_list_models()
    print("\n" + "=" * 50 + "\n")

    chat_input = input("Enter a message for chat completion: ")
    test_chat_completion(chat_input)
    print("\n" + "=" * 50 + "\n")
