import json
import time
import os
from openai import AzureOpenAI
from dotenv import load_dotenv 


class GPT_Interface:
    def __init__(self):
        # Constants
        self.TOKEN_LIMIT = 4096
        self.DEPLOYMENT_NAME = "NAO4"
        self.WORDS_TO_TOKENS_MULTIPLIER = 1.3
        self.conversation_context = []
        self.pose_description = None
        self.prompt = None

        load_dotenv()  # loading variables from .env file

        # Client for OpenAI API
        self.client = AzureOpenAI(api_key=os.environ["AZURE_OPENAI_KEY"],
                                  api_version="2024-02-01",
                                  azure_endpoint=os.environ["AZURE_OPENAI_ENDPOINT"])

        # Load pose descriptions
        script_dir = os.path.dirname(os.path.realpath(__file__))
        poses_file_path = os.path.join(script_dir, 'pose_description.json')
        with open(poses_file_path) as pose_file:
            self.pose_description = json.load(pose_file)
        

        # Load prompt text
        description_dir = os.path.dirname(os.path.realpath(__file__))
        description_file_path = os.path.join(description_dir, 'job_description.txt')
        with open(description_file_path) as job_description_txt:
            self.prompt =  job_description_txt.read()

        # Merge prompt text and poses into a single prompt
        self.job_description = self.prompt + json.dumps(self.pose_description)  # TODO check if an empty .json file would work.
        self.conversation_context.append({"role" : "system", "content": self.job_description})


    def get_gpt_response(self, conversation_context):
        """
        Sends the call to the API which interacts with the LLM to get the response from ChatGPT

        :param conversation_context: The entire context of the current conversation with ChatGPT
        :return: The response from the LLM
        """
        start = time.time()
        response = self.client.chat.completions.create(
            model=self.DEPLOYMENT_NAME,
            messages=conversation_context
        )
        end = time.time()
        print(f"{response.model} took {end - start} seconds to respond")
        context = response.choices[0].message.content
        print(f"Nao: {context}")
        print("-" * 60)
        return context


    def estimate_gpt_token_count(self, messages):
        """
        Counts the tokens in the message for GPT in a simplified matter

        :param messages: Message that should be sent to ChatGPT
        :return: The current amount of tokens in the messages
        """
        return sum(len(message['content'].split()) for message in messages) * self.WORDS_TO_TOKENS_MULTIPLIER


    def trim_context(self, context):
        """
        Trims the context to fit in the TOKEN_LIMIT which is set by ChatGPT

        :param context: The entire context of the current conversation with ChatGPT
        :return: The context that was shortened if it exceeded the TOKEN_LIMIT
        """
        while self.estimate_gpt_token_count(context) > self.TOKEN_LIMIT and len(context) > 1:
            del context[1]  # Remove the oldest exchanges to reduce the token count without removing the description.
        return context
    

    def ask_gpt(self, new_message):
        # Prepare the new message in the format expected by OpenAI API
        new_conversation_context = {"role": "user", "content": new_message}
        self.conversation_context.append(new_conversation_context)  # Update the conversation context with the new response
        self.conversation_context = self.trim_context(self.conversation_context)  # shorten teh conversation_context to fit the token limit if neccessary

        response = self.get_gpt_response(self.conversation_context)  # Get the response from GPT
        self.conversation_context.append({"role": "system", "content": response})  # append the reponse from teh system to the conversation context.

        # Return the response
        return response
    

def main():
    # This sectino is only for testing and debugging purposes.
    gpt = GPT_Interface()

    while True:
        message = input('Please input your question:')
        response = gpt.ask_gpt(message)


if __name__ == "__main__":
    main()