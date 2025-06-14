import os

# This will get the value of the environment variable named 'GEMINI_API_KEY'
gemini_api_key = os.getenv("GEMINI_API_KEY")

if gemini_api_key is None:
    print("Error: GEMINI_API_KEY environment variable not set.")
else:
    # Use your API key here
    print("API Key loaded successfully!")
    # For example, if using the Google Generative AI library:
    # import google.generativeai as genai
    # genai.configure(api_key=gemini_api_key)