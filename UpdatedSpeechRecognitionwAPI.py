import speech_recognition as sr
import os
import json 
import requests 
from PranavFile import send_command_to_jetson # Assuming this is in the same directory 
import pyttsx3
import asyncio 
import sys 

engine = pyttsx3.init() # object creation

# --- Configuration ---
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY", "YOUR_GEMINI_API_KEY") # Replace with your actual key if not using env var
JETSON_IP = "192.168.10.245" # Your Jetson's IP address
JETSON_PORT = 5005          # Your Jetson's listening port

DEFAULT_SPEED = 50 # Speed percentage (0-100)
DEFAULT_DURATION = 2 # Default duration in seconds if not specified for a timed move

# Servo angle definitions
SERVO_MAX_LEFT = -50
SERVO_SHARP_LEFT = -45
SERVO_LEFT = -25
SERVO_SLIGHT_LEFT = -10
SERVO_STRAIGHT = 0.0
SERVO_SLIGHT_RIGHT = 10
SERVO_RIGHT = 25
SERVO_SHARP_RIGHT = 45
SERVO_MAX_RIGHT = 50


if GEMINI_API_KEY == "YOUR_GEMINI_API_KEY":
    print("-------------------------------------------------------------------------")
    print("WARNING: Gemini API Key not found or not set.")
    print("Please set the GEMINI_API_KEY environment variable or replace")
    print("'YOUR_GEMINI_API_KEY' in the script with your actual API key.")
    print("The script will not function correctly without a valid API key.")
    print("-------------------------------------------------------------------------")
    # exit()

# --- Car Control Functions ---

def control_servo(angle): # For stationary turns
    """Controls the car's servo motor to a given angle for stationary turns."""
    angle = max(SERVO_MAX_LEFT, min(SERVO_MAX_RIGHT, angle))
    direction_text = "straight"
    if angle < -1: 
        direction_text = f"left to {abs(angle)} degrees"
    elif angle > 1: 
        direction_text = f"right to {angle} degrees"
    else: 
        direction_text = "straight"

    print(f"Car: Setting servo (stationary turn) to {angle} degrees ({direction_text})...")
    # engine.say(f"Steering {direction_text}.") # TTS for steering removed as requested
    
    command_string = f"1,0.3,2,{angle}" 
    send_command_to_jetson(command_string, JETSON_IP, JETSON_PORT) 
    return 1

def move_forward(speed=DEFAULT_SPEED, duration=DEFAULT_DURATION, angle=SERVO_STRAIGHT):
    """Controls the car to move forward at a given speed, duration, and angle."""
    print(f"Car: Moving Forward at {speed}% speed for {duration}s. Angle: {angle} degrees.")
    engine.say(f"Moving Forward at {speed} percent speed for {duration} seconds.")
    engine.runAndWait() 
    
    speed = speed / 100  # Convert percentage to a fraction
    
    command_string = f"1,{speed},{duration},{angle}"
    send_command_to_jetson(command_string, JETSON_IP, JETSON_PORT) 
    
    if angle == SERVO_STRAIGHT:
        print("Car: Wheels intended straight during forward movement.")
    else:
        print(f"Car: Steering to {angle} degrees during forward movement.")
    return 1

def move_backward(speed=DEFAULT_SPEED, duration=DEFAULT_DURATION, angle=SERVO_STRAIGHT):
    """Controls the car to move backward at a given speed, duration, and angle."""
    print(f"Car: Moving Backward at {speed}% speed for {duration}s. Angle: {angle} degrees.")
    engine.say(f"Moving Backward at {speed} percent speed for {duration} seconds.")
    engine.runAndWait()
    
    speed = speed / 100
    command_string = f"-1,{speed},{duration},{angle}"
    send_command_to_jetson(command_string, JETSON_IP, JETSON_PORT)

    if angle == SERVO_STRAIGHT:
        print("Car: Wheels intended straight during backward movement.")
    else:
        print(f"Car: Steering to {angle} degrees during backward movement.")
    return 0

def unknown_command(user_text=None):
    """Handles unrecognized commands."""
    message = "Car: Command not understood."
    if user_text:
        message += f" (You said: '{user_text}')"
    print(message)
    engine.say("Sorry, I didn't understand that.")
    engine.runAndWait()
    return -1

def exit_program():
    """Handles the program exit sequence."""
    print("Car: Exit command received. Shutting down...")
    engine.say("Goodbye! Shutting down.")
    engine.runAndWait()
    # command_string = f"stop,0,{DEFAULT_DURATION},0" # Example stop command format
    # send_command_to_jetson(command_string, (JETSON_IP, JETSON_PORT))
    sys.exit() 

# --- Speech Recognition Function ---
def listen_for_command():
    """Captures audio from the microphone and returns the recognized text."""
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    with microphone as source:
        print("\nCalibrating microphone for ambient noise (0.5s)...")
        recognizer.adjust_for_ambient_noise(source, duration=0.5)
        print("Listening for command...")
        try:
            audio = recognizer.listen(source, timeout=5, phrase_time_limit=10)
        except sr.WaitTimeoutError:
            print("No speech detected within the time limit.")
            return None

    try:
        print("Recognizing speech...")
        text = recognizer.recognize_google(audio)
        print(f"You said: {text}")
        return text
    except sr.UnknownValueError:
        print("Speech Recognition could not understand audio.")
        return None
    except sr.RequestError as e:
        print(f"Could not request results from Speech Recognition service; {e}")
        return None
    except Exception as e:
        print(f"An unexpected error occurred during speech recognition: {e}")
        return None

# --- Gemini LLM Interaction ---
async def get_command_from_gemini(user_text):
    """
    Sends text to Gemini to determine the car command and parameters.
    Returns a dictionary with the parsed command.
    """
    if not user_text:
        return {"action": "unknown", "parameters": {}}
    if not GEMINI_API_KEY or GEMINI_API_KEY == "YOUR_GEMINI_API_KEY":
        print("Error: Gemini API Key is not configured. Cannot process command.")
        return {"action": "unknown", "parameters": {}}

    prompt = f"""
    You are an intelligent assistant for a voice-controlled car.
    Analyze the user's command and convert it into a structured JSON object.
    The car can perform actions like 'move' (forward/backward, including speed, duration, and steering angle), 
    'turn' (for stationary steering), and 'exit' (stop the program).

    JSON Output Schema:
    {{
      "action": "move" | "turn" | "exit" | "unknown",
      "parameters": {{
        "direction": "forward" | "backward" | null,
        "speed": <number_0_to_100> | null,     // For 'move'. Default: {DEFAULT_SPEED}. Slow: 25, Fast: 75.
        "duration": <number_seconds> | null,   // For 'move'. Default: {DEFAULT_DURATION} if not specified. 
                                               // Interpret "a little bit" or "shortly" as {DEFAULT_DURATION}s if no other duration is given.
                                               // Interpret "a little while" as {DEFAULT_DURATION + 2}s, "for a while" as {DEFAULT_DURATION + 5}s.
        "angle": <number_neg50_to_pos50> | null // For 'move' and 'turn'. Default to {SERVO_STRAIGHT} for 'move' if not specified.
      }}
    }}

    Servo Angle Mapping:
    - "straighten out", "straight": {SERVO_STRAIGHT}
    - "slightly left": {SERVO_SLIGHT_LEFT}, "left": {SERVO_LEFT}, "sharp left": {SERVO_SHARP_LEFT}, "max left": {SERVO_MAX_LEFT}
    - "slightly right": {SERVO_SLIGHT_RIGHT}, "right": {SERVO_RIGHT}, "sharp right": {SERVO_SHARP_RIGHT}, "max right": {SERVO_MAX_RIGHT}

    Examples:
    - User: "move forward" -> {{"action": "move", "parameters": {{"direction": "forward", "speed": {DEFAULT_SPEED}, "duration": {DEFAULT_DURATION}, "angle": {SERVO_STRAIGHT}}}}}
    - User: "go forward for 5 seconds" -> {{"action": "move", "parameters": {{"direction": "forward", "speed": {DEFAULT_SPEED}, "duration": 5, "angle": {SERVO_STRAIGHT}}}}}
    - User: "drive backward slowly for 3 secs turning left" -> {{"action": "move", "parameters": {{"direction": "backward", "speed": 25, "duration": 3, "angle": {SERVO_LEFT}}}}}
    - User: "turn sharp right" (implies stationary turn) -> {{"action": "turn", "parameters": {{"direction": null, "speed": null, "duration": null, "angle": {SERVO_SHARP_RIGHT}}}}}
    - User: "move forward a little bit turning slightly right" -> {{"action": "move", "parameters": {{"direction": "forward", "speed": {DEFAULT_SPEED}, "duration": {DEFAULT_DURATION}, "angle": {SERVO_SLIGHT_RIGHT}}}}}
    - User: "stop program", "exit" -> {{"action": "exit", "parameters": {{}}}}

    User command: "{user_text}"
    JSON Response:
    """

    api_url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash-latest:generateContent?key={GEMINI_API_KEY}"
    
    payload = {
        "contents": [{"role": "user", "parts": [{"text": prompt}]}],
        "generationConfig": {
            "responseMimeType": "application/json",
        }
    }

    try:
        print("Sending command to Gemini for interpretation...")
        response = requests.post(api_url, json=payload, headers={'Content-Type': 'application/json'})
        response.raise_for_status()
        
        gemini_response_text = response.json()["candidates"][0]["content"]["parts"][0]["text"]
        print(f"Gemini raw JSON response: {gemini_response_text}")
        
        parsed_json = json.loads(gemini_response_text)
        print(f"Gemini parsed command: {parsed_json}")
        return parsed_json

    except requests.exceptions.RequestException as e:
        print(f"Error communicating with Gemini API: {e}") # ... (error handling)
        if hasattr(e, 'response') and e.response is not None:
            try:
                print(f"Gemini API Error Response: {e.response.json()}")
            except json.JSONDecodeError:
                print(f"Gemini API Error Response (not JSON): {e.response.text}")
        return {"action": "unknown", "parameters": {}}
    except (json.JSONDecodeError, KeyError, IndexError) as e: # ... (error handling)
        print(f"Error parsing Gemini JSON response: {e}")
        print(f"Received text: {gemini_response_text if 'gemini_response_text' in locals() else 'N/A'}")
        return {"action": "unknown", "parameters": {}}
    except Exception as e: # ... (error handling)
        print(f"An unexpected error occurred while processing Gemini response: {e}")
        return {"action": "unknown", "parameters": {}}

# --- Main Loop ---
async def main():
    """Main function to run the speech-controlled car."""
    print("Starting Advanced Speech Controlled Car...")
    print(f"Default speed: {DEFAULT_SPEED}%, Default duration: {DEFAULT_DURATION}s. Servo range: {SERVO_MAX_LEFT} to {SERVO_MAX_RIGHT} degrees.")
    print("Say 'exit' or 'stop' to quit.")
    print("Press Ctrl+C to exit.")

    try:
        while True:
            user_command_text = listen_for_command()

            if user_command_text:
                command_data = await get_command_from_gemini(user_command_text)
                
                action = command_data.get("action", "unknown")
                parameters = command_data.get("parameters", {})

                if action == "move":
                    direction = parameters.get("direction")
                    speed = int(parameters.get("speed", DEFAULT_SPEED) or DEFAULT_SPEED) 
                    duration = int(parameters.get("duration", DEFAULT_DURATION) or DEFAULT_DURATION) # Extract duration
                    angle = int(parameters.get("angle", SERVO_STRAIGHT) or SERVO_STRAIGHT) 
                    
                    if direction == "forward":
                        move_forward(speed, duration, angle) # Pass duration
                    elif direction == "backward":
                        move_backward(speed, duration, angle) # Pass duration
                    else: 
                        print(f"Move command with unclear direction: {direction}. If angle specified, attempting stationary turn.")
                        if parameters.get("angle") is not None: 
                             control_servo(angle) 
                        else:
                             unknown_command(user_command_text)
                
                elif action == "turn": 
                    angle_param = parameters.get("angle")
                    if angle_param is not None:
                        control_servo(int(angle_param)) 
                    else:
                        unknown_command(user_command_text)
                
                elif action == "exit":
                    exit_program()

                elif action == "unknown":
                    unknown_command(user_command_text)
                
                else: 
                    unknown_command(user_command_text)
            else:
                pass 

    except KeyboardInterrupt:
        print("\nExiting Speech Controlled Car program (Ctrl+C).")
    except SystemExit:
        print("Program terminated successfully.")
    except Exception as e:
        print(f"An unexpected error occurred in the main loop: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    import asyncio
    try:
        asyncio.run(main())
    except RuntimeError as e:
        if " asyncio.run() cannot be called from a running event loop" in str(e):
            print("Detected a running event loop. Attempting to use existing loop.")
            loop = asyncio.get_event_loop()
            if loop.is_running():
                loop.create_task(main())
            else:
                print("Event loop not running, but asyncio.run() failed. Please check your environment.")
        else:
            raise
