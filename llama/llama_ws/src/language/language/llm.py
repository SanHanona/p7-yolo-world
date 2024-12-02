# torchrun src/language/language/llm.py --ckpt_dir '/llama/checkpoints/Llama3.2-3B'

from typing import Optional
import fire
import sys
sys.path.append('/llama/llama3')
from reference_impl.generation import Llama
from termcolor import cprint
import subprocess
import re
import yaml
import subprocess

def run_main(
    ckpt_dir: str,
    temperature: float = 0.6,
    top_p: float = 0.9,
    max_seq_len: int = 512,
    max_batch_size: int = 4,
    max_gen_len: int = 64,
    model_parallel_size: Optional[int] = None,
):
    generator = Llama.build(
        ckpt_dir=ckpt_dir,
        max_seq_len=max_seq_len,
        max_batch_size=max_batch_size,
        model_parallel_size=model_parallel_size,
    )

    available_functions = ['wait(t)', 'find_new_way()', 'pass()', 'stop()']
    
    # Start interactive loop
    print("Interactive mode. Type 'exit' to quit.")
    while True:
        print("start")
        # Read the input (you can use the remote file reading function if needed)
        input = listen_to_topic("/speech")  # Alternatively, use input() for direct user input
        
        if input != "":
            # The prompt includes the context and expected function outputs
            prompts = [
                """You are a robot. You can only respond using the following functions. Call for <|end_of_text|> after giving your output.
                Functions:""" + ', '.join(available_functions) + """
                Example 1:
                    Input: 'Go on' 
                    Output: 'pass()'
                    <|end_of_text|>
                    
                Example 2:
                    Input: 'no, you cannot go this way' 
                    Output: 'find_new_way()'
                    <|end_of_text|>
                    
                Example 3:
                    Input: 'hi robot, can you wait a few minutes please?' 
                    Output: 'wait(300)'
                    <|end_of_text|>
                
                Task:
                Input: '""" + input + """'
                Output:"""
            ]

            # Process the prompt and generate response
            for prompt in prompts:
                result = generator.text_completion(
                    prompt,
                    temperature=temperature,
                    top_p=top_p,
                    max_gen_len=max_gen_len,
                    logprobs=False,
                )

                # Print prompt and result
                cprint(f"{prompt}", end="")
                cprint(f"{result.generation}", color="yellow")
                
                # Process the output (extract the function call)
                output = ""
                for x in result.generation:
                    if x == '<':
                        break
                    if x == ' ' or x == '\n' or x == "'":
                        continue
                    output += x

                print("\n==================================\n\n\n")
                print("--> INPUT : " + input + "\n")
                print("--> OUTPUT : " + output + "\n")
                
                # Validate the output
                if (output in available_functions) or (output.startswith('wait(') and output.endswith(')')):
                    print("Valid output received.")
                    publish_to_topic("/lang_command", "std_msgs/String", f"data: {output}")
                    #input = ""
                else:
                    print("Invalid output, please try again.")
                    #input = ""
                    continue  # Continue the loop if output is not valid

def listen_to_topic(topic_name):
    command = ["ros2", "topic", "echo", topic_name]
    
    try:
        # Run the command and stream the output
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print(f"Listening to topic: {topic_name}")
        
        # Continuously read messages from the topic
        for line in process.stdout:
            print(f"Received message: {line.strip()}")
            if line.strip().startswith('data'):
                # Remove the 'data: ' prefix and return the actual message
                message = line.strip().split("data: ", 1)[1].strip()
                cleaned_message = re.sub(r"^.*?:\s*", "", message)  # Removing the prefix part
                return cleaned_message
        return ""

    except KeyboardInterrupt:
        print("\nTerminating...")
        process.terminate()
    except Exception as e:
        print(f"Error: {e}")
        process.terminate()

    return ""

def publish_to_topic(topic_name, msg_type, message):
    print("publish_to_topic")
    message = message.strip().split("data: ", 1)[1].strip()
    # Ensure the message is properly formatted as YAML
    yaml_message = yaml.dump({"data": message}, default_flow_style=True)

    command = ["ros2", "topic", "pub", topic_name, msg_type, yaml_message, "--once"]
    
    try:
        print("try")
        # Run the command to publish the message
        result = subprocess.run(command, capture_output=True, text=True)
        print("subprocess")
        if result.returncode == 0:
            print(f"Message published to {topic_name}: {yaml_message}")
        else:
            print(f"Failed to publish message to {topic_name}")
            print("Error:", result.stderr)
    except Exception as e:
        print(f"Error while publishing: {e}")


def main():
    fire.Fire(run_main)

if __name__ == "__main__":
    main()
