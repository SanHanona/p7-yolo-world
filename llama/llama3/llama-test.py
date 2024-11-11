# CHECKPOINT_DIR=~/.llama/checkpoints/Llama3.2-3B
# torchrun llama-test.py $CHECKPOINT_DIR

from typing import Optional
import paramiko
import fire

from reference_impl.generation import Llama
from termcolor import cprint


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

    # stat here the available functions for the robot navigation
    available_functions = ['wait(t)', 'find_new_way()', 'pass()']

    # read the input from a txt file
    input = read_input()

    print(input)
    input = 'can you wait for like eight minutes'

    # the prompt explain the context to the llm and gives examples of how he should behave
    # It also gives it the available functions and the input instruction
    prompts = [
    """You are a robot. You can only respond using the following functions. Call for <|end_of_text|> after giving your output.
    Functions:""" + ', '.join(available_functions)  + """
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
    Output: """ 
]

    for prompt in prompts:
        result = generator.text_completion(
            prompt,
            temperature=temperature,
            top_p=top_p,
            max_gen_len=max_gen_len,
            logprobs=False,
        )

        # printing the results in yellow
        cprint(f"{prompt}", end="")
        cprint(f"{result.generation}", color="yellow")
        
        # cutting the part that we want (the output function)
        output = ""
        for x in result.generation:
            if x == '<':
                break
            if x==' ' or x=='\n' or x=="'":
                continue
            output += x
        print("\n==================================\n\n\n")
        print("--> INPUT : " + input + "\n")
        print("--> OUTPUT : " + output + "\n")
        
        # checking if the output is satifying:
        if (output in available_functions) or (output.startswith('wait(') and output.endswith(')')):
            return True
        else:
            return False
    



def read_input():
    ssh_client = paramiko.SSHClient()

    ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    ssh_client.connect('100.108.187.72', username='yabde', password='Ilyacine3')

    sftp_client = ssh_client.open_sftp()
    remote_file = sftp_client.open('D:\Bureau\TC\AAU\PBL\PBL_code\whisper_real_time\last_sentence.txt')
    try:
        for line in remote_file:
            return line
    finally:
        remote_file.close()


def main():
    fire.Fire(run_main)


if __name__ == "__main__":
    main()