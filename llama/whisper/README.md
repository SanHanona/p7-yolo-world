# Real Time Whisper Transcription

![Demo gif](demo.gif)

This is a demo of real time speech to text with OpenAI's Whisper model. It works by constantly recording audio in a thread and concatenating the raw bytes over multiple recordings.

## Installation
```
pip install -r requirements.txt
```

Whisper also requires the command-line tool [`ffmpeg`](https://ffmpeg.org/) to be installed on your system, which is available from most package managers:

```
# on Ubuntu or Debian
sudo apt update && sudo apt install ffmpeg

# on Arch Linux
sudo pacman -S ffmpeg

# on MacOS using Homebrew (https://brew.sh/)
brew install ffmpeg

# on Windows using Chocolatey (https://chocolatey.org/)
choco install ffmpeg

# on Windows using Scoop (https://scoop.sh/)
scoop install ffmpeg
```


## Usage

Start the acquisition by simply running the transcribe_demo.py or by running with the parameters:
```
python transcribe_demo.py ----model "medium" --energy_threshold 1000 --record_timeout 5 --phrase_timeout 3
```

Available models are : ` 'tiny.en', 'tiny', 'base.en', 'base', 'small.en', 'small', 'medium.en', 'medium', 'large-v1', 'large-v2', 'large-v3', 'large', 'large-v3-turbo', 'turbo' `


Your last sentence will be writen in the `last_sentence.txt` file.

## Other

This project is a modified version of https://github.com/davabase/whisper_real_time.git

For more information on Whisper please see https://github.com/openai/whisper

The code in this repository is public domain.