# NeMo-ASR : ROS Package for Speech Recognition using Nvidia NeMo
# 1. Intro
- This package is for offline speech recognition (ASR) using Nvidia NeMo toolkit.
## Nodes
- /nemo_node : node for ASR
- exit : type 'n' for shutdown of the node.
## Publishing Topics
- /speech_recognition : (String) Speech recognition result
# 2. Prerequisites
* Tested on python=3.8.10
### Dependencies
```shell
$ apt-get install sox libsndfile1 ffmpeg portaudio19-dev
$ apt-get install build-essential
``` 
```shell
$ pip install -r requirements.txt
``` 
### Nvidia NeMo
- guide @ https://github.com/NVIDIA/NeMo
```shell
$ pip install nemo_toolkit[all]
``` 
# 3. Usage
## start NeMo node
```shell
$ roslaunch nemo_asr nemo_asr.launch \
    lang:=ko \
    frame:=5 \
    speech_channel:=speech_recognition
``` 
- lang : {"en", "ko"}
- frame : time(sec) to record each voice command
- speech_channel : topic name
## Interface
```shell
[INPUT] 'y' : record for 5 seconds / 'l' : language / 'c' : cli input / 'n' : shutdown  
``` 
- press 'c' to enable command-line input (instead of STT)
- press 'l' to change language
# 4. Pre-trained ASR models
- this package currently uses Conformer-CTC models - https://arxiv.org/abs/2005.08100
### Changing models / languages
* Currently, English and Korean is supported.
* To change the model, edit "src/utils/agent.py"
### Korean ASR model
- "cwwojin/stt_kr_conformer_ctc_medium" - https://huggingface.co/cwwojin/stt_kr_conformer_ctc_medium
- This model is trained on KsponSpeech dataset - https://aihub.or.kr/
- Preprocessing & training scripts using KsponSpeech can be found at - 
https://github.com/rirolab/Co-op/tree/main/Woojin%20Choi/03_nemo_KsponSpeech_train

# Author
- Woojin Choi / cwwojin@kaist.ac.kr