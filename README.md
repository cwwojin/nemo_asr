# NeMo-ASR : ROS Package for Speech Recognition using Nvidia NeMo
# 1. Intro
- This package is for offline speech recognition (ASR) using Nvidia NeMo toolkit.
## Nodes
- /nemo_node : node for ASR
- exit : type 'n' for shutdown of the node.
## Publishing Topics
- /asr_result : (String) Speech recognition result
## Using Pre-trained NeMo models
* Currently, English and Korean is supported.
* To change the model, edit "src/utils/agent.py"
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
### start NeMo node
```shell
$ roslaunch nemo_asr nemo_asr.launch \
    lang:=ko \
    frame:=5
``` 
## Author
- Woojin Choi / cwwojin@kaist.ac.kr