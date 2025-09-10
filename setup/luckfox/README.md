# How to use YOLOv5 on Luckfox
<details>
<summary> <span style="font-size: 22px; font-weight: bold;">1. Convert YOLOv5_model.pt to RKNN </summary>

This section will help you to convert your own YOLOv5 model from Pytorch to ONNX to RKNN based on one of the [Luckfox tutorials](https://wiki.luckfox.com/Luckfox-Pico/Luckfox-Pico-RKNN-Test/) with Ubuntu 20.04 and Python version 3.8.

*NOTE: For others versions please check the [Luckfox tutorials](https://wiki.luckfox.com/Luckfox-Pico/).*

## 1.1 Getting Started
### 1.1.1 Prerequisites

* [YOLOv5 installed](https://github.com/ultralytics/yolov5.git), the trained model.pt with labels and the dataset.
* [RKNN-Toolkit2 downloaded](https://github.com/rockchip-linux/rknn-toolkit2) 
* Install Python Environment

        sudo apt-get update
        sudo apt-get install python3 python3-dev python3-pip
        sudo apt-get install libxslt1-dev zlib1g zlib1g-dev libglib2.0-0 libsm6 libgl1-mesa-glx libprotobuf-dev gcc

* *(Recommended)*  Create a virtual environment. Example: 

        cd rknn-toolkit2
        
        python3 -m venv rknn-toolkit2-env


## 1.2 Install RKNN-Toolkit2 Dependencies
* Activate the virtual environment
    `source rknn-toolkit2-env/bin/activate` and install the requirements. `pip3 install -r rknn-toolkit2/packages/requirements_cp38-1.6.0.txt`.

*NOTE: If an error occurs, try: `pip3 install -r rknn-toolkit2/packages/requirements_cp38-1.6.0.txt -i https://pypi.mirrors.ustc.edu.cn/simple/`*.
## 1.3 Install RKNN-Toolkit2
* `pip3 install rknn-toolkit2/packages/rknn_toolkit2-1.6.0+81f21f4d-cp38-cp38-linux_x86_64.whl`.

If there are no errors after executing the following command, the installation is successful:

        python3
        from rknn.api import RKNN
 ![MENU](/Documentation/Images/successRKNN.png)

 ## 1.2 Convert model.pt to ONNX

 * Use the convert.py script in the yolov5 repository.
 `cd /path/to/yolov5`, run the script with `python export.py --weights <path/to/your/model.pt> --include onnx --imgsz 360 360`.

  *IMPORTANT: All the images of the dataset must have the same size; example: 360x360px.*

 *Note 0: Change -`-imgsz 360 360` if necessary.*

 *Note 1: The path of model.pt is usually found in `yolov5/runs/train/name_of_your_model/weights/best.pt`*


</details>
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
<details>
<summary> <span style="font-size: 22px; font-weight: bold;">2. Run YOLOv5 model on Luckfox </summary>

This section will help you to run a trained model of yolov5 on luckfox pico pro.

## 2.1 Getting Started

### 2.1.1 Prerequisites

* [Luckfox Pico Pro](https://www.luckfox.com/EN-Luckfox-Pico-Pro)
* [Pre-trained YOLOv5 model converted to RKNN](#11-getting-started)
* [Luckfox SDK installed in home directory](https://github.com/LuckfoxTECH/luckfox-pico.git)

## 2.2 How to build
1. Go to the directory `cd Humanoids/setup/luckfox/demos`.
2. Run `./build_yolo.sh`.
 - *NOTE: If you do not have installed the Luckfox SDK in your home directory (`/home/user/luckfox-pico`) the script will fail.*
 - It will show a menu *PUMAS SOFTWARE FOR LUCKFOX BOARD*. Choose 1 and Enter.
 ![MENU](/Documentation/Images/menu_sdk.jpeg)
 3. Once built, `install/` directory is generated, then copy those files using ssh to luckfox root directory.

        cd install
        scp -r luckfox_pico_rtsp_yolov5_demo/root@<your_luckfox_ip>:/root
- *NOTE: The password for Luckfox ssh is always "luckfox"*.
## 2.3 How to run
1. Go to Luckfox ssh `ssh root@<your_luckfox_ip>`, then 
`cd luckfox_pico_rtsp_yolov5_demo/` and run with `./luckfox_pico_rtsp_yolov5`.

2. To only visualize the model running on the luckfox pico camera, on your computer, run: 

        ffplay -flags low_delay -probesize 32 -vf setpts=0 rtsp://<your_luckfox_ip>/live/0


3. Yoy will be able to see the model running.
![YOLO running](/Documentation/Images/yolorunning.jpeg)
