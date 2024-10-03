# LEO_exo_Custom_Control
Description and files to drive a LEO Exoskeleton with an external Raspberry Pi 4/5

You will need an RS485 CAN HAT: https://www.waveshare.com/wiki/RS485_CAN_HAT


![IMG_2964](https://github.com/user-attachments/assets/e7c75833-bd45-424f-a438-c403ef3af450)

## Raspberry Pi 4/5 Configuration [^Note1]

To setup the raspberry Pi 4/5 to drive the LEO exoskeleton you must follow the next steps:
1. Install the latest version of Raspbian OS (64 bits[^1]) in you board
2. Enable the Serial Port (Raspberry Pi Configuration -> Interfaces)
2. Create a virtual environment and activate it
- Install venv
```
sudo apt update
sudo apt upgrade
sudo apt install python-venv
```
- Create virtual environment
```python
python -m venv path_of_the_venv/name_of_the_venv
```
- Activate virtual environment
```
source path_of_the_venv/name_of_the_venv/bin/activate
```
3. Install the following libraries[^2] :
- Pyserial
- PyTorch
- Numpy
4. Download any of the folders in the repos


[^Note1]: This configuration is to run with Inertial Measurement Units (IMUs) feedback. Hence, the BIRO Lab Control Box is needed. 
[^1]: The 64 bits version is needed to use PyTorch
[^2]: Make sure that the versions are compatible | Python 3.12.4 | PySerial 3.5 | PyTorch 2.1.1 | Numpy 1.26.0 |


Left: E9:65:F2:E3:6E:58  
Right: 3B:04:94:38:88:F6  