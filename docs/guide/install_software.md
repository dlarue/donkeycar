#Install Software

* [Overview](#Overview)
* Software:
    * [Step 1: Install Software on Host PC](install_software.md#step-1-install-software-on-host-pc)
    * [Step 2: Install Software on Donkeycar](install_software.md#step-2-install-software-on-donkeycar)
* [Create Donkeycar Application](/guide/create_application/)

## Overview

Donkeycar has components to install on a host PC. This can be a laptop, or desktop machine. The machine doesn't have to be powerful, but it will benefit from faster cpu, more ram, and an NVidia GPU. An SSD hard drive will greatly impact your training times.

Donkeycar software components need to be installed on the robot platform of your choice. Raspberry Pi and Jetson Nano have setup docs. But it has been known to work on Jetson TX2, Friendly Arm SBC, or almost any Debian based SBC ( single board computer ).

After install, you will create the Donkeycar application from a template. This contains code that is designed for you to customize for your particular case. Don't worry, we will get you started with some useful defaults.

Next we will train the Donkeycar to drive on it's own based on your driving style! This uses a supervised learning technique often referred to as behavioral cloning.

<iframe width="560" height="315" src="https://www.youtube.com/embed/BQY9IgAxOO0" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

This is not the only method for getting your Donkeycar to drive itself. But it requires the least amount of hardware and least technical knowledge. Then you can explore other techniques in this Ai mobile laboratory called Donkeycar!

## Step 1: Install Software on Host PC

When controlling your Donkey via behavioral cloning, you will need to setup a host pc to train your machine learning model from the data collected on the robot. Choose a setup that matches your computer OS.


* Setup [Linux Host PC](host_pc/setup_ubuntu.md)
![donkey](/assets/logos/linux_logo.png)
* Setup [Windows Host PC](host_pc/setup_windows.md)
![donkey](/assets/logos/windows_logo.png)
* Setup [Mac Host PC](host_pc/setup_mac.md)
![donkey](/assets/logos/apple_logo.jpg)


# Step 2: Install Software On Donkeycar

This guide will help you to setup the software to run Donkeycar on your Raspberry Pi or Jetson Nano. Choose a setup that matches your SBC type. (SBC = single board computer)

* Setup [RaspberryPi](robot_sbc/setup_raspberry_pi.md)
![donkey](/assets/logos/rpi_logo.png)

* Setup [Jetson Nano](robot_sbc/setup_jetson_nano.md)
![donkey](/assets/logos/nvidia_logo.png)


```
sudo vi /media/userID/UUID/etc/hostname
sudo vi /media/userID/UUID/etc/hosts
```

Now your SD card is ready. Eject it from your computer, put it in the Pi, and plug in the Pi.

### Connecting to the Pi

If you followed the above instructions to add wifi access your Pi should
now be connected to your wifi network. Now you need to find its IP address
so you can connect to it via SSH.

The easiest way (on Ubuntu) is to use the `findcar` donkey command. You can try `ping donkeypi.local`. If you've modified the hostname, then you should try: `ping <your hostname>.local`. This will fail on a windows machine. Windows users will need the full IP address (unless using cygwin).

If you are having troubles locating your Pi on the network, you will want to plug in an HDMI monitor and USB keyboard into the Pi. Boot it. Login with:

* Username: __pi__
* Password: __raspberry__

Then try the command:

```
ifconfig wlan0
```

If this has a valid IPv4 address, 4 groups of numbers separated by dots, then you can try that with your SSH command. If you don't see anything like that, then your wifi config might have a mistake. You can try to fix with

```
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
```

If you don't have a HDMI monitor and keyboard, you can plug-in the Pi with a CAT5 cable to a router with DHCP. If that router is on the same network as your PC, you can try:

```
ping donkeypi.local
```

Hopefully, one of those methods worked and you are now ready to SSH into your Pi. On Mac and Linux, you can open Terminal. On Windows you can install [Putty](http://www.putty.org/) or [one of the alternatives](https://www.htpcbeginner.com/best-ssh-clients-windows-putty-alternatives/2/).

If you have a command prompt, you can try:

```
ssh pi@donkeypi.local
```

or

```
ssh pi@<your pi ip address>
```

or via Putty:
* Username: __pi__
* Password: __raspberry__
* Hostname:`<your pi IP address>`


If you are using the prebuilt image specified above, then your Pi is ready to go. You should see a mycar and donkey directory.

> Note: Check config.py to make sure it uses the correct settings for the PWM channel for steering and throttle. Open config.py ```nano ~/mycar/config.py``` and make sure that you see the lines:
>
> * STEERING_CHANNEL = 1
> * THROTTLE_CHANNEL = 0
>
> The 1 and 0 for the parts arguments should match whichever channel you used to plug your servo/ESC leads in to your 9685 board. Usually this ranges from 0-15 and it numbered on the board.

> Note: If you are using the prebuilt image specified above, your Pi is not using the full capacity of the SD card. To make the full capacity accessible, SSH into the Pi and run `sudo raspi-config` to go into the configuration tool. Select `7 Advanced Options` and `A1 Expand Filesystem`. And then select `<Finish>` to exit the configuration tool and reboot. The Pi can access the full capacity of the SD card now.

### Install Donkeycar

The disk image only has the libraries(tensorflow..) installed, not donkeycar.

```bash
pip install donkeycar[pi]
#test that you are using the most recent version (found in __init__ file)
python -c "import donkeycar as dk; print(dk.__version__)"
```


### Create your car app.

Now generate the drive script, config and folder structure for your car.

```bash
donkey createcar ~/mycar
```

----
Now let's setup the same donkey library on your laptop or server so you can test and train autopilots. Install varies depending on platform.

## Install donkeycar on Linux

Install dependencies, setup virtualenv
```bash
sudo apt-get install virtualenv build-essential python3-dev gfortran libhdf5-dev
virtualenv env -p python3
source env/bin/activate
```

* Install donkey source and create your local working dir:
```bash
git clone https://github.com/autorope/donkeycar
```
* Install the donkeycar package in pip

If you have an NVIDIA GPU - Install donkeycar and use *tensorflow-gpu* dependency:
```bash
pip install donkeycar[tf_gpu]
```

Otherwise, install donkeycar and use *tensorflow* dependency:
```bash
pip install donkeycar[tf]
```

See [https://github.com/tensorflow/tensorflow/issues/7166](https://github.com/tensorflow/tensorflow/issues/7166) for more information.

[Next: Calibrate your car.](./calibrate.md)

----

## Install donkeycar on Windows

* Install [miniconda Python 3.6 64 bit](https://conda.io/miniconda.html). Be sure to check the box to allow it to modify your system path variable to add conda.

* Install [git 64 bit](https://git-scm.com/download/win)

* From the start menu start the Anaconda Prompt.

* Change to a dir you would like to use as the head of your projects.

* Right click can be used to paste into prompt.

```
mkdir projects
cd projects
```

* Get the latest donkey from Github.

```
git clone https://github.com/autorope/donkeycar
cd donkeycar
```

* Navigate to git master branch

```
git checkout master
```

* Create the Python Anaconda environment

```
conda env create -f install\envs\windows.yml
activate donkey
```

* Install donkey source and create your local working dir:

```
pip install -e .
donkey createcar ~/mycar
```



> Note: After closing the Anaconda Prompt, when you open it again, you will need to
> type ```activate donkey``` to re-enable the mappings to donkey specific
> Python libraries

[Next: Calibrate your car.](./calibrate.md)

----

## Install donkeycar on Mac

* Install [miniconda Python 3.6 64 bit](https://conda.io/miniconda.html)

* Install [git 64 bit](https://www.atlassian.com/git/tutorials/install-git)

* Start Terminal

* If Xcode or gcc not installed - run the following command to install Command Line Tools for Xcode.

```
xcode-select --install
```

* Change to a dir you would like to use as the head of your projects.

```
mkdir projects
cd projects
```

* Get the latest donkey from Github.

```
git clone https://github.com/autorope/donkeycar
cd donkeycar
```

* Create the Python anaconda environment

```
conda env create -f install/envs/mac.yml
source activate donkey
```

* Install Tensorflow

```
pip install https://storage.googleapis.com/tensorflow/mac/cpu/tensorflow-1.10.1-py3-none-any.whl
```


* Install donkey source and create your local working dir:

```
pip install -e .
donkey createcar ~/mycar
```

[Next: Calibrate your car.](./calibrate.md)

> Note: After closing the Terminal, when you open it again, you will need to
> type ```source activate donkey``` to re-enable the mappings to donkey specific
> Python libraries

----

## Install donkeycar on AWS Sagemaker

### Introduction

The following instructions will show you how to install donkeycar on an [AWS SageMaker](https://aws.amazon.com/sagemaker/) *Notebook instance*.

The *Notebook instance* is typically used for experimenting and preparing your data and model for training. The convention is then to distribute the training on a separate cluster of *training instances*. This, however, requires you to adapt your training script (and model) to work in a distributed manner, preferably using the [SageMaker Python SDK](https://github.com/aws/sagemaker-python-sdk), which is not currently available in Donkey.

That said, it is still possible to install and train your model on the *Notebook instance*. It will allow you to train your [Keras model](../../donkeycar/parts/keras.py) using a beefy instance type with a large GPU, and, perhaps most importantly, shut it down when finished so that you only pay for what you use.

### Create a Notebook instance

If you havn't already, log in to your [AWS Console](https://console.aws.amazon.com/console/home) and create a new [AWS SageMaker](https://aws.amazon.com/sagemaker/) Notebook instance:

- https://docs.aws.amazon.com/sagemaker/latest/dg/how-it-works-notebooks-instances.html#howitworks-create-ws

Suggest using a `ml.p2.xlarge` instance type. You can find a list of available types here:

- https://aws.amazon.com/sagemaker/pricing/instance-types/

### Clone the donkey git in SageMaker

When you've created your new instance, open it up and create a new [Jupyter Notebook](http://jupyter.org/) (click *New*, *conda_tensorflow_p36*).

* In the first cell, type:
```python
!git clone https://github.com/autorope/donkeycar ~/SageMaker/donkey
```

* Close the Jupyter Notebook (not the instance!). You can delete it if you want.

### Install donkey on SageMaker

After cloning the git, you'll find the donkey folder in the SageMaker Notebook root:

![donkey dir](../assets/sm-tree-donkey.gif)

* Open *donkey/docs/guide/sm-install-donkey.ipynb* and follow the instructions.

-------

### Install another fork of donkeycar

Occasionally you may want to run with changes from a separate fork of donkey. You may uninstall one and install another. That's fastest, but leaves you with only the forked version installed:

```
pip uninstall donkeycar
git clone --depth=1 https://github.com/<username>/donkey donkey_<username>
cd donkey_<username>
pip install -e .
```

To get back to the stock donkey install:

```
pip uninstall donkeycar
pip install donkeycar
```
