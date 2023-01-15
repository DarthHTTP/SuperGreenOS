![SuperGreenLab](assets/sgl.png?raw=true "SuperGreenLab")

[![SuperGreenLab](assets/reddit-button.png?raw=true "SuperGreenLab")](https://www.reddit.com/r/SuperGreenLab)

# Table of Contents

- [Table of Contents](#table-of-contents)
- [SuperGreenOS](#supergreenos)
  - [Who is this document for](#who-is-this-document-for)
  - [Features](#features)
- [Workspace setup](#workspace-setup)
  - [Clone repo, build and run](#clone-repo-build-and-run)
- [How to use](#how-to-use)
- [Up-2-date dev environment setup 06/2020](#up-2-date-dev-environment-setup-062020)
  - [Python 2.7](#python-27)
    - [macos](#macos)
  - [ESP-IDF](#esp-idf)
  - [ejs-cli](#ejs-cli)
  - [mkspiffs](#mkspiffs)
  - [cue](#cue)
- [Using templates.sh for adding a new component](#using-templatessh-for-adding-a-new-component)
- [Up-2-date dev environment setup 01/2023](#up-2-date-dev-environment-setup-012023)

![WeedAppPic](assets/weedapppic.png?raw=true "WeedAppPic")

# SuperGreenOS

SuperGreenOS provides most features used when growing cannabis, all in one package, and controllable from your smartphone, pc, mac, linux, toaster, plumbus, whatnot...

It is the official firmware for the [SuperGreenController](https://github.com/supergreenlab/SuperGreenController).

## Who is this document for

This document is for developpers that want to start playing with there controller's internal stuffs, or just setup their own hardware.
This repository is based on [SuperGreenOSBoilerplate](https://github.com/supergreenlab/SuperGreenOSBoilerplate), please read the doc here first.

## Features

Here's what it can (or will) do:

- Lights on and off schedules
- Up to 6 separate led channels (you can put multiple leds behind one channel)
- Up to 3 separate timers, for full-cycle setups (veg + flo)
- Monitoring a wide range of sensors
- Data sent to a **private** [cloud](https://github.com/supergreenlab/SuperGreenCloud)
- Produce alerts based on sensor values
- Allows remote control (TODO)
- Manual ventilation control
- Automatic ventilation control based on temperature and humidity (TODO)
- `Stretch` mode, allows to choose how much you want your plant to stretch or thicken
- `Sunglass` mode, so you don't burn your eyes when you work on your plants
- More to come..

This is the firmware that runs the [SuperGreenController](https://github.com/supergreenlab/SuperGreenController).

# Workspace setup

If you haven't already done it, you'll to setup esp-idf's toolchain and sdk.

They have a very good quickstart [here](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html).

## Clone repo, build and run

Now you should be able to clone and build the firmware:

```

git clone https://github.com/supergreenlab/SuperGreenOS.git
cd SuperGreenOS
./update_templates.sh config.controller.json
./update_htmlapp.sh config.controller.json
make -j4

```

The plug your controller or any esp32 based board and run the commands:

```

make -j4 flash monitor
./write_spiffs.sh

```

The first command flashes the firmware, the second writes the embedded admin interface on the tiny file system (~20KB available).

# How to use

Once the firmware is flashed you can access the controller's wifi network, once connected go to http://192.168.4.1/fs/app.html,
this will display the html embedded admin interface, which allows you to easily modify any of the controller's parameter.

![Admin](assets/admin.png?raw=true "Admin")


# Up-2-date dev environment setup 06/2020

## Python 2.7

### macos
```bash
brew install python@2
```

## ESP-IDF


```bash
mkdir -p $HOME/esp && cd $HOME/esp
git clone --recursive https://github.com/espressif/esp-idf.git esp-idf_release_3.3.2
cd esp-idf_release_3.3.2
```

Install Docs reference:
https://docs.espressif.com/projects/esp-idf/en/v3.3.2/get-started/index.html

```bash
python2.7 -m pip install --user -r $IDF_PATH/requirements.txt
```

Practically, a virtualenv is created in ~/.espressif where packages are installed and will be activated with the following addition to shell (.bashrc / .zshrc)

```bash
export IDF_PATH=$HOME/esp/esp-idf_release_3.3.2
source $IDF_PATH/export.sh
```

## ejs-cli
```bash
npm -g install ejs-cli
```

## mkspiffs

Please pay attention to *Build configuration name: generic* and version.

https://github.com/igrr/mkspiffs/releases

```bash
mkspiffs ver. 0.2.3
Build configuration name: generic
SPIFFS ver. 0.3.7-5-gf5e26c4
Extra build flags: (none)
SPIFFS configuration:
  SPIFFS_OBJ_NAME_LEN: 32
  SPIFFS_OBJ_META_LEN: 0
  SPIFFS_USE_MAGIC: 1
  SPIFFS_USE_MAGIC_LENGTH: 1
  SPIFFS_ALIGNED_OBJECT_INDEX_TABLES: 0
```

## cue

https://github.com/cuelang/cue/releases

```bash
cue version 0.0.8 darwin/amd64
```

# Using templates.sh for adding a new component

```bash
ejs-cli - version ???
cue - version 0.0.8
```


```bash
./templates.sh new_i2c_device bmebmp280 config.controller.json
./update_config.sh config_gen/config/SuperGreenOS/Controller config.controller.json

./update_templates.sh config.controller.json
./update_htmlapp.sh config.controller.json


./write_spiffs.sh
```

# Up-2-date dev environment setup 01/2023

```bash
brew install pyenv

pyenv global 2.7.18

python2 -m pip install -r $IDF_PATH/requirements.txt

$IDF_PATH/install.sh
```
  

.bashrc
  

```bash
#
#
#
PATH=~/.bin:$PATH
PATH=~/.pyenv/shims:$PATH

#
#
#
export IDF_PATH=$HOME/esp/esp-idf_release_3.3.1
source $IDF_PATH/export.sh
```