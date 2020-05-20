#!/bin/bash

./update_config.sh config_gen/config/SuperGreenOS/Controller config.controller.json
./update_templates.sh config.controller.json
./update_htmlapp.sh config.controller.json