#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

## | --------- change to the directory of this script --------- |

cd "$MY_PATH"

## | ---------------------- install tmux ---------------------- |

bash $MY_PATH/dependencies/tmux/install.sh

## | ------------------- install tmuxinator ------------------- |

bash $MY_PATH/dependencies/tmuxinator.sh
