#!/bin/sh

# シェルスクリプト自体のディレクトリパスを取得
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# シェルスクリプトを実行したディレクトリパスを取得
CURRENT_DIR="$(pwd)"

cd $SCRIPT_DIR

# Pythonの仮想環境がアクティブかどうかを確認
if [[ -n "$VIRTUAL_ENV" ]]; then
    deactivate
fi

PYTHONPATH=$PYTHONPATH:$SCRIPT_DIR/.venv/lib/python3.10/site-packages

PYTHONPATH=$PYTHONPATH:$SCRIPT_DIR/src/mmaction2_ros2/mmaction2

source install/setup.bash

colcon build

# Pythonの仮想環境がアクティブではないかどうかを確認
if [[ -z "$VIRTUAL_ENV" ]]; then
    source .venv/bin/activate
fi

cd $CURRENT_DIR

echo "reload setting."