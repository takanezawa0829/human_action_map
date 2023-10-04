import rclpy
from rclpy.node import Node

import argparse
import time
from collections import deque
from operator import itemgetter
from threading import Thread

import cv2
import numpy as np
import torch
from mmengine import Config, DictAction
from mmengine.dataset import Compose, pseudo_collate

from mmaction.apis import init_recognizer
from mmaction.utils import get_str_type

FONTFACE = cv2.FONT_HERSHEY_COMPLEX_SMALL
FONTSCALE = 1
FONTCOLOR = (255, 255, 255)  # BGR、白
MSGCOLOR = (128, 128, 128)  # BGR、グレー
THICKNESS = 1
LINETYPE = 1
EXCLUED_STEPS = [
    'OpenCVInit', 'OpenCVDecode', 'DecordInit', 'DecordDecode', 'PyAVInit',
    'PyAVDecode', 'RawFrameDecode'
]

def parse_args():
    parser = argparse.ArgumentParser(description='MMAction2のウェブカムデモ')
    parser.add_argument('config', help='テスト設定ファイルのパス')
    parser.add_argument('checkpoint', help='チェックポイントファイル/URL')
    parser.add_argument('label', help='ラベルファイル')
    parser.add_argument(
        '--device', type=str, default='cuda:0', help='CPU/CUDAデバイスのオプション')
    parser.add_argument(
        '--camera-id', type=int, default=0, help='カメラデバイスID')
    parser.add_argument(
        '--threshold',
        type=float,
        default=0.01,
        help='認識スコアの閾値')
    parser.add_argument(
        '--average-size',
        type=int,
        default=1,
        help='予測のために平均化される最新クリップの数')
    parser.add_argument(
        '--drawing-fps',
        type=int,
        default=20,
        help='出力描画の上限FPS値を設定')
    parser.add_argument(
        '--inference-fps',
        type=int,
        default=4,
        help='モデル推論の上限FPS値を設定')
    parser.add_argument(
        '--cfg-options',
        nargs='+',
        action=DictAction,
        default={},
        help='使用される設定ファイルで一部の設定をオーバーライドします。'
        'xxx=yyy形式のキーと値のペアが設定ファイルにマージされます。'
        '例：「--cfg-options model.backbone.depth=18 model.backbone.with_cp=True」')
    args = parser.parse_args()
    assert args.drawing_fps >= 0 and args.inference_fps >= 0, \
        '描画と推論のFPS上限値は正の数、または制限なしのためにゼロに設定する必要があります'
    return args






# メイン
def main():
    # ROS通信の初期化
    rclpy.init()
    
    # ノードの生成   
    node = Node("mmaction2_node")
    
    # ログ出力
    print("mmaction2_node")

    # ノード終了の待機
    rclpy.spin(node)
    
    # ROS通信のシャットダウン
    rclpy.shutdown()

if __name__ == "__main__":
   main()