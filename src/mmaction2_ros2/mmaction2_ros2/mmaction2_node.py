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