# Copyright (c) OpenMMLab. All rights reserved.
# OpenMMLabによる著作権（すべての権利を保有）
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

def show_results():
    print('「Esc」、「q」または「Q」を押して終了します')

    text_info = {}
    cur_time = time.time()
    while True:
        msg = 'Waiting for action ...'
        _, frame = camera.read()
        frame_queue.append(np.array(frame[:, :, ::-1]))

        if len(result_queue) != 0:
            text_info = {}
            results = result_queue.popleft()
            for i, result in enumerate(results):
                selected_label, score = result
                if score < threshold:
                    break
                location = (0, 40 + i * 20)
                text = selected_label + ': ' + str(round(score * 100, 2))
                text_info[location] = text
                cv2.putText(frame, text, location, FONTFACE, FONTSCALE,
                            FONTCOLOR, THICKNESS, LINETYPE)

        elif len(text_info) != 0:
            for location, text in text_info.items():
                cv2.putText(frame, text, location, FONTFACE, FONTSCALE,
                            FONTCOLOR, THICKNESS, LINETYPE)

        else:
            cv2.putText(frame, msg, (0, 40), FONTFACE, FONTSCALE, MSGCOLOR,
                        THICKNESS, LINETYPE)

        cv2.imshow('camera', frame)
        ch = cv2.waitKey(1)

        if ch == 27 or ch == ord('q') or ch == ord('Q'):
            camera.release()
            cv2.destroyAllWindows()
            break

        if drawing_fps > 0:
            # 実際の描画FPS <= 描画FPSの上限値に制限を加える
            sleep_time = 1 / drawing_fps - (time.time() - cur_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
            cur_time = time.time()

def inference():
    score_cache = deque()
    scores_sum = 0
    cur_time = time.time()
    while True:
        cur_windows = []

        while len(cur_windows) == 0:
            if len(frame_queue) == sample_length:
                cur_windows = list(np.array(frame_queue))
                if data['img_shape'] is None:
                    data['img_shape'] = frame_queue.popleft().shape[:2]

        cur_data = data.copy()
        cur_data['imgs'] = cur_windows
        cur_data = test_pipeline(cur_data)
        cur_data = pseudo_collate([cur_data])

        # モデルをフォワードする
        with torch.no_grad():
            result = model.test_step(cur_data)[0]
        scores = result.pred_scores.item.tolist()
        scores = np.array(scores)
        score_cache.append(scores)
        scores_sum += scores

        if len(score_cache) == average_size:
            scores_avg = scores_sum / average_size
            num_selected_labels = min(len(label), 5)

            score_tuples = tuple(zip(label, scores_avg))
            score_sorted = sorted(
                score_tuples, key=itemgetter(1), reverse=True)
            results = score_sorted[:num_selected_labels]

            result_queue.append(results)
            scores_sum -= score_cache.popleft()

            if inference_fps > 0:
                # 実際の推論FPS <= 推論FPSの上限値に制限を加える
                sleep_time = 1 / inference_fps - (time.time() - cur_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                cur_time = time.time()


def main():
    global average_size, threshold, drawing_fps, inference_fps, \
        device, model, camera, data, label, sample_length, \
        test_pipeline, frame_queue, result_queue

    args = parse_args()
    average_size = args.average_size
    threshold = args.threshold
    drawing_fps = args.drawing_fps
    inference_fps = args.inference_fps

    device = torch.device(args.device)

    cfg = Config.fromfile(args.config)
    if args.cfg_options is not None:
        cfg.merge_from_dict(args.cfg_options)

    # 設定ファイルとチェックポイントファイル/URLからRecognizerを構築
    model = init_recognizer(cfg, args.checkpoint, device=args.device)
    camera = cv2.VideoCapture(args.camera_id)
    data = dict(img_shape=None, modality='RGB', label=-1)

    with open(args.label, 'r') as f:
        label = [line.strip() for line in f]

    # カメラ以外のパイプラインからテストパイプラインを準備
    cfg = model.cfg
    sample_length = 0
    pipeline = cfg.test_pipeline
    pipeline_ = pipeline.copy()
    for step in pipeline:
        if 'SampleFrames' in get_str_type(step['type']):
            sample_length = step['clip_len'] * step['num_clips']
            data['num_clips'] = step['num_clips']
            data['clip_len'] = step['clip_len']
            pipeline_.remove(step)
        if get_str_type(step['type']) in EXCLUED_STEPS:
            # フレームをデコードするステップを削除
            pipeline_.remove(step)
    test_pipeline = Compose(pipeline_)

    assert sample_length > 0

    try:
        frame_queue = deque(maxlen=sample_length)
        result_queue = deque(maxlen=1)
        pw = Thread(target=show_results, args=(), daemon=True)
        pr = Thread(target=inference, args=(), daemon=True)
        pw.start()
        pr.start()
        pw.join()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
