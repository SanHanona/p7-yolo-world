# Copyright (c) Tencent Inc. All rights reserved.
# This file is modifef from mmyolo/demo/video_demo.py
import argparse

import cv2
import mmcv
import torch
import sys
from mmengine.dataset import Compose
from mmdet.apis import init_detector
from mmengine.utils import track_iter_progress

from mmyolo.registry import VISUALIZERS


def parse_args():
    parser = argparse.ArgumentParser(description='YOLO-World video demo')
    parser.add_argument('--config', 
                        default='../configs/pretrain/yolo_world_v2_l_clip_large_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_800ft_lvis_minival.py',
                        help='Config file')
    parser.add_argument('--checkpoint', 
                        default='../../weights/yolo_world_v2_l_clip_large_o365v1_goldg_pretrain_800ft-9df82e55.pth',
                        help='Checkpoint file')
    parser.add_argument('--text',
                        type=str,
                        help='text prompts, including categories separated by a comma or a txt file with each line as a prompt.')
    parser.add_argument('--device',
                        default='cuda:0',
                        help='device used for inference')
    parser.add_argument('--score-thr',
                        default=0.4 ,
                        type=float,
                        help='confidence score threshold for predictions.')
    args = parser.parse_args()
    return args

pip install git+https://github.com/lvis-dataset/lvis-api.git
def inference_detector(model, image, texts, test_pipeline, score_thr=0.3):
    data_info = dict(img_id=0, img=image, texts=texts)
    data_info = test_pipeline(data_info)
    data_batch = dict(inputs=data_info['inputs'].unsqueeze(0),
                      data_samples=[data_info['data_samples']])

    with torch.no_grad():
        output = model.test_step(data_batch)[0]
        pred_instances = output.pred_instances
        pred_instances = pred_instances[pred_instances.scores.float() >
                                        score_thr]
    output.pred_instances = pred_instances
    return output


def main():
    args = parse_args()

    model = init_detector(args.config, args.checkpoint, device=args.device)

    # build test pipeline
    model.cfg.test_dataloader.dataset.pipeline[
        0].type = 'mmdet.LoadImageFromNDArray'
    test_pipeline = Compose(model.cfg.test_dataloader.dataset.pipeline)

    if args.text.endswith('.txt'):
        with open(args.text) as f:
            lines = f.readlines()
        texts = [[t.rstrip('\r\n')] for t in lines] + [[' ']]
    else:
        texts = [[t.strip()] for t in args.text.split(',')] + [[' ']]

    # reparameterize texts
    model.reparameterize(texts)

    # init visualizer
    visualizer = VISUALIZERS.build(model.cfg.visualizer)
    # the dataset_meta is loaded from the checkpoint and
    # then pass to the model in init_detector
    visualizer.dataset_meta = model.dataset_meta

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        exit()
    
    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        result = inference_detector(model,
                                    frame,
                                    texts,
                                    test_pipeline,
                                    score_thr=args.score_thr)
        
        visualizer.add_datasample(name='video',
                                image=frame,
                                data_sample=result,
                                draw_gt=False,
                                show=False,
                                pred_score_thr=args.score_thr)
        
        frame = visualizer.get_image()

        # Display the resulting frame
        cv2.imshow('Annotated Video', frame
                   
                   )

        # Press 'q' to exit the video stream
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything is done, release the capture
    cap.release()
    cv2.destroyAllWindows()       


if __name__ == '__main__':
    main()
