@echo off
REM Exit immediately if a command exits with a non-zero status.
setlocal enabledelayedexpansion

REM Set MODEL_DIR if not already set in the environment
if not defined MODEL_DIR (
    set MODEL_DIR=..\weights
)

REM DocString for the script
REM This script builds and runs a Docker container for YOLO-World demos.
REM It supports various pre-trained models and configurations for object detection and segmentation.
REM Usage: 
REM     build_and_run.bat <model-key>
REM Environment Variables:
REM     MODEL_DIR: Path to the directory containing model weights (default: "..\models\models-yoloworld")
REM Arguments:
REM     <model-key>: Key for the desired model configuration (see available keys below)
REM Available model keys:
REM     seg-l, seg-l-seghead, seg-m, seg-m-seghead,
REM     pretrain-l-clip-800ft, pretrain-l-clip, pretrain-l-1280ft, pretrain-l,
REM     pretrain-m-1280ft, pretrain-m, pretrain-s-1280ft, pretrain-s,
REM     pretrain-x-cc3mlite, pretrain-x-1280ft

REM Define model configurations
set "model_key=%~1"
set "MODEL="
set "WEIGHT="

if "%model_key%"=="seg-l" (
    set "MODEL=yolo_world_v2_seg_l_vlpan_bn_2e-4_80e_8gpus_seghead_finetune_lvis.py"
    set "WEIGHT=yolo_world_seg_l_dual_vlpan_2e-4_80e_8gpus_allmodules_finetune_lvis-8c58c916.pth"
) else if "%model_key%"=="seg-l-seghead" (
    set "MODEL=yolo_world_v2_seg_l_vlpan_bn_2e-4_80e_8gpus_seghead_finetune_lvis.py"
    set "WEIGHT=yolo_world_seg_l_dual_vlpan_2e-4_80e_8gpus_seghead_finetune_lvis-5a642d30.pth"
) else if "%model_key%"=="seg-m" (
    set "MODEL=yolo_world_v2_seg_m_vlpan_bn_2e-4_80e_8gpus_seghead_finetune_lvis.py"
    set "WEIGHT=yolo_world_seg_m_dual_vlpan_2e-4_80e_8gpus_allmodules_finetune_lvis-ca465825.pth"
) else if "%model_key%"=="seg-m-seghead" (
    set "MODEL=yolo_world_v2_seg_m_vlpan_bn_2e-4_80e_8gpus_seghead_finetune_lvis.py"
    set "WEIGHT=yolo_world_seg_m_dual_vlpan_2e-4_80e_8gpus_seghead_finetune_lvis-7bca59a7.pth"
) else if "%model_key%"=="pretrain-l-clip-800ft" (
    set "MODEL=yolo_world_v2_l_clip_large_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_800ft_lvis_minival.py"
    set "WEIGHT=yolo_world_v2_l_clip_large_o365v1_goldg_pretrain_800ft-9df82e55.pth"
) else if "%model_key%"=="pretrain-l-clip" (
    set "MODEL=yolo_world_v2_l_clip_large_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py"
    set "WEIGHT=yolo_world_v2_l_clip_large_o365v1_goldg_pretrain-8ff2e744.pth"
) else if "%model_key%"=="pretrain-l-1280ft" (
    set "MODEL=yolo_world_v2_l_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_1280ft_lvis_minival.py"
    set "WEIGHT=yolo_world_v2_l_obj365v1_goldg_pretrain_1280ft-9babe3f6.pth"
) else if "%model_key%"=="pretrain-l" (
    set "MODEL=yolo_world_v2_l_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py"
    set "WEIGHT=yolo_world_v2_l_obj365v1_goldg_pretrain-a82b1fe3.pth"
) else if "%model_key%"=="pretrain-m-1280ft" (
    set "MODEL=yolo_world_v2_m_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_1280ft_lvis_minival.py"
    set "WEIGHT=yolo_world_v2_m_obj365v1_goldg_pretrain_1280ft-77d0346d.pth"
) else if "%model_key%"=="pretrain-m" (
    set "MODEL=yolo_world_v2_m_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py"
    set "WEIGHT=yolo_world_v2_m_obj365v1_goldg_pretrain-c6237d5b.pth"
) else if "%model_key%"=="pretrain-s-1280ft" (
    set "MODEL=yolo_world_v2_s_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_1280ft_lvis_minival.py"
    set "WEIGHT=yolo_world_v2_s_obj365v1_goldg_pretrain_1280ft-fc4ff4f7.pth"
) else if "%model_key%"=="pretrain-s" (
    set "MODEL=yolo_world_v2_s_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py"
    set "WEIGHT=yolo_world_v2_s_obj365v1_goldg_pretrain-55b943ea.pth"
) else if "%model_key%"=="pretrain-x-cc3mlite" (
    set "MODEL=yolo_world_v2_x_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_cc3mlite_train_lvis_minival.py"
    set "WEIGHT=yolo_world_v2_x_obj365v1_goldg_cc3mlite_pretrain-8698fbfa.pth"
) else if "%model_key%"=="pretrain-x-1280ft" (
    set "MODEL=yolo_world_v2_x_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_1280ft_lvis_minival.py"
    set "WEIGHT=yolo_world_v2_x_obj365v1_goldg_cc3mlite_pretrain_1280ft-14996a36.pth"
) else (
    echo Invalid model key.
    echo Usage: %0 ^<model-key^>
    echo Available model keys:
    echo   seg-l
    echo   seg-l-seghead
    echo   seg-m
    echo   seg-m-seghead
    echo   pretrain-l-clip-800ft
    echo   pretrain-l-clip
    echo   pretrain-l-1280ft
    echo   pretrain-l
    echo   pretrain-m-1280ft
    echo   pretrain-m
    echo   pretrain-s-1280ft
    echo   pretrain-s
    echo   pretrain-x-cc3mlite
    echo   pretrain-x-1280ft
    exit /b 1
)

REM Set configuration directory and demo file based on model type
set "config_dir=configs\pretrain"
REM set "demo_file=demo\image_demo.py"
if "%model_key:~0,4%"=="seg-" (
    set "config_dir=configs\segmentation"
    set "demo_file=demo\segmentation_demo.py"
)

REM Run Docker container
echo Running Docker container...
docker run --rm -it --privileged --net=host --runtime nvidia --gpus all -p 8080:8080 yolo-demo:latest 
endlocal
