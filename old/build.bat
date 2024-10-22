@echo off
setlocal

:: Exit immediately if a command fails
set "MODEL_DIR=..\weights"
if defined %MODEL_DIR% (
    echo Model directory is already set to %MODEL_DIR%.
) else (
    set "MODEL_DIR=..\models\models-yoloworld"
)

:: DocString for the script
:: This script builds and runs a Docker container for YOLO-World demos.
:: It supports various pre-trained models and configurations for object detection and segmentation.
::
:: Usage:
::    build_and_run.bat <model-key>
::
:: Environment Variables:
::    MODEL_DIR: Path to the directory containing model weights (default: "../models/models-yoloworld")
::
:: Arguments:
::    <model-key>: Key for the desired model configuration (see available keys below)
::
:: Available model keys:
::    seg-l, seg-l-seghead, seg-m, seg-m-seghead,
::    pretrain-l-clip-800ft, pretrain-l-clip, pretrain-l-1280ft, pretrain-l,
::    pretrain-m-1280ft, pretrain-m, pretrain-s-1280ft, pretrain-s,
::    pretrain-x-cc3mlite, pretrain-x-1280ft

:: Define associative array for model configurations
set "models[seg-l]=yolo_world_v2_seg_l_vlpan_bn_2e-4_80e_8gpus_seghead_finetune_lvis.py yolo_world_seg_l_dual_vlpan_2e-4_80e_8gpus_allmodules_finetune_lvis-8c58c916.pth"
set "models[seg-l-seghead]=yolo_world_v2_seg_l_vlpan_bn_2e-4_80e_8gpus_seghead_finetune_lvis.py yolo_world_seg_l_dual_vlpan_2e-4_80e_8gpus_seghead_finetune_lvis-5a642d30.pth"
set "models[seg-m]=yolo_world_v2_seg_m_vlpan_bn_2e-4_80e_8gpus_seghead_finetune_lvis.py yolo_world_seg_m_dual_vlpan_2e-4_80e_8gpus_allmodules_finetune_lvis-ca465825.pth"
set "models[seg-m-seghead]=yolo_world_v2_seg_m_vlpan_bn_2e-4_80e_8gpus_seghead_finetune_lvis.py yolo_world_seg_m_dual_vlpan_2e-4_80e_8gpus_seghead_finetune_lvis-7bca59a7.pth"
set "models[pretrain-l-clip-800ft]=yolo_world_v2_l_clip_large_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_800ft_lvis_minival.py yolo_world_v2_l_clip_large_o365v1_goldg_pretrain_800ft-9df82e55.pth"
set "models[pretrain-l-clip]=yolo_world_v2_l_clip_large_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py yolo_world_v2_l_clip_large_o365v1_goldg_pretrain-8ff2e744.pth"
set "models[pretrain-l-1280ft]=yolo_world_v2_l_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_1280ft_lvis_minival.py yolo_world_v2_l_obj365v1_goldg_pretrain_1280ft-9babe3f6.pth"
set "models[pretrain-l]=yolo_world_v2_l_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py yolo_world_v2_l_obj365v1_goldg_pretrain-a82b1fe3.pth"
set "models[pretrain-m-1280ft]=yolo_world_v2_m_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_1280ft_lvis_minival.py yolo_world_v2_m_obj365v1_goldg_pretrain_1280ft-77d0346d.pth"
set "models[pretrain-m]=yolo_world_v2_m_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py yolo_world_v2_m_obj365v1_goldg_pretrain-c6237d5b.pth"
set "models[pretrain-s-1280ft]=yolo_world_v2_s_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_1280ft_lvis_minival.py yolo_world_v2_s_obj365v1_goldg_pretrain_1280ft-fc4ff4f7.pth"
set "models[pretrain-s]=yolo_world_v2_s_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py yolo_world_v2_s_obj365v1_goldg_pretrain-55b943ea.pth"
set "models[pretrain-x-cc3mlite]=yolo_world_v2_x_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_cc3mlite_train_lvis_minival.py yolo_world_v2_x_obj365v1_goldg_cc3mlite_pretrain-8698fbfa.pth"
set "models[pretrain-x-1280ft]=yolo_world_v2_x_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_1280ft_lvis_minival.py yolo_world_v2_x_obj365v1_goldg_cc3mlite_pretrain_1280ft-14996a36.pth"

:: Function to display usage information
call :show_usage
if "%~1"=="" (
    echo Invalid model key.
    exit /b 1
)

set "model_key=%1"

:: Validate the model key
if not defined models[%model_key%] (
    echo Invalid model key.
    call :show_usage
    exit /b 1
)

:: Extract model and weight information
for /f "tokens=1,2" %%a in ("%models[%model_key%]%") do (
    set "MODEL=%%a"
    set "WEIGHT=%%b"
)

:: Set configuration directory and demo file based on model type
set "config_dir=configs\pretrain"
set "demo_file=demo\image_demo.py"
if "%model_key:seg-=%" neq "%model_key%" (
    set "config_dir=configs\segmentation"
    set "demo_file=demo\segmentation_demo.py"
)

:: Build Docker image and run container
echo Building Docker image...
docker build -f Dockerfile --no-cache --build-arg MODEL=%MODEL% --build-arg WEIGHT=%WEIGHT% -t yolo-demo:latest .

goto :eof

:show_usage
echo Usage: %~n0 ^<model-key^>
echo Available model keys:
for /f "tokens=1" %%k in ('set models[') do echo  %%k

