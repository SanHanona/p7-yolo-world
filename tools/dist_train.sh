#!/usr/bin/env bash

CONFIG=$1
GPUS=$2
NNODES=${NNODES:-1}
NODE_RANK=${NODE_RANK:-0}
PORT=${MASTER_PORT:-29500}
MASTER_ADDR=${MASTER_ADDR:-"127.0.0.1"}

<<<<<<< HEAD
# PYTHONPATH="$(dirname $0)/..":$PYTHONPATH \
python3 -m torch.distributed.launch \
    # --nnodes=$NNODES \
    # --node_rank=$NODE_RANK \
    # --master_addr=$MASTER_ADDR \
    # --nproc_per_node=$GPUS \
    # --master_port=$PORT \
=======
PYTHONPATH="$(dirname $0)/..":$PYTHONPATH \
python -m torch.distributed.launch \
    --nnodes=$NNODES \
    --node_rank=$NODE_RANK \
    --master_addr=$MASTER_ADDR \
    --nproc_per_node=$GPUS \
    --master_port=$PORT \
>>>>>>> origin/fine_tune_hand_guestures
    $(dirname "$0")/train.py \
    $CONFIG \
    --launcher pytorch ${@:3}
