#!/usr/bin/env bash

CONFIG=$1
GPUS=$2
NNODES=${NNODES:-1}
NODE_RANK=${NODE_RANK:-0}
PORT=${MASTER_PORT:-29500}
MASTER_ADDR=${MASTER_ADDR:-"127.0.0.1"}

<<<<<<< HEAD
=======
<<<<<<< HEAD
# PYTHONPATH="$(dirname $0)/..":$PYTHONPATH \
python3 -m torch.distributed.launch \
    # --nnodes=$NNODES \
    # --node_rank=$NODE_RANK \
    # --master_addr=$MASTER_ADDR \
    # --nproc_per_node=$GPUS \
    # --master_port=$PORT \
=======
>>>>>>> db6c6324138a7ba00265d32bf682105d983d09ab
PYTHONPATH="$(dirname $0)/..":$PYTHONPATH \
python -m torch.distributed.launch \
    --nnodes=$NNODES \
    --node_rank=$NODE_RANK \
    --master_addr=$MASTER_ADDR \
    --nproc_per_node=$GPUS \
    --master_port=$PORT \
<<<<<<< HEAD
=======
>>>>>>> origin/fine_tune_hand_guestures
>>>>>>> db6c6324138a7ba00265d32bf682105d983d09ab
    $(dirname "$0")/train.py \
    $CONFIG \
    --launcher pytorch ${@:3}
