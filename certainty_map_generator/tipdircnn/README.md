# tipdircnn
Train tipdircnn with the following command:
```bash
# For Example
python train_network.py --dataset cornell --dataset-path dataset/cornell/ \
--network efffpn_std --label-type dircen --use-rgb 1 \
--batch-size 16 --tra-batches 500 --epoches 100 \
--val-augment --val-batches 128 --tta-size 4
```

The format of args are listed below, details see the scripts:
```bash
python train_network.py --dataset cornell --dataset-path dataset/cornell/ \
--network $MODEL_ID --label-type dircen --use-rgb $RGB_ID \
--batch-size $BATCH_ID --tra-batches 500 --epoches 100 \
--val-augment --val-batches 128 --tta-size 4 --vis

# The training options are instanced as follows, details see in `train_network.py`.
# TASK=("task1" "task2" "task3" "task4" "task5")
# MODEL=("ggcnn_std" "resfpn_std" "resxfpn_std" "dlafpn_std" "efffpn_std" "segnet")
# RGB=("0" "1")
# BATCH=("8" "16" "32")

```