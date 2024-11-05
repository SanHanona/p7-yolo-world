from ultralytics import YOLOWorld
from ultralytics.nn.tasks import WorldModel
from ultralytics.models.yolo.world.train import WorldTrainer


# Initialize the trainer and model
yaml_path = '/yolo/configs/finetune_handguestures/hand_gestures_v6i.yolov5pytorch/data.yaml'

args = dict(model='yolov8x-worldv2.pt', data=yaml_path, epochs=5, batch=4, imgsz=640, 
            lr0=0.001, weight_decay=0.0005, momentum=0.932, 
            augment=True, save_period=1, patience=5, device=0, val=True, plots=True)

trainer = WorldTrainer(overrides=args)

# Train the model
results = trainer.train()

# model.save("custom_yolov8s.pt") 