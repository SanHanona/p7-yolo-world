# from ultralytics import YOLOWorld
from ultralytics.models.yolo.world import WorldTrainer

args = dict(model="yolov8l-worldv2.pt", data="./geature_data.yaml", epochs=5, batch=4, imgsz=640, 
            lr0=0.001, optimizer='SGD', weight_decay=0.0005, momentum=0.932,
            mosaic=1.0, save_period=1, patience=5, device=0, val=True, plots=True, workers=0)
trainer = WorldTrainer(overrides=args)

trainer.train()