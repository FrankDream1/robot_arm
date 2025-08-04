from ultralytics import YOLO

# load  model
model = YOLO('yolo11n.pt')

# export onnx
# model.export(format='onnx', opset=11, simplify=True, dynamic=False, imgsz=640)
model.export(format='ncnn', opset=11, simplify=True, dynamic=False, imgsz=640)