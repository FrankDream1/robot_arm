from ultralytics import YOLO

# model = YOLO('best.pt')
model = YOLO('yolo11n.pt')

model.export(format='ncnn', opset=12, simplify=True, imgsz=640)

# model.export(format='ncnn', opset=12, simplify=True, imgsz=320, half=True)
# model.export(format='ncnn', imgsz=640)
# model.export(format='onnx', opset=12, simplify=True, dynamic=False, imgsz=640)
# model.export(format='onnx', imgsz=640)

