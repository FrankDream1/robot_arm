import torch
import os
import ultralytics

def main():
    # 设置字体目录
    os.environ['ULTRALYTICS_FONT_LOCATION'] = '/home/lwy/.config/Ultralytics/assets2'
    model = ultralytics.YOLO('yolo11n.pt')
    results = model.train(data='./data.yaml', epochs = 50, imgsz = 640, batch = 16, project='runs',amp=False)

if __name__ == '__main__':
    main()


















