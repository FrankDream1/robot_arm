from ultralytics import YOLO
import cv2
import os
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.text import Text


def visualize_detections(image_path, results, save_path=None, show=True, output_coords=False):
    """
    可视化检测结果，框出目标位置、打上标签、标记中心点并打印坐标

    参数:
        image_path: 原始图像路径
        results: YOLO检测结果对象
        save_path: 保存可视化结果的路径
        show: 是否显示图像
        output_coords: 是否将坐标输出到文件
    """
    # 读取原始图像
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # 转换为RGB格式

    # 创建matplotlib图形
    plt.figure(figsize=(12, 8))
    plt.imshow(image)
    ax = plt.gca()

    # 获取检测结果
    boxes = results[0].boxes.xyxy.cpu().numpy()  # 边界框坐标
    classes = results[0].boxes.cls.cpu().numpy()  # 类别ID
    confidences = results[0].boxes.conf.cpu().numpy()  # 置信度
    names = results[0].names  # 类别名称映射

    # 收集所有中心点坐标
    center_points = []

    print(f"\n检测到的目标数量: {len(boxes)}")
    print("=" * 50)

    # 绘制每个检测结果
    for i, (box, cls, conf) in enumerate(zip(boxes, classes, confidences)):
        # 解包边界框坐标
        x1, y1, x2, y2 = box

        # 计算中心点坐标
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        center_points.append((center_x, center_y))

        # 打印中心点坐标
        print(f"目标 {i + 1}:")
        print(f"  类别: {names[int(cls)]}")
        print(f"  置信度: {conf:.4f}")
        print(f"  中心点坐标: ({center_x:.1f}, {center_y:.1f})")
        print(f"  边界框: x1={x1:.1f}, y1={y1:.1f}, x2={x2:.1f}, y2={y2:.1f}")
        print("-" * 50)

        # 创建矩形框
        rect = Rectangle(
            (x1, y1),
            x2 - x1,
            y2 - y1,
            linewidth=2,
            edgecolor='red',
            facecolor='none'
        )
        ax.add_patch(rect)

        # 添加标签文本
        label = f"{names[int(cls)]} {conf:.2f}"
        text = Text(
            x1,
            y1 - 10,
            label,
            fontsize=12,
            color='red',
            bbox=dict(facecolor='white', alpha=0.7, edgecolor='none', pad=2)
        )
        ax.add_artist(text)

        # 绘制中心点
        center_point = Circle(
            (center_x, center_y),
            radius=5,
            color='cyan',
            fill=True,
            alpha=0.8
        )
        ax.add_patch(center_point)

        # 在中心点旁边添加坐标文本
        coord_text = Text(
            center_x + 10,
            center_y - 10,
            f"({center_x:.0f}, {center_y:.0f})",
            fontsize=10,
            color='yellow',
            bbox=dict(facecolor='blue', alpha=0.5, edgecolor='none', pad=2)
        )
        ax.add_artist(coord_text)

    # 输出坐标到文件
    if output_coords and center_points:
        coord_file = Path(save_path).with_suffix('.txt')
        with open(coord_file, 'w') as f:
            f.write(f"图像: {image_path}\n")
            f.write(f"检测目标总数: {len(center_points)}\n")
            f.write("=" * 50 + "\n")
            for i, (cx, cy) in enumerate(center_points):
                f.write(f"目标 {i + 1}: ({cx:.1f}, {cy:.1f})\n")
        print(f"\n中心点坐标已保存至: {coord_file}")

    plt.axis('off')  # 关闭坐标轴

    # 添加标题
    plt.title(f"检测结果: {len(boxes)}个目标", fontsize=14)

    # 保存结果
    if save_path:
        plt.savefig(save_path, bbox_inches='tight', pad_inches=0.0, dpi=300)
        print(f"可视化结果已保存至: {save_path}")

    # 显示图像
    if show:
        plt.show()

    plt.close()


def main():
    # 加载模型
    model = YOLO("./best.pt")

    # 设置输入源（可以是目录、单个图像或视频）
    source = "ultralytics/assets/"

    # 创建保存结果的目录
    save_dir = Path("runs/detect/visualized")
    save_dir.mkdir(parents=True, exist_ok=True)

    # 处理不同类型的输入源
    if os.path.isdir(source):
        # 处理目录中的所有图像
        image_extensions = ['.jpg', '.jpeg', '.png', '.bmp']
        for file in Path(source).iterdir():
            if file.suffix.lower() in image_extensions:
                print(f"\n{'=' * 50}")
                print(f"处理图像: {file.name}")
                print(f"{'=' * 50}")

                # 执行目标检测
                results = model(file, conf=0.4)

                # 创建保存路径
                save_path = save_dir / f"visualized_{file.stem}.png"

                # 可视化结果并输出坐标
                visualize_detections(
                    str(file),
                    results,
                    save_path=str(save_path),
                    show=False,
                    output_coords=True
                )
    elif os.path.isfile(source):
        # 处理单个图像文件
        print(f"\n{'=' * 50}")
        print(f"处理图像: {source}")
        print(f"{'=' * 50}")

        # 执行目标检测
        results = model(source, conf=0.1)

        # 创建保存路径
        save_path = save_dir / f"visualized_{Path(source).stem}.png"

        # 可视化结果并输出坐标
        visualize_detections(
            source,
            results,
            save_path=str(save_path),
            output_coords=True
        )
    else:
        # 处理视频文件
        cap = cv2.VideoCapture(source)
        frame_count = 0
        video_save_dir = save_dir / "video_frames"
        video_save_dir.mkdir(exist_ok=True)

        print(f"\n开始处理视频: {source}")
        print(f"{'=' * 50}")

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # 进行检测
            results = model(frame, conf=0.1)

            # 可视化当前帧
            temp_image_path = video_save_dir / f"temp_frame_{frame_count}.jpg"
            cv2.imwrite(str(temp_image_path), frame)

            # 创建保存路径
            save_path = video_save_dir / f"frame_{frame_count:04d}.png"

            # 可视化结果并输出坐标
            visualize_detections(
                str(temp_image_path),
                results,
                save_path=str(save_path),
                show=False,
                output_coords=True
            )

            # 删除临时文件
            os.remove(temp_image_path)

            frame_count += 1
            if frame_count % 10 == 0:
                print(f"已处理 {frame_count} 帧")

        cap.release()
        print(f"\n视频处理完成，共处理 {frame_count} 帧")
        print(f"结果保存在: {video_save_dir}")


if __name__ == "__main__":
    main()












