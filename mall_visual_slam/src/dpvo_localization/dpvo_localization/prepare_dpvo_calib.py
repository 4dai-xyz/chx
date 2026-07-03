from pathlib import Path
import argparse


def write_dpvo_calib(path, fx, fy, cx, cy, k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f"{fx:.6f} {fy:.6f} {cx:.6f} {cy:.6f} {k1:.6f} {k2:.6f} {p1:.6f} {p2:.6f} {k3:.6f}\n")


def main():
    parser = argparse.ArgumentParser(description='将 ORB-SLAM3 相机标定文件转换为 DPVO 使用的文本格式')
    parser.add_argument('--input', default='config/KannalaBrandt8.yaml')
    parser.add_argument('--output', default='Opensource code/DPVO-main/calib/custom_mall.txt')
    parser.add_argument('--scale', type=float, default=1.0)
    args = parser.parse_args()

    import cv2
    fs = cv2.FileStorage(args.input, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(args.input)

    fx = fs.getNode('Camera.fx').real() * args.scale
    fy = fs.getNode('Camera.fy').real() * args.scale
    cx = fs.getNode('Camera.cx').real() * args.scale
    cy = fs.getNode('Camera.cy').real() * args.scale
    k1 = fs.getNode('Camera.k1').real()
    k2 = fs.getNode('Camera.k2').real()
    p1 = fs.getNode('Camera.p1').real()
    p2 = fs.getNode('Camera.p2').real()
    k3 = fs.getNode('Camera.k3').real()
    fs.release()

    write_dpvo_calib(args.output, fx, fy, cx, cy, k1, k2, p1, p2, k3)
    print(f'written: {args.output}')


if __name__ == '__main__':
    main()
