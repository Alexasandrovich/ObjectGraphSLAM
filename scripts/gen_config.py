import sys
import os
import shutil
import json

def generate_configs(output_dir, source_config_dir):
    # --- ПАРАМЕТРЫ ТВОЕЙ КАМЕРЫ ---
    WIDTH = 1920
    HEIGHT = 1080
    FX = 1174.0
    FY = 1174.0
    CX = 960.0
    CY = 540.0
    FPS = 30.0
    # ------------------------------

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    print(f"Generating configs in {output_dir}...")

    # 1. ORB-SLAM2 Config (YAML)
    yaml_content = f"""%YAML:1.0
---
Camera.type: "PinHole"
Camera.fx: {FX}
Camera.fy: {FY}
Camera.cx: {CX}
Camera.cy: {CY}
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0
Camera.width: {WIDTH}
Camera.height: {HEIGHT}
Camera.fps: {FPS}
Camera.RGB: 1
ORBextractor.nFeatures: 2000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
"""
    with open(os.path.join(output_dir, "custom.yaml"), "w") as f:
        f.write(yaml_content)

    # 2. ObVi-SLAM Intrinsic (TXT)
    # Format: camera_id, width, height, fx, 0, cx, 0, fy, cy, 0, 0, 1
    with open(os.path.join(output_dir, "custom_camera.txt"), "w") as f:
        f.write("camera_id, img_width, img_height, mat_00, mat_01, mat_02, mat_10, mat_11, mat_12, mat_20, mat_21, mat_22\n")
        f.write(f"1, {WIDTH}, {HEIGHT}, {FX}, 0, {CX}, 0, {FY}, {CY}, 0, 0, 1\n")

    # 3. Extrinsics (Identity)
    with open(os.path.join(output_dir, "extrinsics.txt"), "w") as f:
        f.write("1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1\n")

    # 4. Main Config (JSON Patching)
    # Ищем исходный конфиг в папке проекта
    src_config = None
    for root, dirs, files in os.walk(source_config_dir):
        for file in files:
            if file.endswith(".json") and "config" in file:
                src_config = os.path.join(root, file)
                break
        if src_config: break

    if not src_config:
        print("Warning: Source JSON config not found. Creating minimal default.")
        config_data = {}
    else:
        print(f"Patching config from: {src_config}")
        with open(src_config, 'r') as f:
            try:
                config_data = json.load(f)
            except:
                config_data = {}

    # Добавляем обязательное поле схемы
    config_data["config_schema_version"] = 1

    with open(os.path.join(os.path.dirname(output_dir), "running_config.json"), "w") as f:
        json.dump(config_data, f, indent=4)

    print("Configs generated successfully.")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 gen_config.py <output_calibration_dir> <source_config_dir>")
        sys.exit(1)

    generate_configs(sys.argv[1], sys.argv[2])