# new_ball_detector

## Model conversion

> [!CAUTION]
> The `.engine` model should be built for every system, as the TensorRT libraries are very sensitive to version changes.

This node uses a TensorRT model (`.engine`). To convert a Pytorch model (`.pt`) to a TensorRT model, run the following commands **in the same directory as your `.pt` model**.

### Convert `.pt` to `.onnx`:

```bash
python3 -c "from ultralytics import YOLO; model = YOLO('yolov8_center.pt'); model.export(format='onnx', imgsz=320, opset=17)"
```

### Convert `.onnx` to `.engine`:

For the ***Booster T1***:

```bash
/usr/src/tensorrt/bin/trtexec --onnx=yolov8_center.onnx --saveEngine=yolov8_center_sys_low_t1.engine --fp16 --memPoolSize=workspace:256 --builderOptimizationLevel=0
```

For the ***Booster K1***:

```bash
/usr/src/tensorrt/bin/trtexec --onnx=yolov8_center.onnx --saveEngine=yolov8_center_sys_low_k1.engine --fp16 --memPoolSize=workspace:256 --builderOptimizationLevel=0
```
