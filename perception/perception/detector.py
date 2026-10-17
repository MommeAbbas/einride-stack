"""Single-class YOLO11 adapter; inference/NMS use maintained runtime packages."""
from pathlib import Path
import json
import cv2
import numpy as np


INPUT_SHAPE = (1, 3, 640, 640)  # Model contract, not tuning.
OUTPUT_SHAPE = (1, 5, 8400)


def letterbox(image):
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError('Expected uint8 RGB image')
    h, w = image.shape[:2]
    scale = min(640 / w, 640 / h)
    nw, nh = round(w * scale), round(h * scale)
    left, top = (640 - nw) // 2, (640 - nh) // 2
    padded = np.full((640, 640, 3), 114, np.uint8)  # YOLO training padding value.
    padded[top:top + nh, left:left + nw] = cv2.resize(image, (nw, nh))
    tensor = np.ascontiguousarray(padded.transpose(2, 0, 1)[None], dtype=np.float32) / 255
    return tensor, (scale, left, top, w, h)


def boxes_from_output(output, geometry, confidence, nms_iou):
    output = np.asarray(output)
    if output.shape != OUTPUT_SHAPE:
        raise ValueError(f'Expected output {OUTPUT_SHAPE}, got {output.shape}')
    candidates = output[0].T
    candidates = candidates[np.isfinite(candidates).all(axis=1) & (candidates[:, 4] >= confidence)]
    scale, left, top, width, height = geometry
    boxes, scores = [], []
    for cx, cy, w, h, score in candidates:
        if w <= 0 or h <= 0:
            continue
        x1, x2 = np.clip([(cx - w / 2 - left) / scale, (cx + w / 2 - left) / scale], 0, width)
        y1, y2 = np.clip([(cy - h / 2 - top) / scale, (cy + h / 2 - top) / scale], 0, height)
        if x2 > x1 and y2 > y1:
            boxes.append([float(x1), float(y1), float(x2 - x1), float(y2 - y1)])
            scores.append(float(score))
    indices = cv2.dnn.NMSBoxes(boxes, scores, float(confidence), float(nms_iou)) if boxes else []
    return [(*boxes[int(i)], scores[int(i)]) for i in np.asarray(indices).reshape(-1)]


class OnnxBackend:
    def __init__(self, path, threads):
        import onnxruntime as ort
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        self.session = ort.InferenceSession(str(Path(path).expanduser()), sess_options=options,
                                            providers=['CPUExecutionProvider'])
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError('Expected one input and one output')
        if tuple(inputs[0].shape) != INPUT_SHAPE or tuple(outputs[0].shape) != OUTPUT_SHAPE:
            raise ValueError(f'Wrong model shapes: {inputs[0].shape}, {outputs[0].shape}')
        self.input = inputs[0].name
        self.dtype = np.float16 if inputs[0].type == 'tensor(float16)' else np.float32

    def __call__(self, tensor):
        return self.session.run(None, {self.input: tensor.astype(self.dtype, copy=False)})[0]

    def close(self):
        pass


class TensorRTBackend:
    """TensorRT 10 named-tensor API + official CUDA Python; no engine building."""
    def __init__(self, path):
        import tensorrt as trt
        try:
            from cuda.bindings import runtime as cuda
        except ImportError:
            from cuda import cudart as cuda
        self.cuda = cuda
        self.allocations = []
        self.stream = None
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        data = Path(path).expanduser().read_bytes()
        # Ultralytics exports may prefix the serialized engine with JSON metadata.
        size = int.from_bytes(data[:4], 'little')
        if 0 < size < len(data) - 4 and data[4:5] == b'{':
            json.loads(data[4:4 + size]); data = data[4 + size:]
        self.engine = self.runtime.deserialize_cuda_engine(data)
        if self.engine is None:
            raise RuntimeError('Engine deserialization failed; use the engine built on this robot/TRT version')
        self.context = self.engine.create_execution_context()
        names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        inputs = [n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT]
        outputs = [n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT]
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError('Expected one input and one output')
        self.input, self.output = inputs[0], outputs[0]
        if -1 in self.engine.get_tensor_shape(self.input):
            if not self.context.set_input_shape(self.input, INPUT_SHAPE):
                raise ValueError('Engine rejects required input shape')
        for name, expected in ((self.input, INPUT_SHAPE), (self.output, OUTPUT_SHAPE)):
            if tuple(self.context.get_tensor_shape(name)) != expected:
                raise ValueError(f'Wrong engine shape for {name}: {self.context.get_tensor_shape(name)}')
            if self.engine.get_tensor_location(name) != trt.TensorLocation.DEVICE:
                raise ValueError('Expected device I/O tensors')
        self.dtype = np.dtype(trt.nptype(self.engine.get_tensor_dtype(self.input)))
        self.result = np.empty(OUTPUT_SHAPE, dtype=trt.nptype(self.engine.get_tensor_dtype(self.output)))
        try:
            self.stream = self.check(cuda.cudaStreamCreate())
            self.input_ptr = self.check(cuda.cudaMalloc(int(np.prod(INPUT_SHAPE)) * self.dtype.itemsize))
            self.allocations.append(self.input_ptr)
            self.output_ptr = self.check(cuda.cudaMalloc(self.result.nbytes))
            self.allocations.append(self.output_ptr)
            for name, pointer in ((self.input, self.input_ptr), (self.output, self.output_ptr)):
                if not self.context.set_tensor_address(name, int(pointer)):
                    raise RuntimeError(f'Cannot bind {name}')
        except Exception:
            self.close(); raise

    @staticmethod
    def check(result):
        error, *values = result
        if int(error) != 0:
            raise RuntimeError(f'CUDA runtime error: {error}')
        return values[0] if values else None

    def __call__(self, tensor):
        host = np.ascontiguousarray(tensor, dtype=self.dtype)
        self.check(self.cuda.cudaMemcpy(self.input_ptr, host.ctypes.data, host.nbytes,
                                       self.cuda.cudaMemcpyKind.cudaMemcpyHostToDevice))
        if not self.context.execute_async_v3(stream_handle=int(self.stream)):
            raise RuntimeError('TensorRT execution failed')
        self.check(self.cuda.cudaStreamSynchronize(self.stream))
        self.check(self.cuda.cudaMemcpy(self.result.ctypes.data, self.output_ptr, self.result.nbytes,
                                       self.cuda.cudaMemcpyKind.cudaMemcpyDeviceToHost))
        return self.result

    def close(self):
        for pointer in self.allocations:
            self.cuda.cudaFree(pointer)
        self.allocations.clear()
        if self.stream is not None:
            self.cuda.cudaStreamDestroy(self.stream); self.stream = None


def load_backend(backend, engine_path, onnx_path, threads):
    """Only startup import/load failures select CPU fallback; runtime errors stay visible."""
    if backend == 'onnx':
        return OnnxBackend(onnx_path, threads), 'onnx', None
    if backend != 'tensorrt':
        raise ValueError('backend must be tensorrt or onnx')
    try:
        return TensorRTBackend(engine_path), 'tensorrt', None
    except Exception as error:
        return OnnxBackend(onnx_path, threads), 'onnx', f'{type(error).__name__}: {error}'
