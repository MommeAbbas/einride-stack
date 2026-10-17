import pytest
from perception import detector

@pytest.mark.parametrize('error',[ImportError('cuda absent'),RuntimeError('engine incompatible')])
def test_trt_import_or_load_failure_falls_back_to_cpu(monkeypatch,error):
    def unavailable(*args):raise error
    sentinel=object()
    monkeypatch.setattr(detector,'TensorRTBackend',unavailable)
    monkeypatch.setattr(detector,'OnnxBackend',lambda *args:sentinel)
    backend,name,reason=detector.load_backend('tensorrt','engine','onnx',1)
    assert backend is sentinel and name=='onnx' and str(error) in reason

def test_successful_trt_does_not_load_cpu(monkeypatch):
    sentinel=object()
    monkeypatch.setattr(detector,'TensorRTBackend',lambda *args:sentinel)
    monkeypatch.setattr(detector,'OnnxBackend',lambda *args:pytest.fail('unexpected CPU load'))
    backend,name,reason=detector.load_backend('tensorrt','engine','onnx',1)
    assert backend is sentinel and name=='tensorrt' and not reason
