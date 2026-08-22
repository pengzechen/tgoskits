#!/bin/sh
# 板级测试入口：StarryOS shell 下由 init.sh 调用。
# 逐级验证并在全部通过时打印 SENSEVOICE_RKNN_TEST_PASSED。

PY=/opt/sensevoice/bin/python3
MODEL_DIR=/opt/sensevoice/model
WAV_DIR=/opt/sensevoice/testwavs
export LD_LIBRARY_PATH=/opt/sensevoice/glibc:/opt/sensevoice/lib
export PYTHONPATH=/opt/sensevoice/python

run="$PY /opt/sensevoice/python/sensevoice_rknn_npu.py --model-dir $MODEL_DIR --lib-dir /opt/sensevoice/lib"

# L0: interpreter + imports
if ! $PY /opt/sensevoice/python/sensevoice_rknn_npu.py --help >/tmp/sv0.log 2>&1; then
    echo "SENSEVOICE_RKNN_TEST_FAILED: L0 rc=$?"
    cat /tmp/sv0.log
    exit 1
fi

# L1: missing model -> nonzero exit + diagnostic
if $run --selftest --wav /nonexistent.wav >/tmp/sv1.log 2>&1; then
    echo "SENSEVOICE_RKNN_TEST_FAILED: L1 missing model returned 0"
    exit 1
fi
grep -qiE "not found|failed|error" /tmp/sv1.log || {
    echo "SENSEVOICE_RKNN_TEST_FAILED: L1 no diagnostic"
    exit 1
}

# L2: zh reference clip
if ! $run --wav $WAV_DIR/test.wav >/tmp/sv2.out 2>/tmp/sv2.log; then
    echo "SENSEVOICE_RKNN_TEST_FAILED: L2 rc=$?"
    tail -5 /tmp/sv2.log
    exit 1
fi
grep -q "开饭时间早上九点至下午五点" /tmp/sv2.out || {
    echo "SENSEVOICE_RKNN_TEST_FAILED: L2 transcript mismatch"
    cat /tmp/sv2.out
    exit 1
}

# L3: en reference clip
if ! $run --wav $WAV_DIR/en.wav >/tmp/sv3.out 2>/tmp/sv3.log; then
    echo "SENSEVOICE_RKNN_TEST_FAILED: L3 rc=$?"
    tail -5 /tmp/sv3.log
    exit 1
fi
grep -q "the tribal chieftain" /tmp/sv3.out || {
    echo "SENSEVOICE_RKNN_TEST_FAILED: L3 transcript mismatch"
    cat /tmp/sv3.out
    exit 1
}

grep -q "\[perf\] model load" /tmp/sv2.log && grep "\[perf\]" /tmp/sv2.log
echo "SENSEVOICE_RKNN_TEST_PASSED"
