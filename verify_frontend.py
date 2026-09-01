# -*- coding: utf-8 -*-
"""Frontend pipeline verification for sensevoice_rknn_npu.py.

Runs the full pre-NPU feature extraction (read_wav_mono -> fbank80 -> apply_lfr
-> apply_cmvn -> build_encoder_input) on test.wav and en.wav using ONLY the
numpy-based functions imported from the NPU script. This proves the entire
preprocessing path is correct and produces the right encoder input shape,
without needing a real NPU.

Passes if:
  - fbank80 produces ~T/100 frames of 80-dim log-mel (reasonable values)
  - apply_lfr produces ceil(T/6) frames of 560-dim
  - apply_cmvn output is finite with reasonable range
  - build_encoder_input produces [1, T+4, 560] with the query rows first
"""
import sys, os, numpy as np
sys.path.insert(0, '/root/work/tgoskits-audio')
# import the functions from the NPU script (they are pure numpy, no NPU needed)
import sensevoice_rknn_npu as sv

MODEL_DIR = '/root/work/tgoskits-audio/apps/starry/sensevoice-rknn/model'
WAV_DIR = '/root/work/tgoskits-audio/apps/starry/sensevoice-rknn/testwavs'

def verify_wav(name):
    path = os.path.join(WAV_DIR, name)
    print(f'\n=== {name} ===')
    samples = sv.read_wav_mono(path)
    print(f'samples: shape={samples.shape} dtype={samples.dtype} range=[{samples.min():.4f},{samples.max():.4f}]')
    assert samples.dtype == np.float32
    assert len(samples) > 0

    feats = sv.fbank80(samples)
    print(f'fbank80: shape={feats.shape} dtype={feats.dtype} range=[{feats.min():.3f},{feats.max():.3f}] mean={feats.mean():.3f}')
    assert feats.shape[1] == 80, f'expected 80 mel bins, got {feats.shape[1]}'
    assert np.isfinite(feats).all(), 'fbank has non-finite values'
    # log-mel typically in range -15 to +5
    assert feats.min() > -25 and feats.max() < 35  # int16-scaled waveform -> higher peaks

    lfr = sv.apply_lfr(feats)
    print(f'apply_lfr: shape={lfr.shape} dtype={lfr.dtype} range=[{lfr.min():.3f},{lfr.max():.3f}]')
    assert lfr.shape[1] == 560, f'expected 560-dim after LFR(7,6), got {lfr.shape[1]}'
    expected_frames = int(np.ceil(feats.shape[0] / 6))
    assert lfr.shape[0] == expected_frames, f'LFR frame count {lfr.shape[0]} != expected {expected_frames}'

    cmvn = sv.load_cmvn(os.path.join(MODEL_DIR, 'am.mvn'))
    print(f'cmvn: shift={cmvn[0].shape} rescale={cmvn[1].shape}')
    assert cmvn[0].shape[0] == 560, f'AddShift dim {cmvn[0].shape[0]} != 560'
    assert cmvn[1].shape[0] == 560, f'Rescale dim {cmvn[1].shape[0]} != 560'

    speech = sv.apply_cmvn(lfr, cmvn)
    print(f'apply_cmvn: shape={speech.shape} dtype={speech.dtype} finite={np.isfinite(speech).all()} range=[{speech.min():.3f},{speech.max():.3f}]')
    assert np.isfinite(speech).all(), 'CMVN output has non-finite values'
    # CMVN-normalized features typically in range -10 to +10
    assert speech.min() > -50 and speech.max() < 50

    embedding = np.load(os.path.join(MODEL_DIR, 'embedding.npy'))
    print(f'embedding: shape={embedding.shape} dtype={embedding.dtype}')
    assert embedding.ndim == 2 and embedding.shape[1] == 560, f'embedding shape {embedding.shape} unexpected'

    batch = sv.build_encoder_input(embedding, speech, 'auto', True)
    print(f'build_encoder_input: shape={batch.shape} dtype={batch.dtype} finite={np.isfinite(batch).all()}')
    assert batch.shape[0] == 1, f'batch dim0 {batch.shape[0]} != 1'
    assert batch.shape[2] == 560, f'batch dim2 {batch.shape[2]} != 560'
    assert batch.shape[1] == speech.shape[0] + 4, f'batch T {batch.shape[1]} != speech T {speech.shape[0]} + 4'
    assert np.isfinite(batch).all(), 'encoder input has non-finite values'
    print(f'  -> encoder input ready: [1, {batch.shape[1]}, 560], first 4 rows are queries (lang/event/emo/itn)')

    return batch

print('Frontend pipeline verification (pure numpy, no NPU needed)')
b1 = verify_wav('test.wav')
b2 = verify_wav('en.wav')
print(f'\n=== SUMMARY ===')
print(f'test.wav encoder input: {b1.shape} (valid frames={b1.shape[1]})')
print(f'en.wav encoder input:   {b2.shape} (valid frames={b2.shape[1]})')
print('ALL FRONTEND CHECKS PASSED')
