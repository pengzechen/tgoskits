#!/usr/bin/env python3
"""SenseVoice ASR on RK3588 NPU, CPU frontend + CTC decode.

Port of the upstream happyme531/SenseVoiceSmall-RKNN2 rknnlite recipe to the
librknnrt C API through ctypes, so it runs with only numpy on StarryOS.

Frontend: 16k s16 wav -> kaldi-compatible fbank80 (hamming window, preemph
0.97, remove-dc, snip-edges, int16-scale waveform) -> LFR(7,6) -> 560-wide
am.mvn CMVN as (x + AddShift) * Rescale.

Encoder input: [lang(1), event+emo(2), text_norm(1), speech(T)] frames on the
time axis, zero-padded to the model's fixed input length (read from the input
tensor attr). The speech frames are scaled by SENSEVOICE_INPUT_SCALE (fp16
overflow guard; upstream uses 1/2 for its unscaled model, our fp16-scaled
model defaults to 1.0).

Output: [T, vocab] CTC logits (default layout; SENSEVOICE_OUT_LAYOUT=VT for
the transposed happyme531 conversion) -> greedy CTC over the real (pre-pad)
frames (unique-consecutive, blank 0) -> sentencepiece-style surface join
('_' marks spaces), SenseVoice <|...|> prompt tags stripped.

Levels:
  L0  --help / imports
  L1  missing model file fails with a diagnostic
  L2  zh.wav transcript contains the pinned reference text
  L3  en.wav transcript contains the pinned reference text

Prints SENSEVOICE_RKNN_TEST_PASSED only if every level passes.
"""

import argparse
import ctypes
import json
import math
import os
import re
import sys
import time
import wave

import numpy as np

# ---------------------------------------------------------------------------
# RKNN C API (rknn_api.h subset used here)
# ---------------------------------------------------------------------------

# rknn_query_cmd enum: RKNN_QUERY_IN_OUT_NUM = 0, INPUT_ATTR = 1,
# OUTPUT_ATTR = 2. Off-by-one here silently returns rknn_input_output_num
# packed into the attr struct (the earlier "INPUT_ATTR is all zeros /
# dynamic shape" observation was exactly that misread).
RKNN_QUERY_INPUT_ATTR = 1
RKNN_QUERY_OUTPUT_ATTR = 2
# rknn_tensor_type enum: FLOAT32 = 0, FLOAT16 = 1. The in/out attr prints
# type=1 because the model tensors are fp16; the INPUT buffer type we declare
# in rknn_inputs_set must describe OUR buffer (float32 = 0). Declaring 1 made
# librknnrt reinterpret the float32 byte stream as fp16 (reading half the
# buffer), so the NPU computed on garbage and every output was inf.
RKNN_TENSOR_FLOAT32 = 0

DEBUG = bool(os.environ.get("SENSEVOICE_DEBUG"))

VOICE_COMMANDS = (
    ("forward", ("\u524d\u8fdb", "\u5411\u524d", "\u5f80\u524d", "\u524d\u884c")),
    ("back", ("\u540e\u9000", "\u5411\u540e", "\u5f80\u540e", "\u5012\u9000")),
    ("left", ("\u5de6\u8f6c", "\u5411\u5de6", "\u5f80\u5de6")),
    ("right", ("\u53f3\u8f6c", "\u5411\u53f3", "\u5f80\u53f3")),
    ("stop", ("\u505c\u6b62", "\u505c\u4e0b", "\u505c", "\u5239\u8f66")),
)


class RknnTensorAttr(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_int32),
        ("n_dims", ctypes.c_uint32),
        # librknnrt 2.x's rknn_tensor_attr inserts dims[16] right after n_dims
        # (before name). The old 312-byte layout omitted it; librknnrt 2.3.2's
        # sizeof is 376, and without dims here the field offsets (name/n_elems
        # ...) land on zero regions and rknn_query reads back all-zeros.
        ("dims", ctypes.c_uint32 * 16),
        ("name", ctypes.c_char * 256),
        ("n_elems", ctypes.c_uint32),
        ("size", ctypes.c_uint32),
        ("fmt", ctypes.c_int32),
        ("type", ctypes.c_int32),
        ("qnt_type", ctypes.c_int32),
        ("fl", ctypes.c_int8),
        ("zp", ctypes.c_int32),
        ("scale", ctypes.c_float),
        ("w_stride", ctypes.c_int32),
        ("size_with_stride", ctypes.c_uint32),
        ("pass_through", ctypes.c_uint8),
        ("h_stride", ctypes.c_uint32),
    ]


class RknnInput(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint32),
        ("buf", ctypes.c_void_p),
        ("size", ctypes.c_uint32),
        ("pass_through", ctypes.c_uint8),
        ("type", ctypes.c_int32),
        ("fmt", ctypes.c_int32),
    ]


class RknnOutput(ctypes.Structure):
    _fields_ = [
        ("want_float", ctypes.c_uint8),
        ("is_prealloc", ctypes.c_uint8),
        ("index", ctypes.c_uint32),
        ("buf", ctypes.c_void_p),
        ("size", ctypes.c_uint32),
    ]


class RknnContext:
    """Owns one rknn_context and the loaded librknnrt bindings."""

    def __init__(self, model_path, lib_paths):
        lib = None
        errors = []
        for path in lib_paths:
            try:
                lib = ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
                break
            except OSError as err:
                errors.append(f"{path}: {err}")
        if lib is None:
            raise RuntimeError("cannot load librknnrt.so: " + "; ".join(errors))
        self.lib = lib

        lib.rknn_init.argtypes = [
            ctypes.POINTER(ctypes.c_size_t),
            ctypes.c_void_p,
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_void_p,
        ]
        lib.rknn_init.restype = ctypes.c_int
        lib.rknn_destroy.argtypes = [ctypes.c_size_t]
        lib.rknn_query.argtypes = [
            ctypes.c_size_t,
            ctypes.c_int32,
            ctypes.c_void_p,
            ctypes.c_uint32,
        ]
        lib.rknn_inputs_set.argtypes = [
            ctypes.c_size_t,
            ctypes.c_uint32,
            ctypes.POINTER(RknnInput),
        ]
        lib.rknn_run.argtypes = [ctypes.c_size_t, ctypes.c_void_p]
        lib.rknn_wait.argtypes = [ctypes.c_size_t, ctypes.c_void_p]
        lib.rknn_outputs_get.argtypes = [
            ctypes.c_size_t,
            ctypes.c_uint32,
            ctypes.POINTER(RknnOutput),
            ctypes.c_void_p,
        ]
        lib.rknn_outputs_release.argtypes = [
            ctypes.c_size_t,
            ctypes.c_uint32,
            ctypes.POINTER(RknnOutput),
        ]

        with open(model_path, "rb") as handle:
            blob = handle.read()
        self._model_blob = blob  # keep alive: rknn_init reads in place
        self.ctx = ctypes.c_size_t(0)
        ret = lib.rknn_init(
            ctypes.byref(self.ctx),
            ctypes.c_char_p(blob),
            ctypes.c_uint32(len(blob)),
            0,
            None,
        )
        if ret != 0:
            raise RuntimeError(f"rknn_init failed: {ret}")
        self.in_attr = self._query(RKNN_QUERY_INPUT_ATTR)
        self.out_attr = self._query(RKNN_QUERY_OUTPUT_ATTR)
        if DEBUG:
            for label, attr in (("in", self.in_attr), ("out", self.out_attr)):
                name = attr.name.split(b"\x00")[0].decode(errors="replace")
                print(
                    f"[dbg] {label}: n_dims={attr.n_dims} "
                    f"dims={list(attr.dims[:attr.n_dims])} n_elems={attr.n_elems} "
                    f"type={attr.type} fmt={attr.fmt} qnt={attr.qnt_type} "
                    f"fl={attr.fl} zp={attr.zp} scale={attr.scale} "
                    f"size={attr.size} name={name}",
                    file=sys.stderr,
                )

    def _query(self, cmd):
        attr = RknnTensorAttr()
        ret = self.lib.rknn_query(
            self.ctx, cmd, ctypes.byref(attr), ctypes.sizeof(attr)
        )
        if ret != 0:
            raise RuntimeError(f"rknn_query({cmd}) failed: {ret}")
        return attr

    def run(self, speech_batch):
        """speech_batch: float32 [1, T_in, 560] flattened to the input attr."""
        payload = np.ascontiguousarray(speech_batch, dtype=np.float32)
        inp = RknnInput(
            index=0,
            buf=ctypes.c_void_p(payload.ctypes.data),
            size=ctypes.c_uint32(payload.nbytes),
            pass_through=0,
            type=RKNN_TENSOR_FLOAT32,
            fmt=0,
        )
        ret = self.lib.rknn_inputs_set(self.ctx, 1, ctypes.byref(inp))
        if ret != 0:
            raise RuntimeError(f"rknn_inputs_set failed: {ret}")
        ret = self.lib.rknn_run(self.ctx, None)
        if ret != 0:
            raise RuntimeError(f"rknn_run failed: {ret}")
        ret = self.lib.rknn_wait(self.ctx, None)
        if ret != 0:
            # starry's rknpu driver does not provide a dma_fence fd, so
            # rknn_wait returns -1 ("fence fd = -1 is invalid"). If the run
            # ioctl executed the NPU job synchronously the outputs are already
            # ready; proceed to rknn_outputs_get and let it surface a real
            # failure if not.
            pass
        out = RknnOutput(want_float=1, is_prealloc=0, index=0, buf=None, size=0)
        ret = self.lib.rknn_outputs_get(self.ctx, 1, ctypes.byref(out), None)
        if ret != 0:
            raise RuntimeError(f"rknn_outputs_get failed: {ret}")
        try:
            count = out.size // 4
            arr = np.ctypeslib.as_array(
                ctypes.cast(out.buf, ctypes.POINTER(ctypes.c_float)), shape=(count,)
            ).copy()
            if DEBUG:
                finite = int(np.isfinite(arr).sum())
                print(
                    f"[dbg] out.size={out.size} floats={count} "
                    f"finite={finite}/{count} "
                    f"logits[:5]={arr[:5].tolist()}",
                    file=sys.stderr,
                )
            return arr
        finally:
            self.lib.rknn_outputs_release(self.ctx, 1, ctypes.byref(out))

    def close(self):
        if getattr(self, "ctx", 0):
            self.lib.rknn_destroy(self.ctx)
            self.ctx = 0


# ---------------------------------------------------------------------------
# Frontend: 16k PCM -> fbank80 (kaldi-compatible) -> LFR -> CMVN
# ---------------------------------------------------------------------------


def read_wav_mono(path):
    with wave.open(path, "rb") as handle:
        assert handle.getframerate() == 16000, "expect 16 kHz wav"
        assert handle.getnchannels() == 1, "expect mono wav"
        assert handle.getsampwidth() == 2, "expect s16 wav"
        frames = handle.readframes(handle.getnframes())
    return np.frombuffer(frames, dtype="<i2").astype(np.float32) / 32768.0


def fbank80(samples, fs=16000, n_mels=80, frame_ms=25, shift_ms=10, preemph=0.97):
    """kaldi-compatible log-mel fbank matching kaldi_native_fbank defaults:
    remove_dc_offset -> preemphasize -> hamming window -> |rfft|^2 (kaldi does
    not divide by nfft) -> mel filters -> log. `samples` is float32 in [-1, 1);
    the upstream reference feeds an int16-scale waveform (x * 2^15) because the
    am.mvn statistics were computed at that magnitude.
    """
    samples = samples * 32768.0
    frame_len = int(round(fs * frame_ms / 1000.0))
    frame_shift = int(round(fs * shift_ms / 1000.0))
    if len(samples) < frame_len:
        samples = np.pad(samples, (0, frame_len - len(samples)))

    num_frames = 1 + (len(samples) - frame_len) // frame_shift  # snip_edges=True
    window = 0.54 - 0.46 * np.cos(
        2.0 * math.pi * np.arange(frame_len) / (frame_len - 1)
    )

    frames = np.lib.stride_tricks.as_strided(
        samples,
        shape=(num_frames, frame_len),
        strides=(samples.strides[0] * frame_shift, samples.strides[0]),
    ).astype(np.float64).copy()

    frames -= frames.mean(axis=1, keepdims=True)  # remove_dc_offset
    # preemphasize: s[i] -= 0.97 * s[i-1], s[0] -= 0.97 * s[0]
    frames[:, 1:] -= preemph * frames[:, :-1]
    frames[:, 0] -= preemph * frames[:, 0]
    frames *= window

    nfft = 1
    while nfft < frame_len:
        nfft *= 2
    freq_bins = nfft // 2 + 1
    power = np.abs(np.fft.rfft(frames, nfft)) ** 2

    # Triangular mel filters (HTK mel scale, low=20 Hz, high=nyquist)
    def hz_to_mel(freqs):
        return 1127.0 * np.log1p(np.asarray(freqs, dtype=np.float64) / 700.0)

    low, high = 20.0, fs / 2.0
    mel_points = np.linspace(hz_to_mel(low), hz_to_mel(high), n_mels + 2)
    # FFT bin center frequencies span 0..fs/2 (bin i at i*fs/nfft), not 0..fs
    mel_bin = hz_to_mel(np.arange(freq_bins) * (fs / nfft))
    filters = np.zeros((n_mels, freq_bins))
    for m in range(n_mels):
        left, center, right = mel_points[m], mel_points[m + 1], mel_points[m + 2]
        up = (mel_bin - left) / (center - left)
        down = (right - mel_bin) / (right - center)
        filters[m] = np.maximum(0, np.minimum(up, down))
    # knf floors bin energy at 2^-23 (log -> -15.942); match it so silent
    # bins agree bit-for-bit instead of diverging by ~7 after the log
    feats = np.log(np.maximum(power @ filters.T, 2.0**-23))
    return feats.astype(np.float32)


def load_cmvn(path):
    """Parse kaldi am.mvn: <AddShift> and <Rescale> rows, both already spliced
    to the LFR width (560). Applied as (x + AddShift) * Rescale like the
    upstream reference (AddShift is the negated feature mean, so it is added).
    """
    shift = rescale = None
    lines = open(path, "r", encoding="utf-8").read().splitlines()
    for i, line in enumerate(lines):
        if line.startswith("<AddShift>"):
            vec = _mvn_vector_on(lines, i)
            if vec is not None:
                shift = vec
        elif line.startswith("<Rescale>"):
            vec = _mvn_vector_on(lines, i)
            if vec is not None:
                rescale = vec
    if shift is None or rescale is None:
        raise ValueError(f"am.mvn missing AddShift/Rescale vectors: {path}")
    return shift.astype(np.float32), rescale.astype(np.float32)


def _mvn_vector_on(lines, start):
    """Extract the first bracketed float vector from lines[start:]."""
    buf = ""
    for line in lines[start:]:
        buf += line + " "
        if "]" in line:
            break
    left = buf.find("[")
    right = buf.find("]")
    if left == -1 or right == -1:
        return None
    body = buf[left + 1 : right].split()
    try:
        return np.array([float(v) for v in body], dtype=np.float64)
    except ValueError:
        return None


def apply_lfr(inputs, lfr_m=7, lfr_n=6):
    """Upstream LFR stacking: replicate the first frame (lfr_m-1)//2 times on
    the left, take lfr_m consecutive rows every lfr_n, replicate the last row
    to fill the tail. Output is [ceil(T/6), 560].
    """
    T_lfr = int(np.ceil(inputs.shape[0] / lfr_n))
    left_padding = np.tile(inputs[:1], ((lfr_m - 1) // 2, 1))
    padded = np.vstack((left_padding, inputs))
    rows = padded.shape[0]
    outputs = []
    for i in range(T_lfr):
        if lfr_m <= rows - i * lfr_n:
            outputs.append(padded[i * lfr_n : i * lfr_n + lfr_m].reshape(1, -1))
        else:  # last frame: pad with copies of the final row
            num_padding = lfr_m - (rows - i * lfr_n)
            frame = padded[i * lfr_n :].reshape(-1)
            for _ in range(num_padding):
                frame = np.hstack((frame, padded[-1]))
            outputs.append(frame)
    return np.vstack(outputs).astype(np.float32)


def apply_cmvn(inputs, cmvn):
    shift, rescale = cmvn
    dim = min(inputs.shape[1], shift.shape[0])
    means = np.tile(shift[None, :dim], (inputs.shape[0], 1))
    scale = np.tile(rescale[None, :dim], (inputs.shape[0], 1))
    return ((inputs[:, :dim] + means) * scale).astype(np.float32)


# ---------------------------------------------------------------------------
# Decode: query embedding + CTC greedy + BPE surface join
# ---------------------------------------------------------------------------

# embedding.npy row indices (upstream mapping; ja/ko are 11/12, not the
# sherpa token ids 5/6 used earlier)
LANGUAGES = {"auto": 0, "zh": 3, "en": 4, "yue": 7, "ja": 11, "ko": 12, "nospeech": 13}
TEXT_NORM_ITN = 14
TEXT_NORM_NONE = 15
EVENT_EMO_QUERY = (1, 2)
BLANK_ID = 0
FEATURE_DIM = 560


def build_encoder_input(embedding, speech, language, use_itn):
    """[lang(1), event+emo(2), text_norm(1), speech(T)] on the time axis,
    exactly like the upstream reference. Returns [1, T + 4, 560].
    """
    lang_query = embedding[[LANGUAGES[language]]].reshape(1, 1, -1)  # [1, 1, 560]
    event_emo_query = embedding[list(EVENT_EMO_QUERY)].reshape(
        1, len(EVENT_EMO_QUERY), -1
    )  # [1, 2, 560]
    norm_row = TEXT_NORM_ITN if use_itn else TEXT_NORM_NONE
    text_norm_query = embedding[[norm_row]].reshape(1, 1, -1)  # [1, 1, 560]
    return np.concatenate(
        [lang_query, event_emo_query, text_norm_query, speech[None, ...]],
        axis=1,
    ).astype(np.float32)


def ctc_greedy_ids(logits, vocab, layout, valid):
    """logits: flat float array of vocab*T values; layout selects the memory
    layout: 'TV' means [T, vocab] (our conversion keeps encoder_out as
    [1, 344, 25055]), 'VT' means [vocab, T] (the happyme531 conversion
    transposes it). `valid` limits decoding to the real (pre-pad) frames.
    Returns the greedy CTC id list.
    """
    if logits.size % vocab != 0:
        raise RuntimeError(f"output size {logits.size} not divisible by vocab {vocab}")
    t_out = logits.size // vocab
    if layout == "VT":
        per_frame = logits.reshape(vocab, t_out)[:, :valid].argmax(axis=0)
    else:
        per_frame = logits.reshape(t_out, vocab)[:valid].argmax(axis=1)
    keep = np.append([True], per_frame[1:] != per_frame[:-1])
    kept = per_frame[keep]
    return [int(i) for i in kept[kept != BLANK_ID]]


def ids_to_text(ids, tokens):
    # sentencepiece surface join: '_' marks a space; strip SenseVoice prompt
    # special tokens <|zh|><|NEUTRAL|><|Speech|><|woitn|> like the reference
    # runtime does.
    text = "".join(tokens[i].replace("\u2581", " ") for i in ids if 0 <= i < len(tokens))
    return re.sub(r"<\|[^|]*\|>", "", text).strip()


def load_tokens(path):
    """Load a sherpa-onnx tokens.txt: each line is '<surface> <id>.

    The id space matches the encoder output vocab directly and is the same
    table the .bpe.model encodes, but tokens.txt is trivially parseable and
    ships with the same model family (kept in assets/sensevoice/).
    """
    tokens = {}
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.rstrip("\n")
            if not line:
                continue
            surface, _, ident = line.rpartition(" ")
            try:
                tokens[int(ident)] = surface
            except ValueError:
                continue
    if not tokens:
        raise ValueError(f"no tokens parsed from {path}")
    return [tokens.get(i, "") for i in range(max(tokens) + 1)]


# ---------------------------------------------------------------------------
# main levels
# ---------------------------------------------------------------------------


def transcribe(ctx, embedding, tokens, cmvn, wav_path, language, use_itn, scale):
    samples = read_wav_mono(wav_path)
    feats = fbank80(samples)
    speech = apply_cmvn(apply_lfr(feats), cmvn)
    # RKNN2 fp16 inference can overflow when intermediate activations exceed
    # fp16 max; upstream scales the speech frames (queries stay unscaled).
    speech = (speech * scale).astype(np.float32)

    batch = build_encoder_input(embedding, speech, language, use_itn)
    valid = batch.shape[1]
    # The model input is fixed-length (our fp16-scaled build uses 344 frames,
    # upstream's uses 171); pad to the length from the input tensor attr.
    t_in = ctx.in_attr.n_elems // FEATURE_DIM
    if t_in <= 0 or ctx.in_attr.n_elems % FEATURE_DIM != 0:
        raise RuntimeError(
            f"input attr unusable: n_elems={ctx.in_attr.n_elems}"
        )
    if valid > t_in:
        batch = batch[:, :t_in]
        valid = t_in
    batch = np.pad(batch, ((0, 0), (0, t_in - batch.shape[1]), (0, 0)))

    if DEBUG:
        print(
            f"[dbg] speech={speech.shape} finite={np.isfinite(speech).all()} "
            f"min={speech.min():.3f} max={speech.max():.3f} "
            f"batch={batch.shape} valid={valid} "
            f"min={batch.min():.3f} max={batch.max():.3f}",
            file=sys.stderr,
        )

    logits = ctx.run(batch.reshape(1, -1))

    vocab = len(tokens)
    layout = os.environ.get("SENSEVOICE_OUT_LAYOUT", "TV")
    ids = ctc_greedy_ids(logits, vocab, layout, valid)
    if DEBUG:
        alt = "TV" if layout == "VT" else "VT"
        try:
            print(
                f"[dbg] layout={layout} ids={ids[:32]} "
                f"alt[{alt}]='{ids_to_text(ctc_greedy_ids(logits, vocab, alt, valid), tokens)[:80]}'",
                file=sys.stderr,
            )
        except RuntimeError:
            pass
    return ids_to_text(ids, tokens)


def voice_commands_from_text(text):
    normalized = re.sub(r"\s+", "", text)
    matches = []
    for command, phrases in VOICE_COMMANDS:
        for phrase in phrases:
            start = 0
            while True:
                index = normalized.find(phrase, start)
                if index == -1:
                    break
                matches.append((index, command))
                start = index + len(phrase)
    return [command for _, command in sorted(set(matches))]


def voice_command_from_text(text):
    commands = voice_commands_from_text(text)
    return commands[0] if commands else None


def send_voice_command(command):
    line = "@@RT " + command + "\n"
    console_path = os.environ.get("SENSEVOICE_COMMAND_CONSOLE", "/dev/console")
    try:
        with open(console_path, "w") as console:
            console.write(line)
            console.flush()
    except OSError:
        sys.stdout.write(line)
        sys.stdout.flush()


def run_voice_command_for_two_seconds(command):
    send_voice_command(command)
    if command != "stop":
        time.sleep(2)
        send_voice_command("stop")


def main():
    parser = argparse.ArgumentParser(description="SenseVoice RK3588 NPU ASR")
    parser.add_argument("--model-dir", default="/opt/sensevoice/model")
    parser.add_argument("--lib-dir", default="/opt/sensevoice/lib")
    parser.add_argument(
        "--model",
        default="sense-voice-encoder.rk3588.fp16-scaled.rknn",
        help="model file name inside --model-dir",
    )
    parser.add_argument("--wav", action="append", default=[])
    parser.add_argument("--language", default="auto")
    parser.add_argument("--no-itn", action="store_true")
    parser.add_argument(
        "--input-scale",
        type=float,
        default=float(os.environ.get("SENSEVOICE_INPUT_SCALE", "1.0")),
        help="speech feature scale (upstream uses 0.5 for its unscaled model)",
    )
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    model = os.path.join(args.model_dir, args.model)
    lib_paths = [
        os.path.join(args.lib_dir, "librknnrt.so"),
        "/usr/lib/librknnrt.so",
    ]

    # L1: missing model must fail with a diagnostic.
    if args.selftest and not os.path.exists(model):
        print(f"model not found: {model}", file=sys.stderr)
        return 1

    embedding = np.load(os.path.join(args.model_dir, "embedding.npy"))
    cmvn = load_cmvn(os.path.join(args.model_dir, "am.mvn"))
    tokens = load_tokens(os.path.join(args.model_dir, "tokens.txt"))

    started = time.time()
    ctx = RknnContext(model, lib_paths)
    print(f"[perf] model load: {time.time() - started:.2f}s")
    try:
        for wav_path in args.wav:
            started = time.time()
            text = transcribe(
                ctx, embedding, tokens, cmvn, wav_path, args.language,
                not args.no_itn, args.input_scale,
            )
            print(
                json.dumps(
                    {"wav": wav_path, "text": text, "seconds": time.time() - started},
                    ensure_ascii=False,
                )
            )
            for command in voice_commands_from_text(text):
                run_voice_command_for_two_seconds(command)
    finally:
        ctx.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
