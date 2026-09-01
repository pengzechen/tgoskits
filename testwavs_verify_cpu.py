#!/usr/bin/env python3
# CPU FunASR verify test.wav/en.wav actual content
import sys, os, re

WAV_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'apps', 'starry', 'sensevoice-rknn', 'testwavs')

def strip_tags(text):
    return re.sub(r'<\|[^|]*\|>', '', text).strip()

def main():
    from funasr import AutoModel
    print('[info] loading SenseVoiceSmall ONNX (CPU)...')
    model = AutoModel(model='iic/SenseVoiceSmall', trust_remote_code=True, disable_update=True, use_onnx=True)
    print('[info] model loaded')
    for name in ['test.wav', 'en.wav']:
        path = os.path.join(WAV_DIR, name)
        if not os.path.exists(path):
            print('[skip] ' + name + ' not found')
            continue
        res = model.generate(input=path, language='auto', use_itn=True)
        text = res[0].get('text', '') if res else ''
        clean = strip_tags(text)
        print(name + ' -> ' + clean)
        print(name + ' raw -> ' + text)
    print('done')

if __name__ == '__main__':
    main()
