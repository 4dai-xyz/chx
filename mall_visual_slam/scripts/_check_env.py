import os

cache = '/home/ros/.cache/torch_extensions'
print('torch_extensions exists:', os.path.isdir(cache))
if os.path.isdir(cache):
    for root, dirs, files in os.walk(cache):
        depth = root[len(cache):].count('/')
        if depth > 3:
            continue
        sos = [f for f in files if f.endswith('.so')]
        print(' ', root, 'files:', len(files), 'so:', sos[:3])

print('=== meminfo ===')
with open('/proc/meminfo') as f:
    for i, line in enumerate(f):
        if i < 5:
            print(line.rstrip())
print('cpu count:', os.cpu_count())
