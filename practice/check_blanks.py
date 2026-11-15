#!/usr/bin/env python3
from pathlib import Path
root = Path(__file__).resolve().parent
total = 0
for p in sorted(root.glob('*.cpp')):
    lines = p.read_text().splitlines()
    left = [(i+1, l.strip()) for i, l in enumerate(lines) if '______' in l and not l.strip().startswith('//')]
    total += len(left)
    print(f'{p.name}: 还剩 {len(left)} 个空')
    for n, l in left:
        print(f'  {n}: {l[:90]}')
print(f'\n总剩余：{total}')
print('全部填完！' if total == 0 else '继续填，答案在 PPT。')
