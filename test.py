import os
import re

path = os.getcwd()
emerge_path = os.path.join(path, 'Emergence_results', 'normal')

files = os.listdir(emerge_path)

# expected setup
prefixes = ['E2', 'P1', 'P2']
types = ['anti', 'random', 'pro']
expected_seeds = set(range(42, 72))  # 42–71

# structure: data[prefix][type] = set(seeds_found)
data = {p: {t: set() for t in types} for p in prefixes}

# regex to extract info
pattern = re.compile(r'(E2|P1|P2)_(anti|random|pro).*_seed(\d+)')

# parse files
for f in files:
    match = pattern.match(f)
    if match:
        prefix, t, seed = match.groups()
        seed = int(seed)
        data[prefix][t].add(seed)

# check missing seeds
for p in prefixes:
    for t in types:
        found = data[p][t]
        missing = expected_seeds - found

        if missing:
            print(f"{p} | {t} missing seeds: {sorted(missing)}")
        else:
            print(f"{p} | {t} complete")