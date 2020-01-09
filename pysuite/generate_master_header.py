import os
import re
from sys import argv

import definitions as defs

def generate_master_header(path, indent=0):

    path = path.strip(os.sep)

    path_defn = defs.to_header_guard(path)
    defn_str = f'{defs.HEADER_GUARD_PROJECT_PREFIX}_{path_defn}_HPP'

    base_path = os.path.basename(path)
    p = os.path.join(os.path.dirname(path), f'{base_path}.hpp')

    indent_str = '  ' * indent
    print(indent_str + f'Generating {p}...')
    with open(p, 'w') as header_file:
        header_file.write(f'#ifndef {defn_str}\n')
        header_file.write(f'#define {defn_str}\n\n')
        for entry in os.scandir(path):
            if entry.is_dir(): # recurse downwards
                generate_master_header(entry.path, indent + 1)
        for entry in os.scandir(path):
            if entry.name.endswith('.hpp'):
                header_file.write(f'#include "{base_path}/{entry.name}"\n')
        header_file.write('\n#endif\n')
    print(indent_str + f'Done generating {p}.')


if __name__ == '__main__':
    
    if len(argv) != 2:
        print('Usage: python generate_master_header.py folder')
        print('  Will generate folder.hpp which includes all headers under folder/')
        exit(0)

    _, folder = argv

    generate_master_header(folder)

