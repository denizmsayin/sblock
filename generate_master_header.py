import os
import re
from sys import argv

def get_project_prefix():
    pattern = re.compile(r'project\(([a-zA-Z]+)\)')
    with open('CMakeLists.txt', 'r') as cmf:
        for line in cmf:
            match = pattern.search(line)
            if match:
                prefix = match.groups()[0]
                return prefix.upper()
    raise RuntimeError('Could not find project prefix')

PROJECT_PREFIX = get_project_prefix()

if __name__ == '__main__':
    
    if len(argv) != 2:
        print('Usage: python generate_master_header.py folder')
        print('  Will generate folder.hpp which includes all headers under folder/')
        exit(0)

    _, folder = argv

    folder = folder.strip(os.sep)

    if os.sep in folder:
        print('Script does not work with nested folders')
        exit(0)

    folder_defn = folder.replace(' ', '_').replace('-', '_').upper()
    defn_str = f'__{PROJECT_PREFIX}_{folder_defn}_HPP__'

    with open(f'{folder}.hpp', 'w') as header_file:
        header_file.write(f'#ifndef {defn_str}\n')
        header_file.write(f'#define {defn_str}\n\n')
        for entry in os.scandir(folder):
            if entry.name.endswith('.hpp'):
                header_file.write(f'#include "{folder}/{entry.name}"\n')
        header_file.write('\n#endif\n')


