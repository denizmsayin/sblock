import os
import re
from sys import argv

import definitions as defs

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

    folder_defn = defs.to_header_guard(folder)
    defn_str = f'{defs.HEADER_GUARD_PROJECT_PREFIX}_{folder_defn}_HPP'

    with open(f'{folder}.hpp', 'w') as header_file:
        header_file.write(f'#ifndef {defn_str}\n')
        header_file.write(f'#define {defn_str}\n\n')
        for entry in os.scandir(folder):
            if entry.name.endswith('.hpp'):
                header_file.write(f'#include "{folder}/{entry.name}"\n')
        header_file.write('\n#endif\n')


