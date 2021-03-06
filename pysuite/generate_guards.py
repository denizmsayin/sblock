import os
import re
from sys import argv
from tempfile import TemporaryFile

import definitions as defs

def _skip_until(file_handle, string):
    for line in file_handle:
        lstr = line.strip()
        if lstr != '':
            if lstr.startswith(string):
                return True, lstr
            else:
                print(f'Header file ({file_handle.name}) contains non-whitespace characters'
                      f' before guard {string}, skipping.')
                return False, lstr 
    return False, lstr


def skip_guards(hf_handle):
    skipped, line = _skip_until(hf_handle, '#ifndef')
    if skipped == True:
        _skip_until(hf_handle, '#define')
    else:
        return line
        

if __name__ == '__main__':
    if len(argv) == 1:
        print('Usage: python generate_guards.py header_file_1 header_file_2 ... header_file_n')
        exit(0)

    for header_file in argv[1:]:
        print(f'File: {header_file}')
        defn_str = defs.HEADER_GUARD_PROJECT_PREFIX + '_' + defs.to_header_guard(header_file)
        with TemporaryFile(mode='w+') as tmpf:
            with open(header_file, 'r') as hf:
                tmpf.write(f'#ifndef {defn_str}\n#define {defn_str}\n') # write auto guards
                last_line = skip_guards(hf) # skip #ifndef & #define
                if last_line: # no guards, do not forget last readline
                    tmpf.write(last_line)
                for line in hf: # copy the rest of the header file
                    tmpf.write(line)
                    lstr = line.strip()
                    if lstr != '':
                        last_line = lstr
                if not last_line.startswith('#endif'): # no endif in original header
                    tmpf.write('#endif\n')
            
            # ideally, I should have a context manager that keeps track of nested
            # ifs and endifs, but that is a bit bothersome to code

            # copy the temporary file into the header file
            tmpf.seek(0) # reset tmpf pointer
            with open(header_file, 'w') as hf:
                for line in tmpf:
                    hf.write(line)

