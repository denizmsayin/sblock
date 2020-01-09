import re
import os

_CMAKELISTS_PATH = '/mnt/hdd/Documents/self/sblock/CMakeLists.txt'
_UNDERSCORE_REPLS = (' ', '-', '::', '.', os.sep)

def to_header_guard(name):
    name = name.strip('./')
    for r in _UNDERSCORE_REPLS:
        name = name.replace(r, '_')
    return name.upper()

def _get_cmake_project_prefix(cmakelists_path):
    pattern = re.compile(r'project\(([a-zA-Z_0-9]+)\)')
    with open(cmakelists_path, 'r') as cmf:
        for line in cmf:
            match = pattern.search(line)
            if match:
                prefix = match.groups()[0]
                return prefix
    raise RuntimeError(f'Could not find project prefix in {cmakelists_path}')

USERNAME = 'denizmsayin'
PROJECT_PREFIX = USERNAME + '-' + _get_cmake_project_prefix(_CMAKELISTS_PATH)
HEADER_GUARD_PROJECT_PREFIX = to_header_guard(PROJECT_PREFIX)

