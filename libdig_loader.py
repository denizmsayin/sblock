from ctypes import *

def load(so_path):

    libdig = CDLL(so_path)

    # use void * for DIG * and char * for any output array arguments
    # since we will be passing preallocated byte buffers as output arrays

    libdig.DIG_new.restype = c_void_p
    libdig.DIG_new.argtypes = [c_uint, c_ulong]

    # libdig.DIG_new_gent.restype = c_void_p
    # libdig.DIG_new_gent.argtypes = [c_uint, c_ulong, c_char_p]

    libdig.DIG_delete.argtypes = [c_void_p]

    libdig.DIG_init_batch.argtypes = [c_void_p, c_long, c_long]

    libdig.DIG_get_batch_states.argtypes = [c_void_p, c_char_p, c_char_p]

    libdig.DIG_has_more_neighbors.restype = c_bool
    libdig.DIG_has_more_neighbors.argtypes = [c_void_p]

    libdig.DIG_get_next_neighbors.argtypes = [c_void_p, c_char_p, c_char_p, c_char_p]

    return libdig
