# sblock
sblock is a small library/set of tools in C++ created with the aim of solving the 8, 15 and 24 puzzles

### Building the Project
sblock is built using cmake. After cloning the repository, simply create a build directory and build the project:

```
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DWITH_TORCH=ON -DCMAKE_PREFIX_PATH=<path-to-libtorch> ..
cmake --build .
```

Note that by default, sblock is built with support for TorchScript machine learning models. This requires the pre-compiled C++ front-end libraries of PyTorch. Details on their installation can be found [here](https://pytorch.org/cppdocs/installing.html).

To build sblock without support for TorchScript ML models, simpy unset the option:

```
cmake -DCMAKE_BUILD_TYPE=Release -DWITH_TORCH=OFF ..
```

### Executables and Libraries

Once the build is complete, you will notice a few executables and shared library files. Note that some of them contain multiple variants with postfixes in the form HxW. This is because the height and width of the puzzles are provided with compile-time templates to maximize performance. Thus, different files are generated for different puzzle sizes. The files are as follows:

+ `solver_HxW`: This is the main executable for solving puzzles. It can generate puzzles within given distribution parameters or use an input file to read puzzles from. The search method and heuristic can be selected via the command line, and an output file can also be provided. Can also be sped up via multithreading. Providing the `-?` argument shows a help message. Example: 

   `solver_3x3 -n 1000 -s a* -h cpdb -f pdb3x3.db -o out_file.dat`: solve one thousand 3x3 puzzles generated from the default distribution using the default seed, and solve them with A\* search using the augmented disjoint pattern database heuristic with the database file being `pdb3x3.db`; store the results in `out_file.dat`.

+ `generate_dpdb_HxW`: Used to generate disjoint pattern databases for the puzzles, which can then be used by the solver. Example:
   `generate_dpdb_3x3 0,0,0,0,1,1,1,1,X pdb3x3.db`: Generate a disjoint pattern database with two  groups, having tiles 0, 1, 2 and 3 in the first group and tiles 4, 5, 6 and 7 in the second group. Store the database in file `pdb3x3.db`.

+ `libdig_HxW.so`: A compiled shared library with a C interface used for generating random puzzles for training reinforcement learning models with PyTorch via the `ctypes` interface. 

Other executables can be generated if the `-DWITH_TESTS=ON` is passed to cmake, but those executables are meant for testing purposes only.
