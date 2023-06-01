# Caterpie Prime

This repository stores the source code of our solver *Caterpie Prime* for the Exact track of the [PACE 2023](https://pacechallenge.org/2023/) challenge.

### DOI of Version 1

TBD

### Solver Description

TBD

### Dependencies

- C++ compiler supporting the C++14 standard ([GCC](https://gcc.gnu.org/) recommended)
- [GNU Make](https://www.gnu.org/software/make/)
- [CMake](https://cmake.org/) Version 3.14 or later
- [CLI11](https://github.com/CLIUtils/CLI11)
- [The Kissat SAT Solver 3.0.0](https://github.com/arminbiere/kissat)

### How to build

In this directory, run the following command.

```
make build
```

And the executable file `exact-solver` will be generated under the `dist` directory.

Note: A submission file (`dist/exact-solver.tgz`) can be created by `make publish`.

