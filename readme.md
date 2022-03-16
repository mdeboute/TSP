# Traveling Salesman Problem

## How to compile executable files?

```shell
mkdir -p build
cd build
cmake ..
make
```

## How to run executable files?

In the build directory:

For the MTZ model:

```shell
./mtz <PATH_TO_DAT_FILE>
```

For the flot model:

```shell
./flot <PATH_TO_DAT_FILE>
```

etc...

PS: for each model/executable file, you have the `-nv` (non-verbose) option which will just print the final result of the program on the terminal.

## How to run the tests?

In the project directory:

```shell
chmod u+x benchmark.sh
./benchmark.sh <INSTANCES_DIR> <SOLUTION_DIR> <MODEL>
```

Where `<MODEL>` is the name of the corresponding cpp model file.
