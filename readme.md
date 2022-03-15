# How to compile executable files?

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
