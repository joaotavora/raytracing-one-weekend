## Build

```sh
mkdir build-debug
cd build-debug
conan install --build=missing ../
# inside hello-world/build
CXX=clang++ cmake -DCMAKE_BUILD_TYPE=Release ../
make
```

## Run

```sh
build-debug/bin/rtweekend > ~/tmp/test.ppm
feh test.ppm
```

## Enable use of a LSP language server like `clangd`

```sh
ln -sf build-debug/compile_commands.json
```
