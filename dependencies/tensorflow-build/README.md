# Build

First build container with `docker build -t local/tf-build .`
Then run a TensorFlow build with `docker run --rm -it  -v $PWD/out:/out -e HOST_PERMS="$(id -u):$(id -g)"  -e BUILD_OPTS="--config avx"   local/tf-build:latest` replacing the `BUILD_OPTS` with the desired parameters.

Build options can be:
* `avx` - AVX instruction set
* `avx2` - AVX2 instruction set
* `mkl` - Intel Math Kernel Library
* `xla` - Accelerated Linear Algebra

# Test

First build container with `docker build -t local/tf-test-container -f Dockerfile.test .`
Then run the test with `docker run --rm local/tf-test-container`

Output will show the compile-flag support and whether MKL is compiled in.