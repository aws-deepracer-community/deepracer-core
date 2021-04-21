# Set $BUILD_OPTS and $HOST_PERMS with the -e option
bazel build $BUILD_OPTS //tensorflow/tools/pip_package:build_pip_package
./bazel-bin/tensorflow/tools/pip_package/build_pip_package /out
chown $HOST_PERMS /out/*.whl
