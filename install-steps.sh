
cd sagemaker-containers/
python setup.py sdist
cp dist/sagemaker_containers* ../sagemaker-rl-container/
cd ..

cd sagemaker-tensorflow-container/
python setup.py sdist
docker build . -t local/sagemaker-tensorflow-container:nvidia -f docker/1.12.0/Dockerfile.gpu \
    --build-arg framework_installable=tensorflow_gpu-1.12.0-cp36-cp36m-manylinux1_x86_64.whl \
    --build-arg py_version=3 \
cd ..


#cd intel_coach/
#python setup.py sdist
#cp dist/* ../sagemaker-rl-container/
#cd ..

cd sagemaker-rl-container/
cp ../src/redis.conf .
docker build -t local/sagemaker-rl-container:nvidia -f coach/docker/1.0.0/Dockerfile.tf . \
    --build-arg sagemaker_containers=sagemaker_containers-2.6.5.dev0.tar.gz
rm sagemaker_containers=sagemaker_containers-2.6.5.dev0.tar.gz redis.conf
cd ..

#cd simulation/
#docker build . -f Robomaker-kinetic-debug.docker -t local/robomaker:core-debug
#docker build . -f Robomaker-kinetic-nvidia.docker -t local/robomaker:nvidia
#cd ..
#
#docker run --rm --name dr --env-file ./robomaker.env --network sagemaker-local -d -p 8080:5900 -it local/robomaker:nvidia
 
