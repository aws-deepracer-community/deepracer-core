export MINIO_ACCESS_KEY=minio
export MINIO_SECRET_KEY=miniokey

docker run --rm -d -p 9000:9000 -e "MINIO_ACCESS_KEY=$MINIO_ACCESS_KEY" -e "MINIO_SECRET_KEY=$MINIO_SECRET_KEY" --name minio1 \
  -v /home/deepracer/minio:/data \
  minio/minio server /data
