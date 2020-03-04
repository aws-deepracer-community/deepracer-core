export MINIO_ACCESS_KEY=minio
export MINIO_SECRET_KEY=miniokey

mkdir ~/minio/bucket

docker run --rm -u `id -u` -d -p 9000:9000 -e "MINIO_ACCESS_KEY=$MINIO_ACCESS_KEY" -e "MINIO_SECRET_KEY=$MINIO_SECRET_KEY" --name minio1 \
  -v ~/minio:/data --name minio \
  minio/minio server /data
