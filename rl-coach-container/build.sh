#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
        echo "Requested to stop."
        exit 1
}

PREFIX="awsdeepracercommunity"

while getopts ":a:fp:t:" opt; do
case $opt in
p) PREFIX="$OPTARG"
;;
f) OPT_NOCACHE="--no-cache"
;;
\?) echo "Invalid option -$OPTARG" >&2
exit 1
;;
esac
done

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
VERSION=$(cat $DIR/VERSION)

docker build . ${OPT_NOCACHE} -t $PREFIX/deepracer-rlcoach:${VERSION} -f $DIR/Dockerfile --build-arg IMG_VERSION=$VERSION
