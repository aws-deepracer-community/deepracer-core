#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
        echo "Requested to stop."
        exit 1
}

PREFIX="awsdeepracercommunity"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
VERSION=$(cat $DIR/VERSION)

while getopts ":2cfognp:t:" opt; do
case $opt in
p) PREFIX="$OPTARG"
;;
\?) echo "Invalid option -$OPTARG" >&2
exit 1
;;
esac
done

docker push $PREFIX/deepracer-rlcoach:$VERSION
