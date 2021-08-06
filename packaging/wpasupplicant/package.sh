#!/bin/bash

pushd ../../wpa > /dev/null
params="-m $(realpath .) -c $(git rev-parse HEAD) -g $(git log --date=format:%Y%m%d --pretty=~dirty~git%cd.%h -n 1) -b 0"
./package_wpa.sh $params
popd

exit 0