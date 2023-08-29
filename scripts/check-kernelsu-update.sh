#!/bin/bash

git config --local user.email "41898282+github-actions[bot]@users.noreply.github.com"
git config --local user.name "github-actions[bot]"
git submodule update --init
pushd KernelSU
LOCAL_LATEST=$(git describe --tags --abbrev=0)
git pull origin main
REMOTE_LATEST=$(git describe --tags --abbrev=0)
if [ $LOCAL_LATEST = $REMOTE_LATEST ]; then
echo "No changes: Local tag $LOCAL_LATEST is latest"
exit 0;
fi
git checkout $REMOTE_LATEST
popd
git add KernelSU
git commit -m "KernelSU: Update release tag to $REMOTE_LATEST" -m "Update the tag from $LOCAL_LATEST to $REMOTE_LATEST"
git push
