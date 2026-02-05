#!/bin/sh

git restore --source HEAD -- $(cat rename_constants_paths)
python3 rename_constants.py $(cat rename_constants_paths)
git apply rename_constants_update_ntcore_jinja.patch
