#!/bin/sh

readonly skip_post_flag="--skip-post"

if [ "$1" ]; then
  if [ "$1" != $skip_post_flag ]; then
    echo Unknown argument "$1"!
    exit 11
  fi
fi

file_paths=$(cat translate_units_paths)

start=$(date +%s)
echo "Restoring files and applying pre-patch"
git restore --source 2027 -- $file_paths
git apply translate_units_pre.patch
end=$(date +%s)
printf "  Done in %s seconds\n" $((end - start))

./translate_units.py $file_paths

git restore --source HEAD -- wpimath/src/main/native/include/wpi/units.hpp
git restore --source HEAD -- wpimath/src/main/native/include/wpi/units-usc.hpp
git restore --source HEAD -- wpimath/src/main/native/thirdparty
git restore --source HEAD -- wpimath/src/test/native/cpp/MpUnitsTest.cpp

start=$(date +%s)
echo "Running format"
wpiformat -f $(cat translate_units_format_files)
end=$(date +%s)
printf "  Done formatting %s files in %s seconds\n" $(cat translate_units_format_files | wc -l) $((end - start))

if [ "$1" != $skip_post_flag ]; then
  start=$(date +%s)
  echo "Applying post-patch"
  git apply translate_units_post.patch
  end=$(date +%s)
  printf "  Done in %s seconds\n" $((end - start))
fi

# start=$(date +%s)
# echo "Inverting changes"
# git add $file_paths
# git restore --source HEAD $file_paths
# end=$(date +%s)
# printf "  Done in %s seconds\n" $((end - start))
# 
# remaining_files=$(git diff --name-only -- $file_paths | wc -l)
# 
# if [ $remaining_files = 0 ]; then
#   echo "No remaining changes!"
# else
#   printf "%s files have remaining changes\n" $(git diff --name-only -- $file_paths | wc -l)
#   exit 1
# fi
