#!/bin/sh

scriptstart=$(date +%s)

echo saving changes to generate files
git restore --staged .
git add generate.py generate.sh wpiformat-files.txt
git commit --amend --no-edit --no-verify > /dev/null

echo restoring old files
git restore --source $(git merge-base 2027 HEAD) .
git restore generate.py generate.sh wpiformat-files.txt
git add .

echo running updated generation
echo '  ntcore'
git restore --source HEAD ntcore/src/generate
python3 ntcore/generate_topics.py
git restore ntcore/src/generate/types.json
git restore ntcore/src/generate/main

echo '  wpilibNewCommands'
git restore --source HEAD wpilibNewCommands/generate_hids.py wpilibNewCommands/src/generate
python3 wpilibNewCommands/generate_hids.py
git restore wpilibNewCommands/generate_hids.py wpilibNewCommands/src/generate

echo '  wpilibc'
git restore --source HEAD wpilibc/generate_hids.py wpilibc/src/generate
python3 wpilibc/generate_hids.py
python3 wpilibc/generate_pwm_motor_controllers.py
git restore wpilibc/generate_hids.py wpilibc/src/generate
git restore wpilibc/src/generate/main/native/cpp/motorcontroller

echo '  wpilibj'
git restore --source HEAD wpilibj/generate_hids.py wpilibj/src/generate
python3 wpilibj/generate_hids.py
git restore wpilibj/generate_hids.py wpilibj/src/generate

printf '  %s changed files\n' $(git diff --name-only | wc -l)

echo running replacements
python3 generate.py || exit 1

echo applying c++ format
start=$(date +%s)
wpiformat -f $(cat wpiformat-files.txt) 2> /dev/null
end=$(date +%s)
printf '  Done formatting %s files in around %s seconds\n' $(cat wpiformat-files.txt | wc -l)  $((end - start))

echo applying java format
start=$(date +%s)
./gradlew cscore:spotlessApply wpilibj:spotlessApply wpimath:spotlessApply wpilibjExamples:spotlessApply > /dev/null
end=$(date +%s)
printf '  Done applying java format in around %s seconds\n' $((end - start))

printf '%s total changed files\n' $(git diff --name-only | wc -l)

echo inverting changes
git add .
git restore --source HEAD .

scriptend=$(date +%s)

printf 'done in around %s seconds- %s files remaining\n' $((scriptend - scriptstart)) $(git diff --name-only | wc -l)
