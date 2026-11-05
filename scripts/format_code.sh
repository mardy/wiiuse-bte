#! /bin/sh

for file in $(git ls-files | grep '\.[hc]$')
do
    uncrustify -c .uncrustify.cfg --replace --no-backup "$file"
done
