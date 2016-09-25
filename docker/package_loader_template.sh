declare -a arr=({{.Array}})

for i in "${arr[@]}"
do
    echo "$i"
done
