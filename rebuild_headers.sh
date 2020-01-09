
PROOT="/mnt/hdd/Documents/self/sblock"
PYS="$PROOT/pysuite"

echo "Building master headers..."

for folder in utils search sbpuzzle; do
    eval "python "$PYS"/generate_master_header.py "$folder""
done

echo "**************************"
echo "Generating header guards.."

HEADERS=$(find . -name '*.hpp')
eval "python "$PYS"/generate_guards.py "$HEADERS""

echo "**************************"
echo "All done!"
