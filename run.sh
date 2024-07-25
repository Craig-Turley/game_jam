cd ./build_folder

make

# Check if the build was successful
if [ $? -eq 0 ]; then
	# Launch game
	./mygame
else
	echo "Build failed"
fi