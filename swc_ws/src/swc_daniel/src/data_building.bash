minSpeed=1
speedP=1000
angleP=1

# the ${var} is used for concatenating. this is probably fine
vals="${MinSpeed},${speedP},${angleP}"

scriptResults="/Documents/Github/SCR-SWC-20/results.txt"
valfile="/Documents/Github/SCR-SWC-20/swc-ws/src/values.txt"

# will need this later. Can't use \d because bash doesn't support it. Shame.
regex="Time: [0-9]+\.*[0-9]*"

# > operator overwrites file, >> appends (StackO)
echo vals > valfile

# b/c otherwise it won't recognize roslaunch command
cd Documents/Github/SCR-SWC-20/swc_ws/devel
source ~/.bashrc

# get us back up
cd ..

# main loop (runs for a while, hopefully)
while true; do
	roslaunch swc_daniel swc_daniel.launch
	resultTime=$(grep -F "$regex" "/Downloads/SCR_SWC_20_SIM_5.0_WIN/results.txt")
	newvals="${vals},${resultTime}\n"
	echo newvals >> scriptResults
	
	# and loop!
	minSpeed=minSpeed+1
	speedP=speedP+100
	angleP=angleP+1
	vals="${MinSpeed},${speedP},${angleP}"
	echo vals > valfile
done