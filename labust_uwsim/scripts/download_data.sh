#!/bin/bash
URL=https://www.dropbox.com/s/6f8innn2ewoynzk/labust_uwsim.tar.bz2
DIRECTORY=~/.uwsim

if [ -d "$DIRECTORY" ]; then
	  # Control will enter here if $DIRECTORY exists.
	echo "Found UWSim directory at ${DIRECTORY}"
	echo "Downloading labust_uwsim data..."
	cd ${DIRECTORY}
 	wget -nc --no-check-certificate ${URL} 
	echo "Unpacking data into ${DIRECTORY}..."
	tar xvf `basename ${URL}`
	echo "Do you wish to remove the archive `basename ${URL}`"
	select yn in "Yes" "No"; do
		    case $yn in
					Yes ) rm `basename ${URL}`; break;;
					No ) break;;
			  esac
	done
else
	echo "You must download the UWSim data before the labust_uwsim data."
  echo "Do this by running: rosrun uwsim uwsim"
fi
