#!/bin/bash

end = $((SECONDS+5))

while [ $SECONDS - lt $end ]
do
	sleep 1

	if [! -e /dev/RS485] || [! -e /dev/ZED] || [! -e /dev/NANO]
	then
		/bin/bash udevadm trigger
	fi
done
