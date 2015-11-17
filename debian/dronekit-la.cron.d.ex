#
# Regular cron jobs for the dronekit-la package
#
0 4	* * *	root	[ -x /usr/bin/dronekit-la_maintenance ] && /usr/bin/dronekit-la_maintenance
