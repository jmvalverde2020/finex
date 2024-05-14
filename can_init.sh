sudo slcand -o -c -s8 /dev/ttyACM0 can0

sudo ifconfig can0 txqueuelen 65536

sudo ifconfig can0 up
# ip -details link show can0
