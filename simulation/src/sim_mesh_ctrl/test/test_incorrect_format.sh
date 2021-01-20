
data='{ blahblah'

echo "$data" | nc -u -w 1 localhost 14000
