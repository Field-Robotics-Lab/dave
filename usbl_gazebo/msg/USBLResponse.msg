# USBL response message for message receivers

# the targetted transceiver ID
int32 transceverID

# responseID indicates the response message type, which can be
# used to interpret the data. ID can be found in usblCommandId.hh
int32 responseID

# store the payload of the message
string data