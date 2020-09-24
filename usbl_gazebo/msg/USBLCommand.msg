# USBL command message (usually from transceiver) to command
# USBL trasnponder attached vehicle

# targetted trasnponder ID
int32 transponderID

# commandID indicated which command to send, listed in usblCommandId.hh
int32 commandID

# store command dependent payload
string data