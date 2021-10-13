# add timestamp
Header header

# types of messages
uint8 BRDCST = 0
uint8 UNICST = 1

# raw message array
string msg

# id of the sender (0 - 255)
# -1 if unknown
int16 sender_id

# type of the message
uint8 msg_type
