# This server will verify the passcode sent from the door pad.

# TODO: Count three consecutive errors and disable self
#       Send mail for failed attempts
#       Send replies to Locker
#       Sing Locker messages with a private key
#       Save persistent logs

import socket
import sys
import logging
from time import sleep

PASSCODE_SIZE = 8
HARDCODED_PASSWORD = b"01230123"
MSG_SUCCESS = b"SUCCESS"
MSG_FAIL    = b"FAIL"
SERVER_PORT = ("", 1333)
REPLY_WAIT  = 1 #Wait time before reply 

logging.basicConfig(stream=sys.stdout, level=logging.DEBUG, format='[%(asctime)s] [%(levelname)s] : %(message)s')

logging.debug("Starting on %s:%s" % SERVER_PORT)

while True:
    # Wait for a connection
    logging.debug("Waiting for a passcode...")
    while True:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(SERVER_PORT)
        passcode, addr = sock.recvfrom(PASSCODE_SIZE)
        logging.debug("Connected by %s", addr)
        logging.debug("Received passcode %s", passcode)
        if not passcode: break
        if (passcode == HARDCODED_PASSWORD):
            logging.debug("Passcode matches! Grant access.")
            # Apparently if we reply too fast, the ESP8266 ignores it
            sleep(REPLY_WAIT)
            sock.sendto(MSG_SUCCESS, addr);
        else:
            logging.debug("Passcode does not match. DENY access.")
            # See ESP8266 comment
            sleep(REPLY_WAIT)
            sock.sendto(MSG_FAIL, addr);
        sock.close()


