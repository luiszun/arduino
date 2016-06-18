# This server will verify the passcode sent from the door pad.

# TODO: Count three consecutive errors and disable self
#       Send mail for failed attempts
#       Send replies to Locker
#       Sing Locker messages with a private key
#       Save persistent logs

import socket
import sys
import logging

PASSCODE_SIZE = 4
HARDCODED_PASSWORD = b"1234"
MSG_SUCCESS = b"SUCCESS"
MSG_FAIL    = b"FAIL"
SERVER_PORT = ("", 1333)

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
            sock.sendto(MSG_SUCCESS, addr);
        else:
            logging.debug("Passcode does not match. DENY access.")
            sock.sendto(MSG_FAIL, addr);
        sock.close()


