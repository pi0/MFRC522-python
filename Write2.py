#!/usr/bin/env python
# -*- coding: utf8 -*-

import RPi.GPIO as GPIO
import MFRC522
import signal

continue_reading = True

# Capture SIGINT for cleanup when the script is aborted
def end_read(signal, frame):
    global continue_reading
    print "Ctrl+C captured, ending read."
    continue_reading = False
    GPIO.cleanup()

# Hook the SIGINT
signal.signal(signal.SIGINT, end_read)

# Create an object of the class MFRC522
rfid = MFRC522.MFRC522()


# This loop keeps checking for chips. If one is near it will get the UID and authenticate
while continue_reading:

    # Scan for cards    
    (status, TagType) = rfid.request(rfid.PICC_REQIDL)

    # If a card is found
    if status == rfid.MI_OK:
        print "Card detected"

    # Get the UID of the card
    (status, uid) = rfid.anti_coll()

    # If we have the UID, continue
    if status == rfid.MI_OK:

        # Print UID
        print "Card read UID: " + str(uid[0]) + "," + str(uid[1]) + "," + str(uid[2]) + "," + str(uid[3])

        # This is the default key for authentication
        key = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]

        # Select the scanned tag
        rfid.select_tag(uid)

        # Authenticate
        status = rfid.auth(rfid.PICC_AUTHENT1A, 8, key, uid)

        # Check if authenticated
        if status == rfid.MI_OK:

            use = rfid.read_integer(8)
            use += 1

            rfid.write_integer(8, use)

            print(rfid.read_integer(8))

            rfid.stop_crypto1()
            # continue_reading = False

        else:
            print "Authentication error"
