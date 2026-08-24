#!/usr/bin/env python3

"""
keyboard.py

Simple keyboard interface for changing controller references.

Controls:

s : enable controller
x : disable controller

a : move +Y
d : move -Y

w : increase altitude (+Z target)
z : decrease altitude (-Z target)

r : reset reference to current position

q : emergency stop request
"""


import threading
import time


class KeyboardController:

    def __init__(
        self,
        reference,
        vehicle_state,
    ):

        self.reference = reference
        self.vehicle_state = vehicle_state

        self.running = True

        self.enabled = False
        self.emergency_stop = False

        self.y_step = 0.05
        self.z_step = 0.05

        self.thread = threading.Thread(
            target=self._keyboard_loop,
            daemon=True,
        )

    ##################################################
    # Start
    ##################################################

    def start(self):

        self.thread.start()


    ##################################################
    # Keyboard loop
    ##################################################

    def _keyboard_loop(self):

        while self.running:

            try:

                key = input().strip().lower()

            except EOFError:

                break


            ##################################################
            # Enable
            ##################################################

            if key == "s":

                self.enabled = True

                print(
                    "Controller ENABLED"
                )


            ##################################################
            # Disable
            ##################################################

            elif key == "x":

                self.enabled = False

                print(
                    "Controller DISABLED"
                )


            ##################################################
            # Move +Y
            ##################################################

            elif key == "a":

                self.reference.y += self.y_step

                print(
                    f"Target Y = {self.reference.y:.3f}"
                )


            ##################################################
            # Move -Y
            ##################################################

            elif key == "d":

                self.reference.y -= self.y_step

                print(
                    f"Target Y = {self.reference.y:.3f}"
                )


            ##################################################
            # Increase altitude
            ##################################################

            elif key == "w":

                self.reference.z += self.z_step

                print(
                    f"Target Z = {self.reference.z:.3f}"
                )


            ##################################################
            # Decrease altitude
            ##################################################

            elif key == "z":

                self.reference.z -= self.z_step

                print(
                    f"Target Z = {self.reference.z:.3f}"
                )


            ##################################################
            # Reset reference
            ##################################################

            elif key == "r":

                state = self.vehicle_state.copy()

                self.reference.y = state.y
                self.reference.z = state.z

                self.reference.vy = 0.0
                self.reference.vz = 0.0

                print(
                    "Reference reset to current position"
                )


            ##################################################
            # Emergency stop
            ##################################################

            elif key == "q":

                self.emergency_stop = True

                print(
                    "EMERGENCY STOP REQUESTED"
                )


            time.sleep(0.01)


    ##################################################
    # Stop thread
    ##################################################

    def stop(self):

        self.running = False