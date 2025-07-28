import time
import struct
import serial
import logging

RESET_CMD = struct.pack("<I", 0x7FC00000)
START_TX_CMD = struct.pack("<I", 0x7FC00001)
STOP_TX_CMD = struct.pack("<I", 0x7FC00002)


class Serial_uC:
    """
    Handle communication with a microcontroller (uC) via its USART.
    """

    def __init__(
        self,
        port,
        baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        timeout=0.01,
        write_timeout=0,
    ):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
            timeout=timeout,
            write_timeout=write_timeout,
        )
        self.ep_done = False
        self.ser.write(START_TX_CMD)

    def read_state(self, retries=10):
        """
        Read and decode a 4-byte packet from the microcontroller.
        Retry up to `retries` times if no data is received.
        """
        for _ in range(retries):
            try:
                self.ser.reset_input_buffer()
                packet = self.ser.read(size=4)
                if len(packet) == 4:
                    self.ep_done, ph, th = self._decode_packet(packet)
                    return ph, th
            except serial.SerialException as e:
                logging.warning(f"Serial read failed: {e}. Trying again.")
                self.ser.write(START_TX_CMD)
        raise RuntimeError("Failed to read valid state from uC after multiple retries.")

    def write_action(self, action):
        """
        Send a float action to the uC.
        """
        try:
            self.ser.write(struct.pack("<f", action))
        except serial.SerialException as e:
            logging.error(f"Failed to write action: {e}")

    def reset(self, timeout=20.0):
        """
        Send RESET_CMD and wait for reset completion.
        True if reset succeeds, False on timeout.
        """
        self.read_state()
        self.ser.write(RESET_CMD)
        t_start = time.perf_counter()

        for trigger in [True, False]:
            while self.ep_done != trigger:
                if time.perf_counter() - t_start > timeout:
                    logging.warning(f"Timeout waiting for ep_done == {trigger}.")
                    return False
                self.read_state()
                time.sleep(0.1)

        return True

    def close(self):
        """
        Stop transmission and close the serial port.
        """
        self.ser.write(STOP_TX_CMD)
        self.ser.close()

    @staticmethod
    def _decode_packet(packet):
        """
        Decode a 32-bit packed state from the uC into:
        - done (bool)
        - phi (signed 12-bit int)
        - theta (signed 19-bit int)
        """
        value = int.from_bytes(packet, "little")

        done = not not (value & 0x1)

        # Extract and sign-extend 12-bit phi (bits 1-12)
        phi = (value >> 1) & 0xFFF
        if phi & 0x800:  # Sign bit
            phi -= 0x1000

        # Extract and sign-extend 19-bit theta (bits 13-31)
        theta = (value >> 13) & 0x7FFFF
        if theta & 0x40000:  # Sign bit
            theta -= 0x80000

        return done, phi, theta
