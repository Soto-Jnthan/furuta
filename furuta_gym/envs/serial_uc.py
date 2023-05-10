import serial
import time

USART_SOS = 'A' # Start of string for RxData
USART_EOS = '\r' # End of string for TxData
RESET_CHAR = '*' # Character used to start the reset sequence
START_TX_CHAR = '#' # Character used to start the state transmission
STOP_TX_CHAR = '$' # Character used to stop the state transmission

RESET_CMD = bytes(RESET_CHAR + USART_EOS, 'Ascii')
START_TX_CMD = bytes(START_TX_CHAR + USART_EOS, 'Ascii')
STOP_TX_CMD = bytes(STOP_TX_CHAR + USART_EOS, 'Ascii')

class Serial_uC():
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, 8, stopbits=1, timeout=0.01, writeTimeout=0)
        self.ep_done = False
        self.round_sgnf = lambda x,n: float(f'%.{n - 1}e' % x)  # Rounds x to 'n' significant digits

    def read_state(self):
        data = []
        self.ser.reset_input_buffer()
        while len(data) != 3: # Wait until phi, theta & ep_done have been received
            self.ser.write(START_TX_CMD)
            try:
                input_str = self.ser.readline().decode('Ascii')
                if input_str[0] == USART_SOS:
                    data = [int(i) for i in input_str[1:].split(' ')]
            except Exception: # Allows KeyboardInterrupts to be raised
                pass
        self.ser.write(STOP_TX_CMD)
        self.ep_done = not not data[2]  # 'not not' faster than 'bool()'
        return data[:2]

    def write_speed(self, speed):
        speed = self.round_sgnf(speed, 6)
        self.ser.write(bytes(str(speed) + USART_EOS, 'Ascii'))
        
    def reset(self, timeout=10):
        self.read_state()
        self.ser.write(RESET_CMD)
        t_start = time.perf_counter()
        while not self.ep_done: # If needed, wait for the uC to raise the ep_done flag
            self.read_state()
            if time.perf_counter() - t_start > timeout:
                return False
        while self.ep_done: # Wait for the uC to finish the reset sequence
            self.read_state()
            if time.perf_counter() - t_start > timeout:
                return False
        return True 

    def close(self):
        self.ser.close()