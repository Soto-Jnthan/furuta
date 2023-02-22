import serial
import numpy as np
import time

class Serial_uC():
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, 8, timeout=0.01, stopbits=serial.STOPBITS_ONE)
        
        self.TX_TIMEOUT = 0.2 #s
        self.RESET_TIMEOUT = 1.5 #s
        self.RESET_CHAR = '*' # Character used to start the reset sequence
        self.START_TX_CHAR = '#' # Character used to start the state transmission
        self.STOP_TX_CHAR = '$' # Character used to stop the state transmission
        self.USART_SOS = 'A' # Start of string
        self.USART_EOS = '\r' # End of string
        
        self.RESET_CMD = bytes(self.RESET_CHAR + self.USART_EOS, 'Ascii')
        self.START_TX_CMD = bytes(self.START_TX_CHAR + self.USART_EOS, 'Ascii')
        self.STOP_TX_CMD = bytes(self.STOP_TX_CHAR + self.USART_EOS, 'Ascii')

    def read_state(self):
        data = []
        received = False
        t_start = time.perf_counter()
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.write(self.START_TX_CMD)
        while not received:
            try:
                input_str = self.ser.readline().decode('Ascii')
                if input_str[0] == self.USART_SOS:
                    data = input_str[1:].split(' ')
                    data = [int(i) for i in data]
                    if len(data) == 4:
                        received = True
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            except:
                data = []
            if self.TX_TIMEOUT < time.perf_counter() - t_start:
                self.ser.write(self.START_TX_CMD)
                t_start = time.perf_counter()
        self.ser.write(self.STOP_TX_CMD)
        return data

    def write_speed(self, speed):
        speed = int(np.round(speed))
        self.ser.write(bytes(str(speed) + self.USART_EOS, 'Ascii'))
        
    def reset(self):
        self.ser.write(self.RESET_CMD)
        t_start = time.perf_counter()
        while bool(self.read_state()[3]) is False: #If needed, wait for the uC to raise the ep_done flag 
            if self.RESET_TIMEOUT < time.perf_counter() - t_start:
                self.ser.write(self.RESET_CMD)
                t_start = time.perf_counter()
        while bool(self.read_state()[3]) is True: #Wait for the uC to finish the reset sequence
            if self.RESET_TIMEOUT < time.perf_counter() - t_start:
                self.ser.write(self.RESET_CMD)
                t_start = time.perf_counter()
    
    def close(self):
        self.ser.close()