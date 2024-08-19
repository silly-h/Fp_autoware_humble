import socket
import serial

# TCP configuration
tcp_ip = "192.168.3.199"
tcp_port = 21004

# Serial port configuration
serial_port = "/dev/ttyTHS0"  # Replace with the actual serial port
serial_baudrate = 9600
serial_databits = serial.EIGHTBITS
serial_parity = serial.PARITY_NONE
serial_stopbits = serial.STOPBITS_ONE

# Function to receive TCP data and send it to serial port
def receive_and_send_data():
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((tcp_ip, tcp_port))
    ser_port = serial.Serial(
        port=serial_port,
        baudrate=serial_baudrate,
        # bytesize=serial_databits,
        # parity=serial_parity,
        # stopbits=serial_stopbits
    )
    
    while True:
        data = tcp_socket.recv(204800)
        if not data:
            print("\033[91mLidar GPRMC message empty\033[0m")  # Print in red color
        else:
            print(data.decode())  # Print received TCP message
            ser_port.write(data)
    
    tcp_socket.close()
    ser_port.close()

# Main program
receive_and_send_data()
