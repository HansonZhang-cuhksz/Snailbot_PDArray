import serial
import struct
import matplotlib.pyplot as plt
import time

import numpy as np
from numpy.fft import fft, fftfreq

# Define the struct format:
# '<' for little-endian.
# 'B' for uint8_t, 'H' for uint16_t, 'I' for uint32_t.
# 2 uint8_t headers, 400 uint16_t values, and a uint32_t checksum.
SAMPLE_COUNT = 10000
PACKET_FORMAT = '<BB10000HI'
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

def read_valid_packet(ser):
    """Continuously read from the serial port until a valid packet is found."""
    while True:
        # Read one byte to find the first header (0xEE)
        first_byte = ser.read(1)
        if not first_byte:
            continue

        if first_byte[0] == 0xEE:
            # Read the second byte; expecting 0x99 for header2.
            second_byte = ser.read(1)
            if not second_byte:
                continue

            if second_byte[0] == 0x99:
                # We have detected the proper header sequence.
                # Already read 2 bytes, now read the remaining bytes.
                remaining_bytes = ser.read(PACKET_SIZE - 2)
                if len(remaining_bytes) == (PACKET_SIZE - 2):
                    packet = first_byte + second_byte + remaining_bytes
                    return packet
                else:
                    # Not enough bytes received; flush and try again.
                    ser.reset_input_buffer()

def display_data(data):
    plt.figure()
    plt.plot(data, marker='o', linestyle='-')
    plt.xlabel("Index (x)")
    plt.ylabel("Data[x] value")
    plt.title("Serial Data Plot")
    plt.show()

def compute_fft(data, sampling_rate):
    # Compute FFT
    fft_values = fft(data)
    # Get frequency bins
    freqs = fftfreq(len(data), d=1/sampling_rate)
    return freqs, fft_values

def plot_fft(freqs, fft_values):
    plt.figure()
    # Only plot the positive half of frequencies
    half = len(freqs) // 2
    plt.plot(freqs[:half], np.abs(fft_values[:half]))
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Amplitude")
    plt.title("FFT of the Received Signal")
    plt.show()

sampling_rate = 1000000

def main():
    # Update port and baudrate according to your STM32 configuration.
    # port = '/dev/ttyS3'
    port = 'COM3'
    baudrate = 115200

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print("Serial port opened:", port)
    except Exception as e:
        print("Failed to open serial port:", e)
        return
    
    # Enable interactive mode.
    plt.ion()
    fig, ax = plt.subplots()
    # Initialize the plot with dummy data.
    x = list(range(SAMPLE_COUNT))
    y = [0] * SAMPLE_COUNT
    line, = ax.plot(x, y, marker='o', linestyle='-')
    ax.set_xlabel("Index (x)")
    ax.set_ylabel("Data[x] value")
    ax.set_title("Serial Data Plot")
    fig.canvas.draw()
    fig.canvas.flush_events()

    # Setup realtime plot for FFT
    fig2, ax2 = plt.subplots()
    fft_line, = ax2.plot([], [])
    ax2.set_xlabel("Frequency (Hz)")
    ax2.set_ylabel("Amplitude")
    ax2.set_title("FFT of Received Signal")

    print("Waiting for a valid packet...")

    while True:

        # Read a valid packet.
        packet = read_valid_packet(ser)
        
        # Unpack the packet.
        unpacked = struct.unpack(PACKET_FORMAT, packet)
        header, header2 = unpacked[0], unpacked[1]
        data = unpacked[2:2+SAMPLE_COUNT]  # Slice of 400 uint16_t values.
        checksum = unpacked[-1]

        # Update the plot with new data.
        line.set_ydata(data)
        # Optionally update the x-axis limits if necessary.
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.5)  # A brief pause to allow UI update

        # freqs, fft_values = compute_fft(data, sampling_rate)
        # plot_fft(freqs, fft_values)

        # Compute and update FFT plot
        freqs, fft_values = compute_fft(data, sampling_rate)
        half = len(freqs) // 2
        fft_line.set_xdata(freqs[:half])
        fft_line.set_ydata(np.abs(fft_values[:half]))
        ax2.relim()
        ax2.autoscale_view()
        fig2.canvas.draw()
        fig2.canvas.flush_events()

        # Display the packet information.
        print(f"Header: 0x{header:02X}, 0x{header2:02X}")
        # print("First 10 data values:", data[:10])
        print("DSP Data:", data)
        print(f"Checksum (unused): 0x{checksum:08X}")

        # time.sleep(0.5)

        # # Visualize the data.
        # display_data(data)

    ser.close()

if __name__ == '__main__':
    main()