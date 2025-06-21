import struct
import matplotlib.pyplot as plt

def read_adxl_data(file_path):
    data = []
    with open(file_path, 'rb') as f:
        while True:
            bytes_read = f.read(6)  # 3 * int16_t = 6 bytes
            if len(bytes_read) < 6:
                break  # End of file
            x, y, z = struct.unpack('<hhh', bytes_read)  # little-endian signed shorts
            data.append((x, y, z))
    return data

def plot_data(data):
    xs, ys, zs = zip(*data)
    plt.figure(figsize=(10, 5))
    plt.plot(xs, label='X')
    plt.plot(ys, label='Y')
    plt.plot(zs, label='Z')
    plt.title('ADXL345 Accelerometer Data')
    plt.xlabel('Sample')
    plt.ylabel('Acceleration (raw)')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    file_path = "test.bin"  # change this to the path where you saved the file
    sensor_data = read_adxl_data(file_path)
    print(f"Read {len(sensor_data)} samples.")
    plot_data(sensor_data)
