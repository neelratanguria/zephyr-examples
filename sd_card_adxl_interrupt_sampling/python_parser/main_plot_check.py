import struct
import matplotlib.pyplot as plt

def read_adxl_data_with_dual_timestamp(file_path):
    data = []
    with open(file_path, 'rb') as f:
        while True:
            bytes_read = f.read(15)  # 4 (cycle) + 4 (uptime) + 6 (XYZ) + 1 (checksum)
            if len(bytes_read) < 15:
                break  # End of file

            cycle_ts, uptime_ts, x, y, z, checksum = struct.unpack('<IIhhhB', bytes_read)

            # Verify XOR checksum
            raw = bytes_read[:14]
            computed_checksum = 0
            for b in raw:
                computed_checksum ^= b

            if checksum != computed_checksum:
                print(f"Checksum mismatch at timestamp {uptime_ts}! Skipping sample.")
                continue

            data.append((cycle_ts, uptime_ts, x, y, z))
    return data

def plot_data(data):
    cycle_ts, uptime_ts, xs, ys, zs = zip(*data)

    plt.figure(figsize=(12, 6))
    plt.plot(uptime_ts, xs, label='X')
    plt.plot(uptime_ts, ys, label='Y')
    plt.plot(uptime_ts, zs, label='Z')
    plt.title('ADXL345 Accelerometer Data (with uptime timestamp)')
    plt.xlabel('Uptime (ms)')
    plt.ylabel('Acceleration (raw)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    file_path = "test_22.bin"  # Update this to your actual path
    sensor_data = read_adxl_data_with_dual_timestamp(file_path)
    print(f"Read {len(sensor_data)} valid samples.")
    plot_data(sensor_data)
