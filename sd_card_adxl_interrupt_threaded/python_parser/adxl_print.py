import struct

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

if __name__ == "__main__":
    file_path = "test.bin"  # Change this to your file's path
    sensor_data = read_adxl_data(file_path)

    print(f"Total Samples: {len(sensor_data)}")
    print("X\tY\tZ")
    for i, (x, y, z) in enumerate(sensor_data):
        print(f"{x}\t{y}\t{z}")
