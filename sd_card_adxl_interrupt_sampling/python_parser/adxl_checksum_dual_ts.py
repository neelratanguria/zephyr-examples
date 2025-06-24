import struct
import csv

def calculate_checksum(bytes_slice):
    checksum = 0
    for b in bytes_slice:
        checksum ^= b
    return checksum

def read_with_checksum(file_path):
    data = []
    delta_uptime = []
    delta_cycles = []
    invalid_checksums = 0

    with open(file_path, 'rb') as f:
        prev_uptime = None
        prev_cycles = None

        while True:
            bytes_read = f.read(15)  # 14 data bytes + 1 checksum
            if len(bytes_read) < 15:
                break

            data_bytes = bytes_read[:14]
            checksum_byte = bytes_read[14]
            computed_checksum = calculate_checksum(data_bytes)

            if checksum_byte != computed_checksum:
                print("⚠️  Checksum mismatch detected!")
                invalid_checksums += 1
                continue  # Optionally skip this sample

            cycles, uptime, x, y, z = struct.unpack('<IIhhh', data_bytes)
            data.append((cycles, uptime, x, y, z))

            # Calculate deltas
            delta_uptime.append(0 if prev_uptime is None else uptime - prev_uptime)
            delta_cycles.append(0 if prev_cycles is None else cycles - prev_cycles)

            prev_uptime = uptime
            prev_cycles = cycles

    return data, delta_uptime, delta_cycles, invalid_checksums

def write_to_csv(csv_path, data, delta_uptime, delta_cycles):
    with open(csv_path, mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['Cycle Timestamp', 'Uptime (ms)', 'Delta Uptime (ms)', 'Delta Cycles', 'X', 'Y', 'Z'])

        for (cycles, uptime, x, y, z), d_uptime, d_cycles in zip(data, delta_uptime, delta_cycles):
            writer.writerow([cycles, uptime, d_uptime, d_cycles, x, y, z])

if __name__ == "__main__":
    file_path = "test_22.bin"  # Your binary input
    output_csv = "output_data.csv"

    sensor_data, deltas_uptime, deltas_cycles, bad_checks = read_with_checksum(file_path)
    write_to_csv(output_csv, sensor_data, deltas_uptime, deltas_cycles)

    print(f"✅ Saved {len(sensor_data)} valid samples to {output_csv}")
    if bad_checks > 0:
        print(f"⚠️  Detected {bad_checks} samples with invalid checksum")
