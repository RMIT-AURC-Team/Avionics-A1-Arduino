import csv


def read_frame(data_bytes):

    if len(data_bytes) < 2:
        return None

    # Extract frame length and identifier
    # Mask to get lower 6 bits for frame length
    frame_header = data_bytes[0]
    frame_length = frame_header & 0x3F
    frame_data = data_bytes[2:frame_length-1]

    # Read sync value
    sync = data_bytes[1]

    # Read remaining frame data based on identifier
    if frame_header == 0x54:
        # Unpack sensor data (assuming signed integers)
        accelerometer = {
            'x': int.from_bytes(frame_data[0:2], byteorder='big', signed=True),
            'y': int.from_bytes(frame_data[2:4], byteorder='big', signed=True),
            'z': int.from_bytes(frame_data[4:6], byteorder='big', signed=True),
        }
        gyroscope = {
            'x': int.from_bytes(frame_data[6:8], byteorder='big', signed=True),
            'y': int.from_bytes(frame_data[8:10], byteorder='big', signed=True),
            'z': int.from_bytes(frame_data[10:12], byteorder='big', signed=True),
        }
        magnetometer = {
            'x': int.from_bytes(frame_data[12:14], byteorder='big', signed=True),
            'y': int.from_bytes(frame_data[14:16], byteorder='big', signed=True),
            'z': int.from_bytes(frame_data[16:18], byteorder='big', signed=True),
        }
        return {
            'frame_id': 'highres',
            'frame_length': frame_length,
            'sync': sync,
            'accelerometer': accelerometer,
            'gyroscope': gyroscope,
            'magnetometer': magnetometer,
        }

    elif frame_header == 0x88:
        pressure = int.from_bytes(
            frame_data[0:2], byteorder='big', signed=True)
        temperature = int.from_bytes(
            frame_data[3:5], byteorder='big', signed=True)
        return {
            'frame_id': 'lowres',
            'frame_length': frame_length,
            'sync': sync,
            'pressure': pressure,
            'temperature': temperature,
        }


# Open the data file in binary mode
with open("data.txt", "rb") as data_file:
    # Open separate CSV files for sensor and pressure/temperature data
    with open("data_highres.csv", "w") as highres_csv, open("data_lowres.csv", "w") as lowres_csv:
        # Write header rows for each CSV
        highres_writer = csv.writer(highres_csv)
        highres_writer.writerow([
            "sync",
            "accel_x", "accel_y", "accel_z",
            "gyro_x", "gyro_y", "gyro_z",
            "mag_x", "mag_y", "mag_z"
        ])
        lowres_writer = csv.writer(lowres_csv)
        lowres_writer.writerow([
            "sync", "pressure", "temperature"
        ])

        # Read all data bytes from the file
        data_bytes = bytearray(data_file.read())

        # Process the data stream by reading frames
        while len(data_bytes) > 0:
            frame = read_frame(data_bytes)
            # Write data to appropriate CSV based on frame type
            if frame and (frame['frame_id'] == 'highres'):
                accel_data = [frame['accelerometer'][axis]
                              for axis in ['x', 'y', 'z']]
                gyro_data = [frame['gyroscope'][axis]
                             for axis in ['x', 'y', 'z']]
                mag_data = [frame['magnetometer'][axis]
                            for axis in ['x', 'y', 'z']]
                highres_writer.writerow(
                    [frame['sync']] + accel_data + gyro_data + mag_data)
                data_bytes = data_bytes[frame['frame_length']:]
            elif frame and (frame['frame_id'] == 'lowres'):
                lowres_writer.writerow(
                    [frame['sync'], frame['pressure'], frame['temperature']])
                data_bytes = data_bytes[frame['frame_length']:]
            else:
                data_bytes = data_bytes[1:]
                continue
