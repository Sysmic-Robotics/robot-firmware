import struct
import pandas as pd

input_csv = "stm32_monitor_log.csv"
output_csv = "telemetry_decoded.csv"

# Formato Little-Endian (<) según ARM Cortex-M
STRUCT_FORMAT = "<BHfffffffB"
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)  # 32 bytes


def unpack_telemetry(raw_bytes):
    if len(raw_bytes) != 32:
        return None

    unpacked = struct.unpack(STRUCT_FORMAT, raw_bytes)
    flags = unpacked[0]

    return {
        "robot_id": (flags >> 5) & 0x07,
        "ball_detected": bool((flags >> 4) & 0x01),
        "kicker_charged": bool((flags >> 3) & 0x01),
        "saturations": flags & 0x07,
        "ball_range": unpacked[1],
        "x_est": unpacked[2],
        "y_est": unpacked[3],
        "vx_est": unpacked[4],
        "vy_est": unpacked[5],
        "theta_est": unpacked[6],
        "omega_est": unpacked[7],
        "gyro_z": unpacked[8],
        "ang_acc_limit": unpacked[9] / 10.0,
    }


def process_cube_monitor_log(file_path):
    df = pd.read_csv(file_path)
    decoded_rows = []

    for index, row in df.iterrows():
        try:
            # Casteo explícito int(...) para evitar TypeError con numpy.float64
            raw_bytes = bytes([int(row[f"telemetry_buf[{i}]"]) for i in range(32)])
            decoded_data = unpack_telemetry(raw_bytes)

            if decoded_data:
                # Mapear la columna de tiempo 'time' de STM32CubeMonitor
                if "time" in row:
                    decoded_data["time"] = row["time"]
                decoded_rows.append(decoded_data)
        except (ValueError, KeyError) as e:
            continue

    return pd.DataFrame(decoded_rows)


# Ejecución
df_result = process_cube_monitor_log(input_csv)
df_result.to_csv(output_csv, index=False)
print(f"Procesado exitosamente. {len(df_result)} filas exportadas a {output_csv}")
