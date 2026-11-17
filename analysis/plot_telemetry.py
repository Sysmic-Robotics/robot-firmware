import struct
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# 1. Configuración
INPUT_CSV = "stm32_monitor_log.csv"
CLEAN_CSV = "telemetry_clean_t0.csv"

STRUCT_FORMAT = "<BHfffffffB"


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


def load_and_clean_data(file_path):
    print("Cargando y procesando archivo...")
    df = pd.read_csv(file_path)

    decoded = []
    for _, row in df.iterrows():
        try:
            raw_bytes = bytes([int(row[f"telemetry_buf[{i}]"]) for i in range(32)])
            data = unpack_telemetry(raw_bytes)
            if data:
                data["time_raw"] = row["time"]
                decoded.append(data)
        except (ValueError, KeyError):
            continue

    df_decoded = pd.DataFrame(decoded)

    # --- FILTRADO DE REPETICIONES ---
    state_cols = [
        "x_est",
        "y_est",
        "vx_est",
        "vy_est",
        "theta_est",
        "omega_est",
        "gyro_z",
    ]

    # Quedarse únicamente con muestras donde el buffer realmente cambió
    df_clean = df_decoded.loc[
        (df_decoded[state_cols] != df_decoded[state_cols].shift()).any(axis=1)
    ].copy()

    # --- EJE DE TIEMPO DESDE t = 0.0 s ---
    # Opción A: Tiempo normalizado a 1 kHz perfecto (dt = 1 ms por cada muestra real)
    df_clean["time_sec"] = np.arange(len(df_clean)) * 0.001

    # Opción B (Comentada): Tiempo del log original relativo al inicio (t - t0)
    # t0 = df_clean['time_raw'].iloc[0]
    # df_clean['time_sec'] = df_clean['time_raw'] - t0

    print(
        f"Muestras procesadas: {len(df_clean)} | Duración total: {df_clean['time_sec'].iloc[-1]:.3f} s"
    )

    df_clean.to_csv(CLEAN_CSV, index=False)
    return df_clean


def plot_interactive_plotly(df):
    """Genera dashboard interactivo en HTML (Plotly) con eje X iniciando en 0s"""
    fig = make_subplots(
        rows=3,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.08,
        subplot_titles=(
            "Posición Estimada (x, y)",
            "Velocidad Estimada (vx, vy)",
            "Estado Angular (theta, omega, gyro_z)",
        ),
    )

    # Subplot 1: Posición
    fig.add_trace(
        go.Scatter(
            x=df["time_sec"],
            y=df["x_est"],
            mode="lines",
            name="x_est [m]",
            line=dict(color="#1f77b4"),
        ),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=df["time_sec"],
            y=df["y_est"],
            mode="lines",
            name="y_est [m]",
            line=dict(color="#ff7f0e"),
        ),
        row=1,
        col=1,
    )

    # Subplot 2: Velocidades
    fig.add_trace(
        go.Scatter(
            x=df["time_sec"],
            y=df["vx_est"],
            mode="lines",
            name="vx_est [m/s]",
            line=dict(color="#2ca02c"),
        ),
        row=2,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=df["time_sec"],
            y=df["vy_est"],
            mode="lines",
            name="vy_est [m/s]",
            line=dict(color="#d62728"),
        ),
        row=2,
        col=1,
    )

    # Subplot 3: Angulares
    fig.add_trace(
        go.Scatter(
            x=df["time_sec"],
            y=df["theta_est"],
            mode="lines",
            name="theta_est [rad]",
            line=dict(color="#9467bd"),
        ),
        row=3,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=df["time_sec"],
            y=df["omega_est"],
            mode="lines",
            name="omega_est [rad/s]",
            line=dict(color="#8c564b"),
        ),
        row=3,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=df["time_sec"],
            y=df["gyro_z"],
            mode="lines",
            name="gyro_z [rad/s]",
            line=dict(color="#e377c2", dash="dot"),
        ),
        row=3,
        col=1,
    )

    fig.update_layout(
        title="Telemetría (Tiempo relativo t = 0.0 s)",
        height=800,
        hovermode="x unified",
        template="plotly_white",
    )
    fig.update_xaxes(title_text="Tiempo Relativo [segundos]", row=3, col=1)

    fig.write_html("telemetry_dashboard.html")
    print("Dashboard visual interactivo guardado en 'telemetry_dashboard.html'")
    fig.show()


def plot_static_matplotlib(df):
    """Genera gráfica estática en PNG (Matplotlib) iniciando en 0s"""
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    # Posición
    axs[0].plot(df["time_sec"], df["x_est"], label="X est (m)")
    axs[0].plot(df["time_sec"], df["y_est"], label="Y est (m)")
    axs[0].set_ylabel("Posición [m]")
    axs[0].grid(True)
    axs[0].legend(loc="upper right")

    # Velocidad
    axs[1].plot(df["time_sec"], df["vx_est"], label="Vx est (m/s)")
    axs[1].plot(df["time_sec"], df["vy_est"], label="Vy est (m/s)")
    axs[1].set_ylabel("Velocidad [m/s]")
    axs[1].grid(True)
    axs[1].legend(loc="upper right")

    # Angulares
    axs[2].plot(df["time_sec"], df["theta_est"], label=r"$\theta$ est (rad)")
    axs[2].plot(df["time_sec"], df["omega_est"], label=r"$\omega$ est (rad/s)")
    axs[2].plot(df["time_sec"], df["gyro_z"], label="Gyro Z (rad/s)", linestyle="--")
    axs[2].set_ylabel("Estado Angular")
    axs[2].set_xlabel("Tiempo [s] (Inicio en t = 0.0 s)")
    axs[2].grid(True)
    axs[2].legend(loc="upper right")

    plt.suptitle("Telemetría Reconstruida del Robot")
    plt.tight_layout()
    plt.savefig("telemetry_plot.png", dpi=300)
    plt.show()


if __name__ == "__main__":
    df_clean = load_and_clean_data(INPUT_CSV)

    try:
        plot_interactive_plotly(df_clean)
    except Exception:
        print("Fallback a Matplotlib...")
        plot_static_matplotlib(df_clean)
