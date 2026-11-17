#!/usr/bin/env python3
"""
Radio Control Layer for Sysmic Robotics.

Python port of the engine's radio protocol (packet_serializer.rs + radio.rs)
so a calibration harness can talk to the basestation over USB exactly like
the engine's `sender` does:

    PC ──USB──→ Base Station ──nRF24──→ Robot   (commands, 30 bytes)
    PC ←──USB── Base Station ←──nRF24── Robot   (telemetry, 32 bytes)

TX packet (30 bytes = 5 bytes × 6 robots), per robot:
    Byte 0: ID(3) | dribbler(2) | mode(1) | kick(1) | callback(1)
    Byte 1: sign_vx(1) | abs_vx[6:0]
    Byte 2: sign_vy(1) | abs_vy[6:0]
    Byte 3: sign_omega(1) | abs_omega[6:0]
    Byte 4: abs_vx[8:7](2) | abs_vy[8:7](2) | abs_omega[9:7](4)
  - velocity values scaled ×100, clamped ±511 (→ ±5.11 m/s, ±5.11 rad/s)
  - callback bit (byte0 bit0) = 1 selects TEST MODE (single-motor drive):
      Byte 1: sign_voltage(1) | abs_voltage[11:5]
      Byte 2: motor_idx(2) | abs_voltage[4:0] | unused(1)
      Byte 3-4: zero

RX packet (32 bytes telemetry), per robot:
    Byte 0:  ID(3) | ball(1) | kicker_charged(1) | motor_saturated(4)
    Bytes 1-2:  ball_range (uint16 LE, mm)
    Bytes 3-6:  x_est      (float32 LE, m)
    Bytes 7-10: y_est      (float32 LE, m)
    Bytes 11-14: vx_est    (float32 LE, m/s)
    Bytes 15-18: vy_est    (float32 LE, m/s)
    Bytes 19-22: theta_est (float32 LE, rad)
    Bytes 23-26: omega_est (float32 LE, rad/s)
    Bytes 27-30: gyro_z    (float32 LE, rad/s, diagnostic)
    Byte 31:  ang_acc_limit × 10 (uint8, 0.1 rad/s² resolution)

No sync byte on the wire: the engine re-syncs by sliding 1 byte when a
32-byte window fails validation (invalid ID, NaN/Inf, out-of-bounds).
This module replicates that behaviour.

Usage:
    # Unit self-test (no hardware needed)
    python3 radio_control.py --selftest

    # Live telemetry monitor (basestation on /dev/ttyUSB0)
    python3 radio_control.py --monitor 5

    # Send a velocity command to robot 0 for 2 seconds
    python3 radio_control.py --send 0.5,0.0,0.0 --id 0 --duration 2

    # Drive motor 2 of robot 0 with +600 DAC for 1 second (test mode)
    python3 radio_control.py --test-motor 2,600 --id 0 --duration 1

    # Send a velocity command and capture telemetry in one shot
    python3 radio_control.py --send 1.0,0.0,0.0 --id 0 --duration 3 --monitor 3
"""
import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    serial = None


# ── ANSI ───────────────────────────────────────────────────────────────────
def _supports_color():
    return sys.stdout.isatty()

_USE_COLOR = _supports_color()

def _c(code, text):
    return f'\033[{code}m{text}\033[0m' if _USE_COLOR else text

def green(t):  return _c('32', t)
def yellow(t): return _c('33', t)
def red(t):    return _c('31', t)
def cyan(t):   return _c('36', t)
def bold(t):   return _c('1', t)
def dim(t):    return _c('2', t)


# ── PROTOCOL CONSTANTS ─────────────────────────────────────────────────────
DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 115200
NUM_ROBOTS  = 6          # robots per team in the command packet
BYTES_PER_ROBOT = 5
CMD_PACKET_SIZE = NUM_ROBOTS * BYTES_PER_ROBOT   # 30
TELEMETRY_SIZE  = 32
MAX_VEL_SCALED  = 511    # 10-bit signed after ×100 scaling
MAX_VOLTAGE     = 4095   # DAC range for test mode
ANGULAR_SCALE   = 100.0
POSITION_SCALE  = 100.0
MAX_RX_BUFFER   = 64     # engine's serial_buf cap before discarding

# Calibration telemetry (firmware telemetry_calib_mode, radio_task.c)
CALIB_MAGIC     = 0xCB   # marker byte at offset 29 of calib packets
CONFIG_REG_TELEMETRY_CALIB = 0x01   # config-mode register: calib telemetry on/off


# ── TX: SERIALIZATION (port of packet_serializer.rs) ───────────────────────
def _clamp_scaled(value, scale):
    """Scale a float by `scale`, truncate toward zero (Rust `as i32`), clamp."""
    scaled = int(value * scale)          # Rust: (f64 * f64) as i32 truncates
    return max(-MAX_VEL_SCALED, min(MAX_VEL_SCALED, scaled))


def serialize_velocity(robot_id, vx=0.0, vy=0.0, omega=0.0,
                       dribbler=0, kick=False):
    """
    Serialize one robot's velocity command into 5 bytes.

    Mirrors the engine's normal-velocity branch of `serialize()`.
    Returns a list of 5 ints (0-255).
    """
    vx = _clamp_scaled(vx, 100.0)
    vy = _clamp_scaled(vy, 100.0)
    omega = _clamp_scaled(omega, ANGULAR_SCALE)
    dribb = int(dribbler) & 0x03
    kick_bit = 1 if kick else 0
    mode = 0  # velocity

    b0 = ((robot_id & 0x07) << 5) | (dribb << 3) | (mode << 2) | (kick_bit << 1)
    b1 = ((1 if vx < 0 else 0) << 7) | (abs(vx) & 0x7F)
    b2 = ((1 if vy < 0 else 0) << 7) | (abs(vy) & 0x7F)
    b3 = ((1 if omega < 0 else 0) << 7) | (abs(omega) & 0x7F)
    b4 = (((abs(vx) >> 7) & 0x03) << 6) | (((abs(vy) >> 7) & 0x03) << 4) \
        | ((abs(omega) >> 7) & 0x0F)
    return [b0, b1, b2, b3, b4]


def serialize_test_motor(robot_id, motor_idx, voltage, dribbler=0, kick=False):
    """
    Serialize one robot's test-mode command into 5 bytes.

    Mirrors the engine's test-mode branch of `serialize()`: callback bit
    (byte0 bit0) = 1, voltage clamped ±4095, motor_idx 0-3.
    Returns a list of 5 ints (0-255).
    """
    abs_v = int(abs(voltage))
    abs_v = max(0, min(MAX_VOLTAGE, abs_v))
    sign = 1 if voltage < 0 else 0
    motor = int(motor_idx) & 0x03
    dribb = int(dribbler) & 0x03
    kick_bit = 1 if kick else 0

    b0 = ((robot_id & 0x07) << 5) | (dribb << 3) | (kick_bit << 1) | 0x01
    b1 = (sign << 7) | ((abs_v >> 5) & 0x7F)
    b2 = (motor << 6) | (abs_v & 0x1F)
    return [b0, b1, b2, 0, 0]


def build_command_packet(robot_commands, num_robots=NUM_ROBOTS):
    """
    Build the full 30-byte command packet.

    `robot_commands`: dict {robot_id: [5 bytes]} or {robot_id: bytes}.
    Robots not present are zeroed (idle). Mirrors `serialize()`.
    Returns bytes (length = num_robots * 5).
    """
    packet = bytearray(num_robots * BYTES_PER_ROBOT)
    for rid, cmd in robot_commands.items():
        rid = int(rid)
        if not (0 <= rid < num_robots):
            continue
        off = rid * BYTES_PER_ROBOT
        packet[off:off + BYTES_PER_ROBOT] = bytes(cmd)
    return bytes(packet)


# ── RX: TELEMETRY PARSING (port of parse_telemetry in radio.rs) ────────────
def parse_telemetry(packet):
    """
    Parse a 32-byte telemetry packet into a dict, or None if invalid.

    Validation mirrors the engine: robot ID ≤ 6, all floats finite,
    |x|,|y| ≤ 10 m, |theta| ≤ 10 rad.
    """
    if len(packet) < TELEMETRY_SIZE:
        return None

    id_bits = packet[0] >> 5
    if id_bits > 6:
        return None

    ball_possession = (packet[0] & 0x01) != 0
    kicker_charged  = (packet[0] & 0x02) != 0
    motor_saturated = (packet[0] >> 2) & 0x0F

    ball_range = struct.unpack_from('<H', packet, 1)[0]
    x_est      = struct.unpack_from('<f', packet, 3)[0]
    y_est      = struct.unpack_from('<f', packet, 7)[0]
    vx_est     = struct.unpack_from('<f', packet, 11)[0]
    vy_est     = struct.unpack_from('<f', packet, 15)[0]
    theta_est  = struct.unpack_from('<f', packet, 19)[0]
    omega_est  = struct.unpack_from('<f', packet, 23)[0]
    gyro_z     = struct.unpack_from('<f', packet, 27)[0]
    ang_acc_limit = packet[31] / 10.0

    # Reject NaN/Inf (corrupted serial data or EKF not converged)
    for v in (x_est, y_est, vx_est, vy_est, theta_est, omega_est, gyro_z):
        if v != v or v in (float('inf'), float('-inf')):   # NaN / Inf
            return None

    # Sanity bounds (SSL field ~12m × 9m; robots never leave ±10m)
    if abs(x_est) > 10.0 or abs(y_est) > 10.0:
        return None
    if abs(theta_est) > 10.0:
        return None

    return {
        'id': id_bits,
        'team': 0,
        'ball_possession': ball_possession,
        'kicker_charged': kicker_charged,
        'motor_saturated': motor_saturated,
        'ball_range': ball_range,
        'x_est': x_est,
        'y_est': y_est,
        'vx_est': vx_est,
        'vy_est': vy_est,
        'theta_est': theta_est,
        'omega_est': omega_est,
        'gyro_z': gyro_z,
        'ang_acc_limit': ang_acc_limit,
    }


# ── RX: CALIBRATION TELEMETRY (firmware telemetry_calib_mode) ──────────────
def parse_telemetry_calib(packet):
    """
    Parse a 32-byte calibration telemetry packet, or None if invalid.

    Firmware layout (radio_task.c packTelemetryBuffer, telemetry_calib_mode):
      [0]     ID(3)|ball(1)|kicker(1)|sat(4)
      [1-4]   motor[0].measSpeed  f32
      [5-8]   motor[1].measSpeed  f32
      [9-12]  motor[2].measSpeed  f32
      [13-16] motor[3].measSpeed  f32
      [17-20] vx_est             f32
      [21-24] vy_est             f32
      [25-28] omega_est          f32
      [29]    magic 0xCB
      [30-31] sequence counter   uint16 LE
    """
    if len(packet) < TELEMETRY_SIZE:
        return None
    if packet[29] != CALIB_MAGIC:
        return None

    id_bits = packet[0] >> 5
    if id_bits > 6:
        return None

    ball_possession = (packet[0] & 0x01) != 0
    kicker_charged  = (packet[0] & 0x02) != 0
    motor_saturated = (packet[0] >> 2) & 0x0F

    motor_speed = [struct.unpack_from('<f', packet, 1 + 4 * i)[0]
                   for i in range(4)]
    vx_est    = struct.unpack_from('<f', packet, 17)[0]
    vy_est    = struct.unpack_from('<f', packet, 21)[0]
    omega_est = struct.unpack_from('<f', packet, 25)[0]
    seq       = struct.unpack_from('<H', packet, 30)[0]

    for v in motor_speed + [vx_est, vy_est, omega_est]:
        if v != v or v in (float('inf'), float('-inf')):
            return None

    return {
        'id': id_bits,
        'team': 0,
        'ball_possession': ball_possession,
        'kicker_charged': kicker_charged,
        'motor_saturated': motor_saturated,
        'motor_speed': motor_speed,
        'vx_est': vx_est,
        'vy_est': vy_est,
        'omega_est': omega_est,
        'seq': seq,
    }


# ── RADIO LINK ─────────────────────────────────────────────────────────────
class RadioLink:
    """
    Serial link to the basestation.

    Opens the USB serial port (115200 8N1, no flow control) and provides
    send/read primitives with the engine's re-sync behaviour on RX.
    """

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, timeout=0.01):
        if serial is None:
            raise RuntimeError('pyserial is not installed (pip install pyserial)')
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._ser = None
        self._rx_buf = bytearray()

    # -- lifecycle ----------------------------------------------------------
    def open(self):
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
        )
        return self

    def close(self):
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    @property
    def is_open(self):
        return self._ser is not None and self._ser.is_open

    # -- TX -----------------------------------------------------------------
    def send_packet(self, packet):
        """Write a raw command packet (30 bytes) and flush."""
        if not self.is_open:
            raise RuntimeError('RadioLink not open')
        self._ser.write(packet)
        self._ser.flush()

    def send_velocity(self, robot_id, vx=0.0, vy=0.0, omega=0.0,
                      dribbler=0, kick=False):
        """Send a velocity command for one robot (others idle)."""
        cmd = serialize_velocity(robot_id, vx, vy, omega, dribbler, kick)
        self.send_packet(build_command_packet({robot_id: cmd}))

    def send_test_motor(self, robot_id, motor_idx, voltage,
                        dribbler=0, kick=False):
        """Send a test-mode (single-motor drive) command for one robot."""
        cmd = serialize_test_motor(robot_id, motor_idx, voltage, dribbler, kick)
        self.send_packet(build_command_packet({robot_id: cmd}))

    def send_stop(self, robot_id):
        """Send zero velocities for one robot (safe stop)."""
        self.send_velocity(robot_id, 0.0, 0.0, 0.0)

    def send_config(self, robot_id, register, value):
        """
        Send a config-mode command (test bit + mode bit both set).

        Firmware (drive_task.c processRadioCommand): byte0=(id<<5)|0x05,
        byte1=register, byte2=value. Register 0x01 toggles telemetry_calib_mode.
        """
        cmd = [(robot_id << 5) | 0x05, register & 0xFF, value & 0xFF, 0, 0]
        self.send_packet(build_command_packet({robot_id: cmd}))

    # -- RX -----------------------------------------------------------------
    def _read_available(self):
        """Drain the serial port into the internal buffer."""
        if not self.is_open:
            raise RuntimeError('RadioLink not open')
        chunk = self._ser.read(self._ser.in_waiting or 1)
        if chunk:
            self._rx_buf.extend(chunk)
        if len(self._rx_buf) > MAX_RX_BUFFER:
            self._rx_buf.clear()

    def read_telemetry(self, timeout=None, parser=parse_telemetry):
        """
        Read one valid telemetry packet, or None on timeout.

        Replicates the engine's framing: consume 32-byte windows, validate,
        and slide 1 byte on failure to re-sync. `parser` selects the packet
        layout (normal or calibration).
        """
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            self._read_available()
            while len(self._rx_buf) >= TELEMETRY_SIZE:
                window = bytes(self._rx_buf[:TELEMETRY_SIZE])
                tel = parser(window)
                if tel is not None:
                    del self._rx_buf[:TELEMETRY_SIZE]
                    return tel
                del self._rx_buf[:1]   # misaligned/corrupt → slide 1 byte
            if deadline is not None and time.monotonic() >= deadline:
                return None
            time.sleep(0.001)

    def read_telemetry_calib(self, timeout=None):
        """Read one calibration-mode telemetry packet, or None on timeout."""
        return self.read_telemetry(timeout=timeout, parser=parse_telemetry_calib)

    def read_telemetry_stream(self, duration, callback=None, robot_id=None):
        """
        Collect telemetry for `duration` seconds.

        Returns a list of parsed dicts. If `callback` is given it is called
        with each packet. If `robot_id` is given, only that robot's packets
        are kept.
        """
        out = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            tel = self.read_telemetry(timeout=0.05)
            if tel is None:
                continue
            if robot_id is not None and tel['id'] != robot_id:
                continue
            if callback:
                callback(tel)
            out.append(tel)
        return out


# ── CLI ────────────────────────────────────────────────────────────────────
def _fmt_telemetry(tel):
    return (f"id={tel['id']}  x={tel['x_est']:7.3f}  y={tel['y_est']:7.3f}  "
            f"th={tel['theta_est']:6.3f}  vx={tel['vx_est']:6.3f}  "
            f"vy={tel['vy_est']:6.3f}  om={tel['omega_est']:6.3f}  "
            f"gyro={tel['gyro_z']:6.3f}  sat={tel['motor_saturated']:04b}")


def _fmt_telemetry_calib(tel):
    m = tel['motor_speed']
    return (f"id={tel['id']}  m0={m[0]:8.2f}  m1={m[1]:8.2f}  "
            f"m2={m[2]:8.2f}  m3={m[3]:8.2f}  vx={tel['vx_est']:6.3f}  "
            f"vy={tel['vy_est']:6.3f}  om={tel['omega_est']:6.3f}  "
            f"seq={tel['seq']}")


def _selftest():
    """Unit tests for serialization/parsing — no hardware required."""
    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        status = green('PASS') if cond else red('FAIL')
        print(f'  [{status}] {name} {dim(detail)}')
        if not cond:
            ok = False

    print(bold('TX serialization (velocity):'))

    # vx=1.5 → 150 = 0b10010110 → b1 = 150&0x7F = 22 (0x16),
    # b4 bits[8:7] = (150>>7)&0x03 = 1 → 1<<6 = 0x40
    # vy=-0.5 → -50 → sign=1, abs=50 → b2 = 0x80|50 = 0xB2
    # omega=0.25 → 25 → b3 = 0x19
    b = serialize_velocity(0, 1.5, -0.5, 0.25)
    check('vx=1.5, vy=-0.5, om=0.25, id=0',
          b == [0x00, 0x16, 0xB2, 0x19, 0x40],
          f'got {[hex(x) for x in b]}')

    # id=3, kick=True → byte0 = (3<<5)|(1<<1) = 0x62
    b = serialize_velocity(3, 0.0, 0.0, 0.0, kick=True)
    check('id=3 + kick bit', b[0] == 0x62, f'byte0=0x{b[0]:02x}')

    # dribbler=2 → byte0 bits[4:3] = 2<<3 = 0x10; id=1 → 0x20|0x10 = 0x30
    b = serialize_velocity(1, 0.0, 0.0, 0.0, dribbler=2)
    check('id=1 + dribbler=2', b[0] == 0x30, f'byte0=0x{b[0]:02x}')

    # clamp: vx=6.0 → 600 → clamped 511 = 0b111111111 → b1=0x7F,
    # b4 bits[8:7] = (511>>7)&0x03 = 3 → 3<<6 = 0xC0
    b = serialize_velocity(0, 6.0, 0.0, 0.0)
    check('clamp vx=6.0 → 511', b[1] == 0x7F and b[4] == 0xC0,
          f'b1=0x{b[1]:02x} b4=0x{b[4]:02x}')

    # negative clamp: vx=-6.0 → -511 → sign=1, abs=511
    b = serialize_velocity(0, -6.0, 0.0, 0.0)
    check('clamp vx=-6.0 → -511', b[1] == 0xFF and b[4] == 0xC0,
          f'b1=0x{b[1]:02x} b4=0x{b[4]:02x}')

    # MSB bits: vx=3.0 → 300 = 0b100101100 → b1 = 300&0x7F = 44 (0x2C),
    # b4 bits[8:7] = (300>>7)&0x03 = 2 → 2<<6 = 0x80
    b = serialize_velocity(0, 3.0, 0.0, 0.0)
    check('MSB vx=3.0 → 300 (0x12C)', b[1] == 0x2C and (b[4] & 0xC0) == 0x80,
          f'b1=0x{b[1]:02x} b4=0x{b[4]:02x}')

    print(bold('TX serialization (test mode):'))

    # motor 2, voltage +600 → abs=600=0x258 → b1=(600>>5)=18=0x12, b2=(2<<6)|(600&0x1F=24)=0x98
    b = serialize_test_motor(0, 2, 600)
    check('motor=2, v=+600', b == [0x01, 0x12, 0x98, 0x00, 0x00],
          f'got {[hex(x) for x in b]}')

    # motor 0, voltage -4095 → sign=1, abs=4095=0xFFF → b1=0x80|(4095>>5=127)=0xFF, b2=0x1F
    b = serialize_test_motor(0, 0, -4095)
    check('motor=0, v=-4095', b == [0x01, 0xFF, 0x1F, 0x00, 0x00],
          f'got {[hex(x) for x in b]}')

    # clamp: voltage -5000 → -4095
    b = serialize_test_motor(0, 1, -5000)
    check('clamp v=-5000 → -4095', b[1] == 0xFF and b[2] == 0x5F,
          f'b1=0x{b[1]:02x} b2=0x{b[2]:02x}')

    print(bold('Packet assembly:'))

    pkt = build_command_packet({0: serialize_velocity(0, 1.0, 0.0, 0.0)})
    check('30-byte packet, robot0 set', len(pkt) == 30 and pkt[0] == 0x00
          and pkt[5] == 0x00 and pkt[1] == 100,
          f'len={len(pkt)} b0=0x{pkt[0]:02x} b1={pkt[1]}')

    pkt = build_command_packet({5: serialize_velocity(5, 0.0, 0.0, 0.0)})
    check('robot5 at offset 25', pkt[25] == (5 << 5) and pkt[0] == 0x00,
          f'pkt[25]=0x{pkt[25]:02x}')

    print(bold('RX parsing:'))

    # Build a synthetic 32-byte packet: id=0, x=1.5, y=-0.5, vx=0.3, vy=0.1,
    # theta=0.2, omega=0.05, gyro=0.04, ball_range=123, sat=0b0010
    # (sat occupies bits[5:2]; 0b0010<<2 = 0x08 keeps ID bits[7:5] clear)
    pkt = bytearray(32)
    pkt[0] = (0 << 5) | 0x00 | (0b0010 << 2)
    struct.pack_into('<H', pkt, 1, 123)
    struct.pack_into('<f', pkt, 3, 1.5)
    struct.pack_into('<f', pkt, 7, -0.5)
    struct.pack_into('<f', pkt, 11, 0.3)
    struct.pack_into('<f', pkt, 15, 0.1)
    struct.pack_into('<f', pkt, 19, 0.2)
    struct.pack_into('<f', pkt, 23, 0.05)
    struct.pack_into('<f', pkt, 27, 0.04)
    pkt[31] = 30  # ang_acc_limit = 3.0

    tel = parse_telemetry(bytes(pkt))
    check('valid packet parsed',
          tel is not None and tel['id'] == 0
          and abs(tel['x_est'] - 1.5) < 1e-6
          and abs(tel['vy_est'] - 0.1) < 1e-6
          and tel['ball_range'] == 123
          and tel['motor_saturated'] == 0b0010
          and abs(tel['ang_acc_limit'] - 3.0) < 1e-9,
          f'got {tel}')

    # invalid ID (7) → reject
    bad = bytearray(pkt)
    bad[0] = (7 << 5)
    check('reject id=7', parse_telemetry(bytes(bad)) is None)

    # NaN → reject
    bad = bytearray(pkt)
    struct.pack_into('<f', bad, 3, float('nan'))
    check('reject NaN', parse_telemetry(bytes(bad)) is None)

    # out-of-bounds x=50 → reject
    bad = bytearray(pkt)
    struct.pack_into('<f', bad, 3, 50.0)
    check('reject |x|>10', parse_telemetry(bytes(bad)) is None)

    # short packet → reject
    check('reject short packet', parse_telemetry(bytes(pkt[:16])) is None)

    print(bold('RX parsing (calibration mode):'))

    # Synthetic calib packet: id=0, motor speeds [100, -200, 300.5, -400.25],
    # vx=0.5, vy=-0.25, omega=0.1, seq=1234, magic 0xCB
    cpkt = bytearray(32)
    cpkt[0] = 0x00
    struct.pack_into('<f', cpkt, 1, 100.0)
    struct.pack_into('<f', cpkt, 5, -200.0)
    struct.pack_into('<f', cpkt, 9, 300.5)
    struct.pack_into('<f', cpkt, 13, -400.25)
    struct.pack_into('<f', cpkt, 17, 0.5)
    struct.pack_into('<f', cpkt, 21, -0.25)
    struct.pack_into('<f', cpkt, 25, 0.1)
    cpkt[29] = CALIB_MAGIC
    struct.pack_into('<H', cpkt, 30, 1234)

    tel = parse_telemetry_calib(bytes(cpkt))
    check('calib packet parsed',
          tel is not None and tel['id'] == 0
          and tel['motor_speed'] == [100.0, -200.0, 300.5, -400.25]
          and abs(tel['vx_est'] - 0.5) < 1e-6
          and abs(tel['vy_est'] + 0.25) < 1e-6
          and abs(tel['omega_est'] - 0.1) < 1e-6
          and tel['seq'] == 1234,
          f'got {tel}')

    # wrong magic → reject
    bad = bytearray(cpkt)
    bad[29] = 0x00
    check('reject wrong magic', parse_telemetry_calib(bytes(bad)) is None)

    # calib packets are NOT rejected by the normal parser (motor-speed bytes
    # interleave into in-bounds garbage) — mode consistency is the caller's
    # responsibility; the magic byte only guards calib parsing
    mis = parse_telemetry(bytes(cpkt))
    check('normal parser misparses calib as garbage',
          mis is not None and mis['x_est'] != 100.0,
          f'x_est={mis["x_est"] if mis else None}')
    check('normal packet rejected by calib parser',
          parse_telemetry_calib(bytes(pkt)) is None)

    # NaN motor speed → reject
    bad = bytearray(cpkt)
    struct.pack_into('<f', bad, 5, float('nan'))
    check('reject NaN motor speed', parse_telemetry_calib(bytes(bad)) is None)

    # invalid ID → reject
    bad = bytearray(cpkt)
    bad[0] = (7 << 5)
    check('reject calib id=7', parse_telemetry_calib(bytes(bad)) is None)

    print()
    if ok:
        print(green(bold('All self-tests passed.')))
        return 0
    print(red(bold('Self-tests FAILED.')))
    return 1


def _monitor(link, duration, robot_id=None):
    print(cyan(f'Monitoring telemetry for {duration}s '
               f'(Ctrl+C to stop)...'))
    t0 = time.monotonic()
    count = 0
    try:
        for tel in link.read_telemetry_stream(duration, robot_id=robot_id):
            count += 1
            print(f'  [{time.monotonic() - t0:6.2f}s] {_fmt_telemetry(tel)}')
    except KeyboardInterrupt:
        pass
    print(dim(f'  ({count} packets received)'))
    return count


def _monitor_calib(link, duration, robot_id=None):
    print(cyan(f'Monitoring calibration telemetry for {duration}s '
               f'(Ctrl+C to stop)...'))
    t0 = time.monotonic()
    count = 0
    try:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            tel = link.read_telemetry_calib(timeout=0.05)
            if tel is None:
                continue
            if robot_id is not None and tel['id'] != robot_id:
                continue
            count += 1
            print(f'  [{time.monotonic() - t0:6.2f}s] {_fmt_telemetry_calib(tel)}')
    except KeyboardInterrupt:
        pass
    print(dim(f'  ({count} packets received)'))
    return count


def main():
    ap = argparse.ArgumentParser(description='Radio control layer for Sysmic robots')
    ap.add_argument('--port', default=DEFAULT_PORT, help=f'serial port (default {DEFAULT_PORT})')
    ap.add_argument('--baud', type=int, default=DEFAULT_BAUD, help=f'baud rate (default {DEFAULT_BAUD})')
    ap.add_argument('--selftest', action='store_true', help='run unit self-tests (no hardware)')
    ap.add_argument('--monitor', type=float, metavar='SECONDS', help='monitor telemetry for N seconds')
    ap.add_argument('--monitor-calib', type=float, metavar='SECONDS', help='monitor calibration telemetry for N seconds')
    ap.add_argument('--config', metavar='REG,VALUE', help='send config-mode register write')
    ap.add_argument('--send', metavar='VX,VY,OMEGA', help='send velocity command')
    ap.add_argument('--test-motor', metavar='MOTOR,VOLTAGE', help='send test-mode motor command')
    ap.add_argument('--id', type=int, default=0, help='robot id (default 0)')
    ap.add_argument('--duration', type=float, default=1.0, help='command duration in seconds (default 1)')
    ap.add_argument('--dribbler', type=int, default=0, help='dribbler speed 0-3')
    ap.add_argument('--kick', action='store_true', help='fire kicker')
    args = ap.parse_args()

    if args.selftest:
        return _selftest()

    if serial is None:
        print(red('pyserial is not installed: pip install pyserial'))
        return 1

    if not (args.send or args.test_motor or args.monitor or args.monitor_calib
            or args.config):
        ap.print_help()
        return 1

    with RadioLink(args.port, args.baud) as link:
        print(cyan(f'Radio link open: {args.port} @ {args.baud}'))

        if args.config:
            reg, value = (int(x) for x in args.config.split(','))
            print(yellow(f'Sending config id={args.id}: '
                         f'register={reg}, value={value}'))
            link.send_config(args.id, reg, value)
            time.sleep(0.1)
            link.send_config(args.id, reg, value)
            print(dim('Sent config (×2 for radio reliability).'))

        if args.send:
            vx, vy, omega = (float(x) for x in args.send.split(','))
            print(yellow(f'Sending velocity id={args.id}: '
                         f'vx={vx}, vy={vy}, omega={omega} for {args.duration}s'))
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                link.send_velocity(args.id, vx, vy, omega,
                                   dribbler=args.dribbler, kick=args.kick)
                time.sleep(0.02)
            link.send_stop(args.id)
            print(dim('Sent stop.'))

        if args.test_motor:
            motor, voltage = (int(x) for x in args.test_motor.split(','))
            print(yellow(f'Sending test-mode id={args.id}: '
                         f'motor={motor}, voltage={voltage} for {args.duration}s'))
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                link.send_test_motor(args.id, motor, voltage)
                time.sleep(0.02)
            link.send_test_motor(args.id, motor, 0)
            print(dim('Sent zero voltage.'))

        if args.monitor:
            _monitor(link, args.monitor, robot_id=args.id)

        if args.monitor_calib:
            _monitor_calib(link, args.monitor_calib, robot_id=args.id)

    return 0


if __name__ == '__main__':
    sys.exit(main())