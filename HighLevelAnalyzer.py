# ENCOS joint motor decoder for Saleae Logic 2
# Protocol: ENCOS Motor Debugging Manual V1.20
# Input analyzer: the built-in CAN analyzer (classic CAN, 1 Mbps).
#
# Emits one bubble per decoded field, positioned at that field's real bit
# location within the CAN frame. Bit-stuffing is already resolved by the
# CAN analyzer at the byte level (each data byte's start/end time is
# accurate); position *within* a byte is linearly interpolated, so a field
# can be off by a fraction of a bit period at worst.
#
# Commands and replies share the motor ID as the CAN identifier, so
# direction is inferred from byte 0's top 3 bits + DLC, with request/reply
# pairing to resolve servo-position vs Type 1 feedback.

import math
import struct

from saleae.analyzers import HighLevelAnalyzer, AnalyzerFrame, ChoicesSetting

# ---------------------------------------------------------------------------
# Table 9-1: model -> (torque ±Nm, current ±A, KD max, speed ±rad/s)
# KP is 0..500 and position ±12.5 rad for every model.
# Ranges are also learned per motor ID from range config acks/query replies.
# ---------------------------------------------------------------------------
MODELS = {
    'EC-A4310-P2-36':      (30, 30, 5, 18),
    'EC-A2806-P2-36':      (12, 10, 5, 18),
    'EC-A4315-P2-36':      (70, 30, 5, 18),
    'EC-A6408-P2-25':      (60, 60, 5, 18),
    'EC-A6416-P2-25':      (120, 60, 5, 18),
    'EC-A8112-P1-18':      (90, 60, 5, 18),
    'EC-A8116-P1-18':      (150, 70, 5, 18),
    'EC-A10020-P1-12/6':   (150, 70, 50, 18),
    'EC-A13720-P1-11.4':   (400, 220, 50, 18),
    'EC-A13715-P1-12.67':  (320, 220, 50, 18),
    'EC-A4310-P2-36H':     (30, 30, 5, 18),
    'EC-A6408-P2-16H':     (45, 30, 5, 18),
    'EC-A6408-P2-30.25H':  (60, 60, 5, 18),
    'EC-A6408-P2-30.25HB': (60, 60, 5, 18),
    'EC-A6416-P2-30.25H':  (120, 60, 5, 18),
    'EC-A8112-P1-18H':     (90, 60, 5, 18),
    'EC-A8116-P1-18H':     (130, 70, 5, 18),
    'EC-A10010-P2-16H':    (120, 90, 50, 18),
    'EC-A10015-P2-16H':    (150, 90, 50, 18),
    'EC-A10020-P2-16H':    (180, 90, 50, 18),
    'EC-A10010-P2-24H':    (150, 100, 50, 18),
    'EC-A10015-P2-24H':    (220, 120, 50, 18),
    'EC-A10020-P2-24H':    (300, 140, 50, 18),
    'EC-A3814-H14-107':    (60, 20, 5, 18),
    'EC-A5013-H17-100':    (90, 30, 5, 18),
    'EC-A6013-H20-100':    (130, 35, 5, 18),
    'EC-A6416-H25-100B':   (300, 25, 5, 18),
    'EC-A8116-H32-100B':   (500, 70, 5, 18),
    'EC-A5016-P2-17H':     (36, 40, 50, 30),
    'EC-A5020-P2-17H':     (42, 40, 50, 30),
    'EC-A5025-P2-17H':     (50, 40, 50, 30),
    'EC-A7216-P2-22H':     (140, 100, 50, 30),
    'EC-A7220-P2-22H':     (160, 100, 50, 30),
    'EC-A7225-P2-22H':     (200, 100, 50, 30),
    'EC-A9016-P2-23.625H': (240, 160, 50, 30),
    'EC-A9020-P2-23.625H': (280, 160, 50, 30),
    'EC-A9025-P2-23.625H': (320, 160, 50, 30),
    'EC-A10820-P2-24H':    (400, 220, 50, 30),
    'EC-A10825-P2-24H':    (450, 220, 50, 30),
}

ERRORS = {
    0: 'OK', 1: 'OVER-TEMP', 2: 'OVER-CURRENT', 3: 'OVER-VOLTAGE',
    4: 'UNDER-VOLTAGE', 5: 'ENCODER ERR', 6: 'BRAKE OVER-VOLTAGE', 7: 'DRV FAULT',
}

CURRENT_SUBMODES = {
    0: 'CURRENT', 1: 'TORQUE', 2: 'DAMPING BRAKE', 3: 'DYNAMIC BRAKE',
    4: 'REGEN BRAKE', 5: 'EM BRAKE',
}

CONFIG_NAMES = {
    0x01: 'accel', 0x02: 'comm mode', 0x04: 'Kt', 0x05: 'KP range',
    0x06: 'KD range', 0x07: 'POS range', 0x08: 'SPD range', 0x09: 'TOR range',
    0x0A: 'CUR range', 0x0B: 'CAN timeout', 0x0C: 'current PI',
    0x0D: 'speed PI', 0x0E: 'position PD', 0x0F: 'Kt cal mode',
    0x10: 'Kt table', 0x11: 'zero offset',
}

QUERY_NAMES = {
    1: 'position', 2: 'speed', 3: 'current', 4: 'power', 5: 'accel',
    22: 'Kt', 23: 'KP range', 24: 'KD range', 25: 'POS range',
    26: 'SPD range', 27: 'TOR range', 28: 'CUR range', 29: 'UUID',
    30: 'version', 31: 'CAN timeout', 32: 'current PI', 33: 'speed PI',
    34: 'position PD', 35: 'Kt cal mode', 36: 'Kt table', 37: 'brake',
    38: 'zero offset', 39: 'encoder angle',
}

# Query codes whose reply payload matches a config code's value layout.
QUERY_TO_CONFIG = {
    5: 0x01, 22: 0x04, 23: 0x05, 24: 0x06, 25: 0x07, 26: 0x08, 27: 0x09,
    28: 0x0A, 31: 0x0B, 32: 0x0C, 33: 0x0D, 34: 0x0E, 35: 0x0F, 38: 0x11,
}

# Range config codes: code -> (range key, scale, signed)
RANGE_CODES = {
    0x05: ('kp', 1, False), 0x06: ('kd', 1, False), 0x07: ('pos', 100, True),
    0x08: ('spd', 100, True), 0x09: ('tor', 10, True), 0x0A: ('cur', 10, True),
}

COMM_MODES = {0: 'classic CAN', 1: 'CAN FD', 2: 'CANopen'}


# ---------------------------------------------------------------------------
# Bit helpers (protocol is big-endian / MSB first throughout)
# ---------------------------------------------------------------------------
def bits(data, start, width):
    """Extract `width` bits starting `start` bits from the MSB of `data`."""
    value = int.from_bytes(bytes(data), 'big')
    return (value >> (8 * len(data) - start - width)) & ((1 << width) - 1)


def uscale(raw, width, lo, hi):
    """Linear map 0..2^width-1 -> lo..hi (MIT-style packed fields)."""
    return lo + raw * (hi - lo) / ((1 << width) - 1)


def f32_at(data, start_bit):
    """float32 starting at an arbitrary bit offset."""
    return struct.unpack('>f', bits(data, start_bit, 32).to_bytes(4, 'big'))[0]


def u16(d, i):
    return int.from_bytes(bytes(d[i:i + 2]), 'big')


def i16(d, i):
    return int.from_bytes(bytes(d[i:i + 2]), 'big', signed=True)


def f32(d, i):
    return struct.unpack('>f', bytes(d[i:i + 4]))[0]


def temp(raw):
    return (raw - 50) / 2


def err_str(code):
    return ERRORS.get(code, f'ERR{code}')


class Hla(HighLevelAnalyzer):

    result_types = {
        'command':  {'format': '{{data.summary}}'},
        'feedback': {'format': '{{data.summary}}'},
        'config':   {'format': '{{data.summary}}'},
        'query':    {'format': '{{data.summary}}'},
        'system':   {'format': '{{data.summary}}'},
        'other':    {'format': '{{data.summary}}'},
    }

    def __init__(self):
        tor, cur, kd, spd = MODELS[getattr(self, 'motor_model', 'EC-A4310-P2-36')]
        self.default_ranges = {
            'kp': (0.0, 500.0), 'kd': (0.0, float(kd)), 'pos': (-12.5, 12.5),
            'spd': (-float(spd), float(spd)), 'tor': (-float(tor), float(tor)),
            'cur': (-float(cur), float(cur)),
        }
        self.ranges = {}    # motor ID -> ranges dict (learned from the bus)
        self.pending = {}   # motor ID -> feedback type the last command requested
        self._reset()

    def _reset(self):
        self.start = None
        self.can_id = None
        self.extended = False
        self.remote = False
        self.data = []
        self.byte_times = []   # (start_time, end_time) per byte in self.data

    def ranges_for(self, cid):
        return self.ranges.setdefault(cid, dict(self.default_ranges))

    # -----------------------------------------------------------------------
    # Frame assembly
    # -----------------------------------------------------------------------
    def decode(self, frame: AnalyzerFrame):
        t = frame.type
        if t == 'identifier_field':
            self._reset()
            self.start = frame.start_time
            self.can_id = frame.data['identifier']
            self.extended = frame.data.get('extended', False)
            self.remote = frame.data.get('remote_frame', False)
        elif t == 'data_field':
            if self.start is not None:
                self._add_data_field(frame)
        elif t == 'crc_field':
            if self.start is None:
                return None
            out = self._emit(frame.end_time)
            self._reset()
            return out
        elif t == 'can_error':
            self._reset()
        return None

    def _add_data_field(self, frame):
        """Record each byte plus its own (interpolated, if the analyzer
        packs several bytes into one data_field) start/end time."""
        chunk = frame.data['data']
        n = len(chunk)
        step = (frame.end_time - frame.start_time) / n
        t = frame.start_time
        for k, b in enumerate(chunk):
            t_end = frame.end_time if k == n - 1 else t + step
            self.data.append(b)
            self.byte_times.append((t, t_end))
            t = t_end

    def time_at_bit(self, bit):
        """Interpolate a wall-clock time for an absolute bit offset into
        the payload, using each byte's real start/end time as anchors."""
        total = len(self.byte_times) * 8
        bit = max(0, min(bit, total))
        if bit == total:
            return self.byte_times[-1][1]
        idx, frac = divmod(bit, 8)
        bstart, bend = self.byte_times[idx]
        return bstart + (bend - bstart) * (frac / 8)

    def _emit(self, end_time):
        d, cid = self.data, self.can_id
        if self.extended or self.remote or not d:
            return [AnalyzerFrame('other', self.start, end_time, {
                'id': cid, 'summary': f'ID 0x{cid:X} (not ENCOS)',
                'raw': ' '.join(f'{b:02X}' for b in d),
            })]
        if cid == 0x7FF:
            kind, fields = 'system', self.dec_system(d)
        else:
            try:
                kind, fields = self.dec_motor(cid, d)
            except (IndexError, struct.error):
                kind, fields = 'other', [(0, len(d) * 8, 'malformed')]
        return self._emit_fields(kind, cid, fields, d, end_time)

    def _emit_fields(self, kind, cid, fields, d, end_time):
        """fields: ascending list of (bit_start, bit_width, text). Each gets
        its own bubble at the real time its bits occupy on the bus. A
        leading bubble covers the identifier/control section."""
        raw = ' '.join(f'{b:02X}' for b in d)
        frames = [AnalyzerFrame(kind, self.start, self.time_at_bit(0), {
            'id': cid, 'summary': f'M{cid}', 'raw': raw,
        })]
        for i, (bit_start, _width, text) in enumerate(fields):
            t0 = self.time_at_bit(bit_start)
            t1 = self.time_at_bit(fields[i + 1][0]) if i + 1 < len(fields) else end_time
            frames.append(AnalyzerFrame(kind, t0, t1, {'id': cid, 'summary': text, 'raw': raw}))
        return frames

    # -----------------------------------------------------------------------
    # Dispatch on top 3 bits of byte 0 + DLC
    # -----------------------------------------------------------------------
    def dec_motor(self, cid, d):
        top, n = d[0] >> 5, len(d)
        r = self.ranges_for(cid)

        if top == 0 and n == 8:
            self.pending[cid] = 1          # force-position always replies Type 1
            return 'command', self.dec_force_pos(d, r)
        if top == 1 and n == 8:
            return self.dec_servo_or_type1(cid, d, r)
        if top == 2 and n == 7:
            return 'command', self.dec_speed_cmd(cid, d)
        if top == 2 and n == 8:
            self.pending.pop(cid, None)
            return 'feedback', self.dec_type2(d)
        if top == 3 and n == 3:
            return 'command', self.dec_current_cmd(cid, d)
        if top == 3 and n == 8:
            self.pending.pop(cid, None)
            return 'feedback', self.dec_type3(d)
        if top == 4 and n == 3:
            return 'config', [
                (0, 3, 'CFG-ACK'),
                (3, 5, f'[{err_str(d[0] & 0x1F)}]'),
                (8, 8, CONFIG_NAMES.get(d[1], hex(d[1]))),
                (16, 8, 'ok' if d[2] else 'FAILED'),
            ]
        if top == 5:
            return 'query', self.dec_query_reply(cid, d, r)
        if top == 6 and n == 2:
            self.pending.pop(cid, None)
            state = {0: 'engaged', 1: 'released'}.get(d[1], d[1])
            return 'feedback', [
                (0, 3, 'BRAKE'),
                (3, 5, f'[{err_str(d[0] & 0x1F)}]'),
                (8, 8, str(state)),
            ]
        if top == 6:
            return 'config', self.dec_config_cmd(d)
        if top == 7 and n >= 4 and d[0] == 0xFF and d[1] == 0xFE:
            return 'config', self.dec_config_ack(cid, d)
        if top == 7 and n == 8 and d[0] == 0xEE:
            mode, group = d[1] >> 5, d[1] & 0x1F
            return 'config', [
                (0, 8, 'Kt-REPLY'),
                (8, 3, f'mode{mode}'),
                (11, 5, f'grp{group}'),
            ] + self.kt_fields(d[2:8], 16)
        if top == 7:
            q = d[1] if n > 1 else None
            name = QUERY_NAMES.get(q, f'code {q}')
            fields = [(0, 3, 'QUERY')]
            if q == 36:
                fields.append((3, 5, f'grp{d[0] & 0x1F}'))
            fields.append((8, 8, name))
            return 'query', fields
        return 'other', [(0, n * 8, f'unknown (type {top}, DLC {n})')]

    # -----------------------------------------------------------------------
    # Commands
    # -----------------------------------------------------------------------
    def dec_force_pos(self, d, r):
        kp = uscale(bits(d, 3, 12), 12, *r['kp'])
        kd = uscale(bits(d, 15, 9), 9, *r['kd'])
        pos = uscale(bits(d, 24, 16), 16, *r['pos'])
        spd = uscale(bits(d, 40, 12), 12, *r['spd'])
        tor = uscale(bits(d, 52, 12), 12, *r['tor'])
        return [
            (0, 3, 'FORCE-POS'),
            (3, 12, f'KP={kp:.1f}'),
            (15, 9, f'KD={kd:.2f}'),
            (24, 16, f'pos={pos:+.3f}rad'),
            (40, 12, f'spd={spd:+.2f}rad/s'),
            (52, 12, f'tff={tor:+.2f}Nm'),
        ]

    def dec_servo_cmd(self, d):
        pos = f32_at(d, 3)
        rpm = bits(d, 35, 15) / 10
        cur = bits(d, 50, 12) / 10
        st = bits(d, 62, 2)
        fields = [
            (0, 3, 'SERVO-POS'),
            (3, 32, f'target={pos:.2f}deg'),
            (35, 15, f'speed={rpm:.1f}rpm'),
            (50, 12, f'Ilim={cur:.1f}A'),
            (62, 2, f'reply=T{st}'),
        ]
        return pos, fields, st

    def dec_speed_cmd(self, cid, d):
        st, resv = d[0] & 0x03, (d[0] >> 2) & 0x07
        self._set_pending(cid, st if resv == 0 else 0)
        note = '' if resv == 0 else ' (resv!=0: no reply)'
        return [
            (0, 3, 'SERVO-SPD'),
            (6, 2, f'reply=T{st}{note}'),
            (8, 32, f'{f32(d, 1):.1f}rpm'),
            (40, 16, f'Ilim={u16(d, 5) / 10:.1f}A'),
        ]

    def dec_current_cmd(self, cid, d):
        sub, st, val = (d[0] >> 2) & 0x07, d[0] & 0x03, i16(d, 1)
        name = CURRENT_SUBMODES.get(sub, f'sub{sub}')
        fields = [(0, 3, 'CUR-CMD'), (3, 3, name)]
        if sub == 5:
            self._set_pending(cid, 6)
            state = {0: 'engage', 1: 'release'}.get(val, val)
            fields.append((8, 16, str(state)))
            return fields
        self._set_pending(cid, st)
        fields.append((6, 2, f'reply=T{st}'))
        if sub == 0:
            detail = f'{val / 100:+.2f}A'
        elif sub == 1:
            detail = f'{val / 100:+.2f}Nm'
        elif sub in (3, 4):
            detail = f'threshold {val / 100:.2f}A'
        else:
            detail = ''
        if detail:
            fields.append((8, 16, detail))
        return fields

    def dec_config_cmd(self, d):
        code = d[1]
        if code == 0x10:
            group = d[0] & 0x1F
            return [(0, 16, f'SET Kt-table grp{group}')] + self.kt_fields(d[2:8], 16)
        name = CONFIG_NAMES.get(code, hex(code))
        return [
            (0, 16, f'SET {name}'),
            (16, (len(d) - 2) * 8, f'= {self.fmt_config_value(code, d[2:])}'),
        ]

    def _set_pending(self, cid, reply_type):
        if reply_type:
            self.pending[cid] = reply_type
        else:
            self.pending.pop(cid, None)

    # -----------------------------------------------------------------------
    # The one real ambiguity: 0x2x, DLC 8 = servo-position cmd OR Type 1
    # -----------------------------------------------------------------------
    def dec_servo_or_type1(self, cid, d, r):
        # Type 1: error code must be 0..7 and temps >= 0 degC (raw >= 50).
        fb_ok = (d[0] & 0x1F) <= 7 and d[6] >= 50 and d[7] >= 50
        # Servo cmd: target float must be sane. A Type 1 frame read as a
        # float has an exponent <= 63, i.e. |x| < 1e-19 unless exactly zero.
        pos = f32_at(d, 3)
        cmd_ok = math.isfinite(pos) and (pos == 0 or 1e-3 <= abs(pos) <= 1e5)

        expecting_fb = self.pending.get(cid) == 1
        if fb_ok and (expecting_fb or not cmd_ok):
            self.pending.pop(cid, None)
            return 'feedback', self.dec_type1(d, r)
        if cmd_ok:
            _, fields, st = self.dec_servo_cmd(d)
            self._set_pending(cid, st)
            if fb_ok:
                b, w, text = fields[-1]
                fields[-1] = (b, w, text + ' (?)')
            return 'command', fields
        return 'other', [(0, len(d) * 8, 'servo cmd / Type 1: neither decodes cleanly')]

    # -----------------------------------------------------------------------
    # Feedback
    # -----------------------------------------------------------------------
    def dec_type1(self, d, r):
        pos = uscale(u16(d, 1), 16, *r['pos'])
        spd = uscale(bits(d, 24, 12), 12, *r['spd'])
        cur = uscale(bits(d, 36, 12), 12, *r['cur'])
        return [
            (0, 3, 'T1'),
            (3, 5, f'[{err_str(d[0] & 0x1F)}]'),
            (8, 16, f'pos={pos:+.3f}rad ({math.degrees(pos):+.1f}deg)'),
            (24, 12, f'spd={spd:+.2f}rad/s'),
            (36, 12, f'I={cur:+.2f}A'),
            (48, 8, f'coil={temp(d[6]):.1f}C'),
            (56, 8, f'mos={temp(d[7]):.1f}C'),
        ]

    def dec_type2(self, d):
        return [
            (0, 3, 'T2'),
            (3, 5, f'[{err_str(d[0] & 0x1F)}]'),
            (8, 32, f'pos={f32(d, 1):+.2f}deg'),
            (40, 16, f'I={i16(d, 5) / 100:+.2f}A'),
            (56, 8, f'coil={temp(d[7]):.1f}C'),
        ]

    def dec_type3(self, d):
        return [
            (0, 3, 'T3'),
            (3, 5, f'[{err_str(d[0] & 0x1F)}]'),
            (8, 32, f'spd={f32(d, 1):+.1f}rpm'),
            (40, 16, f'I={i16(d, 5) / 100:+.2f}A'),
            (56, 8, f'coil={temp(d[7]):.1f}C'),
        ]

    # -----------------------------------------------------------------------
    # Config / query payloads
    # -----------------------------------------------------------------------
    def fmt_config_value(self, code, p):
        if code == 0x01:
            return f'{u16(p, 0) / 100:.2f}rad/s^2'
        if code == 0x02:
            return COMM_MODES.get(p[0], str(p[0]))
        if code == 0x04:
            return f'{u16(p, 0) / 100:.2f}Nm/A'
        if code in RANGE_CODES:
            _, lo, hi = self.parse_range(code, p)
            return f'{lo:g}..{hi:g}'
        if code == 0x0B:
            return f'{u16(p, 0)}ms' + (' (disabled)' if u16(p, 0) == 0 else '')
        if code == 0x0C:
            return f'KP={u16(p, 0) / 10000:g} KI={u16(p, 2) / 10:g}'
        if code == 0x0D:
            return f'KP={u16(p, 0) / 100000:g} KI={u16(p, 2) / 100000:g}'
        if code == 0x0E:
            return f'KP={u16(p, 0) / 100000:g} KD={u16(p, 2) / 100000:g}'
        if code == 0x0F:
            return {0: 'off', 1: 'single-table', 2: 'dual-table'}.get(p[0], str(p[0]))
        if code == 0x11:
            return f'{i16(p, 0) / 100:+.2f}deg'
        return ' '.join(f'{b:02X}' for b in p)

    def parse_range(self, code, p):
        key, scale, signed = RANGE_CODES[code]
        read = i16 if signed else u16
        return key, read(p, 0) / scale, read(p, 2) / scale

    def learn_range(self, cid, code, p):
        key, lo, hi = self.parse_range(code, p)
        self.ranges_for(cid)[key] = (lo, hi)

    def kt_fields(self, p, base_bit):
        """4 packed 12-bit Kt-table entries, each its own bubble."""
        return [(base_bit + 12 * i, 12, f'{bits(p, 12 * i, 12) / 100:.2f}')
                for i in range(4)]

    def dec_config_ack(self, cid, d):
        code, p = d[2], d[3:]
        name = CONFIG_NAMES.get(code, hex(code))
        if code in RANGE_CODES and len(p) >= 4:
            self.learn_range(cid, code, p)
        return [
            (0, 16, 'CFG-OK'),
            (16, 8, name),
            (24, len(p) * 8, f'= {self.fmt_config_value(code, p)}'),
        ]

    def dec_query_reply(self, cid, d, r):
        err, q, p = d[0] & 0x1F, d[1], d[2:]
        name = QUERY_NAMES.get(q, f'code {q}')
        fields = [(0, 3, 'REPLY'), (3, 5, f'[{err_str(err)}]'), (8, 8, name)]
        if q == 1:
            fields.append((16, 32, f'= {f32(p, 0):+.2f}deg'))
        elif q == 2:
            fields.append((16, 32, f'= {f32(p, 0):+.2f}rpm'))
        elif q == 3:
            fields.append((16, 32, f'= {f32(p, 0):+.2f}A'))
        elif q == 4:
            fields.append((16, 32, f'= {f32(p, 0):.2f}W'))
        elif q == 29:
            fields.append((16, 8, f'pkt {p[0]}'))
            fields.append((24, 32, ' '.join(f'{b:02X}' for b in p[1:5])))
        elif q == 30:
            fields.append((16, 24, f'hw {p[0]}.{p[1]}.{p[2]}'))
            fields.append((40, 24, f'sw {p[3]}.{p[4]}.{p[5]}'))
        elif q == 37:
            fields.append((16, 8, {0: 'engaged', 1: 'released'}.get(p[0], str(p[0]))))
        elif q == 39:
            fields.append((16, 8, 'cal=on' if p[0] else 'cal=off'))
            fields.append((24, 16, f'raw={i16(p, 1) / 100:.2f}deg'))
            fields.append((40, 16, f'cal={i16(p, 3) / 100:.2f}deg'))
        elif q in QUERY_TO_CONFIG:
            code = QUERY_TO_CONFIG[q]
            if code in RANGE_CODES:
                self.learn_range(cid, code, p)
            fields.append((16, len(p) * 8, f'= {self.fmt_config_value(code, p)}'))
        else:
            fields.append((16, len(p) * 8, ' '.join(f'{b:02X}' for b in p)))
        return fields

    # -----------------------------------------------------------------------
    # Identifier 0x7FF: ID / zero-point setup and ID query
    # -----------------------------------------------------------------------
    def dec_system(self, d):
        if len(d) < 4:
            return [(0, len(d) * 8, 'SYS (short)')]
        mid, dirn, cmd = (d[0] << 8) | d[1], d[2], d[3]
        if dirn == 0:
            if cmd == 0x82:
                return [(0, 16, f'M{mid}'), (16, 8, 'host'), (24, 8, 'QUERY ID')]
            if cmd == 0x03:
                fields = [(0, 16, f'M{mid}'), (16, 8, 'host'), (24, 8, 'SET ZERO')]
                if len(d) >= 6:
                    fields.append((32, 16, f'offset {i16(d, 4) / 100:+.2f}deg'))
                return fields
            if cmd == 0x04 and len(d) >= 6:
                return [
                    (0, 16, f'M{mid}'), (16, 8, 'host'), (24, 8, 'SET ID'),
                    (32, 16, f'-> M{(d[4] << 8) | d[5]}'),
                ]
            if cmd == 0x05:
                return [(0, 16, f'M{mid}'), (16, 8, 'host'), (24, 8, 'RESET ID -> 1')]
            return [(0, 16, f'M{mid}'), (16, 8, 'host'), (24, 8, f'cmd 0x{cmd:02X}')]
        if dirn == 1:
            if d[0] == 0xFF and d[1] == 0xFF and len(d) >= 5:
                return [(0, 16, 'motor'), (16, 8, 'ID query OK'),
                        (24, 16, f'ID is M{(d[3] << 8) | d[4]}')]
            if d[0] == 0x80 and d[1] == 0x80:
                return [(0, 16, 'motor'), (16, 16, 'ID query FAILED')]
            if d[0] == 0x7F and d[1] == 0x7F and cmd == 0x05:
                return [(0, 16, 'motor'), (16, 16, 'ID reset OK')]
            if cmd == 0x03:
                return [(0, 16, f'M{mid}'), (16, 8, 'motor'), (24, 8, 'zero set OK')]
            if cmd == 0x04:
                return [(0, 16, f'M{mid}'), (16, 8, 'motor'), (24, 8, 'ID set OK')]
            if cmd == 0x00:
                return [(0, 16, f'M{mid}'), (16, 8, 'motor'), (24, 8, 'setting FAILED')]
        return [(0, len(d) * 8, 'SYS ' + ' '.join(f'{b:02X}' for b in d))]
