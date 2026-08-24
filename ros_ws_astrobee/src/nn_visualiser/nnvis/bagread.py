"""Minimal ROS 1 bag v2.0 reader. No rospy/rosbag dependency."""
import struct, bz2, io

try:
    import lz4.frame as _lz4
except ImportError:
    _lz4 = None


def _read_header(buf, pos):
    (hlen,) = struct.unpack_from('<I', buf, pos); pos += 4
    end = pos + hlen
    fields = {}
    while pos < end:
        (flen,) = struct.unpack_from('<I', buf, pos); pos += 4
        field = buf[pos:pos + flen]; pos += flen
        i = field.index(b'=')
        fields[field[:i].decode()] = field[i + 1:]
    return fields, pos


def _read_record(buf, pos):
    fields, pos = _read_header(buf, pos)
    (dlen,) = struct.unpack_from('<I', buf, pos); pos += 4
    data = buf[pos:pos + dlen]; pos += dlen
    return fields, data, pos


class Bag:
    def __init__(self, path):
        self.buf = open(path, 'rb').read()
        assert self.buf[:13] == b'#ROSBAG V2.0\n'
        self.conns = {}          # conn id -> dict(topic, type, msgdef)

    def read_messages(self):
        """Yield (topic, conn_id, raw_bytes, (sec, nsec)) in chunk order."""
        pos = 13
        n = len(self.buf)
        while pos < n:
            fields, data, pos = _read_record(self.buf, pos)
            op = fields['op'][0]
            if op == 0x07:                                  # connection
                self._connection(fields, data)
            elif op == 0x05:                                # chunk
                comp = fields['compression'].decode()
                if comp == 'none':
                    raw = data
                elif comp == 'bz2':
                    raw = bz2.decompress(data)
                elif comp == 'lz4':
                    if _lz4 is None:
                        raise RuntimeError('lz4 not installed')
                    raw = _lz4.decompress(data)
                else:
                    raise RuntimeError('unknown compression ' + comp)
                yield from self._chunk(raw)
            # 0x03 bag header, 0x04 index, 0x06 chunk info -> skip

    def _connection(self, fields, data):
        cid = struct.unpack('<I', fields['conn'])[0]
        sub, _ = _read_header(b'\x00\x00\x00\x00' + data if False else
                              struct.pack('<I', len(data)) + data, 0)
        self.conns[cid] = {
            'topic': fields['topic'].decode(),
            'type': sub.get('type', b'').decode(),
            'msgdef': sub.get('message_definition', b'').decode(),
        }

    def _chunk(self, raw):
        pos, n = 0, len(raw)
        while pos < n:
            fields, data, pos = _read_record(raw, pos)
            op = fields['op'][0]
            if op == 0x07:
                self._connection(fields, data)
            elif op == 0x02:
                cid = struct.unpack('<I', fields['conn'])[0]
                sec, nsec = struct.unpack('<II', fields['time'])
                yield self.conns[cid]['topic'], cid, data, (sec, nsec)


# ---------- message deserialisers (hand-written for the five topics) ----------

def _str(b, p):
    (l,) = struct.unpack_from('<I', b, p); p += 4
    return b[p:p + l].decode('utf-8', 'replace'), p + l


def model_states(b):
    """gazebo_msgs/ModelStates -> (names, poses Nx7, twists Nx6) as lists."""
    p = 0
    (n,) = struct.unpack_from('<I', b, p); p += 4
    names = []
    for _ in range(n):
        s, p = _str(b, p); names.append(s)
    (np_,) = struct.unpack_from('<I', b, p); p += 4
    poses = []
    for _ in range(np_):
        poses.append(struct.unpack_from('<7d', b, p)); p += 56
    (nt,) = struct.unpack_from('<I', b, p); p += 4
    twists = []
    for _ in range(nt):
        twists.append(struct.unpack_from('<6d', b, p)); p += 48
    return names, poses, twists


def wrench_stamped(b):
    """geometry_msgs/WrenchStamped -> (stamp, frame_id, 6-tuple)."""
    p = 0
    seq, sec, nsec = struct.unpack_from('<III', b, p); p += 12
    frame, p = _str(b, p)
    w = struct.unpack_from('<6d', b, p)
    return (sec, nsec), frame, w


def tf_message(b):
    """tf2_msgs/TFMessage -> list of (parent, child, xyz, quat)."""
    p = 0
    (n,) = struct.unpack_from('<I', b, p); p += 4
    out = []
    for _ in range(n):
        seq, sec, nsec = struct.unpack_from('<III', b, p); p += 12
        parent, p = _str(b, p)
        child, p = _str(b, p)
        t = struct.unpack_from('<3d', b, p); p += 24
        q = struct.unpack_from('<4d', b, p); p += 32
        out.append((parent, child, t, q, (sec, nsec)))
    return out


def clock(b):
    return struct.unpack_from('<II', b, 0)


def joy(b):
    p = 0
    seq, sec, nsec = struct.unpack_from('<III', b, p); p += 12
    frame, p = _str(b, p)
    (na,) = struct.unpack_from('<I', b, p); p += 4
    axes = struct.unpack_from('<%df' % na, b, p); p += 4 * na
    (nb,) = struct.unpack_from('<I', b, p); p += 4
    btns = struct.unpack_from('<%di' % nb, b, p)
    return (sec, nsec), axes, btns


def arm_state_stamped(b):
    p = 0
    seq, sec, nsec = struct.unpack_from('<III', b, p); p += 12
    frame, p = _str(b, p)
    joint, grip = struct.unpack_from('<BB', b, p)
    return (sec, nsec), joint, grip


def joint_sample_stamped(b):
    p = 0
    seq, sec, nsec = struct.unpack_from('<III', b, p); p += 12
    frame, p = _str(b, p)
    (n,) = struct.unpack_from('<I', b, p); p += 4
    out = []
    for _ in range(n):
        vals = struct.unpack_from('<6f', b, p); p += 24
        (status,) = struct.unpack_from('<H', b, p); p += 2
        name, p = _str(b, p)
        out.append((name, vals, status))
    return (sec, nsec), out


def arm_action_goal(b):
    p = 0
    seq, sec, nsec = struct.unpack_from('<III', b, p); p += 12
    frame, p = _str(b, p)
    gsec, gnsec = struct.unpack_from('<II', b, p); p += 8
    gid, p = _str(b, p)
    (cmd,) = struct.unpack_from('<B', b, p); p += 1
    pan, tilt, grip = struct.unpack_from('<3f', b, p)
    return (sec, nsec), (gsec, gnsec), gid, cmd, (pan, tilt, grip)


def joy(b):
    p = 0
    seq, sec, nsec = struct.unpack_from('<III', b, p); p += 12
    frame, p = _str(b, p)
    (na,) = struct.unpack_from('<I', b, p); p += 4
    axes = struct.unpack_from('<%df' % na, b, p); p += 4 * na
    (nb,) = struct.unpack_from('<I', b, p); p += 4
    btns = struct.unpack_from('<%di' % nb, b, p)
    return (sec, nsec), axes, btns


def arm_state_stamped(b):
    p = 0
    seq, sec, nsec = struct.unpack_from('<III', b, p); p += 12
    frame, p = _str(b, p)
    joint, grip = struct.unpack_from('<BB', b, p)
    return (sec, nsec), joint, grip


def arm_action_goal(b):
    p = 0
    seq, sec, nsec = struct.unpack_from('<III', b, p); p += 12
    frame, p = _str(b, p)
    gsec, gnsec = struct.unpack_from('<II', b, p); p += 8
    gid, p = _str(b, p)
    (cmd,) = struct.unpack_from('<B', b, p); p += 1
    pan, tilt, grip = struct.unpack_from('<3f', b, p)
    return (sec, nsec), cmd, (pan, tilt, grip)
