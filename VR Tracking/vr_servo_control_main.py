import socket
import json
import serial
import time

# ============================================================
# Utility: linear map with clamping
# ============================================================
def map_range(value, in_min, in_max, out_min, out_max):
    value = max(min(value, in_max), in_min)
    t = (value - in_min) / (in_max - in_min)
    return out_min + t * (out_max - out_min)

def clamp_angle(a):
    a = int(round(a))
    return max(0, min(180, a))

# ============================================================
# FINGER-SPECIFIC MAPPINGS (unchanged, kept from your version)
# ============================================================
def pinky_angles_to_servos(pb, pt, ps):
    VR_BASE_MIN, VR_BASE_MAX = -8.6, 78.75
    VR_TIP_MIN,  VR_TIP_MAX  = -0.5, 63
    VR_S_MIN,    VR_S_MAX    = 0, 22

    SB_MIN, SB_MAX = 25, 90
    ST_MIN, ST_MAX = 20, 150
    SS_MIN, SS_MAX = 10, 50

    return (
        map_range(pb, VR_BASE_MIN, VR_BASE_MAX, SB_MIN, SB_MAX),
        map_range(pt, VR_TIP_MIN,  VR_TIP_MAX,  ST_MIN, ST_MAX),
        map_range(ps, VR_S_MIN,    VR_S_MAX,    SS_MIN, SS_MAX)
    )

def ring_angles_to_servos(rb, rt, rs):
    VR_BASE_MIN, VR_BASE_MAX = 6, 85
    VR_TIP_MIN,  VR_TIP_MAX  = -15, 63
    VR_S_MIN,    VR_S_MAX    = 10.5, 21

    SB_MIN, SB_MAX = 10, 180
    ST_MIN, ST_MAX = 10, 170
    SS_MIN, SS_MAX = 140, 110  # reversed

    return (
        map_range(rb, VR_BASE_MIN, VR_BASE_MAX, SB_MIN, SB_MAX),
        map_range(rt, VR_TIP_MIN,  VR_TIP_MAX,  ST_MIN, ST_MAX),
        map_range(rs, VR_S_MIN,    VR_S_MAX,    SS_MIN, SS_MAX)
    )

def index_angles_to_servos(i_b, i_t, i_s):
    VR_BASE_MIN, VR_BASE_MAX = 8, 63
    VR_TIP_MIN, VR_TIP_MAX = 0, 67
    VR_S_MIN, VR_S_MAX = 6, -7


    SB_MIN, SB_MAX = 20, 90
    ST_MIN, ST_MAX = 20, 180
    SS_MIN, SS_MAX = 20, 40

    return (
        map_range(i_b, VR_BASE_MIN, VR_BASE_MAX, SB_MIN, SB_MAX),
        map_range(i_t, VR_TIP_MIN, VR_TIP_MAX, ST_MIN, ST_MAX),
        map_range(i_s, VR_S_MIN, VR_S_MAX, SS_MIN, SS_MAX)
    )

def middle_angles_to_servos(m_b, m_t, m_s):
    VR_BASE_MIN, VR_BASE_MAX = 9.5, 77
    VR_TIP_MIN, VR_TIP_MAX = -4, 68
    VR_S_MIN, VR_S_MAX = 9, 0

    # TBD
    SB_MIN, SB_MAX = 20, 90
    ST_MIN, ST_MAX = 20, 150
    SS_MIN, SS_MAX = 60, 80

    return (
        map_range(m_b, VR_BASE_MIN, VR_BASE_MAX, SB_MIN, SB_MAX),
        map_range(m_t, VR_TIP_MIN, VR_TIP_MAX, ST_MIN, ST_MAX),
        map_range(m_s, VR_S_MIN, VR_S_MAX, SS_MIN, SS_MAX)
    )

def thumb_angles_to_servos(t_b, t_t, t_s):
    VR_BASE_MIN, VR_BASE_MAX = -11, 31 #BENDING
    VR_TIP_MIN, VR_TIP_MAX = 70, 96 #ROLL
    VR_S_MIN, VR_S_MAX = -2, 34 # SPLAY
    #VR_S_MIN, VR_S_MAX = 0.5, 27 #SPLAY

    SB_MIN, SB_MAX = 180, 20
    ST_MIN, ST_MAX = 20, 80 #Roll
    SS_MIN, SS_MAX = 50, 160

    return (
        map_range(t_b, VR_BASE_MIN, VR_BASE_MAX, SB_MIN, SB_MAX),
        map_range(t_t, VR_TIP_MIN, VR_TIP_MAX, ST_MIN, ST_MAX),
        map_range(t_s, VR_S_MIN, VR_S_MAX, SS_MIN, SS_MAX)
    )

# ============================================================
# Networking + Serial
# ============================================================
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

arduino = serial.Serial("COM4", 9600, timeout=1)
time.sleep(2)

last_vals = None

def send_full_hand(P, R, M, I, T):
    """Send packets in order P,R,M,I,T to match Arduino comments"""
    global last_vals
    # Each param is tuple (base, tip, splay)
    current = (*[clamp_angle(x) for x in P],
               *[clamp_angle(x) for x in R],
               *[clamp_angle(x) for x in M],
               *[clamp_angle(x) for x in I],
               *[clamp_angle(x) for x in T])

    if current == last_vals:
        return
    last_vals = current

    msg = (
        f"P:{current[0]},{current[1]},{current[2]};"
        f"R:{current[3]},{current[4]},{current[5]};"
        f"M:{current[6]},{current[7]},{current[8]};"
        f"I:{current[9]},{current[10]},{current[11]};"
        f"T:{current[12]},{current[13]},{current[14]};\n"
    )

    arduino.write(msg.encode())

# Helper to find bones
def get_bone(packet, name):
    for b in packet.get("bones", []):
        if b.get("name") == name:
            return b.get("x"), b.get("y"), b.get("z")
    return None

print("Listening for full hand data...")

# ============================================================
# Main Loop
# ============================================================
RATE_SLEEP = 0.01  # seconds between loops (~100 Hz); increase if needed

while True:
    try:
        data, _ = sock.recvfrom(8192)
        packet = json.loads(data.decode("utf-8"))

        # THUMB
        th_cmc = get_bone(packet, "XRHand_ThumbMetacarpal") #Splay rot (0), y axis rot (1)?
        th_mcp = get_bone(packet, "XRHand_ThumbProximal")
        th_tip  = get_bone(packet, "XRHand_ThumbDistal") #Bending (0)

        # INDEX
        i_base = get_bone(packet, "XRHand_IndexProximal")
        i_tip  = get_bone(packet, "XRHand_IndexDistal")

        # MIDDLE
        m_base = get_bone(packet, "XRHand_MiddleProximal")
        m_tip  = get_bone(packet, "XRHand_MiddleDistal")

        # RING
        r_base = get_bone(packet, "XRHand_RingProximal")
        r_tip  = get_bone(packet, "XRHand_RingDistal")

        # PINKY
        p_base = get_bone(packet, "XRHand_LittleProximal")
        p_tip  = get_bone(packet, "XRHand_LittleDistal")

        # Ensure required data exists before mapping
        if all([th_cmc, th_mcp, th_tip,
                i_base, i_tip,
                m_base, m_tip,
                r_base, r_tip,
                p_base, p_tip]):

            #print(th_tip[0], th_cmc[1], th_cmc[0])

            P = pinky_angles_to_servos(p_base[0], p_tip[0], p_base[1])
            R = ring_angles_to_servos(r_base[0], r_tip[0], r_base[1])
            M = middle_angles_to_servos(m_base[0], m_tip[0], m_base[1])
            I = index_angles_to_servos(i_base[0], i_tip[0], i_base[1])
            T = thumb_angles_to_servos(th_tip[0], th_cmc[2], th_cmc[0])
            #T = thumb_angles_to_servos(180, 20, 50)

            #print(th_tip[0], th_cmc[2], th_cmc[0])
            # Optional: debugging print
            # print("P,R,M,I,T:", P, R, M, I, T)

            send_full_hand(P, R, M, I, T)

        time.sleep(RATE_SLEEP)

    except BlockingIOError:
        pass
    except KeyboardInterrupt:
        break
    except Exception as e:
        print("Error:", e)

sock.close()
arduino.close()
