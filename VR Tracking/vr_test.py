import socket
import json

def get_bone_angles(packet, bone_name):
    """
    Returns a tuple (x, y, z) of the specified bone's rotation angles.
    Returns None if the bone is not found.
    """
    bones = packet.get("bones", [])
    for bone in bones:
        if bone.get("name") == bone_name:
            return bone.get("x"), bone.get("y"), bone.get("z")
    return None

# --- UDP setup ---
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

print(f"Listening for hand data on {UDP_IP}:{UDP_PORT}...")

while True:
    try:
        data, addr = sock.recvfrom(8192)
        text = data.decode('utf-8')
        #print(f"\nFrom {addr}:")
        #print(text)

        # Try to parse JSON
        try:
            packet = json.loads(text)
            print("Parsed JSON:", get_bone_angles(packet,"XRHand_RingProximal" ))
        except json.JSONDecodeError:
            print("Invalid JSON")

    except BlockingIOError:
        pass
    except KeyboardInterrupt:
        print("\nExiting.")
        break
    except Exception as e:
        print(f"Error: {e}")

sock.close()


"""
List of bone names:
- XRHand_Palm
- XRHand_Wrist

- XRHand_ThumbMetacarpal
- XRHand_ThumbProximal
- XRHand_ThumbDistal
- XRHand_ThumbTip

- XRHand_IndexMetacarpal
- XRHand_IndexProximal
- XRHand_IndexIntermediate
- XRHand_IndexDistal
- XRHand_IndexTip

- XRHand_MiddleMetacarpal
- XRHand_MiddleProximal
- XRHand_MiddleIntermediate
- XRHand_MiddleDistal
- XRHand_MiddleTip

- XRHand_RingMetacarpal
- XRHand_RingProximal
- XRHand_RingIntermediate
- XRHand_RingDistal
- XRHand_RingTip

- XRHand_LittleMetacarpal
- XRHand_LittleProximal #splay Y-coord
- XRHand_LittleIntermediate 
- XRHand_LittleDistal
- XRHand_LittleTip
"""



