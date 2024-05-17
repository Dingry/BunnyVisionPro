import pickle
import time

import tyro
from avp_stream import VisionProStreamer


def main(avp_ip: str):
    """
    Fetch the data from avp_stream and dump it to an offline pkl file

    Args:
        avp_ip: the ip_address of your Apple VisionPro inside a local network.
    """
    s = VisionProStreamer(avp_ip, True)

    time.sleep(30)
    print(len(s.recording))
    with open("data/your_offline_avp_stream.pkl", "wb") as f:
        pickle.dump(s.recording, f)


if __name__ == "__main__":
    tyro.cli(main)
