#!/usr/bin/env python3
"""Client de test : simule un PAMI ou le robot qui parle à la Jetson en UDP.

Bibliothèque standard uniquement (compatible Python 3.8 -> 3.14).

Exemples :
    # demander un ID (cmd 1) puis écouter 30 s les messages de la Jetson
    python3 tools/test_client.py --jetson 192.168.1.50

    # simuler le départ de match côté équipe jaune (cmd 4), en plus
    python3 tools/test_client.py --jetson 192.168.1.50 --start Y

    # test en local, Jetson et client sur la même machine
    python3 tools/test_client.py --jetson 127.0.0.1
"""
import argparse
import socket
import time


def describe(line: str) -> str:
    cmd, *args = line.split("|")
    if cmd == "2" and args:
        return f"cmd 2 : identifiant attribué = {args[0]}"
    if cmd == "3":
        pairs = [f"PAMI {args[i]} -> zone {args[i + 1]}" for i in range(0, len(args) - 1, 2)]
        return "cmd 3 : assignations de dépôt : " + ", ".join(pairs)
    if cmd == "5":
        items = []
        for i in range(0, len(args) - 3, 4):
            mid, x, y, yaw = args[i:i + 4]
            items.append(f"#{mid} ({x} cm, {y} cm, {yaw}°)")
        return "cmd 5 : marqueurs : " + ", ".join(items)
    return f"cmd {cmd} : {args}"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--jetson", required=True, help="adresse IP de la Jetson")
    ap.add_argument("--port", type=int, default=5005, help="port d'écoute de la Jetson (WIFI_LISTEN_PORT)")
    ap.add_argument("--local-port", type=int, default=0, help="port local (0 = automatique)")
    ap.add_argument("--key", default="PAMI-TEST", help="identifiant matériel envoyé avec la cmd 1")
    ap.add_argument("--start", choices=["B", "Y"], help="envoie aussi cmd 4 (départ de match)")
    ap.add_argument("--duration", type=float, default=30.0, help="durée d'écoute en secondes")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.local_port))
    sock.settimeout(0.5)
    dest = (args.jetson, args.port)

    def send(line: str) -> None:
        sock.sendto((line + "\n").encode(), dest)
        print(f"-> {line}")

    send(f"1|{args.key}")  # demande d'ID : la Jetson mémorise aussi notre adresse
    if args.start:
        time.sleep(0.3)
        send(f"4|{args.start}")

    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        try:
            data, addr = sock.recvfrom(1500)
        except socket.timeout:
            continue
        for line in data.decode(errors="replace").splitlines():
            if line.strip():
                print(f"<- {describe(line.strip())}   [{addr[0]}:{addr[1]}]")


if __name__ == "__main__":
    main()
