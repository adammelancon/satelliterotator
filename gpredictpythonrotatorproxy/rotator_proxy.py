#!/usr/bin/env python3
# /// script
# dependencies = ["pyserial"]
# ///
import socket
import threading
import argparse
import logging
import serial
import sys

logging.basicConfig(
    format="%(asctime)s %(levelname)s %(message)s", level=logging.DEBUG
)

class RotctldProxyHandler(threading.Thread):
    def __init__(self, client_sock, rot_serial, lock):
        super().__init__(daemon=True)
        self.client = client_sock
        self.rot = rot_serial
        self.lock = lock

    def run(self):
        peer = self.client.getpeername()
        logging.info("Handler started: GPredict/rotctld → %s, serial → %s @%d",
                     peer, self.rot.port, self.rot.baudrate)

        while True:
            try:
                data = self.client.recv(256)
            except (ConnectionResetError, ConnectionAbortedError):
                logging.info("Connection closed by GPredict/rotctld")
                break
            if not data:
                logging.info("No data; closing handler")
                break

            cmd = data.decode('ascii', errors='ignore').strip('\r\n')
            if not cmd:
                continue

            logging.info("→ rotctld command: '%s'", cmd)

            # 1) get_pos: lowercase 'p'
            if cmd == 'p':
                try:
                    with self.lock:
                        self.rot.reset_input_buffer()
                        self.rot.write(b"AZ\r")
                        az_line = self.rot.read_until(b"\r", size=256).decode('ascii', errors='ignore').strip()
                        self.rot.reset_input_buffer()
                        self.rot.write(b"EL\r")
                        el_line = self.rot.read_until(b"\r", size=256).decode('ascii', errors='ignore').strip()

                    az = az_line[3:].strip() if az_line.upper().startswith("AZ ") else az_line
                    el = el_line[3:].strip() if el_line.upper().startswith("EL ") else el_line

                    resp = f"{az}\n{el}\n"
                    logging.info("← get_pos response:\n%s", resp)
                    self.client.sendall(resp.encode('ascii'))
                    continue
                except Exception as e:
                    logging.error("Error in get_pos: %s", e)

            # 2) set_pos: uppercase 'P '
            elif cmd.startswith('P ') and len(cmd.split()) >= 3:
                try:
                    _, az_txt, el_txt = cmd.split(maxsplit=2)
                    target_az = float(az_txt)
                    target_el = float(el_txt)
                    with self.lock:
                        self.rot.reset_input_buffer()
                        self.rot.write(f"AZ {target_az:.1f}\r".encode('ascii'))
                        self.rot.read_until(b"\r", size=256)
                        self.rot.reset_input_buffer()
                        self.rot.write(f"EL {target_el:.1f}\r".encode('ascii'))
                        self.rot.read_until(b"\r", size=256)
                    self.client.sendall(b"RPRT 0\n")
                    logging.info("← set_pos ack: RPRT 0")
                    continue
                except Exception as e:
                    logging.error("Error in set_pos: %s", e)

            # 3) status
            elif cmd.upper() == 'S':
                self.client.sendall(b"S0\n")
                logging.info("← status: S0")
                continue

            # 4) quit
            elif cmd.upper() in ('Q', 'q'):
                logging.info("Received quit; forwarding to ESP and closing handler")
                try:
                    self.rot.write(b"Q\r")
                except Exception:
                    pass
                break

            # 5) unknown
            else:
                self.client.sendall(b"RPRT -1\n")
                logging.info("← unknown cmd, replied RPRT -1")
                continue

        self.client.close()
        logging.info("Handler exited")


def main():
    parser = argparse.ArgumentParser(description="rotctld→GS-232 proxy")
    parser.add_argument('--listen-host', default='127.0.0.1')
    parser.add_argument('--listen-port', type=int, default=7777)
    parser.add_argument('--serial-port', default='COM12')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--timeout', type=float, default=1.0)
    args = parser.parse_args()

    try:
        rot_ser = serial.Serial(
            port=args.serial_port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=args.timeout
        )
    except serial.SerialException as e:
        logging.error("Could not open serial port %s: %s", args.serial_port, e)
        sys.exit(1)

    lock = threading.Lock()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.listen_host, args.listen_port))
    srv.listen(1)
    srv.settimeout(1.0)  # allow periodic checks for Ctrl+C
    logging.info("Proxy listening on %s:%d → %s @%dbaud",
                 args.listen_host, args.listen_port,
                 args.serial_port, args.baud)

    try:
        while True:
            try:
                client, addr = srv.accept()
            except socket.timeout:
                continue
            logging.info("🎯 GPredict connected from %s:%d", *addr)
            RotctldProxyHandler(client, rot_ser, lock).start()
    except KeyboardInterrupt:
        logging.info("Caught Ctrl+C, shutting down proxy...")
    finally:
        try:
            srv.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        srv.close()
        rot_ser.close()
        logging.info("Serial port closed, exiting.")


if __name__ == '__main__':
    main()
