"""
dashboard.py - zivy webovy dashboard pro telemetrii z radia

Cte JSON-lines telemetrii ze serioveho portu (stejne jako test_ctecka.py),
zaroven ji loguje do JSONL + CSV a streamuje do prohlizece, kde se kresli
prehledne grafy, pristroje (umely horizont, kompas) a mapa GPS stopy.

Pouziti:
    pip install pyserial
    python dashboard.py                     # zivy provoz z COM5
    python dashboard.py --port COM7         # jiny port
    python dashboard.py --list              # vypis dostupnych portu
    python dashboard.py --replay let_20260728_125751.jsonl
    python dashboard.py --replay <soubor> --speed 5    # 5x zrychlene prehravani

Prohlizec se otevre sam na http://127.0.0.1:8765
Podrobnejsi analyticky pohled (vice stranek, velky graf roll/pitch/yaw,
tabulka s detekci vybociujicich hodnot) je na /analyzer.html - bezi
na stejnem serveru a stejnych datech (--replay i zivy prijem).
Ukonceni Ctrl+C -> zapise se CSV.

Zadne zavislosti krome pyserial - server je na stdlib (http.server + SSE).
"""

import argparse
import csv
import json
import os
import queue
import sys
import threading
import time
import webbrowser
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ---- vychozi nastaveni ----
PORT = "COM5"
BAUD = 115200
HTTP_PORT = 8765
BACKLOG = 3000        # kolik poslednich vzorku dostane nove pripojeny prohlizec
# ---------------------------

HERE = os.path.dirname(os.path.abspath(__file__))

rows = []                 # vsechny vzorky (pro CSV na konci)
keys_seen = []            # poradi sloupcu, jak se objevily
backlog = []              # posledni vzorky pro nove klienty
clients = []              # fronty pripojenych prohlizecu
lock = threading.Lock()
stop_event = threading.Event()
# Pauza z prohlizece - tyka se jen --replay (zastavi postup prehravanim).
# Zivy prijem ze serioveho portu se pauzou NEPRERUSUJE - zaznam do JSONL/CSV
# ma bezet vzdy, i kdyz si clovek zrovna prohlizeni v prohlizeci pozastavi.
replay_paused = threading.Event()


def add_keys(d):
    for k in d:
        if k not in keys_seen:
            keys_seen.append(k)


def publish(sample):
    """Ulozi vzorek a rozesle ho vsem pripojenym prohlizecum."""
    with lock:
        rows.append(sample)
        add_keys(sample)
        backlog.append(sample)
        if len(backlog) > BACKLOG:
            del backlog[:len(backlog) - BACKLOG]
        targets = list(clients)
    for q in targets:
        try:
            q.put_nowait(sample)
        except queue.Full:
            pass  # pomaly klient - radeji zahodit vzorek nez brzdit cteni


# ---------------------------------------------------------------- zdroje dat

def open_serial(port, baud):
    import serial
    from serial.tools import list_ports
    try:
        return serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"Nepodarilo se otevrit {port}: {e}\n")
        print("Dostupne porty:")
        found = list(list_ports.comports())
        if not found:
            print("  (zadny) -> radio nejspis neni v USB Serial rezimu")
        for p in found:
            print(f"  {p.device}  {p.description}")
        sys.exit(1)


def serial_source(port, baud, jf):
    ser = open_serial(port, baud)
    print(f"Ctu {port} @ {baud} Bd")
    with ser:
        while not stop_event.is_set():
            raw = ser.readline().decode("utf-8", errors="replace").strip()
            if not raw:
                continue
            try:
                d = json.loads(raw)
            except json.JSONDecodeError:
                continue
            emit(d, jf)


def replay_source(path, speed, jf):
    """Prehraje ulozeny let - uzitecne pro ladeni dashboardu bez radia."""
    print(f"Prehravam {path} rychlosti {speed}x (Ctrl+C ukonci)")
    with open(path, "r", encoding="utf-8") as f:
        lines = [ln for ln in f if ln.strip()]
    prev_t = None
    for ln in lines:
        if stop_event.is_set():
            return
        while replay_paused.is_set():
            if stop_event.is_set():
                return
            time.sleep(0.1)
        try:
            d = json.loads(ln)
        except json.JSONDecodeError:
            continue
        # 't' je getTime() radia v jednotkach 10 ms -> odvod realne rozestupy
        t = d.get("t")
        if isinstance(t, (int, float)) and prev_t is not None:
            dt = (t - prev_t) / 100.0
            if 0 < dt < 5:
                time.sleep(dt / speed)
        if isinstance(t, (int, float)):
            prev_t = t
        d.pop("pc_time", None)
        emit(d, jf)
    print("Prehravani doslo na konec souboru - dashboard bezi dal, Ctrl+C ukonci.")
    while not stop_event.is_set():
        stop_event.wait(0.5)  # kratke useky misto jednoho nekonecneho cekani - jinak na Windows nedojde Ctrl+C


def emit(d, jf):
    now = time.time()
    sample = {
        "pc_time": datetime.fromtimestamp(now).strftime("%H:%M:%S.%f")[:-3],
        "_ts": round(now, 3),
        **d,
    }
    if jf:
        jf.write(json.dumps(sample, ensure_ascii=False) + "\n")
        jf.flush()
    publish(sample)


# ------------------------------------------------------------------- server

class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"   # spojeni se zavira -> SSE proud drzime rucne

    def log_message(self, *a):
        pass  # zadny access log do konzole

    def do_GET(self):
        path, _, query = self.path.partition("?")
        if path == "/":
            self.serve_file("dashboard.html")
        elif path == "/stream":
            self.serve_stream()
        elif path == "/pause":
            self.serve_pause(query)
        elif path.count("/") == 1 and path.endswith(".html"):
            # dalsi stranky (napr. /analyzer.html) vedle dashboard.html ve stejne slozce -
            # pouzivaji ten samy /stream, jen jiny pohled na data
            self.serve_file(path.lstrip("/"))
        else:
            self.send_error(404)

    def serve_pause(self, query):
        # ovlada jen postup prehravani u --replay; zivy prijem ze serioveho
        # portu tim nikdy neni dotcen (viz komentar u replay_paused)
        params = dict(p.split("=", 1) for p in query.split("&") if "=" in p)
        if params.get("on") == "1":
            replay_paused.set()
        else:
            replay_paused.clear()
        body = b"ok"
        self.send_response(200)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def serve_file(self, name):
        full = os.path.abspath(os.path.join(HERE, name))
        if os.path.dirname(full) != HERE:   # zadne unikani z adresare (../..)
            self.send_error(404)
            return
        try:
            with open(full, "rb") as f:
                body = f.read()
        except OSError:
            self.send_error(404, f"{name} nenalezen vedle dashboard.py")
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def serve_stream(self):
        q = queue.Queue(maxsize=2000)
        with lock:
            clients.append(q)
            history = list(backlog)
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        try:
            # nove pripojeny prohlizec dostane historii jednim balikem
            self.send_event("init", {"samples": history})
            while not stop_event.is_set():
                try:
                    sample = q.get(timeout=5)
                except queue.Empty:
                    self.wfile.write(b": ping\n\n")   # udrzi spojeni naive
                    self.wfile.flush()
                    continue
                batch = [sample]
                # slepi vzorky, ktere mezitim dosly - setri rezii
                while len(batch) < 100:
                    try:
                        batch.append(q.get_nowait())
                    except queue.Empty:
                        break
                self.send_event("data", {"samples": batch})
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with lock:
                if q in clients:
                    clients.remove(q)

    def send_event(self, name, payload):
        msg = f"event: {name}\ndata: {json.dumps(payload, ensure_ascii=False)}\n\n"
        self.wfile.write(msg.encode("utf-8"))
        self.wfile.flush()


def start_server(http_port):
    srv = ThreadingHTTPServer(("127.0.0.1", http_port), Handler)
    srv.daemon_threads = True
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    return srv


# ---------------------------------------------------------------------- CSV

def write_csv(csv_path):
    cols = [c for c in ("pc_time", "_ts", "t") if c in keys_seen]
    cols += [k for k in keys_seen if k not in cols]
    with open(csv_path, "w", newline="", encoding="utf-8") as cf:
        w = csv.DictWriter(cf, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)


# --------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="Zivy webovy dashboard telemetrie")
    ap.add_argument("--port", default=PORT, help=f"seriovy port (vychozi {PORT})")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--http-port", type=int, default=HTTP_PORT)
    ap.add_argument("--replay", metavar="SOUBOR.jsonl",
                    help="misto radia prehraj ulozeny let")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="nasobek rychlosti prehravani (jen s --replay)")
    ap.add_argument("--list", action="store_true", help="vypis seriove porty a skonci")
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    if args.list:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(f"{p.device}  {p.description}")
        return

    if args.replay:
        jsonl_path = csv_path = None
    else:
        now = datetime.now()
        # slozka podle dne (12.8) a v ni podslozka podle casu letu (145252) -
        # kazdy let tak ma vlastni slozku s jsonl i csv pohromade
        flight_dir = os.path.join(HERE, f"{now.day}.{now.month}", now.strftime("%H%M%S"))
        os.makedirs(flight_dir, exist_ok=True)
        jsonl_path = os.path.join(flight_dir, "let.jsonl")
        csv_path = os.path.join(flight_dir, "let.csv")

    start_server(args.http_port)
    url = f"http://127.0.0.1:{args.http_port}"
    print(f"Dashboard bezi na {url}   (Ctrl+C = konec a ulozeni)")
    if jsonl_path:
        print(f"Log: {os.path.dirname(jsonl_path)}")
    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()

    jf = open(jsonl_path, "w", encoding="utf-8") if jsonl_path else None
    try:
        if args.replay:
            replay_source(args.replay, max(args.speed, 0.01), jf)
        else:
            serial_source(args.port, args.baud, jf)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if jf:
            jf.close()
        if rows and not args.replay:
            write_csv(csv_path)
            print(f"\nUlozeno {len(rows)} vzorku:")
            print(f"  CSV:   {csv_path}")
            print(f"  JSONL: {jsonl_path}")
        elif not rows:
            print("\nZadna data neprisla (zkontroluj port, rezim radia a prepinac).")


if __name__ == "__main__":
    main()
