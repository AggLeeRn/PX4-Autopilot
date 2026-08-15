"""
log_telemetrie.py  (verze s knihovnou rich - plynuly zivy dashboard bez blikani)

Cte JSON-lines telemetrii z radia po seriovem portu, ukazuje zivou tabulku,
ktera se aktualizuje na miste, a uklada cely let do logu.

Pouziti:
    pip install pyserial rich
    - uprav PORT nize (zjisti pres:  python -m serial.tools.list_ports -v)
    - spust V TERMINALU:  python log_telemetrie.py
    - ukonci Ctrl+C  -> zapise se prehledny CSV

Vzniknou dva soubory:
    let_RRRRMMDD_HHMMSS.jsonl   syrovy log, radek po radku (bezpecny proti padu)
    let_RRRRMMDD_HHMMSS.csv     prehledna tabulka (na konci)
"""

import serial
import json
import csv
import sys
import time
from datetime import datetime
from serial.tools import list_ports

from rich.live import Live
from rich.table import Table
from rich.console import Console
from rich import box

# ---- nastaveni ----
PORT = "COM5"       # uprav dle sveho (viz list_ports)
BAUD = 115200
RENDER_HZ = 5       # jak casto prekreslovat tabulku
# -------------------

console = Console()
rows = []
keys_seen = []
latest = {}
LIVE = sys.stdout.isatty()


def add_keys(d):
    for k in d:
        if k not in keys_seen:
            keys_seen.append(k)


def make_table(n, rate):
    t = Table(
        title=f"[bold cyan]Telemetrie[/]   {n} vzorku   {rate:.1f} Hz",
        caption="Ctrl+C = konec a ulozeni",
        box=box.SIMPLE_HEAVY,
        expand=False,
        pad_edge=False,
    )
    t.add_column("senzor", style="cyan", no_wrap=True)
    t.add_column("hodnota", justify="right", style="bold white")
    for k in keys_seen:
        v = latest.get(k, "")
        if isinstance(v, float):
            v = f"{v:.3f}"
        t.add_row(k, str(v))
    return t


def open_serial():
    try:
        return serial.Serial(PORT, BAUD, timeout=1)
    except serial.SerialException as e:
        console.print(f"[red]Nepodarilo se otevrit {PORT}: {e}[/]\n")
        console.print("Dostupne porty:")
        found = list(list_ports.comports())
        if not found:
            console.print("  (zadny) -> radio nejspis neni v USB Serial rezimu")
        for p in found:
            console.print(f"  {p.device}  {p.description}")
        sys.exit(1)


def read_loop(ser, jf, update):
    """update(n, rate) se vola pro prekresleni; oddeluje logiku od zobrazeni."""
    n = 0
    t0 = time.time()
    last_render = 0.0
    while True:
        raw = ser.readline().decode("utf-8", errors="replace").strip()
        if not raw:
            continue
        try:
            d = json.loads(raw)
        except json.JSONDecodeError:
            continue

        d = {"pc_time": datetime.now().strftime("%H:%M:%S.%f")[:-3], **d}
        jf.write(json.dumps(d, ensure_ascii=False) + "\n")
        jf.flush()

        add_keys(d)
        rows.append(d)
        latest.update(d)
        n += 1

        now = time.time()
        if now - last_render >= 1.0 / RENDER_HZ:
            rate = n / (now - t0) if now > t0 else 0.0
            update(n, rate)
            last_render = now


def main(jsonl_path):
    ser = open_serial()
    with ser, open(jsonl_path, "w", encoding="utf-8") as jf:
        if LIVE:
            with Live(make_table(0, 0.0), console=console,
                      refresh_per_second=RENDER_HZ, screen=False) as live:
                read_loop(ser, jf, lambda n, r: live.update(make_table(n, r)))
        else:
            # presmerovany vystup -> zadny dashboard, jen obcasny status
            def status(n, r):
                if n % 20 == 0:
                    print(f"vzorku: {n}")
            read_loop(ser, jf, status)

    return jsonl_path


def write_csv(csv_path):
    cols = [c for c in ("pc_time", "t") if c in keys_seen]
    cols += [k for k in keys_seen if k not in cols]
    with open(csv_path, "w", newline="", encoding="utf-8") as cf:
        w = csv.DictWriter(cf, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)


if __name__ == "__main__":
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    jsonl_path = f"let_{stamp}.jsonl"
    csv_path = f"let_{stamp}.csv"

    try:
        main(jsonl_path)
    except KeyboardInterrupt:
        pass
    finally:
        if rows:
            write_csv(csv_path)
            console.print(f"\n[green]Ulozeno {len(rows)} vzorku[/]:")
            console.print(f"  CSV:   {csv_path}")
            if jsonl_path:
                console.print(f"  JSONL: {jsonl_path}")
        else:
            console.print("\n[yellow]Zadna data neprisla[/] (zkontroluj port, rezim radia a prepinac).")
