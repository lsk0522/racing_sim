#!/usr/bin/env python3
# LSK H/SEQ Shifter — 설정 CLI (제작: Sunggwan)
"""LSK H/SEQ 쉬프터 설정 도구 (USB 시리얼, 펌웨어 0.6 프로토콜)

  python lsk_shifter.py ports                 연결된 쉬프터 찾기
  python lsk_shifter.py info | get | keys | status
  python lsk_shifter.py set gear_on 9.8 --save
  python lsk_shifter.py monitor [--hz 60]     실시간 θ·φ·게이트·기어
  python lsk_shifter.py wizard                캘리브레이션 안내 (중립 → 게이트 4개 → 앞 스톱 → 저장)
  python lsk_shifter.py save | load | defaults | factory | reboot
  python lsk_shifter.py backup cfg.json / restore cfg.json
  python lsk_shifter.py flash firmware.uf2    펌웨어 업데이트 (BOOTSEL 모드로 바꾼 뒤 RPI-RP2 드라이브에 복사)
  python lsk_shifter.py raw "set layout 1"    명령 그대로 보내기
포트는 자동으로 찾음 (USB VID 2E8A, 이름 "LSK HSEQ Shifter"). 직접 지정: --port COM5 / /dev/ttyACM0
필요: pip install pyserial
"""
import argparse, json, os, sys, time, string, shutil

try:
    import serial
    from serial.tools import list_ports
except ImportError:                      # pragma: no cover
    sys.exit("pyserial이 필요합니다: pip install pyserial")

PRODUCT = "LSK HSEQ Shifter"
VID = 0x2E8A                              # Raspberry Pi (RP2040)
GEAR_NAMES = {0: "N", 8: "R"}
GATE_NAMES = ["1-2", "3-4", "5-6", "7-R"]


def find_ports():
    out = []
    for p in list_ports.comports():
        txt = " ".join(str(x) for x in (p.description, p.product, p.manufacturer) if x)
        if PRODUCT.lower() in txt.lower() or "sunggwan" in txt.lower():
            out.insert(0, p.device)
        elif p.vid == VID:
            out.append(p.device)
    return out


class Shifter:
    def __init__(self, port=None, timeout=1.0):
        if port is None:
            ports = find_ports()
            if not ports:
                raise SystemExit("쉬프터를 못 찾음 — USB 연결 확인, 또는 --port 로 지정")
            port = ports[0]
        self.port = port
        self.s = serial.Serial(port, 115200, timeout=timeout)
        try:
            self.s.dtr = True                             # Arduino-Pico CDC는 DTR이 있어야 송신 (pyserial 기본값도 켬)
        except OSError:
            pass
        time.sleep(0.05)
        self.s.reset_input_buffer()

    def close(self):
        try:
            self.cmd("stream 0", quiet=True)
        except Exception:
            pass
        self.s.close()

    def readline(self, deadline):
        buf = b""
        while time.time() < deadline:
            c = self.s.read(1)
            if not c:
                continue
            if c in b"\r\n":
                if buf:
                    return buf.decode("utf-8", "replace")
                continue
            buf += c
        return None

    def cmd(self, line, timeout=1.5, quiet=False):
        """명령 한 줄 → 응답 JSON (실시간 스트림 줄은 건너뜀)"""
        self.s.write((line.strip() + "\n").encode())
        dl = time.time() + timeout
        while True:
            ln = self.readline(dl)
            if ln is None:
                raise TimeoutError(f"응답 없음: {line}")
            try:
                j = json.loads(ln)
            except ValueError:
                continue
            if "ok" in j:
                if not j["ok"] and not quiet:
                    raise RuntimeError(j.get("err", "오류"))
                return j

    def stream(self, hz):
        self.cmd(f"stream {hz}")

    def samples(self, timeout=1.0):
        dl = time.time() + timeout
        while True:
            ln = self.readline(dl)
            if ln is None:
                return
            try:
                j = json.loads(ln)
            except ValueError:
                continue
            if "s" in j:
                yield j["s"]
                dl = time.time() + timeout


def gear_name(g):
    return GEAR_NAMES.get(g, str(g))


def fmt_live(s):
    ok = "OK" if all(s["ok"]) else f"센서 오류 {s['ok']} (누적 {s['err']})"
    if s["mode"] == "SEQ":
        g = {1: "+ 밀기", -1: "− 당기기"}.get(s["latch"], "·")
    else:
        g = f"{gear_name(s['gear'])}  (게이트 {GATE_NAMES[s['gate']]})"
    bar = lambda v, r: "[" + "".join("|" if i == int((v / r + 1) * 10 + 0.5) else ("+" if i == 10 else "-") for i in range(21)) + "]"
    return (f"θ {s['th']:7.2f}° {bar(s['th'], 16)}  φ {s['ph']:7.2f}° {bar(-s['ph'], 32)}  "
            f"{s['mode']:3}  {g:20}  {ok}  센서 {s['hz'][1]} Hz / 루프 {s['hz'][0]} Hz")


def wait_enter(msg):
    input(f"\n▶ {msg}\n  준비되면 Enter ")


def avg_sample(sh, n=40):
    sh.stream(200)
    th = ph = 0.0; k = 0
    for s in sh.samples():
        th += s["th"]; ph += s["ph"]; k += 1
        if k >= n:
            break
    sh.stream(0)
    return th / max(k, 1), ph / max(k, 1)


def wizard(sh):
    print("캘리브레이션 안내 — 레버를 손으로 옮기면서 Enter를 누르세요. 언제든 Ctrl+C로 취소 (저장 안 됨).")
    cfg = sh.cmd("get")["cfg"]
    lay = cfg["layout"]
    wait_enter("① 중립: 3-4 게이트 가운데(아무 기어도 안 들어간 상태)에 두고 손을 떼세요")
    print("  ", sh.cmd("cal zero"))
    wait_enter("② 앞 스톱: 3단 방향(앞)으로 끝까지 밀고 잡고 계세요")
    print("  ", sh.cmd("cal throw"))
    for g, name in enumerate(GATE_NAMES):
        wait_enter(f"③ 게이트 {name}: 그 게이트의 중립(앞뒤 가운데)에 두고 옆으로 살짝 기대 두세요")
        th, ph = avg_sample(sh)
        print(f"   측정 φ = {ph:.2f}° (θ = {th:.2f}°)")
        print("  ", sh.cmd(f"cal gate {g}"))
    print("\n게이트 각도:", {k: v for k, v in sh.cmd("get")["cfg"].items() if k.startswith("gate")})
    if input("저장할까요? [Y/n] ").strip().lower() in ("", "y", "yes", "ㅛ"):
        print("  ", sh.cmd("save"))
    else:
        print("  ", sh.cmd("load", quiet=True), "(저장 안 함 — 이전 설정으로)")
    print(f"배열: {'도그레그' if lay else '표준'} — 바꾸려면: set layout {1 - lay} --save")


def find_uf2_drive(timeout=15.0):
    dl = time.time() + timeout
    cands = []
    while time.time() < dl:
        if os.name == "nt":
            cands = [f"{d}:\\" for d in string.ascii_uppercase]
        else:
            user = os.environ.get("USER", "")
            cands = [f"/media/{user}/RPI-RP2", "/media/RPI-RP2", f"/run/media/{user}/RPI-RP2", "/Volumes/RPI-RP2"]
        for c in cands:
            if os.path.exists(os.path.join(c, "INFO_UF2.TXT")):
                return c
        time.sleep(0.5)
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(description="LSK H/SEQ Shifter 설정 도구 — 제작: Sunggwan")
    ap.add_argument("--port")
    sp = ap.add_subparsers(dest="cmd", required=True)
    for c in ("info", "get", "keys", "status", "save", "load", "defaults", "factory", "reboot", "wizard", "ports"):
        sp.add_parser(c)
    p = sp.add_parser("set"); p.add_argument("key"); p.add_argument("value"); p.add_argument("--save", action="store_true")
    p = sp.add_parser("monitor"); p.add_argument("--hz", type=int, default=30)
    p = sp.add_parser("backup"); p.add_argument("file")
    p = sp.add_parser("restore"); p.add_argument("file"); p.add_argument("--no-save", action="store_true")
    p = sp.add_parser("flash"); p.add_argument("uf2")
    p = sp.add_parser("raw"); p.add_argument("line")
    a = ap.parse_args(argv)

    if a.cmd == "ports":
        ps = find_ports()
        print("\n".join(ps) if ps else "없음")
        return 0
    sh = Shifter(a.port)
    try:
        if a.cmd in ("info", "get", "keys", "status", "save", "load", "defaults", "factory"):
            r = sh.cmd(a.cmd) if a.cmd != "status" else None
            if a.cmd == "status":
                sh.s.write(b"status\n")
                r = next(sh.samples())
                print(fmt_live(r))
            elif a.cmd == "keys":
                for k in r["keys"]:
                    print(f"{k['k']:10} {k['lo']:>8g} ~ {k['hi']:<8g} {k['help']}")
            else:
                print(json.dumps(r, ensure_ascii=False, indent=1))
        elif a.cmd == "set":
            print(sh.cmd(f"set {a.key} {a.value}"))
            if a.save:
                print(sh.cmd("save"))
        elif a.cmd == "monitor":
            sh.stream(a.hz)
            print("Ctrl+C로 종료")
            try:
                for s in sh.samples(2.0):
                    sys.stdout.write("\r" + fmt_live(s) + "   ")
                    sys.stdout.flush()
            except KeyboardInterrupt:
                print()
        elif a.cmd == "wizard":
            try:
                wizard(sh)
            except KeyboardInterrupt:
                print("\n취소 —", sh.cmd("load", quiet=True))
        elif a.cmd == "backup":
            cfg = sh.cmd("get")["cfg"]
            info = sh.cmd("info")
            json.dump({"device": info, "cfg": cfg}, open(a.file, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
            print("저장:", a.file)
        elif a.cmd == "restore":
            cfg = json.load(open(a.file, encoding="utf-8"))["cfg"]
            # 관계 조건(off < on) 때문에 순서: 먼저 off를 낮추고, on, 그다음 나머지
            order = sorted(cfg, key=lambda k: (0 if k.endswith("_off") else 1 if k.endswith("_on") else 2))
            for k in order:
                r = sh.cmd(f"set {k} {cfg[k]}", quiet=True)
                if not r["ok"]:
                    print(f"  {k}: {r['err']}")
            if not a.no_save:
                print(sh.cmd("save"))
        elif a.cmd == "reboot":
            print(sh.cmd("reboot"))
        elif a.cmd == "flash":
            print("BOOTSEL 모드로 전환…")
            try:
                sh.cmd("bootsel", timeout=0.5, quiet=True)
            except Exception:
                pass
            sh.s.close()
            d = find_uf2_drive()
            if not d:
                raise SystemExit("RPI-RP2 드라이브를 못 찾음 — 탐색기에서 직접 .uf2를 복사하세요")
            shutil.copy(a.uf2, d)
            print(f"복사 완료 → {d}  (자동 재부팅)")
            return 0
        elif a.cmd == "raw":
            print(sh.cmd(a.line, quiet=True))
    finally:
        if sh.s.is_open:
            sh.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
