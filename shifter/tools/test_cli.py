# LSK H/SEQ Shifter — CLI ↔ 펌웨어 프로토콜 종단 시험 (PC 시뮬레이터 사용, 제작: Sunggwan)
# 사용: g++ -std=c++17 ../firmware/test/sim_host.cpp -o /tmp/lsk_sim && python3 test_cli.py /tmp/lsk_sim
import os, pty, subprocess, sys, json, time, tty
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lsk_shifter as L

sim = sys.argv[1] if len(sys.argv) > 1 else "/tmp/lsk_sim"
master, slave = pty.openpty()
tty.setraw(slave)
proc = subprocess.Popen([sim], stdin=master, stdout=master)
port = os.ttyname(slave)
fails = 0
def check(c, m):
    global fails
    print(("PASS " if c else "FAIL ") + m)
    fails += 0 if c else 1
try:
    sh = L.Shifter(port)
    i = sh.cmd("info"); check(i["maker"] == "Sunggwan" and i["proto"] == 1, "info")
    g = sh.cmd("get")["cfg"]; check(g["gear_on"] == 9.5 and len(g) == i["keys"], "get: 모든 키")
    check(sh.cmd("set gear_on 10")["gear_on"] == 10, "set")
    try:
        sh.cmd("set gear_off 12"); check(False, "관계 오류 거부")
    except RuntimeError as e:
        check("관계" in str(e), "관계 오류 거부")
    sh.stream(100); n = 0; gears = set()
    t = time.time()
    for s in sh.samples():
        n += 1; gears.add(s["gear"])
        if time.time() - t > 6.0: break
    sh.stream(0)
    check(n > 400, f"스트림 {n}개 / 6초 (100 Hz 요청)")
    check({3, 4} <= gears, f"가상 레버로 3·4단 판정: {sorted(gears)}")
    check(sh.cmd("cal zero")["ok"], "cal zero (스트림 끈 뒤 응답)")
    check(sh.cmd("save")["ok"] and sh.cmd("load")["ok"], "save / load")
    bk = "/tmp/lsk_bk.json"
    sh.s.close()
    check(L.main(["--port", port, "backup", bk]) == 0 and json.load(open(bk))["cfg"]["gear_on"] == 10, "CLI backup")
    check(L.main(["--port", port, "set", "layout", "1"]) == 0, "CLI set")
    check(L.main(["--port", port, "restore", bk, "--no-save"]) == 0, "CLI restore")
    sh = L.Shifter(port); check(sh.cmd("get")["cfg"]["layout"] == 0, "restore 결과"); sh.s.close()
finally:
    proc.kill()
print("ALL PASS" if not fails else f"{fails} FAILED")
sys.exit(fails)
