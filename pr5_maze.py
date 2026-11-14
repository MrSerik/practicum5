# pr5_maze.py - ПР5, вариант Б в CoppeliaSim: обход препятствий в мини-лабиринте, CoppeliaSim 4.6+
# pip install coppeliasim-zmqremoteapi-client numpy matplotlib
#
# Это перенос листинга 2 (функция avoidObstacles) с Arduino на Python:
# тот же конечный автомат FORWARD -> BACKWARD -> TURN, те же параметры варианта.
# Вместо HC-SR04 - передние ультразвуковые датчики Pioneer P3-DX,
# вместо ШИМ на L298N - скорости колёс по формулам ПР4.
#
# Перед запуском: открыта сцена pr5_maze.ttt (её создаёт build_maze.py), симуляция ОСТАНОВЛЕНА,
# файл build_maze.py лежит в той же папке.
# Запуск:  python3 pr5_maze.py
# Результат: данные для таблицы 5 в консоли, maze_path.png, maze_dist.png, maze_log.csv
#
# Испытания Б1-Б3: мышью поставьте робота перед стеной / стенкой под 45 град / ножкой стула,
# сохранять сцену не нужно, задайте T_RUN = 10 и запустите программу.
import csv
import math
import random
import signal
import numpy as np
import matplotlib.pyplot as plt
import zmq
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from build_maze import draw_maze, set_flags, const

# ============ ИСХОДНЫЕ ДАННЫЕ ВАРИАНТА (меняет студент, таблица 6) ============
BASE = 140                  # базовая скорость, ШИМ 0...255
D_STOP, D_SLOW = 15, 35     # пороги остановки и снижения скорости, см
TURN_MODE = 1               # 0 - всегда вправо, 1 - чередование, 2 - случайно
T_BACK, T_TURN = 400, 300   # отъезд назад и минимальный поворот, мс (как в листинге 2)
T_RUN = 120                 # длительность испытания, с (Б4 - 2 мин)
# ============================================================================
V_MAX = 0.50   # скорость колеса при ШИМ 255, м/с (модель «ШИМ -> скорость» вместо мотор-редуктора)
R_W = 0.0975   # радиус колеса P3-DX, м
CONE = 35      # сонары в пределах ±CONE град от курса вместе играют роль одного HC-SR04.
               # Pioneer в 2-2,5 раза шире учебного шасси: один луч ±10 град не видит углы корпуса,
               # поэтому берутся 4 передних сонара (±10 и ±30 град). CONE = 15 - только два центральных.
NAMES = ("FORWARD", "BACKWARD", "TURN")
FORWARD, BACKWARD, TURN = 0, 1, 2

client = RemoteAPIClient()
sim = client.require("sim")
client.socket.setsockopt(zmq.RCVTIMEO, 15000)  # не ждать ответа CoppeliaSim дольше 15 с
HANG = ("\nCoppeliaSim не отвечает 15 с. Нажмите Stop в CoppeliaSim и закройте открытые "
        "диалоговые окна. Не помогло - закройте CoppeliaSim, запустите ./coppeliaSim.sh, "
        "откройте сцену и запустите программу снова.")
W = sim.handle_world

maze = sim.getObject("/Maze", {"noError": True})
if maze == -1:
    raise SystemExit("В сцене нет лабиринта. Откройте pr5_maze.ttt или запустите build_maze.py.")
robot = sim.getObject("/PioneerP3DX")
if sim.getObject("/PioneerP3DX/Script", {"noError": True}) != -1:
    raise SystemExit("У Pioneer остался встроенный Script - удалите его, иначе робот ездит сам.")
mL = sim.getObject("/PioneerP3DX/leftMotor")
mR = sim.getObject("/PioneerP3DX/rightMotor")


def pose():
    """Середина оси колёс и курс - как в ПР4."""
    xl, yl, _ = sim.getObjectPosition(mL, W)
    xr, yr, _ = sim.getObjectPosition(mR, W)
    return (xl + xr) / 2, (yl + yr) / 2, math.atan2(-(xl - xr), yl - yr)


# ---- «HC-SR04»: передние сонары Pioneer (ось z датчика смотрит вдоль курса) ----
_, _, phi0 = pose()
front = []
for h in sim.getObjectsInTree(robot, const(sim, "sceneobject_proximitysensor",
                                           "object_proximitysensor_type"), 0):
    m = sim.getObjectMatrix(h, W)
    a = math.degrees(math.atan2(m[6], m[2]) - phi0)
    a = (a + 180) % 360 - 180
    if abs(a) <= CONE:
        front.append(h)
        print(f"датчик {sim.getObjectAlias(h, 1)}: направление {a:+.0f} град от курса")
if not front:
    raise SystemExit("Не найдены передние ультразвуковые датчики Pioneer P3-DX.")

# ---- проверка касаний: робот и лабиринт - две коллекции ----
for h in sim.getObjectsInTree(robot, const(sim, "sceneobject_shape", "object_shape_type"), 0):
    set_flags(sim, h, collidable=True)
c_robot, c_maze = sim.createCollection(0), sim.createCollection(0)
sim.addItemToCollection(c_robot, sim.handle_tree, robot, 0)
sim.addItemToCollection(c_maze, sim.handle_tree, maze, 0)


def setMotors(left, right):
    """Как в листинге 2: ШИМ -255...255 для левого и правого колеса."""
    for motor, pwm in ((mL, left), (mR, right)):
        pwm = max(-255, min(255, pwm))
        sim.setJointTargetVelocity(motor, pwm / 255 * V_MAX / R_W)   # рад/с


dBuf, dIdx = [400.0] * 3, 0


def readDistanceCm():
    """Медиана трёх последних измерений, как в листинге 2. Нет эха -> 400 см."""
    global dIdx
    d = 400.0
    for h in front:
        res, dist, *_ = sim.readProximitySensor(h)
        if res > 0:
            d = min(d, dist * 100)
    dBuf[dIdx] = d
    dIdx = (dIdx + 1) % 3
    return sorted(dBuf)[1], d          # (медиана, сырое значение)


turnDir = 1                            # +1 - вправо, -1 - влево


def chooseDir():
    if TURN_MODE == 0:
        return 1
    if TURN_MODE == 1:
        return -turnDir
    return random.choice((1, -1))


# ======================== испытание ========================
st, tState = FORWARD, 0.0
log, stops = [], []                    # журнал шагов; моменты перехода в BACKWARD
touches = stuck = 0
was_hit, anchor, t_anchor, stuck_flag = False, None, 0.0, False

sim.setStepping(True)
sim.startSimulation()
try:
    sim.step()
    while (t := sim.getSimulationTime()) < T_RUN:
        millis = t * 1000
        d, d_raw = readDistanceCm()
        # ---------- конечный автомат: строка в строку с avoidObstacles() ----------
        if st == FORWARD:
            if d < D_STOP:
                st, tState = BACKWARD, millis
                setMotors(-BASE, -BASE)
                stops.append(t)
            elif d < D_SLOW:
                setMotors(BASE // 2, BASE // 2)
            else:
                setMotors(BASE, BASE)
        elif st == BACKWARD:
            if millis - tState > T_BACK:
                st, tState = TURN, millis
                turnDir = chooseDir()
                setMotors(turnDir * BASE, -turnDir * BASE)
        elif st == TURN:                       # поворот, пока путь не свободен
            if millis - tState > T_TURN and d > D_SLOW:
                st = FORWARD
        # ---------------------------------------------------------------------------
        sim.step()
        x, y, _ = pose()
        hit = sim.checkCollision(c_robot, c_maze)[0] > 0
        if hit and not was_hit:
            touches += 1
            print(f"  t = {t:5.1f} с: касание в точке ({x:+.2f}, {y:+.2f})")
        was_hit = hit
        if anchor is None or math.dist(anchor, (x, y)) > 0.05:   # сдвинулся на 5 см
            anchor, t_anchor, stuck_flag = (x, y), t, False
        elif t - t_anchor > 10 and not stuck_flag:
            stuck, stuck_flag = stuck + 1, True
            print(f"  t = {t:5.1f} с: застревание > 10 с в точке ({x:+.2f}, {y:+.2f})")
        log.append((t, x, y, d_raw, d, st, int(hit)))
except zmq.error.Again:
    raise SystemExit(HANG)
finally:
    try:
        sim.stopSimulation()
        sim.setStepping(False)
        sim.destroyCollection(c_robot)
        sim.destroyCollection(c_maze)
    except Exception:
        print("Не удалось остановить симуляцию из Python - нажмите Stop в CoppeliaSim.")

# ======================== результаты ========================
L = np.array(log)
t, xs, ys, draw, dfilt, sts, hits = L.T
path = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
# расстояние «после остановки»: минимум сонара за T_BACK после команды стоп
gaps = [draw[(t >= ts) & (t <= ts + T_BACK / 1000)].min() for ts in stops]
ok = touches <= 1 and stuck == 0

print(f"\nИспытание: {t[-1]:.0f} с, BASE = {BASE}, D_STOP / D_SLOW = {D_STOP} / {D_SLOW} см, "
      f"TURN_MODE = {TURN_MODE}")
print(f"  пройденный путь {path:.1f} м, средняя скорость {path / t[-1]:.2f} м/с")
print(f"  остановок перед препятствием: {len(stops)}", end="")
if gaps:
    print(f"; расстояние после остановки: мин {min(gaps):.1f} см, среднее {np.mean(gaps):.1f} см")
else:
    print()
print(f"  касаний: {touches} (критерий <= 1), застреваний > 10 с: {stuck} (критерий 0)")
print(f"  Б4: {'критерий приёмки ВЫПОЛНЕН' if ok else 'критерий приёмки НЕ выполнен'}")

with open("maze_log.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_s", "x_m", "y_m", "d_raw_cm", "d_median_cm", "state", "touch"])
    for r in log:
        w.writerow([f"{r[0]:.3f}", f"{r[1]:.4f}", f"{r[2]:.4f}", f"{r[3]:.1f}", f"{r[4]:.1f}",
                    NAMES[r[5]], r[6]])

# траектория на плане лабиринта
fig, ax = plt.subplots(figsize=(7, 7))
draw_maze(ax)
ax.plot(xs, ys, "b-", lw=1, label="траектория робота")
stop_xy = [(xs[t >= ts][0], ys[t >= ts][0]) for ts in stops]
if stop_xy:
    ax.plot(*zip(*stop_xy), "k.", ms=6, label="команда «стоп» (d < D_STOP)")
hit_on = np.flatnonzero(np.diff(np.r_[0, hits]) > 0)
if len(hit_on):
    ax.plot(xs[hit_on], ys[hit_on], "rx", ms=10, mew=2, label="касание")
ax.plot(xs[0], ys[0], "go", ms=9, label="старт")
ax.plot(xs[-1], ys[-1], "ks", ms=7, label="финиш")
ax.set(xlabel="x, м", ylabel="y, м",
       title=f"Обход препятствий: касаний {touches}, застреваний {stuck}")
ax.grid(alpha=0.3)
ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
plt.savefig("maze_path.png", dpi=150)

# показания дальномера и состояние автомата во времени
fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 5), sharex=True,
                             gridspec_kw={"height_ratios": [3, 1]})
a1.plot(t, np.minimum(draw, 150), color="0.7", lw=0.8, label="сонар (сырое, обрезано 150 см)")
a1.plot(t, np.minimum(dfilt, 150), "b-", lw=1, label="медиана 3 измерений")
a1.axhline(D_STOP, color="r", ls="--", label=f"D_STOP = {D_STOP} см")
a1.axhline(D_SLOW, color="orange", ls="--", label=f"D_SLOW = {D_SLOW} см")
a1.set(ylabel="d, см", ylim=(0, 155))
a1.grid(alpha=0.3)
a1.legend(fontsize=8, loc="upper right")
a2.step(t, sts, where="post", color="k")
a2.set(yticks=[0, 1, 2], yticklabels=NAMES, xlabel="t, с")
a2.grid(alpha=0.3)
plt.tight_layout()
plt.savefig("maze_dist.png", dpi=150)

print("\nСохранено: maze_path.png, maze_dist.png, maze_log.csv")
print("Закройте окна графиков, чтобы завершить программу (или Ctrl+C в терминале).")
signal.signal(signal.SIGINT, signal.SIG_DFL)
plt.show()
