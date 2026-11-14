# build_maze.py - ПР5, вариант Б: мини-лабиринт 3,6 x 3,6 м для Pioneer P3-DX, CoppeliaSim 4.6+
# pip install coppeliasim-zmqremoteapi-client numpy matplotlib
#
# Перед запуском: CoppeliaSim открыта, симуляция ОСТАНОВЛЕНА.
# Удобнее всего открыть сцену pr4_diff.ttt из ПР4 (там уже стоит Pioneer без скрипта).
# Если робота в сцене нет, программа попробует добавить его сама.
# Запуск:  python3 build_maze.py
# Результат: лабиринт в сцене (объект Maze) и сохранённая сцена pr5_maze.ttt
#
# Вид сверху (клетка 1,2 м, робот стартует в центре S):
#
#   +-----------------------------------+
#   |                       |       /   |   / - стенка под 45 град (испытание Б2)
#   |  карман     o ножка   |     /     |   o - ножка стула d = 4 см (испытание Б3)
#   |                       |  карман   |
#   |-----------+                       |
#   |                                   |
#   |  ( ) ведро      S                 |   S - старт робота
#   |                       +-----------|
#   |           |                       |
#   |  карман   |  [ ] коробка   . банка|
#   |           |                       |
#   +-----------------------------------+
import math
import os

# ---------------- геометрия лабиринта, м ----------------
H = 1.8            # половина стороны арены (арена 3,6 x 3,6 м)
C = 0.6            # половина клетки (клетка 1,2 м: Pioneer 0,45 x 0,38 м должен разворачиваться)
WALL_T = 0.05      # толщина стен
WALL_H = 0.30      # высота стен и препятствий (сонары Pioneer примерно на 0,2 м)

WALLS = [          # отрезки (x1, y1, x2, y2)
    (-H, -H, H, -H), (H, -H, H, H), (H, H, -H, H), (-H, H, -H, -H),   # внешний борт
    (-H, C, -C, C),        # внутренние стены «вертушка»: в каждом углу -
    (C, C, C, H),          # тупиковый «карман» шириной 1,2 м
    (C, -C, H, -C),
    (-C, -H, -C, -C),
    (1.2, H, H, 1.2),      # стенка под 45 град в правом верхнем кармане (Б2)
]
POSTS = [          # круглые препятствия (x, y, диаметр, подпись)
    (0.0, 1.2, 0.04, "ножка стула"),   # узкое - сонар может не увидеть (Б3)
    (-1.2, 0.0, 0.20, "ведро"),
    (1.2, -1.2, 0.07, "банка"),
]
BOXES = [          # коробки (x, y, сторона, поворот в град, подпись)
    (0.0, -1.2, 0.30, 30, "коробка"),
]
START = (0.0, 0.0)  # робот ставится в центр лабиринта


def draw_maze(ax):
    """Нарисовать лабиринт на осях matplotlib (вид сверху) - для графиков pr5_maze.py."""
    from matplotlib.patches import Circle, Polygon
    for x1, y1, x2, y2 in WALLS:
        ax.add_patch(Polygon(_rect(x1, y1, x2, y2, WALL_T), color="0.35"))
    for x, y, d, _ in POSTS:
        ax.add_patch(Circle((x, y), d / 2, color="tab:orange"))
    for x, y, a, deg, _ in BOXES:
        ax.add_patch(Polygon(_square(x, y, a, deg), color="tab:orange"))
    ax.set(xlim=(-H - 0.1, H + 0.1), ylim=(-H - 0.1, H + 0.1), aspect="equal")


def _rect(x1, y1, x2, y2, t):
    """Углы прямоугольника-стены вокруг отрезка (с продлением на t/2 для стыков)."""
    L = math.hypot(x2 - x1, y2 - y1)
    ux, uy = (x2 - x1) / L, (y2 - y1) / L
    nx, ny = -uy * t / 2, ux * t / 2
    ax_, ay_ = x1 - ux * t / 2, y1 - uy * t / 2
    bx, by = x2 + ux * t / 2, y2 + uy * t / 2
    return [(ax_ + nx, ay_ + ny), (bx + nx, by + ny), (bx - nx, by - ny), (ax_ - nx, ay_ - ny)]


def _square(x, y, a, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [(x + c * px - s * py, y + s * px + c * py)
            for px, py in ((-a/2, -a/2), (a/2, -a/2), (a/2, a/2), (-a/2, a/2))]


def set_flags(sim, h, **flags):
    """Флажки объекта. CoppeliaSim 4.8+: sim.setBoolProperty; 4.6-4.7: старые функции."""
    try:
        for name, value in flags.items():
            sim.setBoolProperty(h, name, value)
        return
    except Exception:
        pass
    if "dynamic" in flags:
        sim.setObjectInt32Param(h, sim.shapeintparam_static, 0 if flags["dynamic"] else 1)
    if "respondable" in flags:
        sim.setObjectInt32Param(h, sim.shapeintparam_respondable, int(flags["respondable"]))
    bits = 0
    for name, const in (("collidable", "objectspecialproperty_collidable"),
                        ("measurable", "objectspecialproperty_measurable"),
                        ("detectable", "objectspecialproperty_detectable")):
        if flags.get(name):
            bits |= getattr(sim, const)
    if bits:
        sim.setObjectSpecialProperty(h, sim.getObjectSpecialProperty(h) | bits)


def const(sim, *names):
    """Константа API: в новых версиях CoppeliaSim некоторые имена изменились."""
    for n in names:
        try:
            return getattr(sim, n)
        except AttributeError:
            continue
    raise AttributeError(names[0])


# ======================== сборка сцены ========================
def main():
    import zmq
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    client = RemoteAPIClient()
    sim = client.require("sim")
    client.socket.setsockopt(zmq.RCVTIMEO, 15000)
    W = sim.handle_world

    if sim.getSimulationState() != sim.simulation_stopped:
        raise SystemExit("Остановите симуляцию в CoppeliaSim (кнопка Stop) и запустите снова.")

    def add_shape(kind, size, x, y, yaw, color, name, parent):
        h = sim.createPrimitiveShape(kind, size, 0)
        sim.setObjectPosition(h, [x, y, size[2] / 2], W)
        sim.setObjectOrientation(h, [0, 0, yaw], W)
        sim.setObjectAlias(h, name)
        sim.setShapeColor(h, None, sim.colorcomponent_ambient_diffuse, color)
        # неподвижное, твёрдое (робот в него упирается), видимое для сонаров и проверки касаний
        set_flags(sim, h, dynamic=False, respondable=True,
                  collidable=True, measurable=True, detectable=True)
        sim.setObjectParent(h, parent, True)
        return h

    # 1) старый лабиринт, если программу запускают повторно, - удалить
    old = sim.getObject("/Maze", {"noError": True})
    if old != -1:
        sim.removeObjects(sim.getObjectsInTree(old))
        print("Старый лабиринт удалён.")

    # 2) робот: найти или добавить, удалить его встроенный скрипт
    robot = sim.getObject("/PioneerP3DX", {"noError": True})
    if robot == -1:
        robot = load_pioneer(sim)
    script = sim.getObject("/PioneerP3DX/Script", {"noError": True})
    if script != -1:
        sim.removeObjects([script])
        print("Встроенный скрипт Pioneer удалён (иначе робот ездит сам).")
    x, y, z = sim.getObjectPosition(robot, W)
    sim.setObjectPosition(robot, [START[0], START[1], z], W)   # курс не меняем

    # 3) стены и препятствия
    maze = sim.createDummy(0.05)
    sim.setObjectAlias(maze, "Maze")
    sim.setObjectPosition(maze, [0, 0, 0], W)
    for i, (x1, y1, x2, y2) in enumerate(WALLS):
        L = math.hypot(x2 - x1, y2 - y1) + WALL_T
        add_shape(sim.primitiveshape_cuboid, [L, WALL_T, WALL_H],
                  (x1 + x2) / 2, (y1 + y2) / 2, math.atan2(y2 - y1, x2 - x1),
                  [0.80, 0.80, 0.75], f"Wall{i}", maze)
    for i, (x, y, d, _) in enumerate(POSTS):
        add_shape(sim.primitiveshape_cylinder, [d, d, WALL_H], x, y, 0,
                  [0.95, 0.55, 0.15], f"Post{i}", maze)
    for i, (x, y, a, deg, _) in enumerate(BOXES):
        add_shape(sim.primitiveshape_cuboid, [a, a, WALL_H], x, y, math.radians(deg),
                  [0.95, 0.55, 0.15], f"Box{i}", maze)

    path = os.path.abspath("pr5_maze.ttt")
    sim.saveScene(path)
    print(f"Лабиринт построен: {len(WALLS)} стен, {len(POSTS) + len(BOXES)} препятствия.")
    print(f"Сцена сохранена: {path}")
    print("Дальше: python3 pr5_maze.py")


def load_pioneer(sim):
    """Добавить Pioneer P3-DX из библиотеки моделей CoppeliaSim."""
    dirs = []
    for p in ("stringparam_resourcesdir", "stringparam_application_path"):
        try:
            dirs.append(sim.getStringParam(getattr(sim, p)))
        except Exception:
            pass
    for d in dirs:
        f = os.path.join(d, "models", "robots", "mobile", "pioneer p3dx.ttm")
        try:
            h = sim.loadModel(f)
            print(f"Pioneer P3-DX добавлен из {f}")
            return h
        except Exception:
            continue
    raise SystemExit("Робот не найден. Перетащите в сцену robots -> mobile -> pioneer p3dx "
                     "(или откройте pr4_diff.ttt) и запустите программу снова.")


if __name__ == "__main__":
    main()
