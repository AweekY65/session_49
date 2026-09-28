# physics2d — 二维刚体物理仿真核心

纯 Python 标准库实现的二维刚体物理引擎。**不依赖** Unity / Unreal / PhysX / 任何在线服务或外部物理引擎；所有状态只存在于本地内存。碰撞检测与响应核心逻辑全部自行实现，仅使用 `math` 基础数学库。

## 功能

- 刚体形状：圆形（circle）与轴对齐矩形（AABB）
- 属性：位置、速度、质量、恢复系数（restitution）、摩擦系数、静态标记
- 固定时间步积分（半隐式欧拉），施加重力与外力
- 碰撞检测：circle-circle、AABB-AABB、circle-AABB，输出法线与穿透深度
- 基于冲量（impulse）的碰撞响应 + Coulomb 摩擦
- 完全确定性：相同输入 + 相同步数 ⇒ 逐位相同的结果

## 目录结构

```
physics2d/
  vec2.py       # 二维向量
  body.py       # Body：圆形 / AABB 刚体
  collision.py  # 窄相碰撞检测，生成 Manifold（法线 + 穿透深度 + 接触点）
  world.py      # World：固定步长积分、冲量求解器、位置修正
tests/
  test_physics.py
```

## 积分方式

固定时间步 `dt`（如 1/60 s），半隐式欧拉：

```
v += (g + F/m) * dt        # 先更新速度
x += v * dt                # 再更新位置
```

外力通过 `body.apply_force(f)` 累积，每步结束后自动清零。每步内求解器迭代 10 次冲量。仿真无任何随机性与挂钟依赖，保证确定性。

## 碰撞检测

`collide(a, b)` 返回 `Manifold(normal, penetration, contact)`，法线方向恒为 A → B：

- **circle-circle**：圆心距 < 半径和则碰撞；法线为圆心连线方向，穿透深度 = 半径和 − 圆心距。圆心重合时取确定性法线 `(1, 0)`。
- **AABB-AABB**：两轴重叠量均 > 0 则碰撞；沿穿透最小轴给出法线与深度。
- **circle-AABB**：求圆心在 AABB 上的最近点，距离 < 半径则碰撞；圆心在盒内时沿最近面推出。

## 碰撞响应公式

法向冲量（无旋转，质心冲量）：

```
j = -(1 + e) * (v_b - v_a)·n / (1/m_a + 1/m_b)
v_a -= j * n / m_a ;  v_b += j * n / m_b
```

- `e = max(e_a, e_b)`；当法向接近速度 < 1.0（`RESTITUTION_VELOCITY_THRESHOLD`）时强制 `e = 0`，避免静止堆叠因数值误差持续微弹跳、积累能量。
- 摩擦：法向冲量作用后重算切向相对速度，`jt = -(rv·t) / (1/m_a+1/m_b)`，并按 Coulomb 锥截断 `|jt| ≤ μ·j`，`μ = sqrt(μ_a·μ_b)`。
- 位置修正（防沉陷）：`correction = 0.8 * max(penetration - 0.01, 0) / (1/m_a+1/m_b)`，按质量比把两物体沿法线推开（Baumgarte/slop 方案）。

## 边界与极端输入规则

| 输入 | 规则 |
|---|---|
| 质量 ≤ 0 | 视为无限大质量（`inv_mass = 0`），与静态物体一样不可动 |
| 静态物体（`is_static=True`） | 永不获得速度，每步速度强制清零 |
| 极端质量比（如 1e6 : 1e-6） | 冲量按质量比分配，重物体几乎不动，数值保持稳定 |
| 初始重叠（含圆心重合） | 由位置修正逐步分离；重合圆取确定性法线 `(1,0)` |
| 高速物体 | 速度每步截断到 `MAX_SPEED = 1000`，限制单步穿透距离（无 CCD，超过该速度的隧穿不保证检测） |
| restitution / friction | 分别截断到 `[0,1]` 与 `[0,∞)` |
| `dt ≤ 0`、非法形状、非正半径/半边长 | 抛出 `ValueError` |

## 运行测试

```bash
python3 -m pytest tests/ -q
```

测试全部在终端执行，不依赖任何渲染窗口。覆盖：自由落体（对照解析解）、弹性碰撞（动量 + 动能守恒，误差 < 1e-6）、完全非弹性碰撞（动量守恒、能量不增）、静态地面、弹性反弹、三箱体堆叠稳定性（残余动能 ≈ 0）、摩擦减速、零/负质量、极端质量比、初始重叠分离、高速截断、非法输入等。

## 简单示例

```python
from physics2d import Body, Vec2, World

w = World(gravity=Vec2(0, -9.81))
w.add_body(Body.aabb(Vec2(0, -0.5), 50, 0.5, is_static=True))   # 地面
ball = w.add_body(Body.circle(Vec2(0, 5), radius=1, mass=1, restitution=0.5))

dt = 1.0 / 60.0
for _ in range(240):          # 前进 4 秒
    w.step(dt)
print(ball.position, ball.velocity)
```
