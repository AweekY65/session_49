# rigid2d — 自实现二维刚体物理仿真核心

一个**纯 C++17、无外部依赖**的二维刚体物理引擎核心。碰撞检测、冲量求解、位置修正、
摩擦、休眠全部自行实现；不依赖 Unity / Unreal / PhysX / Box2D / 任何在线服务。
所有物体状态、碰撞流形与仿真结果只存在于进程内存中。测试与示例均在终端运行，
**不需要任何渲染窗口**。

- 形状：圆形（circle）与轴对齐矩形（AABB）
- 刚体属性：位置、线速度、质量、恢复系数（restitution）、摩擦系数、静态标志
- 固定时间步 + 自适应子步长积分（半隐式欧拉），结果确定可复现
- 碰撞对：circle–circle、AABB–AABB、circle–AABB，输出法线与穿透深度
- 基于 sequential impulse 的碰撞响应，静态物体绝不获得速度
- Coulomb 摩擦（切向冲量）+ 接触图（contact-island）整岛休眠，抑制静止堆叠抖动
- 对极端质量比、零/非法质量、初始重叠、高速穿透均有明确处理规则

## 目录结构

```
include/rigid2d/
  vec2.hpp        二维向量（点积/叉积/长度/归一化/逐分量 clamp）
  body.hpp        刚体 Body（形状、质量、恢复、摩擦、静态、休眠）
  collision.hpp   碰撞流形 Contact 与三种形状对检测接口
  world.hpp       World / WorldConfig：积分、求解、休眠、诊断量
src/
  collision.cpp   三种碰撞检测实现（法线、穿透、signed gap）
  world.cpp       固定步积分、sequential-impulse 法向/摩擦求解、
                  Baumgarte 位置修正、推测接触、整岛休眠、自适应子步长
tests/            终端自动化测试（自带极简断言框架，无第三方库）
examples/         终端示例
Makefile
```

## 构建与测试

只需一个支持 C++17 的编译器（g++ / clang++），无第三方库：

```bash
make test     # 编译并运行全部测试（终端输出，无窗口）
make demo     # 编译并运行终端示例
make clean
```

单个命令等价于：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Iinclude -Itests \
    src/*.cpp tests/test_*.cpp -o build/rigid2d_tests
./build/rigid2d_tests
```

测试覆盖：自由落体、外力/冲量、弹性 / 非弹性碰撞（圆与盒）、静态地面、
多体堆叠、堆叠受击唤醒、摩擦停滑、确定性复现，以及极端质量比、非法质量、
零半径、初始重叠、高速防穿透、对静态物体施力等边界输入，并在适用条件下
校验动量守恒与动能不增。退出码 0 表示全部通过。

用 AddressSanitizer / UndefinedBehaviorSanitizer 检查：

```bash
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -Iinclude -Itests \
    src/*.cpp tests/test_*.cpp -o build/rigid2d_asan
./build/rigid2d_asan
```

## 使用方法

```cpp
#include "rigid2d/world.hpp"
using namespace rigid2d;

WorldConfig cfg;            // 重力 (0,-9.81)，固定步长 1/60 s
World world(cfg);

Body* ground = world.add(Body::aabb({0, -1}, {10, 1}, 1.0));
ground->makeStatic();                       // 静态：inverseMass = 0

Body* ball = world.add(
    Body::circle({0, 5}, 0.5, /*mass*/1.0,
                 /*restitution*/0.7, /*friction*/0.4));

world.applyImpulse(ball, {2.0, 0.0});       // 瞬时冲量 J/m
for (int i = 0; i < 600; ++i) {
    world.applyForce(ball, {1.0, 0.0});     // 每步外力（步末清零）
    world.step();                           // 恰好推进 fixedDt 秒
}
world.totalMomentum();      // 诊断：总线动量
world.totalKineticEnergy(); // 诊断：总动能
world.contacts();           // 最近一次流形（法线/穿透/gap）
```

## 数学与公式

### 1. 固定时间步积分（半隐式 / symplectic Euler）

每个固定步长 `h = fixedDt`（默认 1/60 s），内部按需要细分为若干子步。
对每个动态物体：

```
v ← v + (g + F/m) · h
x ← x + v · h
```

先更新速度再更新位置（半隐式欧拉，比显式欧拉能量行为更稳）。重力与累积外力
先作用于速度，再用速度移动位置。运算顺序固定、无随机数、不读墙钟，因此
**相同输入必然产生逐位相同的结果**（测试 `determinism_*` 校验）。

外力 `applyForce` 只在当前步有效，步末清零；`applyImpulse` 直接改变速度 `J/m`。

### 2. 碰撞检测与流形

统一约定：`Contact.normal` 为**单位向量，由物体 a 指向物体 b**；把
相对速度 `rv = vB − vA` 沿 `+normal` 方向视为**分离**。同时给出
`penetration`（穿透深度 ≥ 0）与 `gap = −penetration`（沿法线的有符号间距，
重叠为负）。

- **circle–circle**：`d = |xB − xA|`，`gap = d − (rA+rB)`，法线 `(xB−xA)/d`；
  圆心重合时无法线方向，退化为朝上 (0,1)，穿透取半径和，交由位置修正分离。
- **AABB–AABB**：两轴重叠量
  `ox = (hAx+hBx) − |dx|`、`oy = (hAy+hBy) − |dy|`，两轴都重叠才碰撞；
  沿**最小穿透轴**推出，法线取该轴、符号由相对位置决定。
- **circle–AABB**：圆心 clamp 到 AABB 得最近点 `q`，
  `gap = |q − xc| − r`；圆心在盒外时法线为 `(q − xc)/|q − xc|`（圆→盒），
  能正确处理面接触与角接触；圆心在盒内时取最近面，穿透为
  `r − 圆心到该面距离`。

求解器还使用一个**推测接触阈值（contact threshold = slop）**：两物体间距小于
该小量时仍保留流形（`gap > 0`、`penetration = 0`），使静止接触不会因位置修正
留下的极小缝隙而在下一步被误判为“全新碰撞”。

### 3. 冲量碰撞响应（sequential impulse）

对每个接触，设法线为 `n`、切向为 `t`，逆质量 `invMA、invMB`（静态/休眠为 0），
相对法向速度 `vn = (vB − vA)·n`（< 0 表示接近）。

每个接触维护三个**累积冲量**（跨速度迭代累加，满足不等式约束）：

- 支撑冲量 `Pn`（防穿透，可卸载但不能为负）
- 恢复冲量 `Pb`（一次性，只在新碰撞的首次迭代施加，**不做热启动**）
- 摩擦冲量 `Pt`，Coulomb 钳制 `|Pt| ≤ μ·Pn`

**法向（非穿透）**，每轮迭代：

```
dPn = −vn / (invMA + invMB)
dPn = max(−Pn, dPn)          # 单向：只推不拉，允许卸载到 0
vA ← vA − dPn·invMA·n
vB ← vB + dPn·invMB·n
```

**恢复系数**，仅在一次碰撞“片段（episode）”首次接触、且接近速度超过
`restitutionThreshold`（默认 1 m/s）时施加一次：

```
e  = max(eA, eB)
目标分离速度 = e·(−vn)
dPb = (e·(−vn) − vn) / (invMA + invMB),  钳制 Pb ≥ 0
```

低于阈值的弱碰撞按完全非弹性处理（e=0），从而静止接触不会反复微弹、
持续注入能量。弹性碰撞（e=1）时一维对心碰撞严格满足
`vA' = ((mA−mB)/(mA+mB)) uA + (2mB/(mA+mB)) uB`（另有对称式），测试逐项校验，
并验证动量守恒、动能不增。

**摩擦（Coulomb）**：切向取相对表面速度方向，

```
jt = −(rv·t) / (invMA + invMB)
Pt = clamp(prevPt + jt, −μ·Pn, +μ·Pn),   μ = sqrt(fA·fB)（几何均值）
vA ← vA − jt·invMA·t ;  vB ← vB + jt·invMB·t
```

静态物体 `invM = 0`：不出现在任何速度/位置更新中，**碰撞永远不会让静态物体
获得速度或位移**。

**热启动（warm starting）**：持续接触跨子步保留支撑冲量与摩擦冲量，并在求解前
重新施加，使静止堆叠在很少迭代内即承载住自身重量；恢复冲量特意**不**热启动，
否则静止接触会被重新“弹”一次。

### 4. 位置修正（Baumgarte，split impulse）

速度求解后单独做位置稳定，不改变速度（因此不注入动量）：

```
correction = max(penetration − slop, 0) / (invMA+invMB) · β
xA ← xA − correction·invMA·n
xB ← xB + correction·invMB·n
```

`β = baumgarte`（默认 0.2）、`slop`（默认 0.005 m）保留一点重叠以维持接触、
防止抖动。静态物体不承担位移。

### 5. 静止堆叠不累积能量（休眠）

- 速度低于 `restSpeed`（默认 0.05 m/s）开始累积静止时间；
- 用并查集把当前接触图划分为若干**接触岛**，判断每个岛是否被静态几何体支撑；
- 当一个**被支撑**的岛整体都足够慢、且静止最浅的成员也持续了 `sleepTime`
  （默认 0.5 s），把整岛一起休眠（速度清零、跳过求解）。短暂的数值尖峰只
  **暂停**计时而不是清空计时，因此数值抖动的堆叠仍能稳定入睡。
- 任何与运动物体的接触都会沿接触图把整串物体重新唤醒（掉落物砸醒堆叠）。

这样静止堆叠不会因为数值残差持续产生可见能量；即便关闭休眠，
`resting_stack_does_not_accumulate_energy` 也验证稳态动能不增长。

### 6. 高速物体：自适应子步长防穿透

每个固定步内，按当前最快动态物体的位移与最小形状尺寸计算子步数：

```
substeps = clamp( ceil( |v|max·fixedDt / (velocityFraction·shapeMin) ),
                  1, maxSubsteps )
```

默认单步位移不超过最小特征尺寸的 0.25，最多 64 子步。子步数只由位置/速度决定，
不含随机或墙钟，保持确定性。测试用 100 m/s 小球撞薄墙验证不会隧穿。

## 边界输入的明确处理规则

| 情形 | 规则 |
|------|------|
| 零质量 / 负质量 / 无穷 / NaN | 动态物体构造直接抛 `std::invalid_argument`；要不可动物体请用 `makeStatic()` |
| 想要静态物体 | `makeStatic()`：`mass=0、inverseMass=0`，不积分、不受力、不吃冲量、不移动 |
| 零 / 负半径或半厚 | 工厂函数抛异常，必须为有限正值 |
| 恢复系数越界 | 钳制到 `[0,1]`（>1 会注入能量，<0 无意义） |
| 摩擦为负 / NaN | 钳制为 ≥ 0 |
| 极端质量比（如 1e10:1） | 逆质量法天然稳定，轻体近似撞固定墙，重体速度变化在机器精度内；动量仍守恒 |
| 圆心重合 / 初始重叠 | 检测给出确定法线与穿透，位置修正把它们推出；不产生 NaN/Inf |
| 高速 | 自适应子步长把单步位移限制在形状尺寸的一小部分，防止隧穿 |
| 对静态物体施力/冲量 | 直接忽略 |

## 局限

- 仅线速度、无旋转（无角速度、无转动惯量、角冲量）；
- 碰撞为非旋转的圆与轴对齐矩形（无 OBB / 多边形 / SAT 旋转）；
- 宽相为 O(n²) 朴素遍历，面向教学与中小规模场景。
