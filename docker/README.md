# INAV SITL × 虚幻引擎 使用说明

把 INAV 飞控当成 SITL 跑在容器里，通过 UDP 接到虚幻引擎里的拦截机机体上。

镜像 `inav-sitl:9.1` 是**自包含**的，里面已经有：

- SITL 二进制（`/opt/inav/inav_9.1.0_SITL`）
- **两份配置好的 eeprom**：`eeprom.default.bin`（`receiver_type = SIM`）和 `eeprom.msp.bin`（`receiver_type = MSP`）
- `entrypoint.sh`：自动播种 eeprom、自动补 `--simip`、给不出必要参数就快速失败
- `launch_sitl.sh` + `sitl.conf.example`：宿主侧的机队编排脚本（见第九节）

**除了镜像，宿主机上不需要放任何文件，也不需要挂任何卷。**

---

## 一、快速开始：`docker run` 怎么启动

前提：**UE 编辑器已经点了 Play**（关卡在运行，插件的端口表已经打出来）。顺序反了看第十节第 2 条。

### 1.1 最简形式（杆量来自 UE 里的 `SetRcChannel`）

```bash
docker run -d --name inav-sitl-0 --network host \
  -e SIM=ue -e SIM_IP=172.16.2.104 -e SIM_PORT=49000 \
  inav-sitl:9.1 \
  --tcpbaseport=5760 --chanmap=M01-01,M02-02,M03-03,M04-04
```

`SIM_IP` 填**跑 UE 的那台机器**的地址，见第四节。

> ⚠️ **`-e SIM=ue` 不能省。** `SIM_IP` 和 `SIM_PORT` 只有在 `SIM` 设了之后才会被读 —— 不设 `SIM` 就是纯 Configurator 模式，那两个变量**被静默忽略**，容器照样 Up、照样打印 `[EEPROM] Loaded`，只是**永远不连 UE**。这是本节最容易踩的坑：命令看起来"参数都给全了"。

### 1.2 MSP 模式（杆量来自算法）

和你平时那条只差**一个 `-e`**：

```bash
docker run -d --name inav-sitl-0 --network host \
  -e SIM=ue -e SIM_IP=172.16.2.104 -e SIM_PORT=49000 \
  -e RECEIVER=msp \
  inav-sitl:9.1 \
  --tcpbaseport=5760 --chanmap=M01-01,M02-02,M03-03,M04-04
```

不需要从宿主机找任何 eeprom 文件，也不需要写 `-e EEPROM_TEMPLATE=` 那种长路径。详见第三节。

> **UE 仍然必须在跑。** `RECEIVER=msp` 换掉的只是杆量来源，物理和传感器照旧走 UE（第三节末尾）。

### 1.3 纯 Configurator 模式（不连 UE）

**不设 `SIM`** 即可 —— 注意是**不设 `SIM` 这个变量**，不是不设 `SIM_IP`。飞控只监听 TCP，不建立仿真链路，适合单独验 MSP 链路：

```bash
docker run -d --name inav-sitl-test --network host \
  -e RECEIVER=msp \
  inav-sitl:9.1 \
  --tcpbaseport=5790 --chanmap=M01-01
```

三种写法的区别：

| 命令里有什么 | 结果 |
|---|---|
| `-e SIM=ue -e SIM_IP=... -e SIM_PORT=...` | 连 UE（**正常用法**） |
| `-e SIM_IP=... -e SIM_PORT=...`（没有 `SIM`） | **不连 UE**，那两个变量被静默忽略 |
| 什么都没有 | 不连 UE，`--simip` 回落到 `127.0.0.1` |

> **`--chanmap` 也要注意顺序**：`--chanmap` 排在 `--sim` 前面时，写错的映射会被静默吞掉（5.3 节）。走 entrypoint 不会遇到，因为 entrypoint 补的 `--sim` 永远排在前面。

### 1.4 多架

每架错开 `SIM_PORT` 和 `--tcpbaseport`（见第二节）：

```bash
docker run -d --name inav-sitl-1 --network host \
  -e SIM=ue -e SIM_IP=172.16.2.104 -e SIM_PORT=49001 \
  inav-sitl:9.1 \
  --tcpbaseport=5770 --chanmap=M01-01,M02-02,M03-03,M04-04
```

四架以上建议直接用 `launch_sitl.sh`（第九节），手敲容易错位。

### 1.5 怎么确认成功了

等约 12 秒（SITL 的握手窗口是 10 秒），看日志：

```bash
docker logs inav-sitl-0
```

四条**必须全中**（顺序不定）：

```
[SOCKET] ue address = 172.16.2.104:49000, fd=3
[SIM] Connection with UE simulator successfully established.     <- 和 UE 握上手
[EEPROM] Loaded '/data/eeprom.bin' (32768 of 32768 bytes)        <- 用的是配置好的 eeprom
[SOCKET] Bind TCP [::]:5760 to UART1                             <- MSP 端口绑上了
```

缺任何一条都算失败，对照第七节排查。纯 Configurator 模式下后两条不适用，看 `[EEPROM] Loaded` 和 `Bind TCP` 即可。

---

## 二、核心概念：VehicleIndex

**SITL 没有「车辆编号」这个参数。** 编号是**隐含在端口里**的：UE 那边第 `N` 架车监听 `49000 + N`，容器必须把 `-e SIM_PORT` 设成同一个数，两边才对得上。

| VehicleIndex | `--name`（随便起） | `-e SIM_PORT` | `--tcpbaseport` | MSP 客户端连 |
|---|---|---|---|---|
| 0 | inav-sitl-0 | **49000** | 5760 | 5760 |
| 1 | inav-sitl-1 | **49001** | 5770 | 5770 |
| 2 | inav-sitl-2 | **49002** | 5780 | 5780 |
| 3 | inav-sitl-3 | **49003** | 5790 | 5790 |

- `SIM_PORT` 填错 → **连不上**。这是最常见的错。
- `--tcpbaseport` 是 MSP / Configurator 用的。**只跑一个容器可以不写**（默认就是 5760）；跑多个必须错开，步长至少 8（一个容器要占 8 个连续端口，留给 8 个 UART），所以习惯上用 10。
- `--name` 纯粹是给你自己看的，不影响任何行为。

### 坑：UE 日志里的编号是 1 基的

插件从 pawn 名字结尾的数字推编号，而且**减 1**：

```
BP_InavInterceptorPawn1  ->  VehicleIndex 0  ->  udp/49000
BP_InavInterceptorPawn2  ->  VehicleIndex 1  ->  udp/49001
```

进 Play 时 UE 日志会打一行以 `INAV Sim:` 开头、带 `1:udp/49000` 字样的端口表：

```
INAV Sim:  -- 1:udp/49000 BP_InavInterceptorPawn1 | 2:udp/49001 BP_InavInterceptorPawn2. Start containers now.
```

**拿端口去对，别拿前面的编号去对。** 想让 2 号机飞，就是 `-e SIM_PORT=49001`。

> 插件里 `UInavSimBridge > VehicleIndex` 属性可以手动钉死（-1 = 按 pawn 名字自动推）。钉重复了是硬报错、不会自动退让，所以要么别动，要么记住钉的是哪个。

### 运行时生成的 pawn 必须显式指定 VehicleIndex

上面那套「从 pawn 名字推编号」只对**手动摆进关卡**的 pawn 成立。关卡里的 pawn 是你命名的，`BP_InavInterceptorPawn1` 的 `1` 就是编号。

**蓝图在运行时 Spawn 出来的 pawn 没有这个名字。** 引擎给它起的是自动名，那个名字里的数字是**重名计数器，不是你的编号** —— 而且它可能和手动摆的 pawn 撞上。实测过一次：spawn 出来的 pawn 标签正是 `BP_InavInterceptorPawn4`，而关卡里已经有一个同名的 `BP_InavInterceptorPawn4` 占着槽位 3。插件按名字推会得到槽位 3、发现被占、再退到**当前最小的空槽**，日志里留一行 Warning，然后就飞到一个你没打算的端口上去了。

所以运行时生成的 pawn 要在 spawn 节点上把编号钉死：

```
Spawn Actor from Class            ← Class 选 BP_InavInterceptorPawn
  ├─ Class: BP_InavInterceptorPawn
  ├─ Vehicle Index: 2             ← 就是这里，0 基
  └─ ...
```

这个 **Vehicle Index** 引脚是 `AInavInterceptorPawn` 上带 `ExposeOnSpawn` 的属性，插件的建议器会在 `FinishSpawning` **之前**把它写进去，而这个时刻早于桥接组件开始连接 —— 所以不管组件和 actor 的 BeginPlay 谁先跑，读到的一定是这个值。

用 `Spawn Actor Deferred` 也一样，节点上有同一个引脚；也可以在两个节点之间直接 `Set Vehicle Index`。

两个必须注意的点：

- **同一个编号不能钉两次。** 撞车是硬报错、不会自动退让：后一个 pawn 的链路**根本不会启动**（它会在没有飞控的情况下自由落体），日志里是 `pinned VehicleIndex N, but ... already holds that slot`。
- 引脚如果没出现，右键那个 spawn 节点 → **Refresh Nodes**。新加的 C++ 属性不会自己长到已有节点上去。

> 放在 pawn 上而不是只放在桥接组件上，是被 UE 的顺序逼的：组件是在 `FinishSpawning` 里才被创建出来的，而蓝图的 spawn 节点只能在 `SpawnActorDeferred` 和 `FinishSpawning` 之间设值 —— 那个窗口里组件**还不存在**。pawn 本身则一直都在。桥接组件自己的 `VehicleIndex` 仍然是 -1 时就会回退去读 pawn 上的这个值。

---

## 三、`SIM=` 和 `RECEIVER=`：谁决定什么

这两个变量回答的是**不同的问题**，都要有：

| | 回答什么 | 值域 | 决定了 |
|---|---|---|---|
| `SIM` | 连哪个**仿真器后端** | `rf` \| `xp` \| `ue` | 订阅哪些 dataref、传感器从哪读、电机往哪写 |
| `RECEIVER` | 杆量**从哪来** | `sim` \| `msp` | 播种哪份 eeprom（即 `receiver_type`） |

**它们正交**，`SIM=xp RECEIVER=msp` 是完全合理的组合 —— 这不是一个变量能表达的事。

### `SIM=ue` 本身不管杆量

它建立的是数据流：`ue.c` 订阅 `inav_ue/rc/channel[*]`，在 [ue.c:576](src/main/target/SITL/sim/ue.c#L576) 转成 PWM，**无条件**写进 `rxSimSetChannelValue()` 的静态缓冲区。

**但那个缓冲区只有 `receiver_type = SIM` 时才有人读** —— [rx.c:299](src/main/rx/rx.c#L299)：

```c
switch (rxConfig()->receiverType) {
    case RX_TYPE_MSP:  rxMspInit(...);  break;   // rcReadRawFn → rxMspReadRawRC
    case RX_TYPE_SIM:  rxSimInit(...);  break;   // rcReadRawFn → rxSimReadRawRC
    case RX_TYPE_NONE: rcReadRawFn = nullReadRawRC;  break;  // 没人读
}
```

所以：

| | `receiver_type=SIM` | `receiver_type=MSP` |
|---|---|---|
| **`SIM=ue`** | UE 的 `SetRcChannel` 生效 ✓ | dataref 照收照转换，**写进缓冲区没人读** |
| **不设 `SIM`** | 缓冲区永远空 → 无信号 | MSP 客户端控 ✓ |

左下角那格正是"链路全绿但永远解不了锁"的经典失败。

**结论：UE 里 `SetRcChannel` 那条路要两个条件同时成立 —— `SIM=ue` 且 `receiver_type=SIM`。**

### `RECEIVER` 的两份 eeprom

| `-e RECEIVER=` | 播种的模板（镜像内） | `receiver_type` | 杆量来源 |
|---|---|---|---|
| `sim`（默认，可省） | `/opt/inav/eeprom.default.bin` | `SIM` (3) | 仿真器 dataref（`SetRcChannel`） |
| `msp` | `/opt/inav/eeprom.msp.bin` | `MSP` (2) | MSP 客户端，`MSP_SET_RAW_RC` |

两份只差**一个字段** `receiver_type`，加上重算的 CRC16。

### `-e SIM=msp` 是别名，不是第四个后端

entrypoint 把它改写成 `SIM=ue` + MSP eeprom。**不能透传给固件**：[target.c:295](src/main/target/SITL/target.c#L295) 的 `--sim` 只认 `rf|xp|ue`，别的值打 `[SIM] Unsupported simulator msp.` 然后把 `sitlSim` 置成 `SITL_SIM_NONE`（[target.c:72](src/main/target/SITL/target.c#L72)）—— 那是**完全没有仿真链路**，UE 里什么都不会动，跟你要的效果正好相反。

组合行为：

| `SIM=` | `RECEIVER=` | 结果 |
|---|---|---|
| `msp` | 不设 | → `SIM=ue` + MSP eeprom |
| `msp` | `msp` | 同上，**不报警告**（说的是同一件事） |
| `msp` | `sim` | **警告**，以显式的 `RECEIVER` 为准 |
| 其他 | 任意 | 各管各的 |

### 两条链路是正交的

```
算法工程师 ──MSP_SET_RAW_RC──> INAV ──motor[]──> ue.c ──DREF──> UE 显示
                                ^                                |
                                └──── 传感器 (IMU/气压/GPS) ──────┘
```

换成 MSP **不是**"用 MSP 替代仿真器"，而是"用 MSP 替代仿真器的遥控通道"。物理和传感器照旧走 UE —— `simUeInit()` 的调用条件里没有 receiverType，电机输出和传感器写入也都不看接收机类型。**所以 UE 仍然必须在跑。**

---

## 四、`SIM_IP` 怎么填

`SIM_IP` = **跑虚幻引擎的那台机器**的地址。这是镜像唯一没法自带的东西，因为它取决于你的网络。

| 场景 | 填什么 |
|---|---|
| UE 和容器在**同一台机器**（WSL2 + Windows UE） | 填本机局域网 IP，**或者干脆不写** —— entrypoint 会自动用 WSL 默认网关，那正是 Windows 宿主机 |
| UE 在**另一台服务器**上 | 填那台机器的 IP，**必须显式写**，没有默认值 |

不写 `SIM_IP` 时：

- **WSL2 上**：自动回退到默认网关并打一行 warning，能正常工作。
- **其他任何机器上**：直接 `exit 2` 拒绝启动，**不会**静默去连网关。因为别处的默认网关是路由器，包发过去不会有人应答，症状表现为"握手 10 秒后超时"，极难排查。

> 回退**只在 `--network host` 下成立**。省掉 `--network host` 容器会进 default bridge，网关变成 docker bridge（`172.17.0.1`），那只到 WSL 虚拟机、到不了 Windows 上的 UE。

**防火墙**：容器从本机一个**临时源端口**往 `SIM_IP:49000 + N` 发订阅包，UE 把回复发回这个源端口（`ue.c` 不 `bind`，源端口由内核临时分配）。所以需要放行的是 **UE 那台机器的入站 UDP 49000 起这一段**（4 架就是 49000-49003/udp）—— 容器自己并不在 49000 上监听。

---

## 五、参数参考

### 5.1 容器环境变量（entrypoint 读的）

| 变量 | 默认 | 作用 |
|---|---|---|
| `SIM` | 不设 | 仿真后端 `rf` \| `xp` \| `ue`。**不设 = 纯 Configurator 模式**（只监听 TCP），且**`SIM_IP` / `SIM_PORT` 会被静默忽略**。`msp` 是别名，见第三节 |
| `SIM_IP` | 见第四节 | 仿真器（UE）所在机器地址。**只在 `SIM` 设了之后才被读** |
| `SIM_PORT` | `49000` | 仿真器端口 = `49000 + VehicleIndex`。**只在 `SIM` 设了之后才被读** |
| `RECEIVER` | `sim` | `sim` \| `msp`，选哪份内置 eeprom 播种 |
| `EEPROM_PATH` | `/data/eeprom.bin` | 活 eeprom 放哪 |
| `EEPROM_TEMPLATE` | 由 `RECEIVER` 决定 | 显式指定模板，**优先级高于 `RECEIVER`**；只有用镜像外的模板时才需要 |
| `RESEED` | 不设 | `1` = 强制丢弃现有 eeprom 重新播种 |
| `ALLOW_GATEWAY_FALLBACK` | `auto` | `auto` = 只在 WSL 上回退；`1` 强制开；`0` 强制关 |

### 5.2 SITL 命令行参数（`docker run` 里镜像名之后的那些）

这些是**直接传给固件二进制**的。entrypoint 会额外补上 `--path`（和 `--sim` / `--simip` / `--simport`，如果设了对应环境变量）。

| 参数 | 默认 | 作用 |
|---|---|---|
| `--path=<file>` | `eeprom.bin`（程序目录） | eeprom 文件路径。**由 entrypoint 接管**，手写会绕过播种逻辑 |
| `--sim=rf\|xp\|ue` | 不设 | 仿真后端。**由 `SIM` 环境变量接管**，别手写 |
| `--simip=<ip>` | `127.0.0.1` | 仿真器地址。**由 `SIM_IP` 接管**。省略会让 SITL 跟自己说话 |
| `--simport=<port>` | `49000` | 仿真器端口。**由 `SIM_PORT` 接管** |
| `--tcpbaseport=<port>` | `5760` | UART socket 的起始 TCP 端口，占 `SERIAL_PORT_COUNT`(8) 个连续端口 |
| `--chanmap=<mapstring>` | 不设 | 电机/舵机映射，见 5.3。**至少要映射一个电机** |
| `--useimu` | 关 | 用仿真器的 IMU 原始数据而不是直接取姿态。**实验性，不建议** |
| `--serialuart=<1-8>` | — | 串口接收机用哪个 UART（1-8），如 `3` = UART3 |
| `--serialport=<dev>` | — | 串口接收机连的宿主机设备，如 `COM3` / `/dev/ttyACM3`（长度 1-63） |
| `--baudrate=<n>` | `115200` | 串口接收机波特率，最小 1200 |
| `--stopbits=One\|Two` | `One` | 串口接收机停止位 |
| `--parity=Even\|None\|Odd` | `None` | 串口接收机校验位 |
| `--fcproxy` | 关 | 把 inav/betaflight 飞控当作串口接收机的代理 |
| `--help` | — | 打印选项后退出 |
| `--version` | — | 打印版本后退出 |

> **`--serialuart` / `--serialport` / `--baudrate` / `--stopbits` / `--parity` / `--fcproxy` 这一组是给「SITL 当串口接收机」用的**，和 UE 链路无关，本项目的部署里用不到。列在这里是为了完整性。

> ⚠️ **帮助文本和实现不一致的地方**（我照 `--help` 抄过一遍，实测发现对不上，以这里为准）：
>
> - `--stopbits` 的帮助文本写 `None|One|Two`，但 [target.c:217](src/main/target/SITL/target.c#L217) 的 `parseStopBits()` **只认 `One` 和 `Two`**，传 `None` 会打 `[stopbits] Invalid argument` 然后 `exit(0)`。
> - `--tcpbaseport` 的取值范围比帮助文本严格：必须 `1..65535`，**且 `basePort + 8 - 1` 不能超过 65535**（[target.c:376](src/main/target/SITL/target.c#L376)，8 = SITL 的 `SERIAL_PORT_COUNT`），否则同样 `exit(0)`。
> - 无效参数一律是 `exit(0)` —— **退出码 0**，容器看起来是"正常结束"。别拿退出码判断这类错误，看日志。

**UE 后端的硬限制**（[ue.h:87-89](src/main/target/SITL/sim/ue.h#L87-L89)）：最多 **4 个电机 + 4 个舵机**，超出会在启动时报 `[SIM] Mapping error` 并**放弃建立链路**。

### 5.3 `--chanmap` 是什么

「INAV 的电机编号 ↔ 仿真器输出槽位」的接线表。格式 `M<pwm>-<槽位>`：

```
M 01 - 01
│  │    └─ 仿真器那边的第几个输出槽（UE 机体上的电机位）
│  └────── INAV 这边的第几号电机
└───────── M = 电机，S = 舵机
```

`M01-01,M02-02,M03-03,M04-04` 是**恒等映射**：INAV 的 1/2/3/4 号电机依次送到 UE 的 1/2/3/4 号槽位。

电机编号在 INAV 这边由混控器（`QUADX_MMIX`）定，UE 机体那边四个电机座有自己的顺序，这张表是两边唯一的对齐点。**如果哪台电机在 UE 里转错了角，调的是这里，不是固件。**

三个会静默失败的地方：

- 每个 token **必须正好 6 个字符**，数字必须是**两位**，`M1-1` 之类会被拒（[target.c:256](src/main/target/SITL/target.c#L256)）。
- **至少要映射一个电机**，否则 SITL 打 `No motor mapped, check --chanmap.` 然后不建立链路 —— 但进程**不会退出**，表现为"容器 Up、日志没红字、UE 连不上"，跟端口填错长得一模一样。
- **`--chanmap` 的合法性校验是顺序相关的。** 校验条件是 `!parseMapping(optarg) && sitlSim != SITL_SIM_NONE`（[target.c:307](src/main/target/SITL/target.c#L307)），所以 `--chanmap` 排在 `--sim` **前面**时，`sitlSim` 还是 `NONE`，**写错的 chanmap 会被静默吞掉**、当作没给。手敲时把 `--sim` 放前面，或者干脆别手写（entrypoint 补的 `--sim` 永远排在前面，所以走 entrypoint 不会遇到）。

**除非 UE 里发现电机位置反了，否则保持恒等映射。**

### 5.4 退出码

entrypoint 的失败都是**快速且响亮**的，不会给你留一个"看起来正常"的容器：

| 码 | 含义 |
|---|---|
| 1 | 镜像里找不到 SITL 二进制 |
| 2 | 没给 `SIM_IP`，且这台机器推不出来 |
| 3 | eeprom 不可用：找不到可写位置，或 `RECEIVER` 值没有对应模板 |
| 139 | SIGSEGV —— 固件打不开 eeprom。绕过 entrypoint 直接给 `--path` 才会遇到 |

---

## 六、MSP 客户端

`docker/msp_rc_client.py` 是一份**参考实现**，不是库 —— 四十行协议加一段启动序列，方便照抄到算法里：

```bash
python3 msp_rc_client.py --port 5760 --arm
python3 msp_rc_client.py --port 5760 --arm --seconds 10 --throttle 0.3
```

端口是 `5760 + 10 * VehicleIndex`。参数：`--host --port --rate --seconds --arm --throttle --roll --pitch --yaw`。

### 五个必须知道的坑

1. **必须持续发。** MSP 接收机 200ms 收不到包就报无信号（`rx/msp.c` 的 `rxSignalTimeout = DELAY_5_HZ`），直接进 failsafe。**50Hz 起**，而且飞机在天上的每一秒都要发。

2. **读写通道序不一样。** `MSP_SET_RAW_RC` 发的是 rcmap **之前**的原始序：`0=Roll, 1=Pitch, 2=Throttle, 3=Yaw`（油门在 **2**）。而 `MSP_RC`(105) 回读的是内部序，油门在 **3**。**把读回来的原样发回去，油门和偏航就对调了。** 这是这套东西最常见的翻车点。

3. **没发的通道会被清零。** `rxMspFrameReceive` 把没覆盖到的通道全部置 0，包括 AUX。所以帧要发满，或者至少每次都把解锁通道带上。

4. **解锁是个通道，不是命令。** MSP 里没有"arm"这个报文。客户端要把 AUX1 拉高并**一直保持**。阈值由 ARM 那个模式框的 range 决定 —— 参考实现直接给 2000，不猜。

5. **`DATA_ROOT` 会和 `RECEIVER` 打架。** 模板只在 eeprom **不存在**时才播种，所以持久化过的 eeprom 会盖过 `RECEIVER`：机队看着一切正常，但杆量还来自仿真器，MSP 客户端发现飞机毫无反应。启动脚本检测到这种情况会打一行 note；要真生效就删掉那个 `eeprom.bin` 或者用 `RESEED=1`（后者会一并丢弃通过 MSP 改过的配置）。

> 模板是用 `docker/make_msp_eeprom.py` 从 `eeprom.default.bin` 生成的，**只改了三个字节**：`receiver_type` 那个字节，加上重新计算的 CRC16。eeprom 的每条记录都被 CRC 覆盖，校验不过固件会静默回落到出厂默认 —— 正是这套东西一直在避免的那种失败。脚本带 `--verify`，可以拿去核对任何一个 eeprom。

---

## 七、故障排查

| 症状 | 原因 | 处理 |
|---|---|---|
| 日志有 `[SIM] No interface specified. Configurator only.` | **忘了 `-e SIM=ue`** —— `SIM_IP`/`SIM_PORT` 都被静默忽略了 | 补上 `-e SIM=ue`，删容器重跑。这条最坑，因为命令看着"参数都齐了"、`[EEPROM] Loaded` 也正常 |
| 日志 `missed the handshake window` | UE 没在 Play / `SIM_PORT` 填错 / `SIM_IP` 指错 | 确认关卡在运行；用 UE 日志里那个端口核对 `SIM_PORT` |
| 容器 Up，但 UE 连不上，日志没红字 | `--chanmap` 写错（少映射了电机） | 检查 token 是不是正好 6 字符、数字是不是两位 |
| 第二个容器的 Configurator 连不上 | `--tcpbaseport` 和第一个撞了 | 改成 `5760 + 10*index` |
| `[EEPROM] Created` | 播种没生效，跑的是**出厂默认** | `receiver_type = NONE` → 永远解不了锁。见下 |
| 能连上、能收数据，但一直 `disarmed` / `ARMING_DISABLED_RC_LINK` | 十有八九是上一条：出厂 eeprom 的 `receiver_type = NONE` | 删容器重建（`docker rm -f` 后重跑） |
| MSP 客户端连上了，飞机毫无反应 | `receiver_type` 还是 `SIM`，dataref 写进没人读的缓冲区 | 确认 `-e RECEIVER=msp` 生效，日志里应是 `from /opt/inav/eeprom.msp.bin` |
| 容器 **Up 但一个端口都没绑** | 固件卡在 `failureMode`：eeprom 能打开、但写不进去（磁盘满、I/O 错） | 这是最难查的一种，因为容器状态是健康的。挂卷的话检查卷的空间 |
| 容器**退出了** | 大概率是固件打不开 eeprom（SIGSEGV） | `docker inspect -f '{{.State.ExitCode}}'`，139 就是它，对照 5.4 |

> **`[EEPROM] Loaded` 这一条不能省。** 它是区分"配置好的 eeprom"和"出厂默认"的唯一信号，而后者能正常握手、正常收 dataref，唯独永远解不了锁 —— 看起来完全像插件出了问题，实际是固件在等一个永远不来的 receiver。所以排查任何"连上了但飞不了"的问题，先看这行。

---

## 八、常用操作

```bash
docker logs -f inav-sitl-0      # 跟踪日志
docker stop inav-sitl-0         # 停
docker rm -f inav-sitl-0        # 删（重跑前必须先删，名字会撞）
docker ps --format '{{.Names}}  {{.Status}}'   # 看当前有哪些

# 一次性看清楚启动状态
docker inspect -f '{{.State.Status}} exit={{.State.ExitCode}}' inav-sitl-0

# 确认容器里那份 eeprom 是哪一份
docker exec inav-sitl-0 md5sum /data/eeprom.bin
#   default: 75b151875732554cf220d472cbaf2cf5
#   msp:     df8eeaa19162de035d50ba10491e2fd7
```

**eeprom 的持久性**：默认每次都是全新的 —— eeprom 在容器的可写层里，`docker rm` 就没了。这反而是好事，重跑 = 回到已知良好状态。要在容器重建之间保留 CLI 里改过的配置，才需要挂卷（见 `sitl.conf.example` 里的 `DATA_ROOT`）。

---

## 九、一次起多架：`launch_sitl.sh`

`launch_sitl.sh` 是**宿主侧的编排脚本**，就是上面那条 `docker run` 循环 N 次，外加启动后的健康检查。它**不是**容器的 entrypoint（它自己会调 `docker run`，放进容器里就成容器套容器了）。

镜像里也烤了一份（`/opt/inav/launch_sitl.sh`），方便目标服务器直接从镜像里取出来，不用额外传文件：

```bash
docker run --rm --entrypoint cat inav-sitl:9.1 /opt/inav/launch_sitl.sh > launch_sitl.sh
chmod +x launch_sitl.sh
```

### 用法

```bash
./launch_sitl.sh                 # 起默认 4 架（VehicleIndex 0..3）
./launch_sitl.sh 2               # 只起 0、1 两架
./launch_sitl.sh --index 0,2     # 指定架次
./launch_sitl.sh --receiver msp  # 杆量走 MSP
./launch_sitl.sh --stop          # 停掉
./launch_sitl.sh --help          # 全部用法
```

| 参数 | 作用 |
|---|---|
| `[count]` | 起 `VehicleIndex 0..count-1`（默认 4） |
| `--index N[,M...]` | 起指定的架次（0 基）。和 `[count]` 二选一 |
| `--simip IP` | 仿真器地址，覆盖 `sitl.conf` |
| `--simport PORT` | 覆盖监听端口，**只能配单个 `--index`** |
| `--receiver sim\|msp` | 杆量来源，见第三节 |
| `--stop` | 停掉配置里的机队，不起新的 |
| `-h`, `--help` | 打印用法和配置模板 |

### 配置：`sitl.conf`

放在脚本旁边（`cp sitl.conf.example sitl.conf` 起手）。它是给运维改的**地址簿**，只有两个值会因机器而异：

```sh
SIM_IP=192.168.1.50      # 跑 UE 的那台机器
INDICES=0,1,2,3          # 起哪几架
```

| 变量 | 默认 | 作用 |
|---|---|---|
| `SIM_IP` | 空 | 仿真器地址（非 WSL 上必填） |
| `INDICES` | `0,1,2,3` | 起哪些架次，等同于 `--index` |
| `IMAGE` | `inav-sitl:9.1` | 镜像名 |
| `IMAGE_TAR` | 空 | 镜像不在本地时自动 `docker load` 的 tar.gz |
| `SIM_BASE_PORT` | `49000` | 必须等于 `InavSimSettings > BaseListenPort` |
| `TCP_BASE_PORT` | `5760` | MSP 起始端口，每架步长 10 |
| `CHANMAP` | `M01-01,M02-02,M03-03,M04-04` | 电机映射 |
| `RECEIVER` | `sim` | `sim` \| `msp` |
| `DATA_ROOT` | 空 | 取消注释才挂卷持久化每架的 eeprom |
| `DOCKER` | `docker` | WSL 上写 `sudo\ docker` |
| `RESEED` | 空 | 传给容器的 `RESEED` |

优先级：**命令行参数 > `sitl.conf` > 环境变量 > 默认值**。

### 脚本比手敲 `docker run` 多做的事

- 起容器**之前**先做宿主机预检（docker 通不通、非 WSL 上 `SIM_IP` 有没有给），避免瞬间起一堆立刻退出的容器。
- 启动后逐架检查第一节那四条，并把失败翻译成人话。
- 每架自动推导 `SIM_PORT` 和 `--tcpbaseport`，不会错位。
- `DATA_ROOT` 会和 `RECEIVER` 打架时打 note（见第六节第 5 条）。

---

## 十、搬到别的服务器

构建分成两步，第二步可以完全离线跑（内网 CI 就是这一步）：

```bash
# 第一步：在有外网的机器上做两个「预装依赖」基础镜像，推到内网 Harbor
#   builder-base 基于 ubuntu:20.04（glibc 2.31，见下方说明），带整套编译工具链
#   runtime-base 基于 ubuntu:22.04，只有 ca-certificates + iproute2（约 128MB）
REG=registry.ghostcloud.cn/integration
docker build -f Dockerfile.sitl-base --target builder-base -t $REG/inav-sitl-builder-base:20.04 .
docker build -f Dockerfile.sitl-base --target runtime-base -t $REG/inav-sitl-runtime-base:22.04 .
docker push $REG/inav-sitl-builder-base:20.04
docker push $REG/inav-sitl-runtime-base:22.04

# 第二步：内网 CI 直接构建，无需任何 --build-arg，全程不碰 apt/外网
docker build -f Dockerfile.sitl -t inav-sitl:9.1 .

docker save inav-sitl:9.1 | gzip > inav-sitl-9.1.tar.gz
sha256sum inav-sitl-9.1.tar.gz > inav-sitl-9.1.tar.gz.sha256
```

也可以用 `docker/build_sitl_image.sh base|image` 把上面两步各跑一条命令。

> **为什么要拆成两个**：builder 和 runtime 用同一个胖基础镜像的话，runtime 会继承整套工具链，最终镜像会从 135MB 涨到 ~730MB。拆开后 builder-base 只在编译阶段存在，最终镜像仍是 135MB。
>
> **builder-base 为什么用 20.04**：某些宿主机的 Docker seccomp 把 `clone3` 拦成 EPERM；Ubuntu 22.04 的 glibc 2.34+ 在 `posix_spawn` 里用 `clone3` 且**不回退**，导致 ninja/make 报 `posix_spawn: Operation not permitted`、CMake 判“编译器 broken”。20.04 的 glibc 2.31 用的是 `clone`，不会被拦，从而**无需改宿主机 seccomp** 也能编译。runtime-base 保持 22.04——glibc 2.31 编出的二进制在 2.35 上向后兼容。
>
> **默认就指向内网 Harbor**：`Dockerfile.sitl` 的 `BUILDER_BASE_IMAGE` / `RUNTIME_BASE_IMAGE` 两个 `ARG` 默认已是 `registry.ghostcloud.cn/integration/...`（builder `:20.04`、runtime `:22.04`），所以 CI 里**不加参数**即可。要临时改用干净 `ubuntu:22.04` 构建，用 `--build-arg BUILDER_BASE_IMAGE=ubuntu:22.04 --build-arg RUNTIME_BASE_IMAGE=ubuntu:22.04` 覆盖，Dockerfile 里的依赖守卫会回退到 `apt-get`（需外网）。
>
> 这步在 WSL 上 `docker` 已经在组里，不用 `sudo`。如果你的机器要，全程加 `sudo` 即可，脚本那条改 `DOCKER=sudo\ docker`。

传到目标服务器后：

```bash
sha256sum -c inav-sitl-9.1.tar.gz.sha256
gunzip -c inav-sitl-9.1.tar.gz | docker load

# 从镜像里取脚本和配置模板
docker run --rm --entrypoint cat inav-sitl:9.1 /opt/inav/launch_sitl.sh > launch_sitl.sh
docker run --rm --entrypoint cat inav-sitl:9.1 /opt/inav/sitl.conf.example > sitl.conf
chmod +x launch_sitl.sh

vim sitl.conf            # 只需要改 SIM_IP 和 INDICES
./launch_sitl.sh         # 之后每天都是这一条
```

服务器上 docker 一般不需要 `sudo`（操作员在 docker 组里）；如果确实要，在 `sitl.conf` 里写 `DOCKER=sudo\ docker`。

---

## 十一、几个必须知道的坑

1. **PIE 会话期间，绝对不要往 tcp/5760 发 `#`。** 一进 CLI 就会锁上 `ARMING_DISABLED_CLI`，只能重跑容器才能解除。

2. **握手是一次性的。** SITL 只在启动后 10 秒内发订阅包，错过就再也不发了 —— 容器会**永久失聪**（进程还在跑、端口还绑着，但永远收不到数据）。所以：**先让 UE 进 Play，再起容器**。顺序反了就把容器删了重起。

3. **`--network host` 不要省。** 省了仿真链路和 WSL 网关回退都会失效。

4. **`docker ps` 显示 Up 不代表正常。** 第一节那四条日志才是判据。

5. **同一个 `SIM_PORT` 起两个容器** → 抢同一个 UDP 端口，两边都在收包，日志会刷 `UE simulator lost / reconnected`，看起来像网络抖动，实际是端口冲突。

6. **改 eeprom 里的配置要通过 CLI，而 CLI 会锁解锁。** 需要改 `msp_override_channels` 之类的设置时，单独起一个容器进去改、`save`、然后**用挂卷或 `docker cp` 把 eeprom 拿出来**，别在 PIE 会话里改。
