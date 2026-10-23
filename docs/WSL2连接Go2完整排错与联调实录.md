# WSL2 连接 Unitree Go2 完整排错与联调实录

> 记录时间：2026-10-21（最终复测 11:05）
> 环境：Windows 10 Enterprise LTSC 2024（24H2 / Build 26100.9457）+ WSL2 Ubuntu 22.04 + ROS 2 Humble
> 目标：在**不安装原生 Ubuntu 双系统**的前提下，用 Windows 的 USB 以太网卡直连 Go2，并在 WSL2 里用 ROS 2 读到真机话题。
> 结论：已完全打通。`ros2 topic list` 可见 **121 个话题**，`/lf/sportmodestate` 稳定 **20 Hz**，`/lf/lowstate` 约 **21.8 Hz**，机载雷达点云约 **15 Hz**。

## 目录

- [0. 先说结论：这个坑为什么难](#0-先说结论这个坑为什么难)
- [1. 硬件与软件清单](#1-硬件与软件清单)
- [2. 正确的心智模型：DDS 发现到底走了什么](#2-正确的心智模型dds-发现到底走了什么)
- [3. 第一阶段：把 Windows → Go2 的 IP 层打通](#3-第一阶段把-windows--go2-的-ip-层打通)
- [4. 第二阶段：WSL2 镜像网络](#4-第二阶段wsl2-镜像网络)
- [5. 第三阶段：定位“IP 通但 ROS 2 看不到话题”](#5-第三阶段定位ip-通但-ros-2-看不到话题)
- [6. 第四阶段：真正的修复](#6-第四阶段真正的修复)
- [7. 最终验证（真机实测数据）](#7-最终验证真机实测数据)
- [8. 日常使用（最短路径）](#8-日常使用最短路径)
- [9. 安全红线（来自教程第 0 章）](#9-安全红线来自教程第-0-章)
- [10. 完整排错决策树](#10-完整排错决策树)
- [11. 常见坑速查表](#11-常见坑速查表)
- [12. 本次改动清单（可回滚）](#12-本次改动清单可回滚)
- [13. 参考链接](#13-参考链接)
- [14. 一句话总结](#14-一句话总结)
- [附录 A：换一台机器复现的最短清单](#附录-a换一台机器复现的最短清单)
- [附录 B：本次排错时间线](#附录-b本次排错时间线)
- [附录 C：本次实测原始判据](#附录-c本次实测原始判据)

---

## 0. 先说结论：这个坑为什么难

教程（[Go2 机器狗实验指导书](https://ztl3106742440-hub.github.io/go2-tutorial/01-foundation/01-install/)）默认你有一台**原生 Ubuntu**，插网线、配 IP、改 `setup.sh` 里的网卡名就能通。

换成 Windows + WSL2 后，会出现一个非常迷惑的“假连通”状态：

| 检查项 | 现象 | 是否代表真正连通 |
|---|---|---|
| `ping 192.168.123.161` | 4/4 通，`<1ms` | ❌ 只证明 IP 层通 |
| `ping 192.168.123.18` | 4/4 通 | ❌ `.18` 是升级服务，不是 ROS 主机 |
| WSL `ip -br addr` 能看到 `192.168.123.99/24` | 正常 | ❌ 只证明地址配上了 |
| Windows 网卡统计 | 10 秒收到 11.9 MB、上百万组播包 | ✅ 包确实到了 Windows |
| WSL 内 `ip -s link show eth7` 的 `mcast` | **0** | ❌ 包没进 WSL |
| `ros2 topic list` | 只有 `/rosout`、`/parameter_events` | ❌ DDS 没发现 |

也就是说：**IP 通 ≠ DDS 通**。CycloneDDS 依赖 UDP 组播做发现（SPDP/SEDP），而 WSL2 的镜像网络在默认配置下会把这类入站组播挡在门外。

最终定位到**四件事，缺一不可**：

1. `.wslconfig` 必须是 **`networkingMode=mirrored`**（默认 NAT 收不到入站组播）。
2. **Hyper-V 防火墙**必须放行发往 WSL 的**全部入站流量**（只开 7400-7440 不够，DDS 数据端口是动态的）。
3. **WSL 内的 DDS 组播路由**（`239.255.0.0/16`）必须指向接狗的网卡（通常是 `eth7`），不能跟随默认路由走到 Wi-Fi 或代理 TUN。
4. **Clash 等代理的 TUN 模式必须关闭**——它会插入 `eth8`（`198.18.0.1/30`）并抢占默认路由（metric=1），把组播也带走。

### 0.1 先分清 PowerShell 和 WSL

本次排错前期有一半时间浪费在“命令敲错了终端”。两个环境的命令**不能混用**：

| 想做的事 | 在哪执行 | 正确写法 | 典型报错 |
|---|---|---|---|
| 配网卡 IP、防火墙、看适配器 | Windows PowerShell | `netsh`、`Get-NetIPAddress`、`New-NetFirewallHyperVRule` | —— |
| ping 狗 | 两边都行 | PowerShell：`ping -n 4 192.168.123.161`；WSL：`ping -c 4 192.168.123.161` | `Bad value for option -c` |
| 看网卡/路由 | WSL | `ip -br addr`、`ip route get 239.255.0.1` | `ip : 无法将“ip”项识别为 cmdlet` |
| 加载 ROS 环境 | WSL | `source ~/unitree_ros2/setup_go2.sh` | `source : 无法将“source”项识别为...` |
| 跑 ROS 2 命令 | WSL | `ros2 topic list` | `ros2 : 无法将“ros2”项识别为...` |

只要看到 `无法将“xxx”项识别为 cmdlet`，就说明**当前还在 PowerShell**，先输入 `wsl` 回车，或者直接用：

```powershell
wsl -d Ubuntu-22.04 -u seven
```

> 注意用 `-u seven`。`root` 的 `$HOME` 是 `/root`，`setup_go2.sh` 里的 `$HOME/unitree_ros2` 会找不到。

---

## 1. 硬件与软件清单

### 1.1 本次实际环境

| 项目 | 值 |
|---|---|
| 主机系统 | Windows 10 Enterprise LTSC 2024，24H2，Build 26100.9457 |
| WSL 版本 | 2.7.11.0 → **升级到 2.7.13.0** |
| WSL 内核 | `6.18.33.2-microsoft-standard-WSL2` |
| WSL 发行版 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| DDS | `rmw_cyclonedds_cpp`（CycloneDDS 0.10.x，随 `unitree_ros2` 编译） |
| 接狗网卡（Windows） | `以太网 2`，Realtek USB FE Family Controller #2，**USB 网卡** |
| 接狗网卡 MAC | `00-E0-2A-89-3F-B5` |
| 接狗网卡（WSL） | `eth7`，MAC `00:e0:2a:89:3f:b5` |
| Windows 侧 IP | `192.168.123.99/24`，Manual |
| Go2 地址 | 本体 `192.168.123.161`（DDS）；升级服务 `192.168.123.18`（HTTP/SSH） |

> 关键点：这块接狗网卡是 **USB 网卡**，不是笔记本内置网口。USB 网卡在 WSL 镜像模式下的命名是动态的（`eth7`/`eth8` 会变），所以**不要写死网卡名**，要按 MAC 识别。

### 1.2 为什么必须是 USB 网卡

Windows 主机只有一块内置 Wi-Fi（`WLAN`，MediaTek MT7921），没有板载有线网口；接狗用的是 USB 转 RJ45（RTL8152）。这一点很关键：

- 如果以后要做 USB 直通（`usbipd-win`）给 WSL，这块网卡是可行的；
- 本次没有走 USB 直通，而是用 WSL 镜像网络 + 防火墙放行 + 组播路由解决。

---

## 2. 正确的心智模型：DDS 发现到底走了什么

理解下面这条链路，后面的排错会非常快。

```text
Go2 (192.168.123.161)
  │  每 1s 向 239.255.0.1:7400 发 SPDP（参与者发现）
  │  向 239.255.0.1:7401 发 SEDP（话题/类型发现）
  ▼
USB 网卡（Windows: 以太网 2 → WSL: eth7，同一 MAC）
  │
  ├─ Windows 协议栈：能收到组播（统计里 mcast 很大）
  │
  └─ WSL2 镜像网络：需要 Hyper-V 防火墙放行 + 组播路由指对
        │
        ▼
     WSL 内 CycloneDDS（绑定 eth7）
        │  收到 7400/7401 后，再用动态单播端口建数据通道
        ▼
     ros2 topic list / echo
```

三个容易误解的点：

1. **7400/7401 只是发现端口**，真正传数据的端口是动态分配的（`7400 + 250*domain + ...` 再叠加 participant index）。所以只放行 7400-7440 会出现“能发现一部分、拿不到数据”的假象。
2. **组播地址是 `239.255.0.1`**（CycloneDDS 默认 SPDP 地址），不是 `224.0.0.1`。
3. **DDS 是 UDP**，`ping`（ICMP）通完全不能说明 DDS 通。`.161` 上甚至没有开放任何 TCP 端口。

---

## 3. 第一阶段：把 Windows → Go2 的 IP 层打通

### 3.1 关闭会抢网卡的软件

排查前先关掉：

- **VMware 桥接**（如果 VMnet 绑到了接狗网卡）
- **Meta / Quest 串流**（会抢 `192.168.123.161` 的路由）
- **Clash / v2ray 的 TUN 模式**（会插 `eth8` 抢默认路由）

VMware 桥接可以只对这块网卡禁用：

```powershell
Disable-NetAdapterBinding -Name "以太网 2" -ComponentID vmware_bridge
```

需要恢复时：

```powershell
Enable-NetAdapterBinding -Name "以太网 2" -ComponentID vmware_bridge
```

### 3.2 设置静态 IP

接狗网卡必须在 `192.168.123.0/24`，且**不能是 `.161`**（那是狗）。

```powershell
netsh interface ipv4 set address name="以太网 2" static 192.168.123.99 255.255.255.0
netsh interface ipv4 delete arpcache
```

验证：

```powershell
Get-NetIPAddress -InterfaceAlias "以太网 2" -AddressFamily IPv4 |
  Format-List IPAddress,PrefixLength,PrefixOrigin,AddressState
```

期望：`IPAddress=192.168.123.99`、`PrefixLength=24`、`PrefixOrigin=Manual`、`AddressState=Preferred`。

### 3.3 重启网卡让配置生效

```powershell
Restart-NetAdapter -Name "以太网 2"
```

### 3.4 IP 层验证

```powershell
ping -n 4 192.168.123.161
ping -n 4 192.168.123.18
```

本次结果（Windows）：

```text
Pinging 192.168.123.161 with 32 bytes of data:
Reply from 192.168.123.161: bytes=32 time<1ms TTL=64
...
Packets: Sent = 4, Received = 4, Lost = 0 (0% loss)
```

`.18` 同样 4/4。

> 到这里只是 IP 层通。**不要**据此认为已经能跑 ROS 2。

---

## 4. 第二阶段：WSL2 镜像网络

### 4.1 为什么必须用 mirrored，不能用默认 NAT

WSL2 默认是 NAT：WSL 在 `172.x` 私有网段，出站被 NAT，**入站组播无法到达**。Go2 的 DDS 发现包是组播，NAT 模式下必然收不到。

`.wslconfig`（`C:\Users\Seven\.wslconfig`）：

```ini
[wsl2]
memory=10GB
# 镜像网络：WSL 直接共享主机网卡
networkingMode=mirrored
# 关闭 WSL 自身防火墙过滤
firewall=false
```

应用：

```powershell
wsl --shutdown
```

### 4.2 验证镜像网卡

```bash
ip -br addr
```

期望能看到与 Windows 同 IP、同 MAC 的网卡：

```text
eth7   UP   192.168.123.99/24  fe80::d5ee:a1f:4b94:c205/64
eth1   UP   172.16.49.169/22   ...   # Wi-Fi
```

验证 WSL 也能 ping 通：

```bash
ping -c 3 -W 1 192.168.123.161
ping -c 3 -W 1 192.168.123.18
```

本次结果：两个地址都是 3/3、0% loss。

> 注意：`ping` 通只说明**单播 ICMP** 通。后面会看到，WSL 的**组播入站**仍然是 0，这才是真正的病根。

### 4.3 一个必须知道的坑：端口表是共享的

在 mirrored 模式下，Windows 和 WSL 共用 UDP 端口空间。典型现象：

```text
# Windows 先绑定 7400
OSError: [Errno 98] Address already in use   # WSL 再绑定 7400 就失败
```

所以排查时**不要**让 Windows 和 WSL 同时监听同一个 DDS 端口，否则会互相抢。这也解释了为什么“在 Windows 上跑个组播接收器观察”会影响 WSL 的测试结果。

---

## 5. 第三阶段：定位“IP 通但 ROS 2 看不到话题”

### 5.1 先看 ROS 2 侧

```bash
source /home/seven/unitree_ros2/setup_go2.sh
ros2 topic list -t --no-daemon
```

当时的错误输出：

```text
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
```

没有 `/lf/sportmodestate`、`/lf/lowstate`、`/api/sport/request`、`/wirelesscontroller`。

### 5.2 关键证据 1：Windows 收到了组播，WSL 收到 0

在 Windows 上监听 `239.255.0.1:7400`：

```text
PKT 416 bytes from 192.168.123.161:58579
PKT 116 bytes from 192.168.123.161:46621
...
TOTAL=38   # 10 秒
```

再看 Windows 网卡统计：

```text
ReceivedMulticastPackets : 1216127
ReceivedMulticastBytes   : 1613013255   # 约 1.6 GB
```

而 WSL 内：

```text
eth7: RX packets ... mcast 0
```

**结论：包到了 Windows，但没有被镜像进 WSL。**

### 5.3 关键证据 2：WSL 的单播是通的

为了排除“整条链路都不通”，从 Windows 向 WSL 发单播 UDP：

```text
WSL UDP listener on 192.168.123.99:55555
GOT 22 from ('192.168.123.99', 52229) data b'WINDOWS-TO-WSL-UNICAST'
```

WSL 里也能抓到 `.161` 的 ARP 回包：

```text
ARP, Reply 192.168.123.161 is-at 7e:1d:75:60:f5:89
```

**结论：镜像网卡的单播路径正常，只有组播入站被挡。**

### 5.4 排除“不是路由问题”的过程

一开始怀疑 WSL 组播路由走错（确实会走 Wi-Fi）：

```bash
ip route get 239.255.0.1
# multicast 239.255.0.1 dev eth1 src 172.16.49.169
```

手动加路由：

```bash
sudo ip route replace 224.0.0.0/4 dev eth7
ip route get 239.255.0.1
# multicast 239.255.0.1 dev eth7 src 192.168.123.99
```

**但收包仍然是 0。** 说明路由不是唯一原因，但它是必要条件（后面验证：路由对了 + 防火墙放行，才终于通）。

> 附带一个 `ip route` 的坑：组播路由前缀长度必须 ≥ 4，`ip route add 239.255.0.0/8` 会报
> `Error: Invalid prefix for given prefix length.`。用 `/16` 或 `/4`。

### 5.5 查 WSL / Windows 官方与社区资料

查到的高相关 issue：

| 来源 | 结论 |
|---|---|
| [microsoft/WSL#12344](https://github.com/microsoft/WSL/issues/12344) | mirrored 模式组播支持不完整，官方 open |
| [microsoft/WSL#12122](https://github.com/microsoft/WSL/issues/12122) | WSL→Windows 的 UDP 组播收不到 |
| [microsoft/WSL#10535](https://github.com/microsoft/WSL/issues/10535) | mirrored 下 LAN 入站包收不到 |
| [CSDN《WSL2 和 multicast》](https://blog.csdn.net/sxd2001/article/details/144649381) | 详细分析了 mirrored 的组播镜像机制；提到 `CONFIG_IP_MULTICAST` 和“主机也加入组播组” |
| [microsoft/WSL#40296](https://github.com/microsoft/WSL/issues/40296) | 残留代理设置（`ProxyEnable=0` 但 `ProxyServer` 非空）会让 mirrored 网络黑屏 |

本次内核已经启用了组播：

```bash
zcat /proc/config.gz | grep CONFIG_IP_MULTICAST
# CONFIG_IP_MULTICAST=y
```

所以不是内核缺配置的问题。

### 5.6 排除的无效尝试（避免你重复踩）

以下都试过，**都不能单独解决问题**：

- 改 `CYCLONEDDS_URI` 的 `AllowMulticast=false` + 单播 `Peers`（`.161` 的 7410 等端口明确返回 ICMP port unreachable）
- 绑定组播地址 `bind(("239.255.0.1", 7400))`、设 `IP_MULTICAST_IF`、设 `IP_MULTICAST_ALL=0`
- 让 Windows 主机也在同端口加入组播组（会与 WSL 抢端口）
- 升级 WSL 2.7.11 → 2.7.13（版本本身不是根因）
- 重启 WSL、清 ARP、禁用 VMware 桥接（这些是必要的清理，但不是根因）

顺带记录一个**误导性证据**：早期 `tcpdump -ni eth7` 抓不到任何包，看起来像“网卡完全没数据”。但在镜像模式下 `tcpdump` 对 AF_PACKET 的可见性并不可靠；后来同样的命令能正常抓到 ARP/ICMP/DDS，说明不能只凭 `tcpdump` 下结论，要结合 `ip -s link` 的 `mcast` 计数。

### 5.7 升级 WSL 的插曲（不是根因，但值得记录）

排查中途怀疑过 WSL 版本，尝试把 `2.7.11.0` 升到 `2.7.13.0`，过程并不顺利：

| 尝试 | 结果 | 原因 |
|---|---|---|
| 直接双击 MSI 安装 | 失败，`1603`（安装时发生致命错误） | 安装程序需要写 `Program Files` 和系统服务，权限不足 |
| 静默安装 `msiexec /i ... /qn` | 失败，`1730`（安装程序包有问题） | 同样是权限/会话上下文问题 |
| 用管理员 PowerShell 重新执行 | 成功 | UAC 提权后才有写权限 |

结论：**升级 WSL 并没有让 DDS 通**。升完版本后 `mcast` 仍然是 0，直到加了 Hyper-V 入站规则才出现话题。记录下来是为了避免下次把“升级成功”误当成“问题解决”。

回滚方式：安装旧版本 MSI。本次 `2.7.13` 的安装包留在
`C:\Users\Seven\AppData\Local\Temp\wsl2.msi`，需要回退时从 GitHub Releases 下载对应旧版安装即可。

---

## 6. 第四阶段：真正的修复

### 6.1 修复一：放行 Hyper-V 防火墙入站

`firewall=false` 只关了 WSL 自己的规则，**Hyper-V 防火墙仍会丢包**。微软文档给出的开关：

```powershell
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```

更推荐**最小化**做法——只加一条入站规则：

```powershell
$vm = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
New-NetFirewallHyperVRule -Name 'WSL-DDS-In-Any' `
  -DisplayName 'WSL DDS inbound any' `
  -Direction Inbound -VMCreatorId $vm -Protocol Any
```

必须用**管理员 PowerShell**。

> 实测：只放行 `UDP 7400-7440` 时，能收到发现包，但 `ros2 topic list` 仍然只有 2 个话题；改成 `Protocol Any` 后立刻出现 121 个。原因是 DDS 数据端口动态分配。

如果按顺序试过端口规则，机器上可能同时残留两条（本次就是如此）：

```text
WSL-DDS-In-Any   Inbound  Any   LocalPorts: Any         ← 真正起作用
WSL-DDS-In-UDP   Inbound  UDP   LocalPorts: 7400-7440   ← 早期试验，可删
```

`Any` 已经覆盖 UDP 规则，端口规则留着不会出错，但建议删掉，减少以后排错时的干扰：

```powershell
Get-NetFirewallHyperVRule | Where-Object Name -eq 'WSL-DDS-In-UDP' | Remove-NetFirewallHyperVRule
```

加完规则后立刻验证（不用重启 WSL）：

```bash
ip -s link show eth7
# mcast 计数开始增长
```

本次加规则后 10 秒收到 66 个 DDS 包，`ros2 topic list` 直接出现全部话题。

### 6.2 修复二：组播路由指向接狗网卡

即使防火墙放行，如果 `239.255.0.1` 走的是 Wi-Fi / TUN，仍然收不到。需要：

```bash
sudo ip route replace 239.255.0.0/16 dev eth7
```

验证：

```bash
ip route get 239.255.0.1
# multicast 239.255.0.1 dev eth7 src 192.168.123.99
```

### 6.3 修复三：干掉代理 TUN 和残留代理

Clash TUN 开启后，WSL 里会多出：

```text
eth8   UP   198.18.0.1/30
default via 198.18.0.2 dev eth8 proto kernel metric 1
```

`metric 1` 会抢走所有默认路由，包括组播。**联调前关闭 TUN。**

关闭后还要检查 Windows 的残留代理（这是 `#40296` 描述的黑屏条件）：

```powershell
Get-ItemProperty -Path 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Internet Settings' |
  Select-Object ProxyEnable,ProxyServer
```

本次发现：

```text
ProxyEnable : 0
ProxyServer : 127.0.0.1:7897     # ← 残留！
```

清理：

```powershell
Set-ItemProperty -Path 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Internet Settings' -Name ProxyServer -Value ''
Set-ItemProperty -Path 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Internet Settings' -Name ProxyEnable -Value 0 -Type DWord
```

### 6.4 把修复做成开机自启

网卡名 `eth7` 是动态的，重启 WSL 可能变。所以按 **MAC 识别**，并注册 systemd 服务。

`/usr/local/sbin/go2-mcast-route.sh`：

```bash
#!/bin/bash
set -euo pipefail
GO2_MAC="00:e0:2a:89:3f:b5"
IFACE=$(ip -br link | awk -v m="$GO2_MAC" '$3==m {print $1; exit}')
if [ -z "${IFACE:-}" ]; then
  echo "go2-mcast-route: adapter $GO2_MAC not present" >&2
  exit 0
fi
ip route replace 239.255.0.0/16 dev "$IFACE"
echo "go2-mcast-route: 239.255.0.0/16 -> $IFACE"
```

`/etc/systemd/system/go2-mcast-route.service`：

```ini
[Unit]
Description=Go2 DDS multicast route (WSL mirrored)
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/go2-mcast-route.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

启用：

```bash
sudo chmod 755 /usr/local/sbin/go2-mcast-route.sh
sudo systemctl daemon-reload
sudo systemctl enable --now go2-mcast-route.service
sudo systemctl status go2-mcast-route.service
```

本次验证：`systemctl is-enabled` 返回 `enabled`，`is-active` 返回 `active`。

> 需要 `/etc/wsl.conf` 里有 `[boot] systemd=true`，否则 systemd 服务不会跑。本次已是该配置。

---

## 7. 最终验证（真机实测数据）

### 7.1 冷启动验证

```powershell
wsl --shutdown
```

等待 6 秒后重新进入 WSL，**不手动加任何路由**，直接：

```bash
source ~/unitree_ros2/setup_go2.sh
ip route get 239.255.0.1
ros2 topic list -t --no-daemon
```

输出：

```text
Setup unitree ros2 environment (iface=eth7)
multicast 239.255.0.1 dev eth7 src 192.168.123.99 rt_offload_failed uid 1000
```

话题数：**121**。

### 7.2 关键话题与频率

| 话题 | 类型 | 实测频率 | 说明 |
|---|---|---|---|
| `/lf/sportmodestate` | `unitree_go/msg/SportModeState` | **20.2 Hz** | 高层运动状态（位置/速度/姿态/足端力） |
| `/lf/lowstate` | `unitree_go/msg/LowState` | **21.8 Hz** | 低层状态（电机、BMS、IMU） |
| `/utlidar/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` | **约 15 Hz** | 机载雷达去畸变点云 |
| `/wirelesscontroller` | `unitree_go/msg/WirelessController` | 有发布者（1） | 遥控器未操作时无数据，正常 |
| `/api/sport/request` | `unitree_api/msg/Request` | - | **控制入口，本次未发送任何消息** |

### 7.3 一帧真实状态

```bash
ros2 topic echo /lf/sportmodestate --once --no-daemon --no-arr
```

```yaml
stamp:
  sec: 1792551140
  nanosec: ...
error_code: 100
imu_state:
  temperature: 79
mode: 0
gait_type: 0
body_height: 0.31737419962882996
yaw_speed: -0.0074568502604961395
```

### 7.4 电池状态

```bash
ros2 topic echo /lf/lowstate --once --no-daemon --no-arr
```

```yaml
bms_state:
  soc: 53
  current: -2695
  cycle: 18
temperature_ntc1: 44
temperature_ntc2: 39
power_v: 28.906518936157227
power_a: 2.050905704498291
```

### 7.5 点云

```yaml
header:
  frame_id: odom
height: 1
width: 10580
point_step: 32
row_step: 338560
is_dense: true
```

约 1 万点/帧，与 15 Hz 的频率吻合。

---

## 8. 日常使用（最短路径）

### 8.1 开机顺序

1. 狗上电，等它自检完成（不要急着发指令）
2. Windows 确认 `以太网 2` 是 `Up`、`192.168.123.99/24`
3. **不要开 Clash TUN**
4. 打开 WSL 终端

### 8.2 每次新终端

```bash
source ~/unitree_ros2/setup_go2.sh
ros2 topic list
ros2 topic echo /lf/sportmodestate --once
```

`setup_go2.sh` 会自动按 MAC 找到网卡并设置 `CYCLONEDDS_URI`，不需要改官方 `setup.sh`。

### 8.3 一键自检脚本

```bash
#!/bin/bash
set -u
source ~/unitree_ros2/setup_go2.sh || exit 1
echo "=== iface ==="; ip -br addr show "$GO2_IFACE"
echo "=== route ==="; ip route get 239.255.0.1
echo "=== ping ==="; ping -c 2 -W 1 192.168.123.161
echo "=== topics ==="; timeout 20 ros2 topic list -t --no-daemon | grep -E \
  "lf/sportmodestate|lf/lowstate|wirelesscontroller|api/sport/request"
echo "=== rate ==="; timeout 5 ros2 topic hz /lf/sportmodestate
```

期望：网卡是 `eth7`，路由是 `dev eth7`，ping 0% loss，四个话题都在，频率约 20 Hz。

---

## 9. 安全红线（来自教程第 0 章）

本次全程**只做只读操作**：

- ✅ `ping`、`arp`、`ip neigh`、`ip route`
- ✅ `tcpdump`、`ros2 topic list/info/type/hz/echo --once`
- ❌ 没有向 `/lowcmd` 发任何消息
- ❌ 没有向 `/api/sport/request` 发任何消息
- ❌ 没有调用 `stand_up` / `stand_down` / 任何运动服务

以后要做运动测试，先满足：

1. 狗在平地，周围留空（至少 2 m）
2. 遥控器在手，随时能接管
3. 电池/温度正常（本次 53%、NTC 44/39°C）
4. 网线不会被腿勾到
5. 明确这次要做什么动作，以及异常时第一步怎么停

出现抖动、误动、异常发热、原地旋转——**先停，再查**，不要一边觉得不对一边继续发指令。

---

## 10. 完整排错决策树

```text
ros2 topic list 看不到 /lf/sportmodestate
│
├─ ping 192.168.123.161 不通？
│   ├─ 检查接狗网卡是否 Up、IP 是否 192.168.123.99/24
│   ├─ 检查是否有 VMware/Meta/代理抢路由
│   └─ 换网线 / 换 USB 口
│
├─ ping 通，但 WSL 里 ip -s link show eth7 的 mcast=0？
│   ├─ 是否 networkingMode=mirrored？           → 改 .wslconfig
│   ├─ 是否加了 Hyper-V 入站 Any 规则？          → 加规则
│   ├─ 是否有代理 TUN（eth8 / 198.18.x.x）？     → 关 TUN
│   └─ 是否有残留 ProxyServer？                  → 清注册表值
│
├─ mcast 在涨，但仍然只有 2 个话题？
│   ├─ ip route get 239.255.0.1 是否 dev eth7？  → 加 /16 路由
│   ├─ CYCLONEDDS_URI 的网卡名是否对？           → 用 setup_go2.sh
│   └─ ROS_DOMAIN_ID 是否与狗一致（默认 0）？
│
└─ 有话题但 echo 没数据？
    ├─ ros2 topic info 看 Publisher count
    ├─ 用 --no-daemon 排除 daemon 缓存
    └─ 检查是否是 /wirelesscontroller 这类“无操作就无数据”的话题
```

---

## 11. 常见坑速查表

| 现象 | 原因 | 处理 |
|---|---|---|
| `ros2 topic list` 只有 `/rosout` | Hyper-V 防火墙拦截入站 | 加 `WSL-DDS-In-Any` 规则 |
| `mcast=0` 但 Windows 统计很大 | 镜像入站被挡 | 同上 + 检查组播路由 |
| 路由走 `eth1`/`eth8` | 默认路由优先 | `ip route replace 239.255.0.0/16 dev eth7` |
| 多出 `eth8 198.18.0.1/30` | 代理 TUN | 关闭 TUN |
| `ProxyEnable=0` 但 `ProxyServer` 有值 | 代理残留 | 清空 `ProxyServer` |
| `Address already in use` | Windows/WSL 共享端口 | 不要两边同时监听同一端口 |
| `ip route add 239.255.0.0/8` 报错 | 组播前缀必须 ≥4 | 用 `/16` 或 `/4` |
| `ros2 topic echo` 提示 type invalid | 只 source 了 `/opt/ros/humble`，没 source 工作空间 | `source ~/unitree_ros2/setup_go2.sh` |
| `root` 用户下 `$HOME` 不对 | `setup_go2.sh` 用了 `$HOME/unitree_ros2` | 用 `seven` 用户执行 |
| daemon 卡在 `connect(127.0.0.1:11511)` | ROS 2 daemon 异常 | 加 `--no-daemon`，或 `ros2 daemon stop && ros2 daemon start` |
| `ping -c` 报 `Bad value for option -c` | 在 PowerShell 里跑了 Linux 命令 | 进 `wsl` 再执行 |
| `source: 无法将...识别为 cmdlet` | 同上 | 进 `wsl` 再执行 |

---

## 12. 本次改动清单（可回滚）

### 12.1 Windows 侧

| 改动 | 位置 | 回滚 |
|---|---|---|
| 接狗网卡静态 IP | `以太网 2` = `192.168.123.99/24` | `netsh interface ipv4 set address name="以太网 2" dhcp` |
| 禁用 VMware 桥接 | `以太网 2` | `Enable-NetAdapterBinding -Name "以太网 2" -ComponentID vmware_bridge` |
| Hyper-V 入站规则 | `WSL-DDS-In-Any`、`WSL-DDS-In-UDP`（早期试验） | `Get-NetFirewallHyperVRule \| Where-Object Name -like 'WSL-DDS-*' \| Remove-NetFirewallHyperVRule` |
| 清空残留代理 | `HKCU\...\Internet Settings\ProxyServer` | 用代理软件重新设置 |
| WSL 升级 | 2.7.11 → 2.7.13 | 安装旧版 MSI 回滚 |

### 12.2 WSL 侧

| 改动 | 位置 | 回滚 |
|---|---|---|
| 组播路由脚本 | `/usr/local/sbin/go2-mcast-route.sh` | `sudo rm` |
| 自启服务 | `/etc/systemd/system/go2-mcast-route.service` | `sudo systemctl disable --now go2-mcast-route.service && sudo rm` |
| ROS 环境脚本 | `~/unitree_ros2/setup_go2.sh` | 删除即可，官方 `setup.sh` 未改动 |

### 12.3 保留未动

- 官方 `~/unitree_ros2/setup.sh`（仍是 Foxy + `enp3s0`，**不要直接用**）
- `~/unitree_ros2/cyclonedds_ws` 编译产物

---

## 13. 参考链接

- 教程第 1 章：<https://ztl3106742440-hub.github.io/go2-tutorial/01-foundation/01-install/>
- 教程安全红线：<https://ztl3106742440-hub.github.io/go2-tutorial/00-overview/04-safety/>
- Unitree ROS 2 官方仓库：<https://github.com/unitreerobotics/unitree_ros2>
- WSL 网络文档：<https://learn.microsoft.com/en-us/windows/wsl/networking>
- WSL issue #12344（mirrored 组播）：<https://github.com/microsoft/WSL/issues/12344>
- WSL issue #12122（UDP 组播）：<https://github.com/microsoft/WSL/issues/12122>
- WSL issue #10535（LAN 入站）：<https://github.com/microsoft/WSL/issues/10535>
- WSL issue #40296（代理残留）：<https://github.com/microsoft/WSL/issues/40296>
- CSDN《WSL2 和 multicast》：<https://blog.csdn.net/sxd2001/article/details/144649381>

---

## 14. 一句话总结

> **IP 通只是起点。WSL2 要接 Go2，必须同时做到：`mirrored` + Hyper-V 入站 Any 放行 + `239.255.0.0/16` 走接狗网卡 + 关掉代理 TUN。**

四件事缺一不可，而且每一件单独看都不会报错，只会表现成“`ros2 topic list` 里空空如也”。

---

## 附录 A：换一台机器复现的最短清单

如果以后在另一台 Windows 上重来，按这个顺序做，通常 10 分钟内能通：

1. 插 USB 网卡，记下它的 MAC（`Get-NetAdapter | Format-Table Name,MacAddress,Status`）。
2. 设静态 IP：`192.168.123.99/24`，禁用该网卡上的 VMware 桥接。
3. `.wslconfig` 写 `networkingMode=mirrored`、`firewall=false`，然后 `wsl --shutdown`。
4. 管理员 PowerShell 加 Hyper-V 入站 Any 规则。
5. 关闭 Clash TUN、清空残留 `ProxyServer`。
6. 进 WSL，按 MAC 找到网卡，把 `239.255.0.0/16` 指过去，用 systemd 固化。
7. 写 `setup_go2.sh`（按 MAC 自动找网卡 + 设 `CYCLONEDDS_URI`）。
8. 验证三连：`ping` → `ip route get 239.255.0.1` → `ros2 topic list -t --no-daemon`。

判据：`ros2 topic list` 出现 **121 个话题**，`/lf/sportmodestate` 约 **20 Hz**。

---

## 附录 B：本次排错时间线

按实际发生顺序记录，方便对照“我当时卡在哪一步”。

| # | 动作 | 现象/结果 | 判断 |
|---|---|---|---|
| 1 | 在 PowerShell 里直接敲 `ip`、`source`、`ros2` | 全部报“无法识别为 cmdlet” | 命令跑错终端，先进 `wsl` |
| 2 | 设置 `以太网 2` 为 `192.168.123.99/24` | 配置成功 | 基础就绪 |
| 3 | Windows `ping 192.168.123.161` | 4/4 通 | 只证明 IP 层通 |
| 4 | WSL `ping 192.168.123.161` | 3/3 通 | 单播镜像正常 |
| 5 | WSL `ros2 topic list` | 只有 `/rosout`、`/parameter_events` | DDS 发现失败 |
| 6 | Windows 监听 `239.255.0.1:7400` | 持续收到 416/116 字节包 | 狗在正常广播 |
| 7 | WSL `ip -s link show eth7` | `mcast` 计数为 0 | 组播没进 WSL |
| 8 | 手动加 `224.0.0.0/4 dev eth7` | 路由对了，`mcast` 仍为 0 | 路由是必要条件，不是唯一原因 |
| 9 | 试 `AllowMulticast=false` + 单播 Peers | `.161` 的 7410 返回 port unreachable | 狗不支持这种发现方式 |
| 10 | 关闭 VMware 桥接、清 ARP、重启网卡 | 无变化 | 属于清理，不是根因 |
| 11 | 升级 WSL 2.7.11 → 2.7.13 | 安装先报 1603/1730，提权后成功；`mcast` 仍为 0 | 版本不是根因 |
| 12 | 发现 Clash TUN 插入 `eth8 198.18.0.1/30`（metric 1） | 关 TUN 后路由不再被抢 | 必要修复之一 |
| 13 | 清空残留 `ProxyServer=127.0.0.1:7897` | 排除 #40296 描述的黑屏条件 | 必要清理 |
| 14 | 给 Hyper-V 加 `WSL-DDS-In-Any`（Inbound / Any） | `mcast` 开始增长 | **关键修复** |
| 15 | 把 `239.255.0.0/16` 指向 `eth7` | `ros2 topic list` 出现 121 个话题 | **打通** |
| 16 | 重启做冷启动验证 | 不手动加路由也保持 121 话题 | 持久化成功 |

> 最容易误判的是第 3、4 步：`ping` 通会让人以为“网络没问题”，但 DDS 走的是 UDP 组播 + 动态端口，和 ICMP 完全是两条路径。

---

## 附录 C：本次实测原始判据

| 判据 | 命令 | 本次结果 |
|---|---|---|
| IP 可达 | `ping -c 3 192.168.123.161` | 3/3，0% loss，约 1.1 ms |
| 组播出口 | `ip route get 239.255.0.1` | `dev eth7 src 192.168.123.99` |
| 话题数量 | `ros2 topic list -t --no-daemon \| wc -l` | 121 |
| 运动状态频率 | `ros2 topic hz /lf/sportmodestate` | 20.2 Hz |
| 低层状态频率 | `ros2 topic hz /lf/lowstate` | 21.8 Hz |
| 点云频率 | `ros2 topic hz /utlidar/cloud_deskewed` | 约 15 Hz |
| 电池 | `ros2 topic echo /lf/lowstate --once` | SOC 53%，28.9 V |
| 自启服务 | `systemctl is-enabled/is-active` | enabled / active |
