from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


ENV_PREFIX = "GO2_"


def env_arg(name, default_value, description):
    """launch 参数的默认值取自环境变量 GO2_<NAME>（站点 site.env 由 run_site.sh 导出）。

    环境变量名就是 `GO2_` + launch 参数名的大写形式，例如 launch 参数 `max_linear`
    读的是 `GO2_MAX_LINEAR`。必须带这个前缀：站点侧（render_site.py 的 SAFETY_KEYS /
    site.yaml 的 env: 段）导出的全是 `GO2_*` 名字，不加前缀就永远读不到、静默退回
    默认值——安全阈值写进 site.yaml 却完全不生效，是比报错更难查的失败方式。

    EnvironmentVariable 的 default_value 只在变量**不存在**时生效；变量存在但为空串时
    会原样返回空串，所以取值一律走下面的安全解析，不直接丢给 ParameterValue。
    命令行仍可直接覆盖：`ros2 launch ... max_linear:=0.4`。
    """
    return DeclareLaunchArgument(
        name,
        default_value=EnvironmentVariable(
            ENV_PREFIX + name.upper(), default_value=default_value
        ),
        description=description,
    )


def _raw(context, name):
    try:
        return LaunchConfiguration(name).perform(context)
    except Exception as exc:  # noqa: BLE001 - 参数没声明/没设置时按默认值处理
        print(f"[driver.launch] 读取参数 {name} 失败（{exc}），改用默认值")
        return None


def _float_param(context, name, default):
    """把 launch 参数解析成浮点数，解析不了就退回默认值并喊一声。

    为什么不直接用 ParameterValue(value_type=float)：环境变量写成 `GO2_MAX_LINEAR=`（空串）
    或被写错成中文/字母时，float("") 会在 launch 期抛异常，launch 自己捕获后 return_code=1，
    整个 driver.launch.py 以及上层 Nav2 一并不起来，日志只有一句
    "Caught exception in launch"。一个笔误把整栈弄挂，且看不出是哪个站点参数的问题。
    """
    raw = _raw(context, name)
    if raw is None:
        return default
    try:
        return float(raw)
    except (TypeError, ValueError):
        print(f"[driver.launch] 参数 {name}={raw!r} 不是数字，改用默认值 {default}")
        return default


def _bool_param(context, name, default):
    raw = _raw(context, name)
    if raw is None:
        return default
    text = str(raw).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    print(f"[driver.launch] 参数 {name}={raw!r} 不是布尔值，改用默认值 {default}")
    return default


def twist_bridge(context):
    """twist_bridge 节点（参数逐个安全解析后再交给节点）。"""
    return [
        Node(
            package="go2_patrol_driver",
            executable="twist_bridge",
            output="screen",
            parameters=[
                {
                    "scan_topic": LaunchConfiguration("scan_topic").perform(context),
                    "enabled": _bool_param(context, "allow_cmd_vel", True),
                    "require_scan": _bool_param(context, "require_scan", True),
                    "scan_timeout": _float_param(context, "scan_timeout", 1.0),
                    "max_linear": _float_param(context, "max_linear", 0.7),
                    "max_angular": _float_param(context, "max_angular", 1.0),
                    "front_half_angle_deg": _float_param(context, "front_half_angle_deg", 30.0),
                    "emergency_distance": _float_param(context, "emergency_distance", 0.20),
                    "stop_distance": _float_param(context, "stop_distance", 0.35),
                    "slow_distance": _float_param(context, "slow_distance", 0.60),
                    "cmd_vel_timeout": _float_param(context, "cmd_vel_timeout", 0.5),
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            # 运动闸门：默认允许（导航/巡检需要）。建图、只观测、台架测试时必须显式
            # 传 allow_cmd_vel:=false —— mapping.launch.py 已经这么做了。
            env_arg(
                "allow_cmd_vel",
                "true",
                "是否允许把 /cmd_vel 转成官方 MOVE 驱动狗（建图/观测时必须 false）",
            ),
            # 前置急停看门狗：扫描断流时停车而不是"当作前方畅通"继续走。
            # 无雷达的台架 / 离线夹具可用 GO2_REQUIRE_SCAN=false 关掉（不推荐）。
            env_arg("require_scan", "true", "没有新鲜 /scan 时是否停车保护（默认 true）"),
            env_arg("scan_timeout", "1.0", "多久没收到 /scan 就按断流处理（秒）"),
            # 安全阈值：现场只改站点 site.yaml 的 safety: 段
            # （apply_site.sh 生成 site.env，run_site.sh 以 set -a 导出给本进程），
            # 这样改限速/急停距离不用动已安装的软件包。也可用 launch 参数临时覆盖。
            env_arg("max_linear", "0.7", "前进速度上限（m/s）"),
            env_arg("max_angular", "1.0", "转向角速度上限（rad/s）"),
            env_arg("front_half_angle_deg", "30.0", "前置急停只看前方这个半角（度）"),
            env_arg("emergency_distance", "0.20", "前方距离小于该值立即急停并保持刹车（米）"),
            env_arg("stop_distance", "0.35", "前方距离小于该值停车（米）"),
            env_arg("slow_distance", "0.60", "前方距离小于该值时速度减半（米）"),
            env_arg("cmd_vel_timeout", "0.5", "多久没收到 /cmd_vel 就补发刹车（秒）"),
            # TF 补链策略：auto 探测后按需补 base_footprint → base_link（详见 tf_chain_guard.py）
            env_arg(
                "base_footprint_tf",
                "auto",
                "base_footprint→base_link 补链：auto / always / never",
            ),
            Node(
                package="go2_patrol_driver",
                executable="odom_bridge",
                output="screen",
            ),
            Node(
                package="go2_patrol_driver",
                executable="battery_bridge",
                output="screen",
            ),
            OpaqueFunction(function=twist_bridge),
            Node(
                package="go2_patrol_driver",
                executable="tf_chain_guard",
                parameters=[{"mode": LaunchConfiguration("base_footprint_tf")}],
                output="screen",
            ),
        ]
    )
