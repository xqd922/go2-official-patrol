# RViz 配置目录

自定义的 `.rviz` 配置文件放在这里，方便在 `navigation.launch.py` 中通过
`rviz -d <path>` 引用。

当前导航 launch 使用 rviz2 默认配置；若需要固定视图，可在此放一份配置并修改
`launch/navigation.launch.py` 中的 rviz 节点。
