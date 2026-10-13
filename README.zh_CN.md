# 法奥机器人 ROS2 驱动

本项目是法奥机器人对 [ros2_control](https://control.ros.org/jazzy/index.html) 的硬件驱动适配与 [moveit2](https://moveit.picknik.ai/main/index.html) 的配置适配。

本项目基于 ROS2 jazzy 开发，你可以自行尝试适配其他发行版。  
本项目使用 FR5 v6 调试，如果其他机械臂有问题请自行适配。

官方提供[专有实现](https://github.com/FAIR-INNOVATION/frcobot_ros2)，但是功能过于简陋。

## 参考链接

法奥：
- CN: https://www.frtech.cc/
- EN: https://www.frtech.fr/

法奥文档：
- CN: https://fairino-doc-zhs.readthedocs.io/
- EN: https://manual.fairino.support/
- source: https://github.com/FAIR-INNOVATION/fr_documentation


## License
AGPL-3.0（见 [LICENSE](LICENSE)）

`fr_description` 的参数与 mesh 提取自 [frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2) 的 URDF。官方未明确协议，[声称](assets/license-unknown.webp)允许闭源使用。

“法奥”“法奥意威”商标由相关方所有。