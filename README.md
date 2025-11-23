# src 目录说明

本 `src` 目录包含本工作区的 ROS2 源代码包（例如 `face_detect_interfaces`、`service_python_detect` 等）。

本文件为快速使用指南，包含构建、运行、测试和常见问题排查要点。

## 包结构（示例）
- `face_detect_interfaces`：定义自定义服务接口 `srv/FaceDetector.srv`。
- `service_python_detect`：包含 Python 实现的服务端与客户端节点（`face_detect_node.py`、`face_detect_client_node.py`）。

（以实际目录为准，使用 `ls src` 查看本目录下包名）

## 快速构建与运行
1. 在工作区根目录（上级目录包含 `src`、`install`、`build`）执行构建：
```bash
cd ~/ros_learn4/face_detect_ws
colcon build
```
2. 构建完成后加载环境：
```bash
source install/setup.bash
```
3. 启动服务端节点（示例）：
```bash
ros2 run service_python_detect face_detect_node
```
4. 启动客户端或发送请求（示例）：
```bash
ros2 run service_python_detect face_detect_client_node
```

## 依赖与安装
- 系统包（Ubuntu/Debian）通常需安装 `python3-opencv`、`ros-<distro>-cv-bridge` 等。
- Python 包（可在虚拟环境中安装，推荐）：
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```
示例 `requirements.txt` 里常见项：`face_recognition`、`numpy`、`opencv-python`（注意：系统 `python3-opencv` 与 `opencv-python` 可能冲突，推荐在 venv 中使用 pip 版本）。

## 接口说明
- 自定义服务文件：`face_detect_interfaces/srv/FaceDetector.srv`。
  - 请求（request）中包含图像（`sensor_msgs/Image image`）。
  - 响应（response）包含检测到的人脸数量、位置信息数组以及检测耗时等字段。

## 常见问题与排查
- 构建报错关于 `rosidl_adapter`：检查 `.srv` 文件中是否存在单独的分隔符行 `---`（不可行注释）。
- 运行时报错 `cv_bridge.core.CvBridgeError: Unrecognized image encoding []`：说明传入的 `sensor_msgs/Image.encoding` 为空或不受支持，确认客户端发送的是 `sensor_msgs/Image` 且 `encoding`（例如 `bgr8`）正确，或在服务端用 `CvBridge` 加 try/except 并打印 `request.image.encoding`、`width`、`height`、`len(data)` 进行调试。
- 如果 `cv2` 无法导入或版本不对：检查系统包 `python3-opencv` 或虚拟环境中的 `opencv-python` 是否正确安装。
- 构建后记得运行 `source install/setup.bash`，否则运行 `ros2 run` 时可能找不到安装的可执行脚本。

## 贡献与变更提交建议
- 本仓库遵循常见 Git 工作流：在新分支上开发，使用 `git add` / `git commit` / `git push` 提交与推送。建议在提交前把构建输出目录（`build/`、`install/`、`log/`）加入 `.gitignore`。

## 联系
如需我帮助修改示例客户端、运行一次端到端测试、或把 `requirements.txt` / `.gitignore` 写入仓库，请告诉我，我可以为你执行这些操作并演示具体命令。
