# State Machine

该目录预留用于后续创建 FreeRTOS 状态机任务。

建议后续将状态机的任务入口、状态定义和事件处理文件集中放在此目录，
并在根目录 `CMakeLists.txt` 的 `target_sources` 和
`target_include_directories` 中注册新增源码与头文件。
