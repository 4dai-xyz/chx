# Isaac Go2 仿真接口

本目录用于保留 Unitree Go2 在 Isaac Lab / Isaac Sim 中的仿真扩展入口。

当前策略：

```text
1. 不提交官方 IsaacLab 全量源码。
2. 不提交 Isaac assets_cache、日志、下载包。
3. 只保留自定义 Go2 task/env/wrapper/extension 和启动脚本。
4. Isaac Lab 的安装与运行说明见 go2_control/docs/ 中的 Isaac 相关文档。
```

后续如果新增自定义 Isaac Lab 扩展，建议放置为：

```text
isaac_go2_sim/
├── docs/
├── scripts/
└── source_extensions/
```

当前 Go2 + Isaac 相关脚本主要位于：

```text
go2_control/scripts/isaaclab_*.sh
go2_control/scripts/run_isaaclab_go2_viser.sh
```

相关说明文档主要位于：

```text
go2_control/docs/IsaacSimLab_Go2复现指南.md
go2_control/docs/Isaac云服务器迁移指南.md
```
