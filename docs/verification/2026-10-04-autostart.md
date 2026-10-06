# 2026-10-04 开机自启动配置与验证

开机使用 `appli.service` 执行 `/root/dev_ws/appli/start_all.sh 30`，先障碍预扫描，再二维码和物块识别。
服务 `enabled`，验证结束时 `active (running)`，持有 `/dev/ttyS1` 并等待下位机启停帧。

- `APPLI_START_TIMEOUT=0`：初始 `[4]` / `[24]` 持续等待；普通前台入口未设置该变量时仍使用原有起点超时。
- 收到起点后的三次 `[shot]` 各保留 30 秒超时。
- `Restart=on-failure`、`RestartSec=5`：设备未就绪或任务异常退出时重试；手动停止不重启。
- 使用工作区目录与无缓冲 Python 日志；开机链接指向 `/etc/systemd/system/appli.service`。

44 项功能测试通过，包括起点延迟到达、有限起点等待、初始/拍摄超时独立，以及此前的二维码、面积过滤、相机和录像测试。
服务连续观察 36 秒，跨过原有 30 秒起点超时，主 PID 不变、重试计数为 0，日志记录 `START_WAIT_INDEFINITE`。
此项确认服务配置、启用链接与当前运行；本轮未执行整机重启。

完整备份：`/root/dev_ws/RDK_CJLU_V09_autostart_20261004`。结构化结果见 [2026-10-04-autostart.json](2026-10-04-autostart.json)。

```bash
systemctl status appli.service --no-pager
journalctl -u appli.service -b -f
systemctl stop appli.service
# 停止开机服务后，可前台运行简化流程
bash /root/dev_ws/appli/start_simple.sh
```
