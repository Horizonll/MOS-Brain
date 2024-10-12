# MOS Brain

## visualization

主机

在 config.py 、field.js 修改 IP

```shell
python server.py
python -m http.server 8080 --bind 0.0.0.0 # 在index.html所在目录运行
```

机器人

```shell
python send.py
```

浏览器打开

<http://ip:8080>

或者

<http://localhost:8080>

## state machine

基于 python 的 [transitions](state_machine/transitions.md) 库实现的状态机。
