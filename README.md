# pymavlink相关安装使用记录

## 下载mavlink

```bash
git clone https://github.com/mavlink/mavlink.git
```

## 安装pymavlink

### 克隆仓库

```bash
git clone https://github.com/ArduPilot/pymavlink.git
```

### 安装依赖

```bash
sudo apt-get install libxml2-dev libxslt-dev
sudo apt-get install python3-numpy python3-pytest
sudo python3 -m pip install --upgrade future lxml
```

### 编译安装（For developers）

先将mavlink包中的message_definitions文件夹复制粘贴到pymavlink文件夹下，并切换进pymavlink文件夹

```bash
sudo MDEF=PATH_TO_message_definitions python3 -m pip install . -v
sudo python3 setup.py install
```
