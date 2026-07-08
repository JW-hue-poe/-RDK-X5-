#!/bin/bash
# deploy_to_rdk.sh
# 将 rescue_robot 项目部署到 RDK 开发板
# 使用前请修改下面的 RDK_USER 和 RDK_IP

set -e

RDK_USER="sunrise"
RDK_IP="192.168.31.174"
RDK_PORT="22"
RDK_WORKSPACE="/home/sunrise/ros2_ws"
LOCAL_PKG_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PKG_NAME="rescue_robot"

echo "开始部署 $PKG_NAME 到 RDK ($RDK_IP)..."

# 1. 清理并打包项目
echo "[1/4] 打包项目..."
cd "$LOCAL_PKG_DIR"
rm -f /tmp/${PKG_NAME}.tar.gz
tar --exclude='.git' --exclude='__pycache__' --exclude='*.pyc' \
    -czf /tmp/${PKG_NAME}.tar.gz "$PKG_NAME"

# 2. 上传到 RDK
echo "[2/4] 上传到 RDK..."
scp -P "$RDK_PORT" /tmp/${PKG_NAME}.tar.gz ${RDK_USER}@${RDK_IP}:/tmp/

# 3. 在 RDK 上解压并编译
echo "[3/4] 在 RDK 上解压并编译..."
ssh -p "$RDK_PORT" ${RDK_USER}@${RDK_IP} << EOF
    set -e
    mkdir -p ${RDK_WORKSPACE}/src
    rm -rf ${RDK_WORKSPACE}/src/${PKG_NAME}
    tar -xzf /tmp/${PKG_NAME}.tar.gz -C ${RDK_WORKSPACE}/src/
    cd ${RDK_WORKSPACE}
    source /opt/tros/setup.bash
    colcon build --packages-select ${PKG_NAME} --symlink-install
    echo "编译完成"
EOF

# 4. 启动提示
echo "[4/4] 部署完成！"
echo ""
echo "请在 RDK 上执行以下命令启动："
echo "  ssh -p $RDK_PORT ${RDK_USER}@${RDK_IP}"
echo "  source ${RDK_WORKSPACE}/install/setup.bash"
echo "  ros2 launch ${PKG_NAME} rescue_robot.launch.py"
