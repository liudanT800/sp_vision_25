FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

# 基础依赖
RUN apt update && apt install -y \
    git g++ cmake build-essential pkg-config wget curl ca-certificates \
    libopencv-dev libceres-dev libeigen3-dev \
    libfmt-dev libspdlog-dev libyaml-cpp-dev \
    libusb-1.0-0-dev nlohmann-json3-dev \
    can-utils openssh-server screen udev unzip lsb-release gnupg software-properties-common

# 安装 ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && apt install -y \
    ros-humble-ros-core \
    ros-humble-geometry2 \
    ros-humble-tf2 \
    ros-humble-std-msgs \
    python3-colcon-common-extensions

# 源码 ROS 环境
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 安装 OpenVINO
RUN mkdir -p /opt && cd /opt && \
    wget -q https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.0/linux/l_openvino_toolkit_ubuntu22_2024.0.0.14509.34caeefd078_x86_64.tgz && \
    tar -xzf l_openvino_toolkit_ubuntu22_2024.0.0.14509.34caeefd078_x86_64.tgz && \
    mv l_openvino_toolkit_ubuntu22_2024.0.0.14509.34caeefd078_x86_64 openvino_2024 && \
    ln -s openvino_2024 openvino && \
    rm l_openvino_toolkit_ubuntu22_2024.0.0.14509.34caeefd078_x86_64.tgz && \
    cd openvino/install_dependencies && bash install_openvino_dependencies.sh -y

# 设置 OpenVINO 环境变量（修正 cmake 路径）
ENV OPENVINO_ROOT=/opt/openvino

# 先初始化可能不存在的宿主环境变量，避免“未定义变量”警告
ENV LD_LIBRARY_PATH=""
ENV PKG_CONFIG_PATH=""
ENV CMAKE_PREFIX_PATH=""

ENV PATH="${OPENVINO_ROOT}/runtime/bin:${OPENVINO_ROOT}/tools:${PATH}"
ENV LD_LIBRARY_PATH="${OPENVINO_ROOT}/runtime/lib/intel64:${OPENVINO_ROOT}/runtime/3rdparty/tbb/lib:${LD_LIBRARY_PATH}"
ENV PKG_CONFIG_PATH="${OPENVINO_ROOT}/runtime/lib/pkgconfig:${PKG_CONFIG_PATH}"
ENV CMAKE_PREFIX_PATH="${OPENVINO_ROOT}/runtime/cmake:${CMAKE_PREFIX_PATH}"

# 复制并安装大恒（MindVision）SDK
COPY linuxSDK_V2.1.0.37.tar.gz /tmp/
RUN set -eux; \
    cd /tmp; \
    tar -xzf linuxSDK_V2.1.0.37.tar.gz; \
    # 确保 install.sh 在当前目录并以该目录为工作目录执行
    if [ -f ./install.sh ]; then \
        chmod +x ./install.sh; \
        bash -ex ./install.sh; \
    else \
        echo "install.sh not found in /tmp after extraction" >&2; ls -la /tmp; exit 1; \
    fi; \
    rm -f /tmp/linuxSDK_V2.1.0.37.tar.gz

# 更新 LD_LIBRARY_PATH 包含 MindVision
ENV LD_LIBRARY_PATH="/usr/lib/mindvision:${LD_LIBRARY_PATH}"

# 项目代码
WORKDIR /workspace
COPY . /workspace

# 编译（使用 CMAKE_PREFIX_PATH 而非 -DOPENVINO_DIR）
RUN bash -c "source /opt/ros/humble/setup.bash && \
    cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=${OPENVINO_ROOT}/runtime/cmake && \
    cmake --build build -j$(nproc)"

# 创建启动脚本
RUN echo '#!/bin/bash\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["tail", "-f", "/dev/null"]