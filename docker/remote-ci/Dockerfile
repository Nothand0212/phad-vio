FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG GTSAM_COMMIT=2f3e56c0ddbd3a1aa54ed043643b553d26a069f6

ARG UBUNTU_MIRROR=http://mirrors.aliyun.com/ubuntu/

RUN sed -i \
        -e "s|http://archive.ubuntu.com/ubuntu/|${UBUNTU_MIRROR}|" \
        -e "s|http://security.ubuntu.com/ubuntu/|${UBUNTU_MIRROR}|" \
        /etc/apt/sources.list.d/ubuntu.sources \
    && apt-get -o Acquire::Retries=5 -o Acquire::http::Timeout=30 \
        -o Acquire::http::Pipeline-Depth=0 update \
    && apt-get -o Acquire::Retries=5 -o Acquire::http::Timeout=30 \
        -o Acquire::http::Pipeline-Depth=0 \
        install --yes --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        git \
        libboost-chrono-dev \
        libboost-date-time-dev \
        libboost-filesystem-dev \
        libboost-program-options-dev \
        libboost-regex-dev \
        libboost-serialization-dev \
        libboost-system-dev \
        libboost-thread-dev \
        libboost-timer-dev \
        libeigen3-dev \
        libgtest-dev \
        libmetis-dev \
        libopencv-dev \
        libssl-dev \
        libyaml-cpp-dev \
        ninja-build \
        nlohmann-json3-dev \
        python3 \
        python3-pip \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

ARG GTSAM_ARCHIVE_SHA256=50bd99ddbb363f03f145d814995df234c83ad38f867080fba5b60f6c151b348a
COPY --from=gtsam-source gtsam.tar.gz /tmp/gtsam.tar.gz
RUN echo "${GTSAM_ARCHIVE_SHA256}  /tmp/gtsam.tar.gz" | sha256sum --check - \
    && mkdir /tmp/gtsam \
    && tar -xzf /tmp/gtsam.tar.gz --strip-components=1 -C /tmp/gtsam \
    && rm /tmp/gtsam.tar.gz \
    && cmake -S /tmp/gtsam -B /tmp/gtsam-build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_PYTHON=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_WITH_TBB=OFF \
    && cmake --build /tmp/gtsam-build --parallel "$(nproc)" \
    && cmake --install /tmp/gtsam-build \
    && ldconfig \
    && rm -rf /tmp/gtsam /tmp/gtsam-build

RUN apt-get -o Acquire::Retries=5 -o Acquire::http::Timeout=30 \
        -o Acquire::http::Pipeline-Depth=0 update \
    && apt-get -o Acquire::Retries=5 -o Acquire::http::Timeout=30 \
        -o Acquire::http::Pipeline-Depth=0 \
        install --yes --no-install-recommends python3-numpy \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv --system-site-packages /opt/phad-venv

COPY smoke /tmp/smoke
RUN cmake -S /tmp/smoke -B /tmp/smoke-build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/smoke-build --parallel "$(nproc)" \
    && install -m 0755 /tmp/smoke-build/phad-remote-ci-smoke \
        /usr/local/bin/phad-remote-ci-smoke \
    && /usr/local/bin/phad-remote-ci-smoke \
    && rm -rf /tmp/smoke /tmp/smoke-build

COPY write_toolchain_manifest.py /tmp/write_toolchain_manifest.py
RUN GTSAM_COMMIT="${GTSAM_COMMIT}" \
    python3 /tmp/write_toolchain_manifest.py /opt/phad-toolchain.json \
    && rm /tmp/write_toolchain_manifest.py

ENV PATH="/opt/phad-venv/bin:${PATH}"

CMD ["/bin/bash"]
