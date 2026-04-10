FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV SYSTEMC_VERSION=2.3.4
ENV SYSTEMC_HOME=/usr/local/systemc
ENV CMAKE_BUILD_PARALLEL_LEVEL=4

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    make \
    g++ \
    wget \
    tar \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp

RUN wget https://github.com/accellera-official/systemc/archive/refs/tags/${SYSTEMC_VERSION}.tar.gz -O systemc.tar.gz && \
    tar -xzf systemc.tar.gz && \
    cd systemc-${SYSTEMC_VERSION} && \
    mkdir build && cd build && \
    cmake .. \
      -DCMAKE_INSTALL_PREFIX=${SYSTEMC_HOME} \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_STANDARD=17 \
      -DCMAKE_CXX_STANDARD_REQUIRED=ON && \
    make -j"$(nproc)" && \
    make install && \
    rm -rf /tmp/systemc-${SYSTEMC_VERSION} /tmp/systemc.tar.gz

ENV LD_LIBRARY_PATH=${SYSTEMC_HOME}/lib:${SYSTEMC_HOME}/lib-linux64:${LD_LIBRARY_PATH}
ENV CMAKE_PREFIX_PATH=${SYSTEMC_HOME}:${CMAKE_PREFIX_PATH}

WORKDIR /workspace

CMD ["/bin/bash"]