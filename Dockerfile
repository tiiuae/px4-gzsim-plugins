FROM ghcr.io/tiiuae/gz-sim-server:main AS builder

RUN apt-get update -y \
    && apt install -y \
    wget lsb-core \
    build-essential \
    cmake \
    git \
    libboost-filesystem-dev \
    libtinyxml-dev

# Clone c_library_v2 commit matching with current px4-firmware mavlink commit
# => mavlink/c_library_v2:fbdb7c29 is built from mavlink/mavlink:08112084
RUN git clone -q https://github.com/mavlink/c_library_v2.git  /usr/local/include/mavlink && \
    cd /usr/local/include/mavlink && git checkout -q fbdb7c29e47902d44eeaa58b4395678a9b78f3ae && \
    rm -rf /usr/local/include/mavlink/.git

ENV _MAVLINK_INCLUDE_DIR  /usr/local/include/mavlink

WORKDIR /px4-plugins
ADD . .

RUN ./build.sh

#---------------------------------------------------------------------

FROM busybox

WORKDIR /artifacts
COPY --from=builder /px4-plugins/build/*.so /artifacts