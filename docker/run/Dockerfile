FROM ubuntu:22.04

LABEL maintainer="Tim Schneider <tim@robot-learning.de>"

ARG LIBFRANKA_VERSION=0.13.3

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get -y install build-essential cmake git libpoco-dev catch2 python3-dev

WORKDIR /tmp

RUN git clone https://gitlab.com/libeigen/eigen.git \
    && cd eigen \
    && git checkout 3.4.0 \
    && mkdir build && cd build \
    && cmake .. \
    && make install \
    && cd /tmp && rm -rf eigen

RUN git clone --recursive https://github.com/frankaemika/libfranka.git \
    && cd libfranka \
    && git checkout "${LIBFRANKA_VERSION}" \
    # Cherry-pick missing imports commit
    && git config user.email "john.doe@example.com" \
    && git config user.name "John Doe" \
    && if ! git merge-base --is-ancestor f1f46fb HEAD; then git cherry-pick -m 1 f1f46fb; fi \
    && git submodule update \
    && mkdir build && cd build \
    && cmake -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF .. \
    && make -j$(nproc) \
    && make install \
    && cd /tmp && rm -rf libfranka

RUN git clone https://github.com/catchorg/Catch2.git \
    && cd Catch2 \
    && git checkout v2.13.8 \
    && mkdir build && cd build \
    && cmake -DCATCH_BUILD_TESTING=OFF -DCATCH_ENABLE_WERROR=OFF -DCATCH_INSTALL_DOCS=OFF -DCATCH_INSTALL_HELPERS=OFF .. \
    && make install \
    && cd /tmp && rm -rf Catch2

RUN git clone https://github.com/pybind/pybind11.git \
    && cd pybind11 \
    && git checkout v2.11.1 \
    && mkdir build && cd build \
    && cmake -DPYBIND11_TEST=OFF .. \
    && make -j$(nproc) \
    && make install \
    && cd /tmp && rm -rf pybind11

RUN git clone https://github.com/TimSchneider42/franky.git --recurse-submodules \
    && cd franky \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && cd /tmp && rm -rf franky
