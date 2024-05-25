FROM ubuntu:22.04

LABEL maintainer="Tim Schneider <tim@robot-learning.de>"

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get -y install build-essential cmake git libpoco-dev catch2 python3-dev

ADD dependency_versions /tmp/
ADD docker/common/ /tmp/common/
ARG LIBFRANKA_VERSION
RUN set -a && . /tmp/dependency_versions && set +a \
    && /tmp/common/install-dependencies

RUN git clone https://github.com/TimSchneider42/franky.git --recurse-submodules \
    && cd franky \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && cd /tmp && rm -rf franky

# Cleaning up
RUN rm -rf /tmp/common