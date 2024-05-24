FROM quay.io/pypa/manylinux2014_x86_64:latest

LABEL maintainer="Tim Schneider <tim@robot-learning.de>"

RUN yum -y install poco-devel*

WORKDIR /tmp

RUN git clone https://gitlab.com/libeigen/eigen.git \
    && cd eigen \
    && git checkout 3.4.0 \
    && mkdir build && cd build \
    && cmake .. \
    && make install \
    && cd /tmp && rm -rf eigen

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

ARG PYBIN_SUPPORTED_VERSIONS="cp3(7m|8|9|10|11|12)"
ENV PYBIN_SUPPORTED_VERSIONS=${PYBIN_SUPPORTED_VERSIONS}
RUN for PYBIN in /opt/python/*/bin; do \
      if [[ "${PYBIN}" =~ ${PYBIN_SUPPORTED_VERSIONS} ]]; then \
        "${PYBIN}/pip" install cmake setuptools wheel --user; \
      fi; \
    done \
    && ln -f -s $HOME/.local/bin/cmake /usr/bin/cmake

ARG LIBFRANKA_VERSION=0.13.3
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

# Set working directory
RUN mkdir -p /code
RUN mkdir -p /build
VOLUME /code
VOLUME /build
WORKDIR /build
ADD build-wheels /usr/bin/
ADD run-tests /usr/bin/
