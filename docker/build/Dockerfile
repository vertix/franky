FROM quay.io/pypa/manylinux2014_x86_64:latest

LABEL maintainer="Tim Schneider <tim@robot-learning.de>"

RUN yum -y install poco-devel*

ADD dependency_versions /tmp/
ADD docker/common/ /tmp/common/
ARG LIBFRANKA_VERSION
RUN set -a && . /tmp/dependency_versions && set +a \
    && /tmp/common/install-dependencies

ARG PYBIN_SUPPORTED_VERSIONS="cp3(7m|8|9|10|11|12)"
# Preserve argument for runtime
ENV PYBIN_SUPPORTED_VERSIONS=${PYBIN_SUPPORTED_VERSIONS}
RUN /tmp/common/manylinux-setup-pip

# Set working directory
RUN mkdir -p /code
RUN mkdir -p /build
VOLUME /code
VOLUME /build
WORKDIR /build
ADD docker/common/manylinux-build-wheels /usr/bin/build-wheels
ADD docker/build/run-tests /usr/bin/

# Cleaning up
RUN rm -rf /tmp/common
