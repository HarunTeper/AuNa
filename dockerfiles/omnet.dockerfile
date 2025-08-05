# syntax=docker/dockerfile:1-labs
# Build from the base dockerfile
FROM base:latest

USER root

# Copy and set up entrypoint script
COPY dockerfiles/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Install wget and download/extract OMNeT++
RUN apt-get update && apt-get install -y wget \
    && wget https://github.com/omnetpp/omnetpp/releases/download/omnetpp-5.6.2/omnetpp-5.6.2-src-linux.tgz \
    && tar -xvzf omnetpp-5.6.2-src-linux.tgz -C /home/ubuntu/ \
    && rm omnetpp-5.6.2-src-linux.tgz

# Install OMNeT++ dependencies
RUN apt-get update && apt-get -y install \
    build-essential clang lld gdb bison flex perl python3 python3-pip \
    qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libqt5opengl5-dev \
    libxml2-dev zlib1g-dev doxygen graphviz libwebkit2gtk-4.0-37 \
    libopenscenegraph-dev mpi-default-dev default-jre \
    libprotobuf-dev protobuf-compiler libcurl4-openssl-dev libgdal-dev

RUN echo "WITH_OSGEARTH=no" >> /home/ubuntu/omnetpp-5.6.2/configure.user

RUN chown -R ubuntu:ubuntu /home/ubuntu/omnetpp-5.6.2

USER ubuntu

RUN python3 -m pip install --user --upgrade numpy pandas matplotlib scipy seaborn posix_ipc

# Set OMNeT++ environment variables
RUN echo "export OMNETPP_PATH=/home/ubuntu/omnetpp-5.6.2" >> /home/ubuntu/.bashrc \
    && echo "export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH}:\${OMNETPP_PATH}/lib" >> /home/ubuntu/.bashrc \
    && echo "export PATH=\${PATH}:\${OMNETPP_PATH}/bin" >> /home/ubuntu/.bashrc

RUN cd /home/ubuntu/omnetpp-5.6.2 && \
    export OMNETPP_PATH=/home/ubuntu/omnetpp-5.6.2 && \
    export PATH=$PATH:$OMNETPP_PATH/bin && \
    ./configure && \
    make -j$(nproc)

USER root

USER ubuntu

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]