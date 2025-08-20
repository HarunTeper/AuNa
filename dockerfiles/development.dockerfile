# syntax=docker/dockerfile:1-labs
FROM base:latest

USER root

# Development tools installation with cleanup [2]
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    clangd \
    clang-format \
    clang-tidy \
    gdb \
    valgrind \
    htop \
    tree \
    vim \
    nano \
    tmux \
    build-essential \
    cmake

# Copy dependency files for caching optimization [2]
COPY --parents packages/src/**/package.xml /tmp/package_xmls/
COPY --parents packages/**/requirements.txt /tmp/requirements/
COPY --parents packages/**/*.toml /tmp/toml/

# Fix ownership
RUN chown -R ubuntu:ubuntu /tmp/package_xmls /tmp/requirements /tmp/toml

USER ubuntu

# Update rosdep database
RUN rosdep update

# Install all dependencies
RUN rosdep install --from-paths /tmp/package_xmls --ignore-src -r -y || echo "Some rosdep packages failed" \
    && rm -rf /tmp/package_xmls

# Install all Python dependencies with error handling
RUN find /tmp/requirements -name "requirements.txt" -exec pip install --no-cache-dir -r {} \; 2>/dev/null || true \
    && find /tmp/toml -name "pyproject.toml" -exec pip install --no-cache-dir -e {} \; 2>/dev/null || true \
    && rm -rf /tmp/requirements /tmp/toml

# Bash configuration for development
RUN echo 'if [ -f /home/ubuntu/workspace/packages/install/setup.bash ]; then source /home/ubuntu/workspace/packages/install/setup.bash; fi' >> /home/ubuntu/.bashrc \
    && echo 'alias cb="colcon build --symlink-install"' >> /home/ubuntu/.bashrc \
    && echo 'alias cbp="colcon build --symlink-install --packages-select"' >> /home/ubuntu/.bashrc \
    && echo 'alias cbt="colcon test"' >> /home/ubuntu/.bashrc \
    && echo 'alias cbtr="colcon test-result --verbose"' >> /home/ubuntu/.bashrc \
    && echo 'function ll() { ls -alF "$@"; }' >> /home/ubuntu/.bashrc \
    && echo 'function la() { ls -A "$@"; }' >> /home/ubuntu/.bashrc

# Copy and set up entrypoint
COPY dockerfiles/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN sudo chmod +x /usr/local/bin/entrypoint.sh

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]
