# syntax=docker/dockerfile:1-labs
# Build from the base dockerfile
FROM base:latest

USER root

# Copy the package.xml, requirements.txt, and pyproject.toml files for dependency resolution to maximize cache efficiency
COPY --parents packages/src/**/package.xml /tmp/package_xmls/
COPY --parents packages/**/requirements.txt /tmp/requirements/
COPY --parents packages/**/*.toml /tmp/toml/

# Fix ownership after COPY (COPY creates files owned by root)
RUN chown -R ubuntu:ubuntu /tmp/package_xmls
RUN chown -R ubuntu:ubuntu /tmp/requirements
RUN chown -R ubuntu:ubuntu /tmp/toml

# Copy and set up entrypoint script
COPY dockerfiles/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

RUN apt update

RUN sudo apt update && sudo apt install -y clangd

# Update rosdep database
RUN rosdep update

ARG PIP_BREAK_SYSTEM_PACKAGES
ENV PIP_BREAK_SYSTEM_PACKAGES=${PIP_BREAK_SYSTEM_PACKAGES}

# Install dependencies for all packages using only package.xml files
RUN rosdep install --from-paths /tmp/package_xmls --ignore-src -r -y && \
    rm -rf /tmp/package_xmls

# Install Python packages from requirements files
RUN find /tmp/requirements -name "requirements.txt" -exec pip install -r {} \; && \
    rm -rf /tmp/requirements

# Install Python packages from pyproject.toml files  
RUN find /tmp/toml -name "pyproject.toml" -exec pip install -e {} \; && \
    rm -rf /tmp/toml

USER ubuntu

# Source the workspace in bashrc if setup.bash exists
RUN echo 'if [ -f /home/ubuntu/workspace/packages/install/setup.bash ]; then source /home/ubuntu/workspace/packages/install/setup.bash; fi' >> /home/ubuntu/.bashrc

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]