FROM ros:humble-ros-base-jammy

# Install Python dependencies and validation tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-flake8 \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install black formatter
RUN pip3 install --no-cache-dir black

# Copy validation script
COPY validate-chapter.sh /usr/local/bin/validate-chapter.sh
RUN chmod +x /usr/local/bin/validate-chapter.sh

# Set working directory
WORKDIR /workspace

# Default command
CMD ["/bin/bash"]
