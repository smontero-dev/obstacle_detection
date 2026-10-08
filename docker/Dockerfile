# Development image: ROS 2 Humble desktop (RViz included) plus the project's
# Python dependencies. Built and started by docker/run.sh.
FROM osrf/ros:humble-desktop

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt
