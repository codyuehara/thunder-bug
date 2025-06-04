#! /bin/bash
docker build -f Dockerfile.host-build --build-arg WORKSPACE=$workspace -t codyuehara/thunder-bug .
