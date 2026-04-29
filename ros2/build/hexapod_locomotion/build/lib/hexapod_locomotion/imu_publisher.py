#!/usr/bin/env python3

"""Backward-compatible entry point for the BNO055 IMU publisher."""

from .bno055_publisher import BNO055Publisher, main

ImuPublisher = BNO055Publisher


if __name__ == '__main__':
    main()
