#!/usr/bin/env python
# -*- coding: utf-8 -*-


class F_ARM(object):

    def __init__(self):
        self.joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.delta_thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.link_lenghts = [26.5, 28.1831, 20.0]