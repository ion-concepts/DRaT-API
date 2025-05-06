#
# Copyright 2008,2009 Free Software Foundation, Inc.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

# The presence of this file turns this directory into a Python package

'''
DRaT

This GNU Radio out-of-tree module implements sources and sinks for
DRaT, the Digital Radio Transport.
'''
import os

# import pybind11 generated symbols into the drat namespace
from .drat_python import *

# import any pure python here
from .time_source import time_source
