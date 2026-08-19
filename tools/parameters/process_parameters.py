#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2013-2017 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################


#
# Dima parameter processor.
#
# The firmware build supplies an explicit source set and always emits the XML
# and JSON catalogues consumed by the remaining generation chain.
#

from __future__ import print_function

import argparse
import sys

from dima_params import jsonout, srcparser, srcscanner, xmlout


def main():
    parser = argparse.ArgumentParser(
        description="Process the explicit Dima parameter source set."
    )
    parser.add_argument(
        "--src-file",
        required=True,
        metavar="FILE",
        nargs="+",
        help="explicit source files to scan for parameters",
    )
    parser.add_argument(
        "-x",
        "--xml",
        required=True,
        metavar="FILENAME",
        help="XML catalogue output path",
    )
    parser.add_argument(
        "-j",
        "--json",
        required=True,
        metavar="FILENAME",
        help="JSON catalogue output path",
    )
    parser.add_argument(
        "-b",
        "--board",
        required=True,
        metavar="BOARD",
        help="board identity written to the generated catalogues",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="print explicit inputs and generated outputs",
    )
    args = parser.parse_args()

    scanner = srcscanner.SourceScanner()
    parser = srcparser.SourceParser()

    if args.verbose:
        print("Scanning explicit source files: " + ", ".join(args.src_file))

    for source_file in args.src_file:
        if not scanner.ScanFile(source_file, parser):
            sys.exit(1)

    if not parser.Validate():
        sys.exit(1)
    param_groups = parser.GetParamGroups()

    if not param_groups:
        print("Warning: no parameters found")

    if args.verbose:
        print("Creating XML file " + args.xml)
    xmlout.XMLOutput(param_groups, args.board).Save(args.xml)

    if args.verbose:
        print("Creating JSON file " + args.json)
    jsonout.JsonOutput(param_groups, args.board).Save(args.json)


if __name__ == "__main__":
    main()
