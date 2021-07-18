#!/usr/bin/python3

# Copyright (c) 2021 Skyward Experimental Rocketry
# Authors: Alberto Nidasio
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import re
from argparse import ArgumentParser
from os import walk
from os.path import join

# Copyright template for C++ code files and headers
CPP_TEMPLATE = r'\/\* Copyright \(c\) 20\d\d(?:-20\d\d)? Skyward Experimental Rocketry\n' + \
r' \* Authors: (.+)\n' + \
r' \*\n' + \
r' \* Permission is hereby granted, free of charge, to any person obtaining a copy\n' + \
r' \* of this software and associated documentation files \(the "Software"\), to deal\n' + \
r' \* in the Software without restriction, including without limitation the rights\n' + \
r' \* to use, copy, modify, merge, publish, distribute, sublicense, and\/or sell\n' + \
r' \* copies of the Software, and to permit persons to whom the Software is\n' + \
r' \* furnished to do so, subject to the following conditions:\n' + \
r' \*\n' + \
r' \* The above copyright notice and this permission notice shall be included in\n' + \
r' \* all copies or substantial portions of the Software.\n' + \
r' \*\n' + \
r' \* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR\n' + \
r' \* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,\n' + \
r' \* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE\n' + \
r' \* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER\n' + \
r' \* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,\n' + \
r' \* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN\n' + \
r' \* THE SOFTWARE.\n' + \
r' \*\/'

def config_cmd_parser():
    parser = ArgumentParser(
    description='The Copyright Checker script verifies that all files in the specified directory contains the correct copyright notice.\nSupport cpp code and header files.')
    parser.add_argument('directory', nargs='?',
                        help='Directory where to search files')
    return parser

def print_banner():
    # Font: Ivrit
    print("+------------------------------------------------+")
    print("|    ___                       _       _     _   |")
    print("|  / ___|___  _ __  _   _ _ __(_) __ _| |__ | |_ |")
    print("| | |   / _ \\| '_ \\| | | | '__| |/ _` | '_ \\| __||")
    print("| | |__| (_) | |_) | |_| | |  | | (_| | | | | |_ |")
    print("|  \\____\\___/| .__/ \\__, |_|  |_|\\__, |_| |_|\\__||")
    print("|        ____|_|    |___/     _  |___/           |")
    print("|       / ___| |__   ___  ___| | _____ _ __      |")
    print("|      | |   | '_ \\ / _ \\/ __| |/ / _ \\ '__|     |")
    print("|      | |___| | | |  __/ (__|   <  __/ |        |")
    print("|       \\____|_| |_|\\___|\\___|_|\\_\\___|_|        |")
    print("+------------------------------------------------+")

# -------------------------------------------------------------
# MAIN
# -------------------------------------------------------------


parser = config_cmd_parser()
args = parser.parse_args()
totalCheckdFilesCounter = 0
filesWithErrorsCounter = 0

if(not args.directory):
    print('[chopyrightchecker] No directory specified')
    print('')
    parser.print_help()
    exit(-1)

# Walk throgh the directory and check each file
for dirpath, dirnames, filenames in walk(args.directory):
    for filename in [f for f in filenames if f.endswith(('.cpp', '.h'))]:
        totalCheckdFilesCounter += 1

        # Prepare the complete filepath
        currentFilepath = join(dirpath, filename)

        # Check the current file
        with open(currentFilepath) as file:
            match = re.search(CPP_TEMPLATE, file.read())
            if not match:
                filesWithErrorsCounter += 1

                # The file's copyright notice does not match the template!
                print('The file {0} does not match the correct copyright notice!'.format(currentFilepath))


print('Checked {0} files, {1} files do not match with the copyright template'.format(totalCheckdFilesCounter, filesWithErrorsCounter))        
