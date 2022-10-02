#!/usr/bin/python3

# Copyright (c) 2021 Skyward Experimental Rocketry
# Author: Alberto Nidasio
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the 'Software'), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# The linter script offers a lot of functionalities:
# - Checks for the right copyright notice in all .h and .cpp files
# - Checks if all .h and .cpp files respects the clang format specified in .clang-format file
# - Checks if some .h or .cpp files have printfs, asserts or some string used in them

import re
from collections import Counter
from subprocess import DEVNULL, STDOUT, call, run, check_output, CalledProcessError
from argparse import ArgumentParser, RawTextHelpFormatter
from os import pathconf, walk
from os.path import join

# Copyright template for C++ code files and headers
CPP_TEMPLATE = r'\/\* Copyright \(c\) 20\d\d(?:-20\d\d)? Skyward Experimental Rocketry\n' + \
    r' \* (Authors?): (.+)\n' + \
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
    r' \* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE\n' + \
    r' \* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER\n' + \
    r' \* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,\n' + \
    r' \* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN\n' + \
    r' \* THE SOFTWARE.\n' + \
    r' \*\/'
AUTHOR_DELIMITER = ','


def config_cmd_parser():
    parser = ArgumentParser(
        description='The linter script offers a lot of functionalities but it does not modify any file. If no option is specified everything will be checked.',
        formatter_class=RawTextHelpFormatter)
    parser.add_argument('directory', nargs='?',
                        help='Directory where to search files')
    parser.add_argument(
        '--copyright', dest='copyright',
        action='store_true', help='Checks for the right copyright notice in all .h and .cpp files')
    parser.add_argument(
        '--format', dest='format',
        action='store_true', help='Checks if all .h and .cpp files respects the clang format specified in .clang-format file')
    parser.add_argument(
        '--find', dest='find',
        action='store_true', help='Checks if some .h or .cpp files have printfs, asserts or some string used in them')
    parser.add_argument(
        '--cppcheck', dest='cppcheck',
        action='store_true', help='Runs cppcheck for static code analysis')
    parser.add_argument('-q', '--quiet', dest='quiet',
                        action='store_true', help='Output only essential messages')
    return parser


def print_banner():
    # Font: Ivrit
    print('+------------------------------+')
    print('|  _     _       _             |')
    print('| | |   (_)_ __ | |_ ___ _ __  |')
    print('| | |   | | \'_ \| __/ _ \ \'__| |')
    print('| | |___| | | | | ||  __/ |    |')
    print('| |_____|_|_| |_|\__\___|_|    |')
    print('+------------------------------+')


class Colors():
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'
    UNDERLINE = '\033[4m'
    RESET = '\033[0m'


# Checks for the right copyright notice in all .h and .cpp files
def check_copyright(directory):
    print(Colors.GREEN + 'Copyright check' + Colors.RESET)

    # Statistics
    totalCheckdFilesCounter = 0
    filesWithErrorsCounter = 0
    authors = {}
    averageAuthorsPerFile = 0

    # Walk through the directory and check each file
    for dirpath, dirnames, filenames in walk(directory):
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
                    print(Colors.YELLOW + 'Wrong copyright notice in file {0}'.format(
                        currentFilepath) + Colors.RESET)
                else:
                    fileAuthors = [a.strip()
                                   for a in match.group(2).split(AUTHOR_DELIMITER)]

                    # Check the number of authors against 'Author' or `Authors`
                    if len(fileAuthors) == 1 and match.group(1)[-1] == 's':
                        print('\'Authors\' should to be changed to \'Author\' in {0}'.format(
                            currentFilepath))
                    if len(fileAuthors) > 1 and match.group(1)[-1] != 's':
                        print('\'Author\' should to be changed to \'Authors\' in {0}'.format(
                            currentFilepath))

                    # Save statistics on authors
                    for author in fileAuthors:
                        if author in authors:
                            authors[author] += 1
                        else:
                            authors[author] = 1
                    averageAuthorsPerFile += len(fileAuthors)
    averageAuthorsPerFile /= totalCheckdFilesCounter

    print('Checked {} files'.format(totalCheckdFilesCounter))
    if filesWithErrorsCounter == 0:
        print('All the files have the correct copyright notice')
    else:
        print(Colors.RED + '{:.1f}% ({}/{}) of all analyzed files do not match with the copyright template!'.format(
            100*filesWithErrorsCounter/totalCheckdFilesCounter, filesWithErrorsCounter, totalCheckdFilesCounter) + Colors.RESET)

    if (not args.quiet):
        print('{:.2} authors per file'.format(
            averageAuthorsPerFile))

        print('Number of mentions per author:')
        for author in sorted(authors.items(), key=lambda item: item[1], reverse=True):
            print('{:3} - {}'.format(author[1], author[0]))

    # Exit if error if at least one file isn't correct
    if (filesWithErrorsCounter > 0):
        exit(-1)


# Checks if all .h and .cpp files respects the clang format specified in .clang-format file
def check_format(directory):
    print(Colors.GREEN + 'Formatting check' + Colors.RESET)

    # Statistics
    totalCheckdFilesCounter = 0
    filesWithErrorsCounter = 0

    # Walk throgh the directory and check each file
    for dirpath, dirnames, filenames in walk(directory):
        for filename in [f for f in filenames if f.endswith(('.cpp', '.h', 'c'))]:
            totalCheckdFilesCounter += 1

            # Prepare the complete filepath
            currentFilepath = join(dirpath, filename)

            # Dry run clang-format and check if we have an error
            returnCode = call(
                ['clang-format', '-style=file', '--dry-run', '--Werror', '--ferror-limit=1', currentFilepath], stderr=DEVNULL)

            # If and error occurs warn the user
            if (returnCode != 0):
                filesWithErrorsCounter += 1

                if (not args.quiet):
                    print(Colors.YELLOW + 'Wrong code format for file {1}'.format(
                        returnCode, currentFilepath) + Colors.RESET)

    print('Checked {} files'.format(totalCheckdFilesCounter))
    if filesWithErrorsCounter == 0:
        print('All the files match the Skyward formatting style')
    else:
        print(Colors.RED + '{:4.1f}% ({}/{}) of all analyzed files do not match Skyward formatting style!'.format(
            100*filesWithErrorsCounter/totalCheckdFilesCounter, filesWithErrorsCounter, totalCheckdFilesCounter), Colors.RESET)

    # Exit if error if at least one file isn't correct
    if (filesWithErrorsCounter > 0):
        exit(-1)


def find_in_code(directory, searchTerm, extensionFilters=('.cpp', '.h'), pathFilter=None):
    print(
        Colors.GREEN + 'Checking for \'{}\' in code files'.format(searchTerm) + Colors.RESET)

    # Statistics
    totalCheckdFilesCounter = 0
    filesWithErrorsCounter = 0

    # Walk through the directory and check each file
    for dirpath, dirnames, filenames in walk(directory):
        for filename in [f for f in filenames if f.endswith(extensionFilters) and (not pathFilter or dirpath.find(pathFilter) >= 0)]:
            totalCheckdFilesCounter += 1

            # Prepare the complete filepath
            currentFilepath = join(dirpath, filename)

            # Check the current file
            with open(currentFilepath) as file:
                fileContent = file.read()
                # Check for linter off flag
                if re.search('linter off', fileContent, re.M):
                    continue
                match = re.search(searchTerm, fileContent, re.M)
                if match:
                    filesWithErrorsCounter += 1

                    # The current file has the error
                    if (not args.quiet):
                        print(Colors.YELLOW + 'Found \'{}\' in file {}'.format(searchTerm,
                                                                               currentFilepath) + Colors.RESET)

    print('Checked {} files'.format(totalCheckdFilesCounter))
    if filesWithErrorsCounter == 0:
        print(
            'All the files does not contain \'{}\''.format(searchTerm))
    else:
        print(Colors.RED + '{:.1f}% ({}/{}) of all analyzed files contain \'{}\'!'.format(
            100*filesWithErrorsCounter/totalCheckdFilesCounter, filesWithErrorsCounter, totalCheckdFilesCounter, searchTerm) + Colors.RESET)

    return filesWithErrorsCounter


def check_find(directory):
    sum = find_in_code(directory, r'^using namespace', '.h')
    sum += find_in_code(directory,
                        r'[^a-zA-Z0-9]printf\(', pathFilter='shared')
    sum += find_in_code(directory, r'( |^)assert\(')
    sum += find_in_code(directory, '^ *throw ', pathFilter='catch')

    if sum > 0:
        exit(-1)


def check_cppcheck(directory):
    print(Colors.GREEN + 'cppcheck' + Colors.RESET)
    # Run cppcheck on the directory
    try:
        result = check_output(['cppcheck', '-q', '--language=c++', '--template=gcc', '--std=c++11', '--enable=all', '--inline-suppr',
                               '--suppress=unmatchedSuppression', '--suppress=unusedFunction', '--suppress=missingInclude',
                               directory], stderr=STDOUT)

        # Parse results and count errors
        errors = re.findall(r'\[(\w+)\]', result.decode('utf-8'))
        errors = Counter(errors)

        if (not args.quiet):
            print('cppcheck found the following errors:')
            for error in errors:
                print('{:3} - {}'.format(errors[error], error))

        totalErrors = sum(errors.values())
        if (totalErrors > 0):
            print(
                Colors.RED + 'cppcheck found {} errors in total'.format(totalErrors) + Colors.RESET)
            exit(-1)
        else:
            print('cppcheck did not find any errors')

    except CalledProcessError as e:
        print(e.output.decode('utf-8'))
        exit(-1)

# -------------------------------------------------------------
# MAIN
# -------------------------------------------------------------


parser = config_cmd_parser()
args = parser.parse_args()

if (not args.directory):
    print('No directory specified')
    print('')
    parser.print_help()
    exit(-1)

if (args.copyright):
    check_copyright(args.directory)

if (args.format):
    check_format(args.directory)

if (args.cppcheck):
    check_cppcheck(args.directory)

if (args.find):
    check_find(args.directory)

# Checks everything if no option is specified
if (not args.copyright and not args.format and not args.find and not args.cppcheck):
    check_copyright(args.directory)
    check_format(args.directory)
    check_find(args.directory)
    check_cppcheck(args.directory)
