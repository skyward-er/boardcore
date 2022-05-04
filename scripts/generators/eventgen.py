#!/usr/bin/python3

# Copyright (c) 2018-2021 Skyward Experimental Rocketry
# Authors: Luca Erbetta, Alvise de' Faveri Tron, Alberto Nidasio
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
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import sys
import datetime
from os import walk
from os.path import join, dirname
from argparse import ArgumentParser
from genutils import parse_scxml, read_file, write_file

current_directory = dirname(sys.argv[0])
TEMPLATE_FOLDER = 'templates'
OUTPUT_FOLDER = 'generated'

# Templates file names
EVENTS_H_TEMPLATE_FILE = current_directory + \
    ('/' if current_directory != '' else '') + \
    TEMPLATE_FOLDER + '/Events.h.template'
TOPICS_H_TEMPLATE_FILE = current_directory + \
    ('/' if current_directory != '' else '') + \
    TEMPLATE_FOLDER + '/Topics.h.template'
EVENT_STRINGS_CPP_TEMPLATE_FILE = current_directory + \
    ('/' if current_directory != '' else '') + \
    TEMPLATE_FOLDER + '/EventStrings.cpp.template'


def config_cmd_parser():
    parser = ArgumentParser(
        description='The Eventgen program generates the files Events.h, Topics.hand EventString.cpp gathering information from all scxml files specified.')
    parser.add_argument('-q', '--quiet', dest='quiet',
                        action='store_true', help='Output only essential messages')
    parser.add_argument('-f', '--files', dest='files', metavar='F', nargs='*')
    parser.add_argument('directory', nargs='?',
                        help='Directory where to search files')
    return parser


def print_banner():
    # Font: Ivrit
    print("+-----------------------------------------------------+")
    print("|  ____  _                                _           |")
    print("| / ___|| | ___   ___      ____ _ _ __ __| |          |")
    print("| \\___ \\| |/ / | | \\ \\ /\\ / / _` | '__/ _` |          |")
    print("|  ___) |   <| |_| |\\ V  V / (_| | | | (_| |          |")
    print("| |____/|_|\\_\\\\__, | \\_/\\_/ \\__,_|_|  \\__,_|          |")
    print("|        _____|___/            _                      |")
    print("|       | ____|_   _____ _ __ | |_ __ _  ___ _ __     |")
    print("|       |  _| \\ \\ / / _ \\ '_ \\| __/ _` |/ _ \\ '_ \\    |")
    print("|       | |___ \\ V /  __/ | | | || (_| |  __/ | | |   |")
    print("|       |_____| \\_/ \\___|_| |_|\\__\\__, |\\___|_| |_|   |")
    print("|                                 |___/               |")
    print("+-----------------------------------------------------+")


def extract_events(fsmData):
    # Get all possible events
    events = []
    for state in fsmData[3]:
        for event in state[3]:
            events.append(event[0])

    # Remove doubles
    events = list(dict.fromkeys(events))

    return events


def generate_events_h(events, date):
    template = read_file(EVENTS_H_TEMPLATE_FILE)

    if not events or len(events) == 0:
        return

    # Prepare the event list
    enum_event_list = list(events)
    enum_event_list[0] += ' = EV_FIRST_CUSTOM'
    enum_event_list = '\n'.join(
        [' '*4 + event + ',' for event in enum_event_list])
    vector_event_list = '\n'.join([' '*4 + event + ',' for event in events])

    # Insert information into template file
    content = template.format(
        year=date.year, date=date, enum_event_list=enum_event_list,
        vector_event_list=vector_event_list
    )

    # Save the file
    return write_file(OUTPUT_FOLDER + '/events', 'Events.h', content)


def generate_topics_h(topics, date):
    template = read_file(TOPICS_H_TEMPLATE_FILE)

    if not topics or len(topics) == 0:
        return

    # Prepare the topic list
    topic_list = '\n'.join([' '*4 + topic + ',' for topic in topics])

    # Insert information into template file
    content = template.format(year=date.year, date=date, topic_list=topic_list)

    # Save the file
    return write_file(OUTPUT_FOLDER + '/events', 'Topics.h', content)


def generate_event_string_cpp(events, topics, date):
    template = read_file(EVENT_STRINGS_CPP_TEMPLATE_FILE)

    if not events or len(events) == 0 or not topics or len(topics) == 0:
        return

    # Prepare the event and topic lists
    event_list = '\n'.join(
        [' '*8 + '{' + event + ', "' + event + '"},' for event in events])
    topic_list = '\n'.join(
        [' '*8 + '{' + topic + ', "' + topic + '"},' for topic in topics])

    # Insert information into template file
    content = template.format(
        year=date.year, date=date,
        event_list=event_list, topic_list=topic_list
    )

    # Save the file
    return write_file(OUTPUT_FOLDER + '/events', 'EventStrings.cpp', content)


# -------------------------------------------------------------
# MAIN
# -------------------------------------------------------------


parser = config_cmd_parser()
args = parser.parse_args()
files = []

if(not args.files and not args.directory):
    print('[eventgen] No scxml files specified nor a directory')
    print('')
    parser.print_help()
    exit(-1)

# Fetch files to parse
if args.files:
    files.extend(args.files)
if args.directory:
    for dirpath, dirnames, filenames in walk(args.directory):
        for filename in [f for f in filenames if f.endswith(".scxml")]:
            if not 'template' in filename:
                files.append(join(dirpath, filename))

if(not args.quiet):
    print_banner()
    print('')
    print('[eventgen] Welcome to the eventgen script!')
    print('')

date = datetime.datetime.now()
events = []
topics = []

for file in files:
    print('[eventgen] Now parsing ' + file)

    fsmData = parse_scxml(file)

    # Extract events
    events.extend(extract_events(fsmData))

    # Save topic
    topics.extend(['TOPIC_' + topic for topic in fsmData[5]])

# Remove duplicates
events = list(dict.fromkeys(events))
topics = list(dict.fromkeys(topics))

# Sort
events.sort()
topics.sort()

# Generate Events.h
file_name = generate_events_h(events, date)
if file_name:
    print('[eventgen] Generated ' + file_name)

# Generate Topics.h
file_name = generate_topics_h(topics, date)
if file_name:
    print('[eventgen] Generated ' + file_name)

# Generate EventString.cpp
file_name = generate_event_string_cpp(events, topics, date)
if file_name:
    print('[eventgen] Generated ' + file_name)
