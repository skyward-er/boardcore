#!/usr/bin/python3

# Copyright (c) 2018-2020 Skyward Experimental Rocketry
# Authors: Luca Erbetta, Alvise de' Faveri Tron
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
#
import re
import datetime
from os.path import join
import os

import xml.etree.ElementTree as ET
import sys
from collections import OrderedDict

OUTPUT_FOLDER = "generated"

#
# ASCII art banner
#
def printBanner():
    print("+--------------------------------------------------------+")
    print("|     ____  _                                _           |")
    print("|    / ___|| | ___   ___      ____ _ _ __ __| |          |")
    print("|    \\___ \\| |/ / | | \\ \\ /\\ / / _` | '__/ _` |          |")
    print("|     ___) |   <| |_| |\\ V  V / (_| | | | (_| |          |")
    print("|    |____/|_|\\_\\\\__, | \\_/\\_/ \\__,_|_|  \\__,_|          |")
    print("|           _____|___/            _                      |")
    print("|          | ____|_   _____ _ __ | |_ __ _  ___ _ __     |")
    print("|          |  _| \\ \\ / / _ \\ '_ \\| __/ _` |/ _ \\ '_ \\    |")
    print("|          | |___ \\ V /  __/ | | | || (_| |  __/ | | |   |")
    print("|          |_____| \\_/ \\___|_| |_|\\__\\__, |\\___|_| |_|   |")
    print("|                                    |___/               |")
    print("+--------------------------------------------------------+")

#
# Parse SCXML file(s)
#
def parse_scxml(files):
    events = []
    topics = []

    for file in files:
        print("parsing " + str(file))
        tree = ET.parse(str(file))
        root = tree.getroot()

        # find all transitions and related events
        first = 0
        for child in root:
            for tran in child.iter('{http://www.w3.org/2005/07/scxml}transition'):
                if(tran.attrib['target'] != 'final'):
                    ev = tran.attrib['event']

                    try:
                        topics.append("TOPIC_" + ev.split('.')[0])
                        events.append(ev.split('.')[1])

                    except:
                        print("Cannot parse the event name... Forgot the topic?")
                        print(tran.tag, tran.attrib)

    # remove duplicates
    events = list(OrderedDict.fromkeys(events))
    topics = list(OrderedDict.fromkeys(topics))
    events.sort()
    topics.sort()

    print("{} events loaded.".format(len(events)))
    print("{} topics loaded.".format(len(topics)))

    return events, topics

#
# Generate Events.h
#
def generate_events(events, date):
    enum_str = ""
    event_map_str = ""
    event_list_str = ""

    # generate string
    for e in events:
        endl = ",\n" if e != events[-1] else ""
        enum_str += "    " + e + \
            (" = EV_FIRST_SIGNAL" if e == events[0] else "") + endl
        event_map_str += "        {{ {event}, {string} }}{endl}".format(
            event=e, string='"' + e + '"', endl=endl)
        event_list_str += e + (", " if e != events[-1] else "")

    # read template
    with open('Events.h.template', 'r') as template_file:
        template = template_file.read()

    # write template
    template = template.format(date=date, enum_data=enum_str, event_list=event_list_str)
    with open(join(OUTPUT_FOLDER, 'Events.h'), 'w') as header_file:
        header_file.write(template)

    return event_map_str

#
# Generate Topics.h
#
def generate_topics(topics, date):
    enum_str = ""
    topic_map_str = ""
    topic_list_str = ""

    for t in topics:
        endl = ",\n" if t != topics[-1] else ""
        enum_str += "    " + t + endl
        topic_map_str += "        {{ {topics}, {string} }}{endl}".format(
            topics=t, string='"' + t + '"', endl=endl)
        topic_list_str += t + (", " if t != topics[-1] else "")

    with open('Topics.h.template', 'r') as template_file:
        template = template_file.read()

    template = template.format(date=date, enum_data=enum_str, topic_list=topic_list_str)

    with open(join(OUTPUT_FOLDER, 'Topics.h'), 'w') as header_file:
        header_file.write(template)

    return topic_map_str

#
# Generate EventFunctions.cpp
#
def generate_functions(event_map_str, topic_map_str, date):
    with open('EventStrings.cpp.template', 'r') as cpp_template_file:
        cpp = cpp_template_file.read()

    cpp = cpp.format(date=date, event_map_data=event_map_str, topic_map_data=topic_map_str)

    with open(join(OUTPUT_FOLDER, 'EventStrings.cpp'), 'w') as cpp_file:
        cpp_file.write(cpp)

#
# Main
#
printBanner()
print()
print("Skyward SCXML event generator")
print()

if(len(sys.argv) < 2):
    print("Error: no SCXML files provided.\nUsage: eventgen.py <SCXML_FILES>")
    sys.exit(-1)

if not os.path.exists(OUTPUT_FOLDER):
    os.mkdir(OUTPUT_FOLDER)

date = datetime.datetime.now()

print("-"*30)
print("Parsing SCXML files...")
events, topics = parse_scxml(sys.argv[1:])

print("-"*30)
print("Generating Events.h...")
event_map_str = generate_events(events, date)
print("OK\n")

print("-"*30)
print("Generating Topics.h...")
topic_map_str = generate_topics(topics, date)
print("OK\n")

print("-"*30)
print("Generating EventFunctions.cpp...")
generate_functions(event_map_str, topic_map_str, date)
print("OK\n")


# print
print("-"*30)
print("Events:\n")
for e in events:
    print(e)
print("\n\nTopics:\n")
for t in topics:
    print(t)

print("-"*30)
print("... Done.")
