#!/usr/bin/python3

# Copyright (c) 2018 Skyward Experimental Rocketry
# Authors: Luca Erbetta
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

from googleapiclient.discovery import build
from httplib2 import Http
from oauth2client import file, client, tools

import re
import datetime
import sys


SCOPES = 'https://www.googleapis.com/auth/spreadsheets.readonly'

service = None

SPREADSHEET_ID = '12TecOmDd7Uot-MvXkCbhDJRU48-XO6s5ChKDlr4AOvI'
EVENTS_RANGE_NAME = 'EventList!A2:A'
TOPICS_RANGE_NAME = 'Topics!B3:B'


def auth():
    try:
        store = file.Storage('store.json')
        creds = store.get()
        if not creds or creds.invalid:
            flow = client.flow_from_clientsecrets('credentials.json', SCOPES)
            creds = tools.run_flow(flow, store)

        global service
        service = build('sheets', 'v4', http=creds.authorize(Http()))
        return True
    except:
        print("Authentication error:", sys.exc_info()[0])
        return False


def load_events():
    result = service.spreadsheets().values().get(spreadsheetId=SPREADSHEET_ID,
                                                 range=EVENTS_RANGE_NAME).execute()
    lines = result.get('values', [])

    # Event names start with EV_ and contains only uppercase letters or underscores
    re_event = re.compile(r'(?P<ev>^EV_([A-Z_]+)+)')

    # Only return lines with valid events, remove additional data
    events = []
    for l in lines:
        m = re_event.match(l[0])
        if m is not None:
            events.append(m.group('ev'))
        else:
            print("Skipped line containing invalid event: {}".format(l))

    return events


def load_topics():
    result = service.spreadsheets().values().get(spreadsheetId=SPREADSHEET_ID,
                                                 range=TOPICS_RANGE_NAME).execute()
    lines = result.get('values', [])
    re_topics = re.compile(r'(?P<topic>^TOPIC_([A-Z_]+)+)')

    # Only return lines with valid topics, remove additional data
    topics = []
    for l in lines:
        m = re_topics.match(l[0])
        if m is not None:
            topics.append(m.group('topic'))
        else:
            print("Skipped line containing invalid topics: {}".format(l))

    return topics


def has_duplicates(lst):
    if len(lst) != len(set(lst)):
        return True
    return False


print("Homeone on-board software event header generator v0.2")
print("Google sheets API auth in progress...")
if auth():
    print("Auth successfull.")
else:
    exit()

print("Reading from: https://docs.google.com/spreadsheets/d/12TecOmDd7Uot-MvXkCbhDJRU48-XO6s5ChKDlr4AOvI")

events = load_events()
topics = load_topics()

# Check duplicates
if has_duplicates(events):
    print("Duplicate events found! Terminating.")
    exit()

if has_duplicates(topics):
    print("Duplicate topics found! Terminating.")
    exit()

print("{} events loaded.".format(len(events)))
print("{} topics loaded.".format(len(topics)))

enum_str = ""
map_str = ""

date = datetime.datetime.now()
link = "https://docs.google.com/spreadsheets/d/{id}".format(id=SPREADSHEET_ID)


# Events.h generation

print("Generating Events.h...")

for e in events:
    endl = ",\n" if e != events[-1] else ""
    enum_str += "    " + e + \
        (" = EV_FIRST_SIGNAL" if e == events[0] else "") + endl
    map_str += "        {{ {event}, {string} }}{endl}".format(
        event=e, string='"' + e + '"', endl=endl)

with open('Events.h.template', 'r') as template_file:
    template = template_file.read()

template = template.format(sheet_link=link, date=date,
                           enum_data=enum_str, map_data=map_str)

with open('Events.h', 'w') as header_file:
    header_file.write(template)

print("Events.h successfully generated.")
# Topics.h generation

print("Generating Topics.h...")
enum_str = ""
map_str = ""

for t in topics:
    endl = ",\n" if t != topics[-1] else ""
    enum_str += "    " + t + endl
    map_str += "        {{ {topics}, {string} }}{endl}".format(
        topics=t, string='"' + t + '"', endl=endl)

with open('Topics.h.template', 'r') as template_file:
    template = template_file.read()

template = template.format(sheet_link=link, date=date,
                           enum_data=enum_str, map_data=map_str)

with open('Topics.h', 'w') as header_file:
    header_file.write(template)

print("Topics.h successfully generated.")
print("Done.")
