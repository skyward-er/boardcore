#!/usr/bin/python3

# Copyright (c) 2021 Skyward Experimental Rocketry
# Author: Alberto Nidasio
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

import os
import sys
import datetime
from os import walk
from os.path import join, dirname
from argparse import ArgumentParser
from genutils import parse_scxml, read_file, write_file

current_directory = dirname(sys.argv[0])
TEMPLATE_FOLDER = 'templates'
OUTPUT_FOLDER = 'generated'
DEFAULT_NAMESPACE = 'DeathStackBoard'

# Templates file names
FSM_CONTROLLER_H_TEMPLATE_FILE = current_directory + \
    ('/' if current_directory != '' else '') + \
    TEMPLATE_FOLDER + '/FSMController.h.template'
FSM_CONTROLLER_CPP_TEMPLATE_FILE = current_directory + \
    ('/' if current_directory != '' else '') + \
    TEMPLATE_FOLDER + '/FSMController.cpp.template'
FSM_DATA_H_TEMPLATE_FILE = current_directory + \
    ('/' if current_directory != '' else '') + \
    TEMPLATE_FOLDER + '/FSMData.h.template'
STATE_FUNCTION_DEFINITION_TEMPLATE_FILE = current_directory + \
    ('/' if current_directory != '' else '') + \
    TEMPLATE_FOLDER + '/state_function_definition.template'
FSM_TEST_CPP_TEMPLATE_FILE = current_directory + \
    ('/' if current_directory != '' else '') + \
    TEMPLATE_FOLDER + '/test.cpp.template'

# Templates strings
STATE_FUNCTION_DECLARATION_TEMPLATE = 'void state_{state_name}(const Event& ev);'
STATE_FUNCTION_CASE_TEMPLATE = ' '*8 + 'case {event}:\n' + \
    ' '*8 + '{{\n' + \
    '{actions}\n' + \
    ' '*12 + 'break;\n' + \
    ' '*8 + '}}'
UTILITY_FUNCTION_DECLARATION_TEMPLATE = 'void {function_name}();'
TOPICS_SUBSCIPTION_TEMPLATE = ' '*4 + 'EventBroker::getInstance().subscribe(this, {topic});'
UTILITY_FUNCTION_DEFINITION_TEMPLATE = 'void {state_machine_name}Controller::{function_name}()\n{{\n    // ...\n}}'
STATE_FUNCTION_CASE_TRANSITION_TO_TARGET_TEMPLATE = 'transition(&{state_machine_name}Controller::state_{state_name});'
TEST_CASE_METHOD_TEMPLATE = 'TEST_CASE_METHOD({state_machine_name}ControllerFixture, "Testing transitions from {state}")\n' + \
    '{{\n' + \
    ' '*4 + 'controller->transition(&{state_machine_name}Controller::state_{start_state});\n\n' + \
    '{test_case_sections}' + \
    '\n}}'
TEST_CASE_SECTION_TEMPLATE = ' '*4 + 'SECTION("{event} -> {target_state_upper}")\n' + \
    ' '*4 + '{{\n' + \
    ' '*8 + 'REQUIRE(testFSMTransition(*controller, Event{{{event}}}, &{state_machine_name}Controller::state_{target_state}));\n' + \
    ' '*4 + '}}'
README_TEMPLATE = '```mermaid\nstateDiagram-v2\n{mermaid}\n```'


def config_cmd_parser():
    parser = ArgumentParser(
        description='The FSMGen program generates cpp files (FSMController.h, FSMController.cpp, FSMData.h and test-FMS.cpp) for each scxml file specified.')
    parser.add_argument('-q', '--quiet', help='Output only essential messages', dest='quiet',
                        action='store_true')
    parser.add_argument('-a', '--authors', dest='authors')
    parser.add_argument('-n', '--main_namespace',
                        help=DEFAULT_NAMESPACE + ' as default', dest='main_namespace')
    parser.add_argument('-f', '--files', dest='files', metavar='F', nargs='*')
    parser.add_argument('directory', nargs='?',
                        help='Directory where to search files')
    return parser


def print_banner():
    # Font: Ivrit
    print("+----------------------------------------------------------------------------+")
    print("|  ____  _                                _                                  |")
    print("| / ___|| | ___   ___      ____ _ _ __ __| |                                 |")
    print("| \\___ \\| |/ / | | \\ \\ /\\ / / _` | '__/ _` |                                 |")
    print(
        "|  ___) |   <| |_| |\\ V  V / (_| | | | (_| |                                 |")
    print("| |____/|_|\\_\\\\__, | \\_/\\_/ \\__,_|_|  \\__,_|                                 |")
    print("|        _____|___/  __  __    ____                           _              |")
    print("|       |  ___/ ___||  \\/  |  / ___| ___ _ __   ___ _ __ __ _| |_ ___  _ __  |")
    print("|       | |_  \___ \\| |\\/| | | |  _ / _ \\ '_ \\ / _ \\ '__/ _` | __/ _ \\| '__| |")
    print("|       |  _|  ___) | |  | | | |_| |  __/ | | |  __/ | | (_| | || (_) | |    |")
    print("|       |_|   |____/|_|  |_|  \\____|\\___|_| |_|\\___|_|  \\__,_|\\__\___/|_|    |")
    print("|                                                                            |")
    print("+----------------------------------------------------------------------------+")


def generate_fsm_controller_h(fsmData, year, authors, main_namespace):
    state_machine_name = fsmData[0]

    # Open template file
    template = read_file(FSM_CONTROLLER_H_TEMPLATE_FILE)

    # Prepare the state functions declarations
    states_funcions_declarations = [
        STATE_FUNCTION_DECLARATION_TEMPLATE.format(state_name=state[0])
        for state in fsmData[3]]
    states_funcions_declarations = '\n'.join(
        [' '*4 + declaration for declaration in states_funcions_declarations])

    # Preapare utility functions declarations
    utility_functions_declarations = [
        UTILITY_FUNCTION_DECLARATION_TEMPLATE.format(
            function_name=function.replace('()', ''))
        for function in fsmData[4]]
    utility_functions_declarations = '\n'.join(
        [' '*4 + declaration for declaration in utility_functions_declarations]
    )

    # Insert information into template file
    content = template.format(
        year=year, authors=authors, main_namespace=main_namespace,
        state_machine_name=state_machine_name,
        states_funcions_declarations=states_funcions_declarations,
        start_state=fsmData[2] + '_' + fsmData[1].upper(),
        utility_functions_declarations=utility_functions_declarations
    )

    # Save the file
    return write_file(OUTPUT_FOLDER + '/' + state_machine_name,
                      state_machine_name + 'Controller.h', content)


def generate_fsm_controller_cpp(fsmData, year, authors, main_namespace):
    state_machine_name = fsmData[0]

    # Open template files
    template = read_file(FSM_CONTROLLER_CPP_TEMPLATE_FILE)
    functon_definition_template = read_file(
        STATE_FUNCTION_DEFINITION_TEMPLATE_FILE)

    # Prepare the state functions definitions
    states_functions_definitions = []
    for state in fsmData[3]:
        event_cases = []

        # Prepare onentry and onexit actions
        onentry_actions = '\n'.join(
            [' '*12 + action + ';' for action in state[1]])
        onexit_actions = '\n'.join(
            [' '*12 + action + ';' for action in state[2]])
        if onentry_actions == '':
            onentry_actions = ' '*12 + '// ...'
        if onexit_actions == '':
            onexit_actions = ' '*12 + '// ...'

        # Prepare the cases
        for event in state[3]:
            actions = [action + ';' for action in event[2]]
            if event[1] and event[1] != state[0]:
                actions.append(STATE_FUNCTION_CASE_TRANSITION_TO_TARGET_TEMPLATE.format(
                    state_machine_name=state_machine_name, state_name=event[1]))
            actions = '\n'.join([' '*12 + action for action in actions])
            if actions == '':
                actions = ' '*12 + '// ...'
            event_cases.append(STATE_FUNCTION_CASE_TEMPLATE.format(
                event=event[0], actions=actions
            ))
        event_cases = '\n'.join(event_cases)

        states_functions_definitions.append(
            functon_definition_template.format(
                state_machine_name=state_machine_name,
                state_name=state[0].lower(),
                current_state=state[0].upper(),
                onentry_actions=onentry_actions, onexit_actions=onexit_actions,
                event_cases=event_cases
            )
        )
    states_functions_definitions = '\n\n'.join(states_functions_definitions)

    # Preapare utility functions declarations
    utility_functions_definiton = []
    for function in fsmData[4]:
        utility_functions_definiton.append(
            UTILITY_FUNCTION_DEFINITION_TEMPLATE.format(
                state_machine_name=state_machine_name,
                function_name=function.replace('()', '')
            )
        )
    utility_functions_definiton = '\n\n'.join(utility_functions_definiton)

    # Prepare topics subscriptions
    topics_subscriptions = []
    for topic in fsmData[5]:
        topics_subscriptions.append(TOPICS_SUBSCIPTION_TEMPLATE.format(
            topic='TOPIC_' + topic
        ))
    topics_subscriptions = '\n'.join(topics_subscriptions)

    # Insert information into template file
    content = template.format(
        year=year, authors=authors, state_machine_name=state_machine_name,
        initial_state=fsmData[1], topics_subscriptions=topics_subscriptions,
        topic='TOPIC_' + fsmData[2], main_namespace=main_namespace,
        states_functions_definitions=states_functions_definitions,
        utility_functions_definiton=utility_functions_definiton
    )

    # Save the file
    return write_file(OUTPUT_FOLDER + '/' + state_machine_name,
                      state_machine_name + 'Controller.cpp', content)


def generate_fsm_data_h(fsmData, year, authors, main_namespace):
    state_machine_name = fsmData[0]

    # Open template file
    template = read_file(FSM_DATA_H_TEMPLATE_FILE)

    # Prepare the states list
    states = [state[0].upper() for state in fsmData[3]]
    states[0] += ' = 0'
    states = '\n'.join([' '*4 + state + ',' for state in states])

    # Insert information into template file
    content = template.format(
        year=year, authors=authors, main_namespace=main_namespace,
        state_machine_name=state_machine_name, states=states
    )

    # Save the file
    return write_file(OUTPUT_FOLDER + '/' + state_machine_name,
                      state_machine_name + 'Data.h', content)


def generate_fsm_test_cpp(fsmData, year, authors):
    state_machine_name = fsmData[0]

    # Open template file
    template = read_file(FSM_TEST_CPP_TEMPLATE_FILE)

    # Prepare the test cases list
    test_cases = []
    for state in fsmData[3]:
        # Prepare test case sections
        test_case_sections = []
        for event in state[3]:
            test_case_sections.append(TEST_CASE_SECTION_TEMPLATE.format(
                target_state_upper=event[1].upper(), target_state=event[1],
                state_upper=state[0].upper(), event=event[0],
                state_machine_name=state_machine_name
            ))
        test_case_sections = '\n\n'.join(test_case_sections)

        test_cases.append(TEST_CASE_METHOD_TEMPLATE.format(
            state_machine_name=state_machine_name, state=state[0],
            start_state=state[0],
            test_case_sections=test_case_sections
        ))
    test_cases = '\n\n'.join(test_cases)

    # Insert information into template file
    content = template.format(
        year=year, authors=authors, state_machine_name=state_machine_name,
        test_cases=test_cases
    )

    # Save the file
    return write_file(OUTPUT_FOLDER + '/' + 'tests', 'test-' + fsmData[0].lower() + '.cpp', content)


def generate_readme_with_mermaid(fsmData):
    state_machine_name = fsmData[0]
    lines = []

    # Initial state
    lines.append('[*] --> ' + fsmData[1])

    # Most commented code is because gitlab does not support some features of mermaid

    # Generate descriptions and transitons
    for state in fsmData[3]:
        # Name of the state
        # lines.append(state[0] + ' : ' + state[0])

        # Description with onentry, onexit and other actions
        # for action in state[1]:
        #     lines.append(state[0] + ' : onentry - ' + action)
        # for action in state[2]:
        #     lines.append(state[0] + ' : onexit - ' + action)
        for transiton in state[3]:
            # if transiton[1] == state[0]:
            #     lines.append(state[0] + ' : ' + transiton[0] + (
            #         ' - ' + ''.join(transiton[2])
            #         if len(transiton[2]) > 0 else ''))
            # Transitions
            # else:
            if not transiton[1] == state[0]:
                lines.append(state[0] + ' --> ' +
                             transiton[1] + ' : ' + transiton[0])
                #  + ('\\n' + ''.join(transiton[2]) if len(transiton[2]) > 0 else '')

    mermaid = '\n'.join([' '*4 + line for line in lines])
    mermaid = README_TEMPLATE.format(mermaid=mermaid)

    # Save the file
    return write_file(OUTPUT_FOLDER + '/' + state_machine_name, 'readme.md', mermaid)


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
    print('[fsmgen] Welcome to the FSMgen script!')
    print('')

if(not args.authors):
    print('[fsmgen] WARNING missing authors parameter')
    args.authors = 'Someone'

if(not args.main_namespace):
    print('[fsmgen] WARNING missing namespace parameter')
    args.main_namespace = DEFAULT_NAMESPACE

date = datetime.datetime.now()

for file in files:
    print('[fsmgen] Now parsing ' + file)

    fsmData = parse_scxml(file)

    # Generate FSMController.h
    file_name = generate_fsm_controller_h(
        fsmData, date.year, args.authors, args.main_namespace)
    print('[fsmgen] Generated ' + file_name)

    # Generate FSMController.cpp
    file_name = generate_fsm_controller_cpp(
        fsmData, date.year, args.authors, args.main_namespace)
    print('[fsmgen] Generated ' + file_name)

    # Generate FSMData.h
    file_name = generate_fsm_data_h(
        fsmData, date.year, args.authors, args.main_namespace)
    print('[fsmgen] Generated ' + file_name)

    # Generate test-FMS.cpp
    file_name = generate_fsm_test_cpp(fsmData, date.year, args.authors)
    print('[fsmgen] Generated ' + file_name)

    # Generate readme.md
    file_name = generate_readme_with_mermaid(fsmData)
    print('[fsmgen] Generated ' + file_name)
