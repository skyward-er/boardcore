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

from re import sub
from os import makedirs
from os.path import join, basename
from xml.etree.ElementTree import parse


def camel_case_to_underscores(string):
    return sub(r'((?<=[a-z])[A-Z]|(?<!\A)[A-Z](?=[a-z]))', r'_\1', string)


def parse_scxml(file):
    name = basename(file).split('.')[0]

    # Parse the scxml file
    tree = parse(str(file))
    root = tree.getroot()
    initial_state = root.attrib['initial']

    # Extract states and actions
    topics = []
    states_list = []
    actions_list = []
    for state in root:
        state_id = camel_case_to_underscores(state.attrib['id']).lower()

        # Extract on entry and on exit actions
        onentry_actions = []
        onexit_actions = []
        for onentry in state.iter('{http://www.w3.org/2005/07/scxml}onentry'):
            if 'postD' in onentry.text or 'removeD' in onentry.text:
                onentry_actions.append(onentry.text)
            else:
                onentry_actions.append(
                    camel_case_to_underscores(onentry.text).lower())
        for onexit in state.iter('{http://www.w3.org/2005/07/scxml}onexit'):
            if 'postD' in onexit.text or 'removeD' in onexit.text:
                onexit_actions.append(onexit.text)
            else:
                onexit_actions.append(
                    camel_case_to_underscores(onexit.text).lower())

        # Extract transitions and topics
        transitions_list = []
        for transition in state.iter('{http://www.w3.org/2005/07/scxml}transition'):
            try:
                if '.' in transition.attrib['event']:
                    (topic, event) = camel_case_to_underscores(
                        transition.attrib['event']).split('.')
                    topics.append(topic)
                    event = topic + "_" + event
                else:
                    event = camel_case_to_underscores(
                        transition.attrib['event'])
            except:
                print(
                    '[genutils] Transition is missing event parameter in file ' + file)
                exit(-1)
            try:
                target = camel_case_to_underscores(
                    transition.attrib['target']).lower()
            except:
                target = None

            # Divide each line of the transition text into a single action
            try:
                transition_actions = transition.text.splitlines()
                for i, line in enumerate(transition_actions):
                    transition_actions[i] = line.lstrip().rstrip()
                transition_actions = list(
                    filter(lambda a: a != '', transition_actions))
                transition_actions = [
                    camel_case_to_underscores(action).lower()
                    if transition.text and not 'postD' in transition.text
                    and not 'removeD' in transition.text else action
                    for action in transition_actions]
            except:
                transition_actions = []

            transitions_list.append((event, target, transition_actions))
            actions_list.extend(transition_actions)

        states_list.append(
            (state_id, onentry_actions, onexit_actions, transitions_list))

        # Remove from the actions list all the duplicates, postD and removeD
        actions_list.extend(onentry_actions)
        actions_list.extend(onexit_actions)
        actions_list = list(dict.fromkeys(actions_list))
        actions_list = list(
            filter(lambda a: not 'postD' in a and not 'removeD' in a, actions_list))
        actions_list.sort()

    state_machine_topic = max(set(topics), key=topics.count)
    topics = list(dict.fromkeys(topics))

    return (name, initial_state, state_machine_topic, states_list, actions_list, topics)


def read_file(file_name):
    try:
        with open(file_name, 'r') as r_file:
            return r_file.read()
    except:
        print('Could not open template file "' + file_name + '"')
        exit(-1)


def write_file(file_path, file_name, content):
    file_path = file_path

    try:
        # Ensures the path exists
        makedirs(file_path, exist_ok=True)

        with open(join(file_path, file_name), 'w') as w_file:
            w_file.write(content)

        return file_name
    except:
        print('Could not create and/or write file "' +
              file_path + '/' + file_name + '"')
        exit(-1)
