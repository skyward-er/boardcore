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

import math
import re
import datetime
import sys
from os.path import join
import os
from string import Template

SCOPES = "https://www.googleapis.com/auth/spreadsheets.readonly"

service = None

# Spreadsheet file used to generate the events. The can be found in the link
# to the spreadsheet, for example:
# https://docs.google.com/spreadsheets/d/184kR2OAD7yWV0fYJdiGUDmHmy5_prY3nr-XgNA0Uge0/
# -->
# 184kR2OAD7yWV0fYJdiGUDmHmy5_prY3nr-XgNA0Uge0
spreadsheet_id = "1D5m5WrNXxAL8XA6CKyfe5JcQ5IRnpNHg5ZG5bzt5G4I"
output_folder = "hermes"

RANGE_FIELD_NAME = "{sheet_name}!A2:A"
RANGE_FIELD_TYPE = "{sheet_name}!B2:B"
RANGE_FIELD_SIZE = "{sheet_name}!C2:C"
RANGE_FIELD_MIN = "{sheet_name}!D2:D"
RANGE_FIELD_MAX = "{sheet_name}!E2:E"
RANGE_REPEAT_NUM = "{sheet_name}!J1"
RANGE_MAV_NAME = "{sheet_name}!J2"

with open("templates/TelemetryPacker.h.template", "r") as template_file:
    tm_packer_template = Template(template_file.read())
with open("templates/packFunction.cpp.template", "r") as template_file:
    pack_fun_template = Template(template_file.read())
with open("templates/unpackFunction.cpp.template", "r") as template_file:
    unpack_fun_template = Template(template_file.read())
with open("templates/ConversionFunctions.h.template", "r") as template_file:
    convfuns_template = Template(template_file.read())
with open("templates/Packets.h.template", "r") as template_file:
    header_template = Template(template_file.read())


def auth():
    try:
        store = file.Storage("store.json")
        creds = store.get()
        if not creds or creds.invalid:
            flow = client.flow_from_clientsecrets("credentials.json", SCOPES)
            creds = tools.run_flow(flow, store)

        global service
        service = build("sheets", "v4", http=creds.authorize(Http()))
        return True
    except:
        print("Authentication error:", sys.exc_info()[0])
        return False


def listSheets():
    result = service.spreadsheets().get(spreadsheetId=spreadsheet_id).execute()

    return [x["properties"]["title"] for x in result["sheets"]]


def loadPacketFromSheet(packet_name: str):
    # Strip 'Packet' at the end
    packet = {"name": packet_name[0:-6]}

    result = (
        service.spreadsheets()
        .values()
        .get(
            spreadsheetId=spreadsheet_id,
            range=RANGE_FIELD_NAME.format(sheet_name=packet_name),
        )
        .execute()
    )
    fields = [x[0] for x in result["values"]]

    result = (
        service.spreadsheets()
        .values()
        .get(
            spreadsheetId=spreadsheet_id,
            range=RANGE_FIELD_SIZE.format(sheet_name=packet_name),
        )
        .execute()
    )
    sizes = [int(x[0]) for x in result["values"]]

    result = (
        service.spreadsheets()
        .values()
        .get(
            spreadsheetId=spreadsheet_id,
            range=RANGE_FIELD_TYPE.format(sheet_name=packet_name),
        )
        .execute()
    )
    types = [x[0] for x in result["values"]]

    result = (
        service.spreadsheets()
        .values()
        .get(
            spreadsheetId=spreadsheet_id,
            range=RANGE_FIELD_MIN.format(sheet_name=packet_name),
        )
        .execute()
    )
    rmins = [float(x[0]) for x in result["values"]]

    result = (
        service.spreadsheets()
        .values()
        .get(
            spreadsheetId=spreadsheet_id,
            range=RANGE_FIELD_MAX.format(sheet_name=packet_name),
        )
        .execute()
    )
    rmaxs = [float(x[0]) for x in result["values"]]

    

    result = (
        service.spreadsheets()
        .values()
        .get(
            spreadsheetId=spreadsheet_id,
            range=RANGE_REPEAT_NUM.format(sheet_name=packet_name),
        )
        .execute()
    )

    indices = [0]
    for i in range(1, len(sizes)):
        indices += [indices[i - 1] + sizes[i - 1]]

    packet["repeat"] = int(result["values"][0][0])

    result = (
        service.spreadsheets()
        .values()
        .get(
            spreadsheetId=spreadsheet_id,
            range=RANGE_MAV_NAME.format(sheet_name=packet_name),
        )
        .execute()
    )

    indices = [0]
    for i in range(1, len(sizes)):
        indices += [indices[i - 1] + sizes[i - 1]]

    packet["mav_tm_name"] = result["values"][0][0]

    packet["fields"] = [
        {"name": f, "size": s, "index": i, "type": t, "range": (rmin, rmax)}
        for f, s, i, t, rmin, rmax in zip(fields, sizes, indices, types, rmins, rmaxs)
    ]
    packet["partial_size"] = sum(sizes)
    packet["total_size"] = math.ceil((sum(sizes) * packet["repeat"]) / 8)
    return packet


def generatePackFunction(packet_data, field):
    convert_fun_template = Template(
        "${telemetry_name}Conversion::${field_name_ccase}ToBits"
    )

    subs = {}

    subs["max_index"] = packet_data["repeat"]

    subs["type"] = field["type"]
    name = field["name"].replace("_", " ").title().replace(" ", "")
    name = name[0].lower() + name[1:]
    subs["convert_fun"] = convert_fun_template.substitute(
        telemetry_name=packet_data["name"], field_name_ccase=name,
    )
    subs["field_name_lcase"] = field["name"].lower()
    subs["field_name_ccase"] = field["name"].replace("_", " ").title().replace(" ", "")
    subs["field_name_ucase"] = field["name"].upper()
    subs["mav_tm_name"] = packet_data["mav_tm_name"]

    return pack_fun_template.substitute(**subs)


def generateUnpackFunction(packet_data, field):
    convert_fun_template = Template(
        "${telemetry_name}Conversion::bitsTo${field_name_ccase}"
    )

    subs = {}
    subs["max_index"] = packet_data["repeat"]

    subs["type"] = field["type"]
    subs["convert_fun"] = convert_fun_template.substitute(
        telemetry_name=packet_data["name"],
        field_name_ccase=field["name"].replace("_", " ").title().replace(" ", ""),
    )
    subs["field_name_lcase"] = field["name"].lower()
    subs["field_name_ccase"] = field["name"].replace("_", " ").title().replace(" ", "")
    subs["field_name_ucase"] = field["name"].upper()
    subs["mav_tm_name"] = packet_data["mav_tm_name"]

    return unpack_fun_template.substitute(**subs)


def generatePackerClass(packet_data, output_folder):
    index_template = Template("        INDEX_$field_name = $index")
    size_template = Template("        SIZE_$field_name = $size")

    indices = ""
    sizes = ""
    pack_funcs = ""
    unpack_funcs = ""
    for f in packet_data["fields"]:
        indices += index_template.substitute(field_name=f["name"].upper(), index=f["index"])
        sizes += size_template.substitute(field_name=f["name"].upper(), size=f["size"])

        if f != packet_data["fields"][-1]:
            indices += ",\n"
            sizes += ",\n"

        pack_funcs += generatePackFunction(packet_data, f)
        unpack_funcs += generateUnpackFunction(packet_data, f)

    out = tm_packer_template.substitute(
        mav_tm_name=packet_data["mav_tm_name"],
        telemetry_name_ccase=packet_data["name"],
        tm_total_size=packet_data["total_size"],
        tm_partial_size=packet_data["partial_size"],
        field_indices=indices,
        field_sizes=sizes,
        pack_functions=pack_funcs,
        unpack_functions=unpack_funcs,
        folder=output_folder
    )

    with open(join(output_folder, packet_data["name"] + "Packer.h"), "w") as out_file:
        out_file.write(out)


def generateConversionFunctions(packet_data):
    class_template = Template(
        "\nclass ${telemetry_name_ccase}Conversion\n{\npublic:\n    $functions\n};\n\n\n"
    )
    
    frombits_template = Template(
        "\n    static $type bitsTo${field_name_ccase}(uint64_t bits)\n    "
        + "{\n        return ($type)bits;\n    }\n"
    )

    tobits_template = Template(
        "\n    static uint64_t ${field_name_ccase}ToBits($type $field_name_lcase)"
        + "\n    {\n        return (uint64_t)$field_name_lcase;\n    }\n\n"
    )

    frombits_template_ranged = Template(
        "\n    static $type bitsTo${field_name_ccase}(uint64_t bits)\n    "
        + "{\n        return undiscretizeRange<$type>(bits, $rmin, $rmax, $res);\n    }\n"
    )

    tobits_template_ranged = Template(
        "\n    static uint64_t ${field_name_ccase}ToBits($type $field_name_lcase)"
        + "\n    {\n        return discretizeRange<$type>($field_name_lcase, $rmin, $rmax, $res);\n    }\n\n"
    )

    funs = ""
    for field in packet_data["fields"]:
        subs = {}
        subs["type"] = field["type"]
        subs["field_name_lcase"] = field["name"].lower()
        subs["field_name_ccase"] = field["name"].replace("_", " ").title().replace(" ", "")
        subs["field_name_ucase"] = field["name"].upper()
        subs["rmin"] = field["range"][0]
        subs["rmax"] = field["range"][1]
        subs["res"] = 2**field["size"]
        if subs["rmax"] - subs["rmin"] != 0:
            funs += frombits_template_ranged.substitute(**subs)

            subs["field_name_ccase"] = (
                subs["field_name_ccase"][0].lower() + subs["field_name_ccase"][1:]
            )
            funs += tobits_template_ranged.substitute(**subs)
        else:
            funs += frombits_template.substitute(**subs)

            subs["field_name_ccase"] = (
                subs["field_name_ccase"][0].lower() + subs["field_name_ccase"][1:]
            )
            funs += tobits_template.substitute(**subs)

    return class_template.substitute(
        functions=funs, telemetry_name_ccase=packet_data["name"]
    )


print("Skyward telemetry packet generator")
print("Google sheets API auth in progress...")

if auth():
    print("Auth successfull.")
else:
    print("Authentication failed.")
    exit()

sheets = listSheets()
packet_sheets = [x for x in sheets if x.endswith("Packet")]

includes = ""
conversion_classes = ""
for p in packet_sheets:
    data = loadPacketFromSheet(p)
    generatePackerClass(data, output_folder)
    conversion_classes += generateConversionFunctions(data)
    includes += "#include \"{}\"\n".format(join("bitpacking",output_folder,  data["name"] + "Packer.h"))

with open(join(output_folder, "ConversionFuncions_generated.h"), "w") as out_file:
    out_file.write(convfuns_template.substitute(classes=conversion_classes))

with open(join(output_folder, output_folder.title() + "Packets.h"), "w") as out_file:
    out_file.write(header_template.substitute(includes=includes))
